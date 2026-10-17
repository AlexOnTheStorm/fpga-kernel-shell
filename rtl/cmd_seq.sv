`timescale 1ns / 1ps
// =====================================================================
//  cmd_seq — секвенсор kshell: берёт 128-битный дескриптор из командной
//  FIFO, проверяет (ALIGN/RANGE/BAD_OPCODE) и выдаёт СТРОГО ПО ПОРЯДКУ в
//  очереди rd / wr / ядра; WAIT и SIGNAL исполняет сам (docs/design.md §6, §7).
//
//  Двухступенчатый тракт, одна команда за такт (без пузырей):
//    FETCH   cq_d → комбинаторное декодирование/проверка (33-битная сумма
//            spm+len/BB, 41-битная hbm+len) → регистр cur_q (+ cmd_no);
//    DISPATCH из cur_q (регистры, без логики декодирования): rd_v/wr_v/
//            krn_v/sig_v. Пока команда стоит в cur_q (очередь полна или WAIT),
//            следующая не берётся; освободившись, cur_q принимает следующую в
//            тот же такт.
//  Тяжёлая арифметика проверки живёт на FETCH (cq_d → регистры), выходы
//  движков — прямо из регистров; зато выдача на такт позже взятия.
//
//  FSM: S_IDLE (cur_q пуст) / S_DISP (в cur_q команда движка или SIGNAL) /
//  S_WAIT (в cur_q WAIT; условие $signed(sem − thr) ≥ 0, §11, sem_val —
//  регистровый выход sem_file, т.е. изменение видно со следующего такта).
//
//  Плохая команда НЕ исполняется и не хранится: на её FETCH регистрируется
//  импульс dec_err_v (код + номер команды). Секвенсор сам фиксирует halt_q и
//  работает как при error, не дожидаясь, пока kshell защёлкнет липкий error —
//  иначе после плохой команды успела бы уйти следующая. halt_q снимает только
//  soft_rst, как и error.
//
//  error (или halt_q): текущий WAIT/команда в cur_q сбрасываются, команды из
//  FIFO выкачиваются по одной за такт (cq_pop = cq_v) и отбрасываются, ничего
//  не выдаётся (§12.2). Выброшенные команды в cmd_no не считаются.
//  enable = 0: новые команды не берутся, и уже взятая не выдаётся (ничего не
//  уходит в движки); WAIT в cur_q остаётся и проверяется заново после enable.
//  soft_rst: сброс управления (cmd_no = 0).
// =====================================================================
module cmd_seq
  import kshell_pkg::*;
(
    input logic clk,
    input logic rst_n,
    input logic soft_rst,
    input logic enable,
    // липкий флаг ошибки из kshell
    input logic error,

    // командная FIFO: cq_d = {CMD3, CMD2, CMD1, CMD0}
    input  logic         cq_v,
    output logic         cq_pop,
    input  logic [127:0] cq_d,

    // очередь rd (LOAD / STREAM)
    output logic             rd_v,
    input  logic             rd_rdy,
    output logic             rd_stream,
    output logic [ SEMW-1:0] rd_sem,
    output logic [SPMAW-1:0] rd_spm,
    output logic [   AW-1:0] rd_hbm,
    output logic [  NBW-1:0] rd_nb,
    output logic [     31:0] rd_no,

    // очередь wr (STORE)
    output logic             wr_v,
    input  logic             wr_rdy,
    output logic [ SEMW-1:0] wr_sem,
    output logic [SPMAW-1:0] wr_spm,
    output logic [   AW-1:0] wr_hbm,
    output logic [  NBW-1:0] wr_nb,
    output logic [     31:0] wr_no,

    // очередь ядра (RUN)
    output logic                  krn_v,
    input  logic                  krn_rdy,
    output logic [SEMW-1:0]       krn_sem,
    output logic [     3:0][31:0] krn_arg,
    output logic [    31:0]       krn_no,

    // семафоры
    input  logic [NSEM-1:0][31:0] sem_val,
    output logic                  sig_v,
    output logic [SEMW-1:0]       sig_idx,
    output logic [    31:0]       sig_val,

    // ошибка декодирования (импульс) и статус
    output logic        dec_err_v,
    output err_e        dec_err_code,
    output logic [31:0] dec_err_cmd,
    output logic [31:0] cmd_no,
    output logic        in_wait,
    output logic        pf_wait
);

  // Дескриптор §6. CMD1 = hbm_lo / thr / inc / arg0; CMD2 целиком = arg1 (hbm_hi = [7:0]);
  // CMD3 = len / arg2; arg3 RUN = spm (CMD0[15:0]) с расширением нулями.
  typedef struct packed {
    logic [31:0] len;     // CMD3
    logic [23:0] rsv2;    // CMD2[31:8]
    logic [7:0]  hbm_hi;  // CMD2[7:0]
    logic [31:0] hbm_lo;  // CMD1
    logic [3:0]  op;      // CMD0[31:28]
    logic        rsv0a;   // CMD0[27]
    logic [2:0]  sem;     // CMD0[26:24]
    logic [7:0]  rsv0b;   // CMD0[23:16]
    logic [15:0] spm;     // CMD0[15:0]
  } desc_t;

  // len — в байтах; nb = len / BURST_BYTES, слов spm = len / BB
  localparam int OFFW = $clog2(BURST_BYTES);
  localparam int BBW = $clog2(BB);
  localparam int HSUMW = AW + 1;
  localparam int SSUMW = 33;
  localparam logic [HSUMW-1:0] HBM_LIMIT = HSUMW'(1) << AW;
  localparam logic [SSUMW-1:0] SPM_LIMIT = SSUMW'(SPM_WORDS);
  localparam logic [31:0] ZERO32 = 32'd0;

  typedef enum logic [1:0] {
    S_IDLE,  // cur_q пуст: можно брать команду
    S_DISP,  // в cur_q LOAD/STREAM/STORE/RUN/SIGNAL: ждём очередь, затем выдаём
    S_WAIT  // в cur_q WAIT: ждём семафор
  } seq_state_e;

  seq_state_e state_q, state_d;
  desc_t cq_f, cur_q;
  logic [31:0] no_q;
  logic halt_q, err_v_q;
  err_e err_code_q;
  logic [31:0] err_cmd_q;

  assign cq_f = desc_t'(cq_d);

  // ------------------------------------------------------------ FETCH: декодирование cq_d

  logic f_dma, f_spm_chk, f_op_ok, f_wait;
  logic [HSUMW-1:0] f_hend;
  logic [SSUMW-1:0] f_send;
  logic f_align_bad, f_range_bad, f_bad;
  err_e f_code;

  assign f_dma = (cq_f.op == OP_LOAD) || (cq_f.op == OP_STREAM) || (cq_f.op == OP_STORE);
  // STREAM идёт в AXIS, scratchpad не трогает → RANGE только у LOAD/STORE
  assign f_spm_chk = (cq_f.op == OP_LOAD) || (cq_f.op == OP_STORE);
  assign f_op_ok = f_dma || (cq_f.op == OP_WAIT) || (cq_f.op == OP_SIGNAL) || (cq_f.op == OP_RUN);
  assign f_wait = (cq_f.op == OP_WAIT);

  // суммы без потери переноса: hbm+len в 41 бите (len до 2^32), spm+len/BB в 33 битах
  assign f_hend = HSUMW'({cq_f.hbm_hi, cq_f.hbm_lo}) + HSUMW'(cq_f.len);
  assign f_send = SSUMW'(cq_f.spm) + SSUMW'(cq_f.len[31:BBW]);

  assign f_align_bad = f_dma && ((cq_f.len == ZERO32) || (cq_f.len[OFFW-1:0] != '0)
                                 || (cq_f.hbm_lo[OFFW-1:0] != '0) || (f_hend > HBM_LIMIT));
  assign f_range_bad = f_spm_chk && (f_send > SPM_LIMIT);
  assign f_bad = !f_op_ok || f_align_bad || f_range_bad;
  // приоритет ALIGN > RANGE > BAD_OPCODE (§12.2); у неверного кода поля не проверяются
  assign f_code = f_align_bad ? E_ALIGN : (f_range_bad ? E_RANGE : E_OPCODE);

  // ------------------------------------------------------------ DISPATCH: cur_q

  logic c_rd, c_wr, c_krn, c_sig, disp_rdy;
  logic wait_pass;
  logic err_eff, disp_ok;
  logic [31:0] wait_diff;

  assign c_rd = (cur_q.op == OP_LOAD) || (cur_q.op == OP_STREAM);
  assign c_wr = (cur_q.op == OP_STORE);
  assign c_krn = (cur_q.op == OP_RUN);
  assign c_sig = (cur_q.op == OP_SIGNAL);
  // выбранная команда может уйти: её очередь готова (SIGNAL — всегда)
  assign disp_rdy = (c_rd && rd_rdy) || (c_wr && wr_rdy) || (c_krn && krn_rdy) || c_sig;

  // знаковая разность — безопасна при переполнении счётчика (§11)
  assign wait_diff = sem_val[cur_q.sem] - cur_q.hbm_lo;
  assign wait_pass = !wait_diff[31];

  assign err_eff = error || halt_q;
  assign disp_ok = (state_q == S_DISP) && enable && !err_eff;

  // next-state и cq_pop. cur_q освобождается (free), когда выдан/пройден: тогда в этот же такт
  // берём следующую команду (если enable и она есть).
  logic free;

  always_comb begin
    state_d = S_IDLE;
    cq_pop  = 1'b0;
    free    = 1'b0;
    if (err_eff) begin
      // очистка: всё отбрасываем, state_d = S_IDLE
      cq_pop = cq_v;
    end else begin
      case (state_q)
        S_IDLE:  free = 1'b1;
        S_DISP: begin
          state_d = S_DISP;
          free    = enable && disp_rdy;
        end
        S_WAIT: begin
          state_d = S_WAIT;
          free    = enable && wait_pass;
        end
        default: state_d = S_IDLE;
      endcase
      if (free) begin
        state_d = S_IDLE;
        if (enable && cq_v) begin
          cq_pop = 1'b1;
          // плохая команда не хранится; выдавать нечего
          if (!f_bad) state_d = f_wait ? S_WAIT : S_DISP;
        end
      end
    end
  end

  // взята из FIFO и не отброшена по error
  logic take;
  assign take = cq_pop && !err_eff;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) state_q <= S_IDLE;
    else state_q <= state_d;
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) cmd_no <= '0;
    else if (take) cmd_no <= cmd_no + 32'd1;
  end

  // датапат: грузится на take, читается только в S_DISP/S_WAIT
  always_ff @(posedge clk) begin
    if (take) begin
      cur_q <= cq_f;
      no_q  <= cmd_no;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) halt_q <= 1'b0;
    else if (take && f_bad) halt_q <= 1'b1;
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) err_v_q <= 1'b0;
    else err_v_q <= take && f_bad;
  end

  always_ff @(posedge clk) begin
    if (take && f_bad) begin
      err_code_q <= f_code;
      err_cmd_q  <= cmd_no;
    end
  end

  // ------------------------------------------------------------ выходы

  assign rd_v         = disp_ok && c_rd;
  assign rd_stream    = (cur_q.op == OP_STREAM);
  assign rd_sem       = cur_q.sem;
  assign rd_spm       = cur_q.spm[SPMAW-1:0];
  assign rd_hbm       = {cur_q.hbm_hi, cur_q.hbm_lo};
  assign rd_nb        = cur_q.len[OFFW+:NBW];
  assign rd_no        = no_q;

  assign wr_v         = disp_ok && c_wr;
  assign wr_sem       = cur_q.sem;
  assign wr_spm       = cur_q.spm[SPMAW-1:0];
  assign wr_hbm       = {cur_q.hbm_hi, cur_q.hbm_lo};
  assign wr_nb        = cur_q.len[OFFW+:NBW];
  assign wr_no        = no_q;

  assign krn_v        = disp_ok && c_krn;
  assign krn_sem      = cur_q.sem;
  assign krn_arg[0]   = cur_q.hbm_lo;
  assign krn_arg[1]   = {cur_q.rsv2, cur_q.hbm_hi};
  assign krn_arg[2]   = cur_q.len;
  assign krn_arg[3]   = {16'd0, cur_q.spm};
  assign krn_no       = no_q;

  assign sig_v        = disp_ok && c_sig;
  assign sig_idx      = cur_q.sem;
  assign sig_val      = cur_q.hbm_lo;

  assign dec_err_v    = err_v_q;
  assign dec_err_code = err_code_q;
  assign dec_err_cmd  = err_cmd_q;

  // WAIT «в работе»: ждёт условие (на error прерван — сразу низкий)
  assign in_wait      = (state_q == S_WAIT) && !err_eff;
  assign pf_wait      = in_wait && !wait_pass;

endmodule
