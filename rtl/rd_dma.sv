`timescale 1ns / 1ps
// =====================================================================
//  rd_dma — rd-движок kshell: LOAD (память → spm) и STREAM (память → AXIS
//  ядра) поверх out-of-order AXI4-чтения (docs/design.md §8).
//
//  Резчик (FSM S_IDLE/S_ISSUE) снимает команду с очереди rd сразу, как
//  свободен (R9: одна команда живёт в резчике вне очереди), и режет её на
//  burst'ы по BURST beat'ов. Слот окна = tail[TAGW-1:0] = ARID и занимается
//  в момент handshake AR. Тег = слот, поэтому коммит по порядку бесплатен:
//  голова окна снимает слоты строго в порядке выдачи.
//
//  R-beat'ы (RREADY = 1 всегда, место зарезервировано заранее, §8.4):
//    LOAD   → сразу в spm (порт A, без переупорядочивания);
//    STREAM → в ROB по адресу {слот, beat}; голова выдаёт ROB в AXIS по
//             порядку через 2-местный выходной буфер (чтение ROB — 1 такт).
//  Голова LOAD + done → снять слот (≤ 1 за такт), на last_of_cmd без ошибки
//  команды → sem_inc. Голова STREAM с ошибкой RRESP стоит до abort: данные
//  burst'а с SLVERR в поток не попадают никогда.
//
//  abort (§8.6): AR больше не выдаются (удерживаемый arvalid доводится до
//  handshake), остаток команды в резчике бросается, команды не принимаются;
//  все R принимаются; голова снимает слоты без выдачи в AXIS и без sem_inc.
//  Уже выданные в выходной буфер beat'ы (≤ 2, верные данные) остаются в нём —
//  AXIS запрещает снимать tvalid без handshake; их убирает soft_rst.
//  soft_rst: сброс управления в исходное состояние; разрешён только при
//  axi_quiet (гарантирует kshell, проверяет rd_dma_sva).
//
//  Тайминг: R-канал обрабатывается в такт прихода (spm-запись/ROB/счётчики
//  слота от m_axi_rid). Если нужен запас — регистровый срез AXI ставится
//  в интерконнекте перед портом, логика движка от этого не меняется.
// =====================================================================
// порт abort — имя из интерфейса плана; Verilator предупреждает о совпадении с C++ abort()
/* verilator lint_off SYMRSVDWORD */
module rd_dma
  import kshell_pkg::*;
#(
    // слотов окна (степень 2, ≥ 2)
    parameter int NTAG = 16
) (
    input logic        clk,
    input logic        rst_n,
    // §12.4; только при axi_quiet
    input logic        soft_rst,
    // ENABLE && !error: резчик может выдавать AR
    input logic        go,
    // ошибка: ROB отбрасывается, sem не растёт
    input logic        abort,
    // счётчик тактов (для задержки)
    input logic [31:0] now,

    // команда из очереди rd; cmd_nb — число burst'ов (≥ 1, гарантирует секвенсор)
    input  logic             cmd_v,
    output logic             cmd_rdy,
    input  logic             cmd_stream,
    input  logic [ SEMW-1:0] cmd_sem,
    input  logic [SPMAW-1:0] cmd_spm,
    input  logic [   AW-1:0] cmd_hbm,
    input  logic [  NBW-1:0] cmd_nb,
    input  logic [     31:0] cmd_no,

    // AXI4 AR/R
    output logic                    m_axi_arvalid,
    input  logic                    m_axi_arready,
    output logic [          AW-1:0] m_axi_araddr,
    output logic [$clog2(NTAG)-1:0] m_axi_arid,
    output logic [             7:0] m_axi_arlen,
    output logic [             2:0] m_axi_arsize,
    output logic [             1:0] m_axi_arburst,
    output logic [             3:0] m_axi_arcache,
    input  logic                    m_axi_rvalid,
    output logic                    m_axi_rready,
    input  logic [          DW-1:0] m_axi_rdata,
    input  logic [$clog2(NTAG)-1:0] m_axi_rid,
    input  logic [             1:0] m_axi_rresp,
    input  logic                    m_axi_rlast,

    // запись в spm (порт A, приоритетная)
    output logic             spm_wr_v,
    output logic [SPMAW-1:0] spm_wr_addr,
    output logic [   DW-1:0] spm_wr_data,

    // поток к ядру
    output logic          k_s_tvalid,
    input  logic          k_s_tready,
    output logic [DW-1:0] k_s_tdata,
    output logic          k_s_tlast,

    // коммит LOAD
    output logic            sem_inc_v,
    output logic [SEMW-1:0] sem_inc_idx,

    // ошибка RRESP (импульс на первый плохой beat burst'а) и номер команды
    output logic        err_v,
    output logic [31:0] err_cmd,

    // статус и perf
    // нет выданного (или выставленного) AR без последнего R
    output logic        axi_quiet,
    // резчик пуст, окно пусто, выход AXIS пуст
    output logic        idle,
    // занятых слотов окна
    output logic [ 7:0] inflight,
    // handshake AR
    output logic        pf_ar_hs,
    // у резчика есть burst, но окно полно
    output logic        pf_win_full,
    // импульс: последний R burst'а (на такт позже)
    output logic        pf_lat_v,
    // now − t_issue этого burst'а
    output logic [31:0] pf_lat
);
  /* verilator lint_on SYMRSVDWORD */

  localparam int TAGW = $clog2(NTAG);
  localparam int PW = TAGW + 1;  // указатели окна с битом обёртки: полно ≠ пусто
  localparam int BEATW = $clog2(BURST);
  // адрес burst'а выровнен по BURST_BYTES: младшие OFFW бит всегда 0
  localparam int OFFW = $clog2(BURST_BYTES);
  localparam int BLKW = AW - OFFW;  // адрес в единицах burst'а
  localparam int ROBW = NTAG * BURST;  // слов ROB: строка BURST слов на слот

  localparam logic [PW-1:0] WIN_FULL = PW'(NTAG);
  localparam logic [BEATW-1:0] LAST_BEAT = BEATW'(BURST - 1);
  localparam logic [NBW-1:0] ONE_BURST = NBW'(1);
  localparam logic [1:0] RESP_OKAY = 2'b00;
  localparam logic [1:0] BURST_INCR = 2'b01;
  // Normal Non-cacheable Bufferable: память, не устройство (§4)
  localparam logic [3:0] CACHE_NORMAL_BUF = 4'b0011;

  // ------------------------------------------------------------ резчик

  typedef enum logic [0:0] {
    S_IDLE,  // нет команды; принимаем из очереди
    S_ISSUE  // команда в резчике; выдаём burst'ы
  } sl_state_e;

  sl_state_e state_q, state_d;

  // команда в резчике
  logic c_stream_q;
  logic [SEMW-1:0] c_sem_q;
  logic [31:0] c_no_q;
  logic [SPMAW-1:0] c_spm_q;  // spm-адрес текущего burst'а
  logic [BLKW-1:0] c_blk_q;  // адрес текущего burst'а в единицах BURST_BYTES
  logic [NBW-1:0] c_left_q;  // burst'ов осталось, включая текущий
  logic c_last;  // текущий burst — последний в команде

  // окно
  logic [PW-1:0] head_q, tail_q, occ;
  logic [TAGW-1:0] tslot, hslot, rslot;
  logic win_full, win_empty;
  logic ar_hold_q;  // arvalid выставлен и не принят: держать до handshake
  logic cmd_hs, ar_hs, r_hs, r_end, r_bad;
  logic [PW-1:0] out_cnt_q;  // выданных AR без последнего R

  assign occ       = tail_q - head_q;
  assign win_full  = (occ == WIN_FULL);
  assign win_empty = (occ == '0);
  assign tslot     = tail_q[TAGW-1:0];
  assign hslot     = head_q[TAGW-1:0];
  assign rslot     = m_axi_rid;
  assign c_last    = (c_left_q == ONE_BURST);

  assign cmd_hs    = cmd_v && cmd_rdy;
  assign ar_hs     = m_axi_arvalid && m_axi_arready;
  assign r_hs      = m_axi_rvalid;  // RREADY = 1
  assign r_end     = r_hs && m_axi_rlast;
  assign r_bad     = r_hs && (m_axi_rresp != RESP_OKAY);

  // FSM резчика: next-state + выходы cmd_rdy/arvalid.
  // Новый AR — только при go && !abort и свободном слоте; уже выставленный
  // arvalid (ar_hold_q) держится до handshake даже на паузе/abort (AXI).
  always_comb begin
    state_d       = state_q;
    cmd_rdy       = 1'b0;
    m_axi_arvalid = 1'b0;
    case (state_q)
      S_IDLE: begin
        cmd_rdy = !abort;
        if (cmd_v && !abort) state_d = S_ISSUE;
      end
      S_ISSUE: begin
        m_axi_arvalid = !win_full && ((go && !abort) || ar_hold_q);
        if (m_axi_arvalid && m_axi_arready && c_last) state_d = S_IDLE;
        // abort: остаток команды бросается на границе burst'а
        else if (abort && !(ar_hold_q && !m_axi_arready)) state_d = S_IDLE;
      end
      default: state_d = S_IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) state_q <= S_IDLE;
    else state_q <= state_d;
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) ar_hold_q <= 1'b0;
    else ar_hold_q <= m_axi_arvalid && !m_axi_arready;
  end

  // поля команды (датапат: грузятся при приёме, читаются только в S_ISSUE)
  always_ff @(posedge clk) begin
    if (cmd_hs) begin
      c_stream_q <= cmd_stream;
      c_sem_q    <= cmd_sem;
      c_no_q     <= cmd_no;
    end
  end

  // адреса текущего burst'а: грузятся при приёме, растут на каждый AR
  always_ff @(posedge clk) begin
    if (cmd_hs) begin
      c_spm_q <= cmd_spm;
      c_blk_q <= cmd_hbm[AW-1:OFFW];
    end else if (ar_hs) begin
      c_spm_q <= c_spm_q + SPMAW'(BURST);
      c_blk_q <= c_blk_q + BLKW'(1);
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) c_left_q <= '0;
    else if (cmd_hs) c_left_q <= cmd_nb;
    else if (ar_hs) c_left_q <= c_left_q - ONE_BURST;
  end

  assign m_axi_araddr  = {c_blk_q, OFFW'(0)};
  assign m_axi_arid    = tslot;
  assign m_axi_arlen   = 8'(BURST - 1);
  assign m_axi_arsize  = 3'($clog2(BB));
  assign m_axi_arburst = BURST_INCR;
  assign m_axi_arcache = CACHE_NORMAL_BUF;
  assign m_axi_rready  = 1'b1;

  // ------------------------------------------------------------ окно

  // Поля слота пишутся при handshake AR (слот свободен — R для него нет).
  // Все массивы слотов resetless: читаются только для занятых слотов
  // [head, tail), а занятый слот всегда проинициализирован при выдаче.
  logic                        w_stream_q[NTAG];
  logic                        w_last_q  [NTAG];
  logic [ SEMW-1:0]            w_sem_q   [NTAG];
  logic [SPMAW-1:0]            w_spm_q   [NTAG];
  logic [     31:0]            w_no_q    [NTAG];
  logic [     31:0]            w_t_q     [NTAG];

  // счётчики/флаги слотов — регистры (обновляются по R с произвольным RID)
  // принятых beat'ов слота
  logic [ NTAG-1:0][BEATW-1:0] w_beat_q;
  // пришёл RLAST
  logic [ NTAG-1:0]            w_done_q;
  // был RRESP ≠ OKAY
  logic [ NTAG-1:0]            w_err_q;

  always_ff @(posedge clk) begin
    if (ar_hs) begin
      w_stream_q[tslot] <= c_stream_q;
      w_last_q[tslot]   <= c_last;
      w_sem_q[tslot]    <= c_sem_q;
      w_spm_q[tslot]    <= c_spm_q;
    end
  end

  always_ff @(posedge clk) begin
    if (ar_hs) begin
      w_no_q[tslot] <= c_no_q;
      w_t_q[tslot]  <= now;
    end
  end

  // Состояние слота: инициализация при выдаче (tslot) и обновление по R (rslot).
  // tslot ≠ rslot всегда: R приходит только для занятого слота (rd_dma_sva).
  always_ff @(posedge clk) begin
    if (ar_hs) w_beat_q[tslot] <= '0;
    if (r_hs) w_beat_q[rslot] <= w_beat_q[rslot] + BEATW'(1);
  end

  always_ff @(posedge clk) begin
    if (ar_hs) w_done_q[tslot] <= 1'b0;
    if (r_end) w_done_q[rslot] <= 1'b1;
  end

  always_ff @(posedge clk) begin
    if (ar_hs) w_err_q[tslot] <= 1'b0;
    if (r_bad) w_err_q[rslot] <= 1'b1;
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) tail_q <= '0;
    else if (ar_hs) tail_q <= tail_q + PW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) out_cnt_q <= '0;
    else if (ar_hs && !r_end) out_cnt_q <= out_cnt_q + PW'(1);
    else if (!ar_hs && r_end) out_cnt_q <= out_cnt_q - PW'(1);
  end

  // ------------------------------------------------------------ приём R

  // LOAD → spm сразу, по адресу spm_addr[s] + beat_cnt[s]
  assign spm_wr_v    = r_hs && !w_stream_q[rslot];
  assign spm_wr_addr = w_spm_q[rslot] + SPMAW'(w_beat_q[rslot]);
  assign spm_wr_data = m_axi_rdata;

  // STREAM → ROB по адресу {s, beat_cnt[s]} (простая 1W1R память)
  logic [DW-1:0] rob_q[ROBW];

  always_ff @(posedge clk) begin
    if (r_hs && w_stream_q[rslot]) rob_q[{rslot, w_beat_q[rslot]}] <= m_axi_rdata;
  end

  // ошибка: один импульс на burst (первый плохой beat слота); kshell берёт первую
  logic err_v_q;
  logic [31:0] err_cmd_q;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) err_v_q <= 1'b0;
    else err_v_q <= r_bad && !w_err_q[rslot];
  end

  always_ff @(posedge clk) begin
    if (r_bad && !w_err_q[rslot]) err_cmd_q <= w_no_q[rslot];
  end

  assign err_v   = err_v_q;
  assign err_cmd = err_cmd_q;

  // perf: задержка AR → последний R (регистр: мукс 16:1 + вычитатель с R-входа)
  logic pf_lat_v_q;
  logic [31:0] pf_lat_q;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) pf_lat_v_q <= 1'b0;
    else pf_lat_v_q <= r_end;
  end

  always_ff @(posedge clk) begin
    if (r_end) pf_lat_q <= now - w_t_q[rslot];
  end

  assign pf_lat_v    = pf_lat_v_q;
  assign pf_lat      = pf_lat_q;
  assign pf_ar_hs    = ar_hs;
  assign pf_win_full = (state_q == S_ISSUE) && win_full;

  // ------------------------------------------------------------ голова окна

  logic h_ready, h_stream, h_last, h_err;
  logic rd_issue, rd_end, retire;
  logic [BEATW-1:0] rd_beat_q;  // beat головы STREAM, читаемый из ROB
  logic rd_pend_q;  // чтение ROB выдано прошлым тактом → данные в буфер сейчас
  logic rd_last_q;
  logic [DW-1:0] rd_data_q;
  logic ob_in_rdy, ob_empty, k_pop, ob_credit;
  logic [1:0] ob_cnt;

  assign h_ready   = !win_empty && w_done_q[hslot];
  assign h_stream  = w_stream_q[hslot];
  assign h_last    = w_last_q[hslot];
  assign h_err     = w_err_q[hslot];

  // Кредит буфера: место под уже летящее чтение и под новое. 2 места хватает
  // на 1 beat/такт, потому что учитываем pop этого такта (tready → чтение ROB).
  assign k_pop     = k_s_tvalid && k_s_tready;
  assign ob_credit = (ob_cnt + 2'(rd_pend_q)) <= (2'd1 + 2'(k_pop));

  // STREAM без ошибки: читать beat; burst с SLVERR стоит до abort (не в поток)
  assign rd_issue  = h_ready && h_stream && !h_err && !abort && ob_credit;
  assign rd_end    = rd_issue && (rd_beat_q == LAST_BEAT);
  // LOAD: снять сразу; STREAM: после последнего beat'а или при abort (отбросить)
  assign retire    = h_ready && (h_stream ? (rd_end || abort) : 1'b1);

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) head_q <= '0;
    else if (retire) head_q <= head_q + PW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) rd_beat_q <= '0;
    else if (retire) rd_beat_q <= '0;
    else if (rd_issue) rd_beat_q <= rd_beat_q + BEATW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) rd_pend_q <= 1'b0;
    else rd_pend_q <= rd_issue;
  end

  // чтение ROB (регистр выхода памяти) + tlast: последний beat burst'а с last_of_cmd
  always_ff @(posedge clk) begin
    if (rd_issue) begin
      rd_data_q <= rob_q[{hslot, rd_beat_q}];
      rd_last_q <= rd_end && h_last;
    end
  end

  fifo_sync #(
      .W    (DW + 1),
      .DEPTH(2)
  ) u_obuf (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (soft_rst),
      .in_v   (rd_pend_q),
      .in_rdy (ob_in_rdy),
      .in_d   ({rd_last_q, rd_data_q}),
      .out_v  (k_s_tvalid),
      .out_rdy(k_s_tready),
      .out_d  ({k_s_tlast, k_s_tdata}),
      .count  (ob_cnt),
      .empty  (ob_empty)
  );

  // Накопитель ошибки команды на голове: команды коммитятся по порядку,
  // поэтому хватает одного регистра (OR по уже снятым слотам команды).
  logic cerr_q;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) cerr_q <= 1'b0;
    else if (retire) cerr_q <= h_last ? 1'b0 : (cerr_q || h_err);
  end

  assign sem_inc_v   = retire && !h_stream && h_last && !cerr_q && !h_err && !abort;
  assign sem_inc_idx = w_sem_q[hslot];

  // ------------------------------------------------------------ статус

  // выставленный arvalid тоже «не тихо»: soft_rst в этот такт потерял бы AR
  assign axi_quiet   = (out_cnt_q == '0) && !m_axi_arvalid;
  assign idle        = (state_q == S_IDLE) && win_empty && ob_empty && !rd_pend_q;
  assign inflight    = 8'(occ);

endmodule
