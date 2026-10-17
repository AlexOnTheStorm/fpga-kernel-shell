`timescale 1ns / 1ps
// =====================================================================
//  wr_dma — wr-движок kshell: STORE (spm → память) поверх AXI4-записи
//  (docs/design.md §9).
//
//  На каждый burst: СНАЧАЛА BURST слов читаются из spm (порт A, только в
//  такты, когда rd_dma не занял порт: spm_rd_gnt) в W-FIFO глубиной
//  2×BURST, ПОТОМ выдаётся AW (когда в FIFO лежит полный burst), ПОТОМ W
//  подряд без пауз со стороны wr_dma (данные уже в FIFO), WLAST на beat'е
//  BURST. Так W никогда не стоит после принятого AW: обёртка не держит
//  канал записи интерконнекта (голодание порта A задерживает только AW).
//
//  Чтение spm и выдача AW развязаны: fetch бежит вперёд по команде, пока
//  есть место в FIFO (счётчик готовых burst'ов rdy_bursts_q — между
//  «последнее слово burst'а легло в FIFO» и «AW выдан»).
//
//  AWID = 0 → B приходят по порядку. FIFO бёрстов в полёте (глубина WR_OUT,
//  заполняется на handshake AW, снимается на B) хранит {last_of_cmd, sem,
//  cmd_no}; её заполнение = inflight, а «полна» запрещает новый AW. Накопитель
//  ошибки команды cerr_q: BRESP ≠ OKAY → err_v/err_cmd и sem команды не
//  растёт (B идут по порядку, одного регистра хватает).
//
//  go = 0: новые AW (и предвыборка) не стартуют, выставленный awvalid
//  доводится до handshake, W/B выданных AW завершаются.
//  abort (§12.2): новый AW не выдаётся, остаток команды бросается, новые
//  команды не принимаются; W уже выданных AW дописывается (их данные в
//  FIFO), B принимаются, sem не растёт. Слова в W-FIFO, не покрытые AW,
//  остаются до soft_rst (она их очищает).
//  soft_rst: сброс управления и очистка FIFO; разрешён только при axi_quiet
//  (гарантирует kshell, проверяет wr_dma_sva). abort должен держаться до
//  soft_rst; снятие abort без soft_rst вне контракта (остатки W-FIFO).
// =====================================================================
// порт abort — имя из интерфейса плана; Verilator предупреждает о совпадении с C++ abort()
/* verilator lint_off SYMRSVDWORD */
module wr_dma
  import kshell_pkg::*;
(
    input logic clk,
    input logic rst_n,
    // §12.4; только при axi_quiet
    input logic soft_rst,
    // ENABLE && !error: резчик может стартовать burst
    input logic go,
    // ошибка: AW больше не выдаются, команды не принимаются
    input logic abort,

    // команда из очереди wr; cmd_nb — число burst'ов (≥ 1, гарантирует секвенсор)
    input  logic             cmd_v,
    output logic             cmd_rdy,
    input  logic [ SEMW-1:0] cmd_sem,
    input  logic [SPMAW-1:0] cmd_spm,
    input  logic [   AW-1:0] cmd_hbm,
    input  logic [  NBW-1:0] cmd_nb,
    input  logic [     31:0] cmd_no,

    // чтение spm (порт A, неприоритетное): gnt = req && !wr; данные через такт после gnt
    output logic             spm_rd_req,
    output logic [SPMAW-1:0] spm_rd_addr,
    input  logic             spm_rd_gnt,
    input  logic [   DW-1:0] spm_rd_data,

    // AXI4 AW/W/B
    output logic          m_axi_awvalid,
    input  logic          m_axi_awready,
    output logic [AW-1:0] m_axi_awaddr,
    output logic [   0:0] m_axi_awid,
    output logic [   7:0] m_axi_awlen,
    output logic [   2:0] m_axi_awsize,
    output logic [   1:0] m_axi_awburst,
    output logic [   3:0] m_axi_awcache,
    output logic          m_axi_wvalid,
    input  logic          m_axi_wready,
    output logic [DW-1:0] m_axi_wdata,
    output logic [BB-1:0] m_axi_wstrb,
    output logic          m_axi_wlast,
    input  logic          m_axi_bvalid,
    output logic          m_axi_bready,
    input  logic [   0:0] m_axi_bid,
    input  logic [   1:0] m_axi_bresp,

    // коммит STORE: последний B команды без ошибки
    output logic            sem_inc_v,
    output logic [SEMW-1:0] sem_inc_idx,

    // ошибка BRESP (импульс на каждый плохой B) и номер команды
    output logic        err_v,
    output logic [31:0] err_cmd,

    // статус и perf
    // нет выданного (или выставленного) AW без B (⊇ AW с недописанным W)
    output logic       axi_quiet,
    // нет команды, W-FIFO пуста, в полёте ничего
    output logic       idle,
    // burst'ов записи в полёте (AW выдан, B не получен)
    output logic [7:0] inflight,
    // handshake AW
    output logic       pf_aw_hs
);
  /* verilator lint_on SYMRSVDWORD */

  localparam int BEATW = $clog2(BURST);
  // адрес burst'а выровнен по BURST_BYTES: младшие OFFW бит всегда 0
  localparam int OFFW = $clog2(BURST_BYTES);
  localparam int BLKW = AW - OFFW;  // адрес в единицах burst'а
  localparam int WF_DEPTH = 2 * BURST;
  localparam int WFCW = $clog2(WF_DEPTH + 1);  // ширина счётчика W-FIFO
  localparam int WOW = $clog2(
      WR_OUT + 1
  );  // ширина счётчика burst'ов в полёте
  localparam int IFW = 1 + SEMW + 32;  // {last_of_cmd, sem, cmd_no}

  localparam logic [BEATW-1:0] LAST_BEAT = BEATW'(BURST - 1);
  localparam logic [NBW-1:0] ONE_BURST = NBW'(1);
  localparam logic [1:0] RESP_OKAY = 2'b00;
  localparam logic [1:0] BURST_INCR = 2'b01;
  // Normal Non-cacheable Non-bufferable: B означает «данные в памяти» (§4)
  localparam logic [3:0] CACHE_NORMAL_NB = 4'b0010;
  localparam logic [WFCW-1:0] WF_FULL_LESS_1 = WFCW'(WF_DEPTH - 1);

  // ------------------------------------------------------------ резчик

  typedef enum logic [0:0] {
    S_IDLE,  // нет команды; принимаем из очереди
    S_RUN    // команда в работе: fetch из spm и выдача AW
  } wr_state_e;

  wr_state_e state_q, state_d;

  // команда в работе
  logic [SEMW-1:0] c_sem_q;
  logic [31:0] c_no_q;
  // fetch: spm-адрес следующего слова, beat в burst'е, burst'ов осталось читать
  logic [SPMAW-1:0] f_spm_q;
  logic [BEATW-1:0] f_beat_q;
  logic [NBW-1:0] f_left_q;
  // AW: адрес следующего burst'а (в единицах BURST_BYTES) и сколько осталось
  logic [BLKW-1:0] a_blk_q;
  logic [NBW-1:0] a_left_q;

  logic cmd_hs, aw_hs, w_hs, b_hs, b_bad;
  logic aw_hold_q;  // awvalid выставлен и не принят: держать до handshake
  logic a_last;  // выдаваемый burst — последний в команде
  assign a_last = (a_left_q == ONE_BURST);

  assign cmd_hs = cmd_v && cmd_rdy;
  assign aw_hs  = m_axi_awvalid && m_axi_awready;
  assign w_hs   = m_axi_wvalid && m_axi_wready;
  assign b_hs   = m_axi_bvalid && m_axi_bready;
  assign b_bad  = b_hs && (m_axi_bresp != RESP_OKAY);

  // W-FIFO и очередь burst'ов в полёте (объявлены ниже, нужны выше)
  logic wf_in_rdy, wf_out_v, wf_empty;
  logic [WFCW-1:0] wf_count;
  logic [  DW-1:0] wf_out_d;
  logic if_in_rdy, if_out_v, if_empty;
  logic [ WOW-1:0] if_count;
  logic [ IFW-1:0] if_out_d;
  logic [WFCW-1:0] rdy_bursts_q;  // полных burst'ов в W-FIFO без AW
  logic
      rd_pend_q,
      rd_last_q;  // чтение spm выдано прошлым тактом / это был последний beat
  logic [WOW-1:0] w_pend_q;  // AW выдано, W не дописан

  // FSM: next-state + cmd_rdy/awvalid. Новый AW — только при go && !abort, полном
  // burst'е в W-FIFO и свободном месте в очереди полёта; уже выставленный awvalid
  // (aw_hold_q) держится до handshake даже на паузе/abort (AXI).
  always_comb begin
    state_d       = state_q;
    cmd_rdy       = 1'b0;
    m_axi_awvalid = 1'b0;
    case (state_q)
      S_IDLE: begin
        cmd_rdy = !abort;
        if (cmd_v && !abort) state_d = S_RUN;
      end
      S_RUN: begin
        m_axi_awvalid = (rdy_bursts_q != '0) && if_in_rdy && ((go && !abort) || aw_hold_q);
        if (m_axi_awvalid && m_axi_awready && a_last) state_d = S_IDLE;
        // abort: остаток команды бросается на границе burst'а
        else if (abort && !(aw_hold_q && !m_axi_awready)) state_d = S_IDLE;
      end
      default: state_d = S_IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) state_q <= S_IDLE;
    else state_q <= state_d;
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) aw_hold_q <= 1'b0;
    else aw_hold_q <= m_axi_awvalid && !m_axi_awready;
  end

  // поля команды (датапат: грузятся при приёме, читаются только в S_RUN)
  always_ff @(posedge clk) begin
    if (cmd_hs) begin
      c_sem_q <= cmd_sem;
      c_no_q  <= cmd_no;
    end
  end

  // сторона AW: адрес растёт на каждый handshake AW
  always_ff @(posedge clk) begin
    if (cmd_hs) a_blk_q <= cmd_hbm[AW-1:OFFW];
    else if (aw_hs) a_blk_q <= a_blk_q + BLKW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) a_left_q <= '0;
    else if (cmd_hs) a_left_q <= cmd_nb;
    else if (aw_hs) a_left_q <= a_left_q - ONE_BURST;
  end

  assign m_axi_awaddr  = {a_blk_q, OFFW'(0)};
  assign m_axi_awid    = 1'b0;
  assign m_axi_awlen   = 8'(BURST - 1);
  assign m_axi_awsize  = 3'($clog2(BB));
  assign m_axi_awburst = BURST_INCR;
  assign m_axi_awcache = CACHE_NORMAL_NB;
  assign m_axi_bready  = 1'b1;
  assign pf_aw_hs      = aw_hs;

  // ------------------------------------------------------------ fetch spm → W-FIFO

  logic rd_fire;
  logic f_active;

  // Запрос только пока есть что читать, go && !abort и гарантированное место в FIFO:
  // слова, уже лежащие, + летящее (rd_pend_q) + это (pop только освобождает место).
  assign f_active    = (state_q == S_RUN) && (f_left_q != '0);
  assign spm_rd_req  = f_active && go && !abort && (wf_count < WF_FULL_LESS_1 + WFCW'(!rd_pend_q));
  assign spm_rd_addr = f_spm_q;
  assign rd_fire     = spm_rd_req && spm_rd_gnt;

  always_ff @(posedge clk) begin
    if (cmd_hs) f_spm_q <= cmd_spm;
    else if (rd_fire) f_spm_q <= f_spm_q + SPMAW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) f_beat_q <= '0;
    else if (cmd_hs) f_beat_q <= '0;
    else if (rd_fire) f_beat_q <= f_beat_q + BEATW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) f_left_q <= '0;
    else if (cmd_hs) f_left_q <= cmd_nb;
    else if (rd_fire && f_beat_q == LAST_BEAT) f_left_q <= f_left_q - ONE_BURST;
  end

  // spm отдаёт данные через такт после gnt
  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) rd_pend_q <= 1'b0;
    else rd_pend_q <= rd_fire;
  end

  always_ff @(posedge clk) begin
    if (rd_fire) rd_last_q <= (f_beat_q == LAST_BEAT);
  end

  // полный burst в FIFO: +1 на последнем слове (после push), −1 на AW
  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) rdy_bursts_q <= '0;
    else if (rd_pend_q && rd_last_q && !aw_hs) rdy_bursts_q <= rdy_bursts_q + WFCW'(1);
    else if (!(rd_pend_q && rd_last_q) && aw_hs) rdy_bursts_q <= rdy_bursts_q - WFCW'(1);
  end

  fifo_sync #(
      .W    (DW),
      .DEPTH(WF_DEPTH)
  ) u_wfifo (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (soft_rst),
      .in_v   (rd_pend_q),
      .in_rdy (wf_in_rdy),
      .in_d   (spm_rd_data),
      .out_v  (wf_out_v),
      .out_rdy(m_axi_wready && (w_pend_q != '0)),
      .out_d  (wf_out_d),
      .count  (wf_count),
      .empty  (wf_empty)
  );

  // ------------------------------------------------------------ W

  // W идёт только после handshake AW (w_pend_q > 0) и не прерывается: данные
  // burst'а уже в FIFO к моменту AW.
  logic [BEATW-1:0] w_beat_q;

  assign m_axi_wvalid = (w_pend_q != '0) && wf_out_v;
  assign m_axi_wdata  = wf_out_d;
  assign m_axi_wstrb  = '1;
  assign m_axi_wlast  = (w_beat_q == LAST_BEAT);

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) w_beat_q <= '0;
    else if (w_hs) w_beat_q <= m_axi_wlast ? '0 : w_beat_q + BEATW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) w_pend_q <= '0;
    else if (aw_hs && !(w_hs && m_axi_wlast)) w_pend_q <= w_pend_q + WOW'(1);
    else if (!aw_hs && (w_hs && m_axi_wlast)) w_pend_q <= w_pend_q - WOW'(1);
  end

  // ------------------------------------------------------------ B и burst'ы в полёте

  // {last_of_cmd, sem, cmd_no} на каждый AW; B идут по порядку (AWID = 0)
  fifo_sync #(
      .W    (IFW),
      .DEPTH(WR_OUT)
  ) u_inflight (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (soft_rst),
      .in_v   (aw_hs),
      .in_rdy (if_in_rdy),
      .in_d   ({a_last, c_sem_q, c_no_q}),
      .out_v  (if_out_v),
      .out_rdy(b_hs),
      .out_d  (if_out_d),
      .count  (if_count),
      .empty  (if_empty)
  );

  logic h_last;
  logic [SEMW-1:0] h_sem;
  logic [31:0] h_no;
  assign {h_last, h_sem, h_no} = if_out_d;

  // Накопитель ошибки команды: B по порядку, поэтому хватает одного регистра
  // (OR по уже принятым B команды; сбрасывается на её последнем B).
  logic cerr_q;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) cerr_q <= 1'b0;
    else if (b_hs) cerr_q <= h_last ? 1'b0 : (cerr_q || b_bad);
  end

  // sem — регистр: импульс на такт позже последнего B, когда axi_quiet уже поднят
  // (хост/kshell, увидев sem, видит и тишину AXI). abort на выходе подавляет импульс.
  logic sem_q;
  logic [SEMW-1:0] sem_idx_q;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) sem_q <= 1'b0;
    else sem_q <= b_hs && h_last && !cerr_q && !b_bad;
  end

  always_ff @(posedge clk) begin
    if (b_hs) sem_idx_q <= h_sem;
  end

  assign sem_inc_v   = sem_q && !abort;
  assign sem_inc_idx = sem_idx_q;

  // ошибка: импульс на каждый плохой B (kshell фиксирует первую)
  logic err_v_q;
  logic [31:0] err_cmd_q;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) err_v_q <= 1'b0;
    else err_v_q <= b_bad;
  end

  always_ff @(posedge clk) begin
    if (b_bad) err_cmd_q <= h_no;
  end

  assign err_v     = err_v_q;
  assign err_cmd   = err_cmd_q;

  // ------------------------------------------------------------ статус

  // выставленный awvalid тоже «не тихо»: soft_rst в этот такт потерял бы AW
  assign axi_quiet = if_empty && !m_axi_awvalid;
  assign idle      = (state_q == S_IDLE) && wf_empty && if_empty && !rd_pend_q && !sem_q;
  assign inflight  = 8'(if_count);

endmodule
