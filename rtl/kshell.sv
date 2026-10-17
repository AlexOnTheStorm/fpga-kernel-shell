`timescale 1ns / 1ps
// =====================================================================
//  kshell — универсальная обёртка вычислительного ядра (docs/design.md §2).
//  Соединяет axil_regs → командная FIFO → cmd_seq → очереди rd/wr/ядра →
//  rd_dma / wr_dma / контроллер ядра; spm и sem_file общие; kshell_perf.
//  Своя логика здесь — только системная:
//
//  Ошибки (§12.2). Источники — регистровые импульсы: rd_dma.err_v (RRESP),
//  wr_dma.err_v (BRESP), cmd_seq.dec_err_v (ALIGN/RANGE/BAD_OPCODE), k_err
//  (KERNEL, регистрируется здесь — путь от чужого ядра не идёт в логику
//  оболочки комбинаторно), watchdog (TIMEOUT). Фиксируется ПЕРВАЯ: код и
//  номер команды защёлкиваются только при !error_q, в одном такте — по
//  приоритету TIMEOUT > RRESP > BRESP > KERNEL > ALIGN > RANGE > BAD_OPCODE.
//  err_now = error_q || импульс этого такта:
//    abort = err_now — липкий до SOFT_RESET (контракт wr_dma: abort держится
//            до soft_rst, иначе остатки W-FIFO уйдут без AW);
//    go    = ENABLE && !err_now;
//    приращения семафоров от rd/wr/ядра гасятся при err_now (§11, R10) —
//    закрывает такт между регистровым err_v движка и липким флагом.
//  go/abort НЕ зависят от axi_quiet (у wr_dma axi_quiet комбинаторно зависит
//  от go/abort через awvalid — была бы петля).
//
//  Контроллер ядра (§7): очередь ядра → k_start (импульс) при !krn_busy и go;
//  krn_busy ставится по k_start, снимается по k_done или soft_rst; по k_done без
//  ошибки — sem += 1 семафору этого RUN. k_start только при !krn_busy_q (не в
//  такте k_done): правило §5.4 «новый k_start после k_done» без гонки.
//
//  Предикаты (§12.1): AXI_QUIET = тишина rd и wr; IDLE = AXI_QUIET, все FIFO
//  пусты, движки idle (окно, ROB-выход/AXIS, W-FIFO), ядро свободно,
//  секвенсор пуст (!seq_busy: взятая команда или WAIT).
//
//  SOFT_RESET (§12.4): soft_rst = soft_rst_req && AXI_QUIET (иначе запрос
//  игнорируется — хост повторяет). Комбинаторно: регистр опоздал бы на такт,
//  за который мог бы выйти новый AR/AW. Очищает FIFO, движки, секвенсор,
//  семафоры, perf, ошибку, cq_ovf; k_rst_n = rst_n && !soft_rst (один такт).
//
//  Watchdog (§12.5): считает такты при !AXI_QUIET без R-beat'а и без B;
//  любой R или B обнуляет; достиг WDOG_CYCLES → TIMEOUT (один импульс:
//  счётчик насыщается). Пауза, WAIT и медленное ядро — это AXI_QUIET.
//
//  cq_ovf (§6): DOORBELL при полной командной FIFO — липкий флаг, НЕ ошибка.
// =====================================================================
module kshell
  import kshell_pkg::*;
#(
    parameter int NTAG        = 16,
    parameter int WDOG_CYCLES = 1 << 20
) (
    input logic clk,
    input logic rst_n,

    // AXI4-Lite slave (хост)
    input  logic        s_axil_awvalid,
    output logic        s_axil_awready,
    input  logic [11:0] s_axil_awaddr,
    input  logic        s_axil_wvalid,
    output logic        s_axil_wready,
    input  logic [31:0] s_axil_wdata,
    input  logic [ 3:0] s_axil_wstrb,
    output logic        s_axil_bvalid,
    input  logic        s_axil_bready,
    output logic [ 1:0] s_axil_bresp,
    input  logic        s_axil_arvalid,
    output logic        s_axil_arready,
    input  logic [11:0] s_axil_araddr,
    output logic        s_axil_rvalid,
    input  logic        s_axil_rready,
    output logic [31:0] s_axil_rdata,
    output logic [ 1:0] s_axil_rresp,

    // AXI4 master (память): AR/R от rd_dma, AW/W/B от wr_dma
    output logic                    m_axi_arvalid,
    input  logic                    m_axi_arready,
    output logic [          AW-1:0] m_axi_araddr,
    output logic [$clog2(NTAG)-1:0] m_axi_arid,
    output logic [             7:0] m_axi_arlen,
    output logic [             2:0] m_axi_arsize,
    output logic [             1:0] m_axi_arburst,
    output logic [             3:0] m_axi_arcache,
    output logic [             2:0] m_axi_arprot,
    output logic [             3:0] m_axi_arqos,
    output logic                    m_axi_arlock,
    input  logic                    m_axi_rvalid,
    output logic                    m_axi_rready,
    input  logic [          DW-1:0] m_axi_rdata,
    input  logic [$clog2(NTAG)-1:0] m_axi_rid,
    input  logic [             1:0] m_axi_rresp,
    input  logic                    m_axi_rlast,
    output logic                    m_axi_awvalid,
    input  logic                    m_axi_awready,
    output logic [          AW-1:0] m_axi_awaddr,
    output logic [             0:0] m_axi_awid,
    output logic [             7:0] m_axi_awlen,
    output logic [             2:0] m_axi_awsize,
    output logic [             1:0] m_axi_awburst,
    output logic [             3:0] m_axi_awcache,
    output logic [             2:0] m_axi_awprot,
    output logic [             3:0] m_axi_awqos,
    output logic                    m_axi_awlock,
    output logic                    m_axi_wvalid,
    input  logic                    m_axi_wready,
    output logic [          DW-1:0] m_axi_wdata,
    output logic [          BB-1:0] m_axi_wstrb,
    output logic                    m_axi_wlast,
    input  logic                    m_axi_bvalid,
    output logic                    m_axi_bready,
    input  logic [             0:0] m_axi_bid,
    input  logic [             1:0] m_axi_bresp,

    // контракт ядра §5
    output logic                  k_rst_n,
    output logic                  k_start,
    output logic [     3:0][31:0] k_arg,
    input  logic                  k_done,
    input  logic                  k_err,
    output logic                  k_s_tvalid,
    input  logic                  k_s_tready,
    output logic [  DW-1:0]       k_s_tdata,
    output logic                  k_s_tlast,
    input  logic                  k_spm_en,
    input  logic                  k_spm_we,
    input  logic [SPMAW-1:0]      k_spm_addr,
    input  logic [  DW-1:0]       k_spm_wdata,
    input  logic [  BB-1:0]       k_spm_wstrb,
    output logic [  DW-1:0]       k_spm_rdata
);

  localparam int RQW = 1 + SEMW + SPMAW + AW + NBW + 32;  // {stream, sem, spm, hbm, nb, no}
  localparam int WQW = SEMW + SPMAW + AW + NBW + 32;  // {sem, spm, hbm, nb, no}
  localparam int KQW = SEMW + 4 * 32 + 32;  // {sem, arg[4], no}
  localparam int CQCW = $clog2(CQ_DEPTH + 1);
  localparam int EQCW = $clog2(EQ_DEPTH + 1);
  localparam int WDW = $clog2(WDOG_CYCLES + 1);
  localparam logic [WDW-1:0] WDOG_LIMIT = WDW'(WDOG_CYCLES);
  localparam logic [WDW-1:0] WDOG_LAST = WDW'(WDOG_CYCLES - 1);

  // ------------------------------------------------------------ объявления

  // регистры хоста
  logic enable, soft_rst_req, perf_snap;
  logic cq_push;
  logic [127:0] cq_in_d;
  logic sem_we;
  logic [SEMW-1:0] sem_idx;
  logic [31:0] sem_wval;
  logic [31:0] status;
  logic [7:0][31:0] perf;

  // системные флаги
  logic soft_rst, axi_quiet, idle;
  logic error_q, err_any, err_now, go, abort;
  err_e err_code_q;
  logic [31:0] err_cmd_q;
  logic cq_ovf_q;
  logic krn_busy_q;
  logic [31:0] now_q;

  // командная FIFO
  logic cq_in_rdy, cq_v, cq_pop, cq_empty;
  logic [127:0] cq_d;
  logic [CQCW-1:0] cq_count;

  // секвенсор
  logic s_rd_v, s_rd_rdy, s_rd_stream;
  logic [SEMW-1:0] s_rd_sem;
  logic [SPMAW-1:0] s_rd_spm;
  logic [AW-1:0] s_rd_hbm;
  logic [NBW-1:0] s_rd_nb;
  logic [31:0] s_rd_no;
  logic s_wr_v, s_wr_rdy;
  logic [SEMW-1:0] s_wr_sem;
  logic [SPMAW-1:0] s_wr_spm;
  logic [AW-1:0] s_wr_hbm;
  logic [NBW-1:0] s_wr_nb;
  logic [31:0] s_wr_no;
  logic s_krn_v, s_krn_rdy;
  logic [SEMW-1:0] s_krn_sem;
  logic [3:0][31:0] s_krn_arg;
  logic [31:0] s_krn_no;
  logic [NSEM-1:0][31:0] sem_val;
  logic sig_v;
  logic [SEMW-1:0] sig_idx;
  logic [31:0] sig_val;
  logic dec_err_v;
  err_e dec_err_code;
  logic [31:0] dec_err_cmd, cmd_no;
  logic in_wait, pf_wait, seq_busy;

  // очереди движков
  logic rq_v, rq_rdy, rq_empty, rq_stream;
  logic [SEMW-1:0] rq_sem;
  logic [SPMAW-1:0] rq_spm;
  logic [AW-1:0] rq_hbm;
  logic [NBW-1:0] rq_nb;
  logic [31:0] rq_no;
  logic [EQCW-1:0] rq_count;
  logic wq_v, wq_rdy, wq_empty;
  logic [SEMW-1:0] wq_sem;
  logic [SPMAW-1:0] wq_spm;
  logic [AW-1:0] wq_hbm;
  logic [NBW-1:0] wq_nb;
  logic [31:0] wq_no;
  logic [EQCW-1:0] wq_count;
  logic kq_v, kq_empty;
  logic [SEMW-1:0] kq_sem;
  logic [3:0][31:0] kq_arg;
  logic [31:0] kq_no;
  logic [EQCW-1:0] kq_count;

  // rd_dma
  logic rd_spm_wr_v;
  logic [SPMAW-1:0] rd_spm_wr_addr;
  logic [DW-1:0] rd_spm_wr_data;
  logic rd_sem_inc_v;
  logic [SEMW-1:0] rd_sem_inc_idx;
  logic rd_err_v;
  logic [31:0] rd_err_cmd;
  logic rd_axi_quiet, rd_idle;
  logic [7:0] rd_inflight;
  logic pf_ar_hs, pf_win_full, pf_lat_v;
  logic [31:0] pf_lat;

  // wr_dma
  logic wr_spm_rd_req, wr_spm_rd_gnt;
  logic [SPMAW-1:0] wr_spm_rd_addr;
  logic [DW-1:0] wr_spm_rd_data;
  logic wr_sem_inc_v;
  logic [SEMW-1:0] wr_sem_inc_idx;
  logic wr_err_v;
  logic [31:0] wr_err_cmd;
  logic wr_axi_quiet, wr_idle;
  logic [7:0] wr_inflight;
  logic pf_aw_hs;

  // ядро
  logic k_err_q;
  logic [SEMW-1:0] krn_sem_q;
  logic [31:0] krn_no_q;
  logic k_done_ok;
  logic inc_rd_v, inc_wr_v;

  // watchdog
  logic [WDW-1:0] wdog_q;
  logic wdog_cnt, wdog_hit_q;

  // ------------------------------------------------------------ регистры хоста

  axil_regs u_regs (
      .clk           (clk),
      .rst_n         (rst_n),
      .s_axil_awvalid(s_axil_awvalid),
      .s_axil_awready(s_axil_awready),
      .s_axil_awaddr (s_axil_awaddr),
      .s_axil_wvalid (s_axil_wvalid),
      .s_axil_wready (s_axil_wready),
      .s_axil_wdata  (s_axil_wdata),
      .s_axil_wstrb  (s_axil_wstrb),
      .s_axil_bvalid (s_axil_bvalid),
      .s_axil_bready (s_axil_bready),
      .s_axil_bresp  (s_axil_bresp),
      .s_axil_arvalid(s_axil_arvalid),
      .s_axil_arready(s_axil_arready),
      .s_axil_araddr (s_axil_araddr),
      .s_axil_rvalid (s_axil_rvalid),
      .s_axil_rready (s_axil_rready),
      .s_axil_rdata  (s_axil_rdata),
      .s_axil_rresp  (s_axil_rresp),
      .enable        (enable),
      .soft_rst_req  (soft_rst_req),
      .perf_snap     (perf_snap),
      .cq_push       (cq_push),
      .cq_d          (cq_in_d),
      .cq_full       (!cq_in_rdy),
      .sem_we        (sem_we),
      .sem_idx       (sem_idx),
      .sem_wval      (sem_wval),
      .status        (status),
      .err_code      (err_code_q),
      .err_cmd       (err_cmd_q),
      .cmd_count     (cmd_no),
      .sem_val       (sem_val),
      .perf          (perf)
  );

  // STATUS §13
  assign status = {
    wr_inflight,
    rd_inflight,
    8'(CQ_DEPTH - int'(cq_count)),
    3'd0,
    krn_busy_q,
    cq_ovf_q,
    error_q,
    axi_quiet,
    idle
  };

  // ------------------------------------------------------------ командная FIFO + секвенсор

  // на ошибке cmd_seq сам выкачивает FIFO (cq_pop = cq_v), clr — только SOFT_RESET
  fifo_sync #(
      .W    (128),
      .DEPTH(CQ_DEPTH)
  ) u_cq (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (soft_rst),
      .in_v   (cq_push),
      .in_rdy (cq_in_rdy),
      .in_d   (cq_in_d),
      .out_v  (cq_v),
      .out_rdy(cq_pop),
      .out_d  (cq_d),
      .count  (cq_count),
      .empty  (cq_empty)
  );

  cmd_seq u_seq (
      .clk         (clk),
      .rst_n       (rst_n),
      .soft_rst    (soft_rst),
      .enable      (enable),
      .error       (err_now),
      .cq_v        (cq_v),
      .cq_pop      (cq_pop),
      .cq_d        (cq_d),
      .rd_v        (s_rd_v),
      .rd_rdy      (s_rd_rdy),
      .rd_stream   (s_rd_stream),
      .rd_sem      (s_rd_sem),
      .rd_spm      (s_rd_spm),
      .rd_hbm      (s_rd_hbm),
      .rd_nb       (s_rd_nb),
      .rd_no       (s_rd_no),
      .wr_v        (s_wr_v),
      .wr_rdy      (s_wr_rdy),
      .wr_sem      (s_wr_sem),
      .wr_spm      (s_wr_spm),
      .wr_hbm      (s_wr_hbm),
      .wr_nb       (s_wr_nb),
      .wr_no       (s_wr_no),
      .krn_v       (s_krn_v),
      .krn_rdy     (s_krn_rdy),
      .krn_sem     (s_krn_sem),
      .krn_arg     (s_krn_arg),
      .krn_no      (s_krn_no),
      .sem_val     (sem_val),
      .sig_v       (sig_v),
      .sig_idx     (sig_idx),
      .sig_val     (sig_val),
      .dec_err_v   (dec_err_v),
      .dec_err_code(dec_err_code),
      .dec_err_cmd (dec_err_cmd),
      .cmd_no      (cmd_no),
      .in_wait     (in_wait),
      .pf_wait     (pf_wait),
      .seq_busy    (seq_busy)
  );

  // ------------------------------------------------------------ очереди движков

  // на ошибке очереди очищаются (§12.2: движки при abort команд не берут)
  logic eq_clr;
  assign eq_clr = soft_rst || error_q;

  fifo_sync #(
      .W    (RQW),
      .DEPTH(EQ_DEPTH)
  ) u_rq (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (eq_clr),
      .in_v   (s_rd_v),
      .in_rdy (s_rd_rdy),
      .in_d   ({s_rd_stream, s_rd_sem, s_rd_spm, s_rd_hbm, s_rd_nb, s_rd_no}),
      .out_v  (rq_v),
      .out_rdy(rq_rdy),
      .out_d  ({rq_stream, rq_sem, rq_spm, rq_hbm, rq_nb, rq_no}),
      .count  (rq_count),
      .empty  (rq_empty)
  );

  fifo_sync #(
      .W    (WQW),
      .DEPTH(EQ_DEPTH)
  ) u_wq (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (eq_clr),
      .in_v   (s_wr_v),
      .in_rdy (s_wr_rdy),
      .in_d   ({s_wr_sem, s_wr_spm, s_wr_hbm, s_wr_nb, s_wr_no}),
      .out_v  (wq_v),
      .out_rdy(wq_rdy),
      .out_d  ({wq_sem, wq_spm, wq_hbm, wq_nb, wq_no}),
      .count  (wq_count),
      .empty  (wq_empty)
  );

  fifo_sync #(
      .W    (KQW),
      .DEPTH(EQ_DEPTH)
  ) u_kq (
      .clk    (clk),
      .rst_n  (rst_n),
      .clr    (eq_clr),
      .in_v   (s_krn_v),
      .in_rdy (s_krn_rdy),
      .in_d   ({s_krn_sem, s_krn_arg, s_krn_no}),
      .out_v  (kq_v),
      .out_rdy(k_start),
      .out_d  ({kq_sem, kq_arg, kq_no}),
      .count  (kq_count),
      .empty  (kq_empty)
  );

  // ------------------------------------------------------------ движки

  // счётчик тактов для задержки чтения (perf RD_LAT_SUM): свободно бегущий
  always_ff @(posedge clk) begin
    if (!rst_n) now_q <= 32'd0;
    else now_q <= now_q + 32'd1;
  end

  rd_dma #(
      .NTAG(NTAG)
  ) u_rd (
      .clk          (clk),
      .rst_n        (rst_n),
      .soft_rst     (soft_rst),
      .go           (go),
      .abort        (abort),
      .now          (now_q),
      .cmd_v        (rq_v),
      .cmd_rdy      (rq_rdy),
      .cmd_stream   (rq_stream),
      .cmd_sem      (rq_sem),
      .cmd_spm      (rq_spm),
      .cmd_hbm      (rq_hbm),
      .cmd_nb       (rq_nb),
      .cmd_no       (rq_no),
      .m_axi_arvalid(m_axi_arvalid),
      .m_axi_arready(m_axi_arready),
      .m_axi_araddr (m_axi_araddr),
      .m_axi_arid   (m_axi_arid),
      .m_axi_arlen  (m_axi_arlen),
      .m_axi_arsize (m_axi_arsize),
      .m_axi_arburst(m_axi_arburst),
      .m_axi_arcache(m_axi_arcache),
      .m_axi_rvalid (m_axi_rvalid),
      .m_axi_rready (m_axi_rready),
      .m_axi_rdata  (m_axi_rdata),
      .m_axi_rid    (m_axi_rid),
      .m_axi_rresp  (m_axi_rresp),
      .m_axi_rlast  (m_axi_rlast),
      .spm_wr_v     (rd_spm_wr_v),
      .spm_wr_addr  (rd_spm_wr_addr),
      .spm_wr_data  (rd_spm_wr_data),
      .k_s_tvalid   (k_s_tvalid),
      .k_s_tready   (k_s_tready),
      .k_s_tdata    (k_s_tdata),
      .k_s_tlast    (k_s_tlast),
      .sem_inc_v    (rd_sem_inc_v),
      .sem_inc_idx  (rd_sem_inc_idx),
      .err_v        (rd_err_v),
      .err_cmd      (rd_err_cmd),
      .axi_quiet    (rd_axi_quiet),
      .idle         (rd_idle),
      .inflight     (rd_inflight),
      .pf_ar_hs     (pf_ar_hs),
      .pf_win_full  (pf_win_full),
      .pf_lat_v     (pf_lat_v),
      .pf_lat       (pf_lat)
  );

  wr_dma u_wr (
      .clk          (clk),
      .rst_n        (rst_n),
      .soft_rst     (soft_rst),
      .go           (go),
      .abort        (abort),
      .cmd_v        (wq_v),
      .cmd_rdy      (wq_rdy),
      .cmd_sem      (wq_sem),
      .cmd_spm      (wq_spm),
      .cmd_hbm      (wq_hbm),
      .cmd_nb       (wq_nb),
      .cmd_no       (wq_no),
      .spm_rd_req   (wr_spm_rd_req),
      .spm_rd_addr  (wr_spm_rd_addr),
      .spm_rd_gnt   (wr_spm_rd_gnt),
      .spm_rd_data  (wr_spm_rd_data),
      .m_axi_awvalid(m_axi_awvalid),
      .m_axi_awready(m_axi_awready),
      .m_axi_awaddr (m_axi_awaddr),
      .m_axi_awid   (m_axi_awid),
      .m_axi_awlen  (m_axi_awlen),
      .m_axi_awsize (m_axi_awsize),
      .m_axi_awburst(m_axi_awburst),
      .m_axi_awcache(m_axi_awcache),
      .m_axi_wvalid (m_axi_wvalid),
      .m_axi_wready (m_axi_wready),
      .m_axi_wdata  (m_axi_wdata),
      .m_axi_wstrb  (m_axi_wstrb),
      .m_axi_wlast  (m_axi_wlast),
      .m_axi_bvalid (m_axi_bvalid),
      .m_axi_bready (m_axi_bready),
      .m_axi_bid    (m_axi_bid),
      .m_axi_bresp  (m_axi_bresp),
      .sem_inc_v    (wr_sem_inc_v),
      .sem_inc_idx  (wr_sem_inc_idx),
      .err_v        (wr_err_v),
      .err_cmd      (wr_err_cmd),
      .axi_quiet    (wr_axi_quiet),
      .idle         (wr_idle),
      .inflight     (wr_inflight),
      .pf_aw_hs     (pf_aw_hs)
  );

  // §4: AxPROT = 000, AxQOS = 0, AxLOCK = 0
  assign m_axi_arprot = 3'b000;
  assign m_axi_arqos  = 4'd0;
  assign m_axi_arlock = 1'b0;
  assign m_axi_awprot = 3'b000;
  assign m_axi_awqos  = 4'd0;
  assign m_axi_awlock = 1'b0;

  // ------------------------------------------------------------ spm

  spm u_spm (
      .clk      (clk),
      .a_wr_v   (rd_spm_wr_v),
      .a_wr_addr(rd_spm_wr_addr),
      .a_wr_data(rd_spm_wr_data),
      .a_rd_req (wr_spm_rd_req),
      .a_rd_addr(wr_spm_rd_addr),
      .a_rd_gnt (wr_spm_rd_gnt),
      .a_rd_data(wr_spm_rd_data),
      .b_en     (k_spm_en),
      .b_we     (k_spm_we),
      .b_addr   (k_spm_addr),
      .b_wdata  (k_spm_wdata),
      .b_wstrb  (k_spm_wstrb),
      .b_rdata  (k_spm_rdata)
  );

  // ------------------------------------------------------------ семафоры

  // приращения от движков гасятся при ошибке, включая такт её импульса (§11, R10)
  assign inc_rd_v = rd_sem_inc_v && !err_now;
  assign inc_wr_v = wr_sem_inc_v && !err_now;

  sem_file u_sem (
      .clk        (clk),
      .rst_n      (rst_n),
      .clr        (soft_rst),
      .inc_rd_v   (inc_rd_v),
      .inc_rd_idx (rd_sem_inc_idx),
      .inc_wr_v   (inc_wr_v),
      .inc_wr_idx (wr_sem_inc_idx),
      .inc_k_v    (k_done_ok),
      .inc_k_idx  (krn_sem_q),
      .inc_sig_v  (sig_v),
      .inc_sig_idx(sig_idx),
      .inc_sig_val(sig_val),
      .host_we    (sem_we),
      .host_idx   (sem_idx),
      .host_val   (sem_wval),
      .sem_val    (sem_val)
  );

  // ------------------------------------------------------------ контроллер ядра

  assign k_rst_n   = rst_n && !soft_rst;
  assign k_start   = kq_v && !krn_busy_q && go;
  assign k_arg     = kq_arg;
  // k_done засчитывается только у запущенного RUN и без ошибки (§7, §11)
  assign k_done_ok = k_done && krn_busy_q && !err_now;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) krn_busy_q <= 1'b0;
    else if (k_start) krn_busy_q <= 1'b1;
    else if (k_done) krn_busy_q <= 1'b0;
  end

  // семафор и номер текущего RUN (датапат: читаются только при krn_busy_q)
  always_ff @(posedge clk) begin
    if (k_start) begin
      krn_sem_q <= kq_sem;
      krn_no_q  <= kq_no;
    end
  end

  // k_err от ядра — регистр (в такте soft_rst ядро в сбросе, импульс игнорируется)
  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) k_err_q <= 1'b0;
    else k_err_q <= k_err;
  end

  // ------------------------------------------------------------ AXI_QUIET / IDLE / SOFT_RESET

  assign axi_quiet = rd_axi_quiet && wr_axi_quiet;
  assign idle = axi_quiet && cq_empty && rq_empty && wq_empty && kq_empty && rd_idle && wr_idle
             && !krn_busy_q && !seq_busy && !in_wait;
  assign soft_rst = soft_rst_req && axi_quiet;

  // ------------------------------------------------------------ ошибки

  assign err_any = wdog_hit_q || rd_err_v || wr_err_v || k_err_q || dec_err_v;
  assign err_now = error_q || err_any;
  assign go      = enable && !err_now;
  assign abort   = err_now;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) error_q <= 1'b0;
    else if (err_any) error_q <= 1'b1;
  end

  // первая ошибка: код и номер команды по приоритету §12.2; dec_err_code/cmd
  // действительны только при dec_err_v. TIMEOUT → последняя взятая команда.
  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) begin
      err_code_q <= E_NONE;
      err_cmd_q  <= 32'd0;
    end else if (!error_q) begin
      if (wdog_hit_q) begin
        err_code_q <= E_TIMEOUT;
        err_cmd_q  <= cmd_no - 32'd1;
      end else if (rd_err_v) begin
        err_code_q <= E_RRESP;
        err_cmd_q  <= rd_err_cmd;
      end else if (wr_err_v) begin
        err_code_q <= E_BRESP;
        err_cmd_q  <= wr_err_cmd;
      end else if (k_err_q) begin
        err_code_q <= E_KERNEL;
        err_cmd_q  <= krn_no_q;
      end else if (dec_err_v) begin
        err_code_q <= dec_err_code;
        err_cmd_q  <= dec_err_cmd;
      end
    end
  end

  // cq_ovf: DOORBELL при полной FIFO — липкий, не ошибка (§6)
  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) cq_ovf_q <= 1'b0;
    else if (cq_push && !cq_in_rdy) cq_ovf_q <= 1'b1;
  end

  // ------------------------------------------------------------ watchdog

  // ждём ответа памяти: не тихо и в этом такте нет ни R-beat'а, ни B
  assign wdog_cnt = !axi_quiet && !m_axi_rvalid && !(m_axi_bvalid && m_axi_bready);

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst || !wdog_cnt) wdog_q <= '0;
    else if (wdog_q != WDOG_LIMIT) wdog_q <= wdog_q + WDW'(1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) wdog_hit_q <= 1'b0;
    else wdog_hit_q <= wdog_cnt && (wdog_q == WDOG_LAST);
  end

  // ------------------------------------------------------------ perf

  kshell_perf u_perf (
      .clk        (clk),
      .rst_n      (rst_n),
      .clr        (soft_rst),
      .enable     (enable),
      .snap       (perf_snap),
      .ev_seq_wait(pf_wait),
      .ev_ar_hs   (pf_ar_hs),
      .ev_win_full(pf_win_full),
      .ev_lat_v   (pf_lat_v),
      .lat        (pf_lat),
      .ev_strm_bp (k_s_tvalid && !k_s_tready),
      .ev_aw_hs   (pf_aw_hs),
      .ev_krn_busy(krn_busy_q),
      .perf       (perf)
  );

endmodule
