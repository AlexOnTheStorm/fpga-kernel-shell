`timescale 1ns / 1ps
// =====================================================================
//  rd_dma_cov — functional coverage rd_dma (bind; собирается только под
//  VCOV=1, синтез и lint дизайна не читают). docs/design.md §8, §16.
//    cg_win  (каждый такт): окно полно при burst'е в резчике; head-of-line —
//            голова STREAM готова, но стоит (выход AXIS не берёт), а следующий
//            слот — уже готовый LOAD.
//    cg_r    (на RLAST): расстояние out-of-order (rid − head) mod NTAG:
//            0 / 1–3 / 4–7 / 8+.
//    cg_beat (на R-beat'е): чередование — beat продолжает burst после beat'а
//            другого ID.
//  Подмножество covergroup у Verilator (docs/coverage.md tick_to_trade): синхронный снимок `_r`
//  против дельта-гонки bound-covergroup, явный sample(), условие внешним if.
// =====================================================================
/* verilator lint_off SYMRSVDWORD */
module rd_dma_cov
  import kshell_pkg::*;
#(
    parameter int NTAG = 16
) (
    input logic                                       clk,
    input logic                                       rst_n,
    input logic                                       abort,
    input logic [  $clog2(NTAG):0]                    head_q,
    input logic [  $clog2(NTAG):0]                    occ,
    input logic                                       pf_win_full,
    input logic                                       m_axi_rvalid,
    input logic [$clog2(NTAG)-1:0]                    m_axi_rid,
    input logic                                       m_axi_rlast,
    input logic [        NTAG-1:0][$clog2(BURST)-1:0] w_beat_q,
    input logic [        NTAG-1:0]                    w_done_q,
    input logic                                       w_stream_q  [NTAG],
    input logic                                       h_ready,
    input logic                                       h_stream,
    input logic                                       h_err,
    input logic                                       rd_issue
);
  /* verilator lint_on SYMRSVDWORD */
  localparam int TAGW = $clog2(NTAG);
  localparam int PW = TAGW + 1;
  localparam logic [PW-1:0] TWO_SLOTS = PW'(2);

  // ------------------------------------------------------------ снимок
  logic [TAGW-1:0] nslot;  // слот за головой
  logic hol;
  assign nslot = head_q[TAGW-1:0] + TAGW'(1);
  // голова STREAM готова и без ошибки, но ROB не читается (кредит выхода = ядро не берёт)
  assign hol = h_ready && h_stream && !h_err && !abort && !rd_issue
            && (occ >= TWO_SLOTS) && !w_stream_q[nslot] && w_done_q[nslot];

  // предыдущий R-beat: был ли и с каким ID (состояние самого cov-модуля)
  logic have_prev_q;
  logic [TAGW-1:0] prev_rid_q;
  always_ff @(posedge clk) begin
    if (!rst_n) have_prev_q <= 1'b0;
    else if (m_axi_rvalid) have_prev_q <= 1'b1;
  end
  always_ff @(posedge clk) begin
    if (m_axi_rvalid) prev_rid_q <= m_axi_rid;
  end

  // beat продолжает burst (у слота уже есть принятые beat'ы) после beat'а другого ID
  logic intl;
  assign intl = m_axi_rvalid && have_prev_q && (prev_rid_q != m_axi_rid)
             && (w_beat_q[m_axi_rid] != '0);

  logic full_r, hol_r, rend_r, intl_r;
  logic [7:0] dist_r;
  always_ff @(posedge clk) begin
    full_r <= pf_win_full;
    hol_r  <= hol;
    rend_r <= m_axi_rvalid && m_axi_rlast;
    dist_r <= 8'(TAGW'(m_axi_rid - head_q[TAGW-1:0]));
    intl_r <= intl;
  end

  // ------------------------------------------------------------ covergroup'ы
  covergroup cg_win;
    option.per_instance = 1;
    cp_full: coverpoint full_r {bins full = {1'b1};}
    cp_hol: coverpoint hol_r {bins hol = {1'b1};}
  endgroup

  covergroup cg_r;
    option.per_instance = 1;
    cp_ooo: coverpoint dist_r {
      bins d0 = {8'd0};
      bins d1_3 = {[8'd1 : 8'd3]};
      bins d4_7 = {[8'd4 : 8'd7]};
      bins d8p = {[8'd8 : 8'd255]};
    }
  endgroup

  covergroup cg_beat;
    option.per_instance = 1;
    cp_intl: coverpoint intl_r {bins interleaved = {1'b1};}
  endgroup

  cg_win  cov_win = new();
  cg_r    cov_r = new();
  cg_beat cov_beat = new();

  always @(posedge clk) begin
    if (rst_n) begin
      cov_win.sample();
      if (rend_r) cov_r.sample();
      if (intl_r) cov_beat.sample();
    end
  end

endmodule

bind rd_dma rd_dma_cov #(.NTAG(NTAG)) u_cov (.*);
