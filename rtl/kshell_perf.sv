`timescale 1ns / 1ps
// =====================================================================
//  kshell_perf — perf-счётчики kshell (docs/design.md §14).
//  8 живых 32-битных счётчиков, считают только при enable (CTRL.ENABLE).
//  snap копирует все живые счётчики в регистры чтения ОДНОВРЕМЕННО (один
//  фронт) — хост читает согласованный снимок по AXI-Lite по одному слову.
//  clr (SOFT_RESET) обнуляет живые счётчики и снимок. Переполнение — по
//  модулю 2^32 (32 бита хватает на ~43 с при 100 МГц; хост делает снимки).
//  Порядок perf[i] = порядок адресов §14: CYCLES, SEQ_WAIT, RD_BURSTS,
//  RD_LAT_SUM, RD_WIN_FULL, STRM_BP, WR_BURSTS, KRN_BUSY.
// =====================================================================
module kshell_perf (
    input  logic            clk,
    input  logic            rst_n,
    input  logic            clr,
    input  logic            enable,
    input  logic            snap,
    // события (по handshake, §14)
    input  logic            ev_seq_wait,
    input  logic            ev_ar_hs,
    input  logic            ev_win_full,
    input  logic            ev_lat_v,
    input  logic [31:0]     lat,
    input  logic            ev_strm_bp,
    input  logic            ev_aw_hs,
    input  logic            ev_krn_busy,
    output logic [ 7:0][31:0] perf
);

  localparam int NCNT = 8;
  localparam int I_LAT_SUM = 3;  // единственный счётчик с приращением ≠ 1

  logic [NCNT-1:0] ev;
  logic [NCNT-1:0][31:0] cnt_q;

  assign ev = {ev_krn_busy, ev_aw_hs, ev_strm_bp, ev_win_full, ev_lat_v, ev_ar_hs, ev_seq_wait, 1'b1};

  for (genvar i = 0; i < NCNT; i++) begin : g_cnt
    logic [31:0] inc;
    if (i == I_LAT_SUM) begin : g_lat
      assign inc = lat;
    end else begin : g_one
      assign inc = 32'd1;
    end

    // живой счётчик
    always_ff @(posedge clk) begin
      if (!rst_n || clr) cnt_q[i] <= 32'd0;
      else if (enable && ev[i]) cnt_q[i] <= cnt_q[i] + inc;
    end

    // снимок для чтения хостом
    always_ff @(posedge clk) begin
      if (!rst_n || clr) perf[i] <= 32'd0;
      else if (snap) perf[i] <= cnt_q[i];
    end
  end

endmodule
