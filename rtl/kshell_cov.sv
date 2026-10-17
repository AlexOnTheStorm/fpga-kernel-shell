`timescale 1ns / 1ps
// =====================================================================
//  kshell_cov — системное functional coverage kshell (bind; только VCOV=1).
//  docs/design.md §9, §11, §12, §16.
//    cg_err (в такте после фиксации первой ошибки): все коды ERR_CODE 1..7.
//    cg_sys (каждый такт):
//      - пауза во время трафика: ENABLE = 0, а AXI ещё не тихо (§12.3);
//      - W-FIFO wr_dma полна (2×BURST слов; сигнал внутренний — через bind);
//      - ≥ 2 приращения одного семафора в одном такте (§11: суммируются) и
//        какие пары источников совпали (rd+wr, rd+SIGNAL, wr+SIGNAL).
//  Паттерн: синхронный снимок `_r` + явный sample() (docs/coverage.md tick_to_trade).
// =====================================================================
module kshell_cov
  import kshell_pkg::*;
#(
    // ширина счётчика W-FIFO wr_dma ($clog2(2·BURST + 1))
    parameter int WFCW = $clog2(2 * BURST + 1)
) (
    input logic            clk,
    input logic            rst_n,
    input logic            enable,
    input logic            axi_quiet,
    input logic            error_q,
    input err_e            err_code_q,
    input logic            inc_rd_v,
    input logic [SEMW-1:0] rd_sem_inc_idx,
    input logic            inc_wr_v,
    input logic [SEMW-1:0] wr_sem_inc_idx,
    input logic            k_done_ok,
    input logic [SEMW-1:0] krn_sem_q,
    input logic            sig_v,
    input logic [SEMW-1:0] sig_idx,
    input logic [WFCW-1:0] wf_count
);
  localparam logic [WFCW-1:0] WF_FULL = WFCW'(2 * BURST);

  // совпадения индексов приращений по парам источников
  logic rd_wr, rd_sig, wr_sig;
  logic [2:0] n_same;  // число источников, попавших в самый «загруженный» семафор
  assign rd_wr  = inc_rd_v && inc_wr_v && (rd_sem_inc_idx == wr_sem_inc_idx);
  assign rd_sig = inc_rd_v && sig_v && (rd_sem_inc_idx == sig_idx);
  assign wr_sig = inc_wr_v && sig_v && (wr_sem_inc_idx == sig_idx);

  always_comb begin
    n_same = 3'd0;
    for (int s = 0; s < NSEM; s++) begin
      logic [2:0] n;
      n = 3'(inc_rd_v && (rd_sem_inc_idx == SEMW'(s))) + 3'(inc_wr_v && (wr_sem_inc_idx == SEMW'(s)))
        + 3'(k_done_ok && (krn_sem_q == SEMW'(s))) + 3'(sig_v && (sig_idx == SEMW'(s)));
      if (n > n_same) n_same = n;
    end
  end

  // ------------------------------------------------------------ снимок
  logic err_p_q;  // error_q такт назад: фронт = первая ошибка, код уже защёлкнут
  always_ff @(posedge clk) begin
    if (!rst_n) err_p_q <= 1'b0;
    else err_p_q <= error_q;
  end

  logic err_rise_r, pause_r, wf_full_r;
  err_e code_r;
  logic [2:0] n_same_r;
  logic [2:0] pair_r;  // {wr_sig, rd_sig, rd_wr}
  always_ff @(posedge clk) begin
    err_rise_r <= error_q && !err_p_q;
    code_r     <= err_code_q;
    pause_r    <= !enable && !axi_quiet;
    wf_full_r  <= (wf_count == WF_FULL);
    n_same_r   <= n_same;
    pair_r     <= {wr_sig, rd_sig, rd_wr};
  end

  // ------------------------------------------------------------ covergroup'ы
  covergroup cg_err;
    option.per_instance = 1;
    cp_code: coverpoint code_r {
      bins rresp = {E_RRESP};
      bins bresp = {E_BRESP};
      bins align = {E_ALIGN};
      bins range = {E_RANGE};
      bins opcode = {E_OPCODE};
      bins kernel = {E_KERNEL};
      bins timeout = {E_TIMEOUT};
    }
  endgroup

  covergroup cg_sys;
    option.per_instance = 1;
    cp_pause: coverpoint pause_r {bins pause_traffic = {1'b1};}
    cp_wf_full: coverpoint wf_full_r {bins wf_full = {1'b1};}
    cp_same_inc: coverpoint n_same_r {bins two = {3'd2}; bins three_plus = {[3'd3 : 3'd4]};}
    cp_pair_rd_wr: coverpoint pair_r[0] {bins rd_wr = {1'b1};}
    cp_pair_rd_sig: coverpoint pair_r[1] {bins rd_sig = {1'b1};}
    cp_pair_wr_sig: coverpoint pair_r[2] {bins wr_sig = {1'b1};}
  endgroup

  cg_err cov_err = new();
  cg_sys cov_sys = new();

  always @(posedge clk) begin
    if (rst_n) begin
      cov_sys.sample();
      if (err_rise_r) cov_err.sample();
    end
  end

endmodule

// wf_count — внутренний счётчик W-FIFO wr_dma: иерархическая ссылка в области kshell
bind kshell kshell_cov u_cov (
    .*,
    .wf_count(u_wr.wf_count)
);
