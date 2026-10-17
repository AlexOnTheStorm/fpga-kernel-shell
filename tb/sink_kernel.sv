`timescale 1ns / 1ps
// =====================================================================
//  sink_kernel — тестовое ядро-приёмник для T2 (docs/design.md §5, §16).
//  Контракт §5: arg0 = адрес spm (слова), arg1 = число beat'ов N (≥ 1).
//  Каждый принятый beat i пишется в spm[arg0 + i] через порт B (полная маска).
//  tready = 1 только между k_start и k_done (S_RUN) и при !stall
//  (tb-вход случайного backpressure для проверки skid-буфера rd_dma).
//  k_done — регистровый импульс в такте после beat'а N−1 с tlast.
//  k_err  — импульс, если tlast пришёл раньше beat'а N−1 или его нет на нём;
//           после k_err ядро возвращается в S_IDLE без k_done (§5.5),
//           поток стоит до SOFT_RESET.
//  k_rst_n — синхронный сброс; в такте сброса порт B и tready молчат.
// =====================================================================
module sink_kernel
  import kshell_pkg::*;
(
    input  logic                  clk,
    input  logic                  k_rst_n,
    input  logic                  stall,
    input  logic                  k_start,
    input  logic [     3:0][31:0] k_arg,
    output logic                  k_done,
    output logic                  k_err,
    input  logic                  k_s_tvalid,
    output logic                  k_s_tready,
    input  logic [  DW-1:0]       k_s_tdata,
    input  logic                  k_s_tlast,
    output logic                  k_spm_en,
    output logic                  k_spm_we,
    output logic [SPMAW-1:0]      k_spm_addr,
    output logic [  DW-1:0]       k_spm_wdata,
    output logic [  BB-1:0]       k_spm_wstrb,
    input  logic [  DW-1:0]       k_spm_rdata
);

  typedef enum logic [0:0] {
    S_IDLE,
    S_RUN
  } sk_state_e;

  sk_state_e state_q, state_d;
  logic [SPMAW-1:0] base_q;
  logic [31:0] n_q, i_q;
  logic hs, at_last, bad, good_end;

  assign hs       = k_s_tvalid && k_s_tready;
  assign at_last  = (i_q == n_q - 32'd1);
  // tlast раньше последнего beat'а или его нет на последнем
  assign bad      = hs && (k_s_tlast != at_last);
  assign good_end = hs && k_s_tlast && at_last;

  always_comb begin
    state_d    = state_q;
    k_s_tready = 1'b0;
    case (state_q)
      S_IDLE: if (k_start) state_d = S_RUN;
      S_RUN: begin
        k_s_tready = k_rst_n && !stall;
        if (good_end || bad) state_d = S_IDLE;
      end
      default: state_d = S_IDLE;
    endcase
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n) state_q <= S_IDLE;
    else state_q <= state_d;
  end

  // аргументы RUN (датапат: грузятся на k_start)
  always_ff @(posedge clk) begin
    if (k_start) begin
      base_q <= k_arg[0][SPMAW-1:0];
      n_q    <= k_arg[1];
    end
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n || k_start) i_q <= 32'd0;
    else if (hs) i_q <= i_q + 32'd1;
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n) begin
      k_done <= 1'b0;
      k_err  <= 1'b0;
    end else begin
      k_done <= good_end;
      k_err  <= bad;
    end
  end

  // порт B: запись принятого beat'а
  assign k_spm_en    = hs;
  assign k_spm_we    = 1'b1;
  assign k_spm_addr  = base_q + SPMAW'(i_q);
  assign k_spm_wdata = k_s_tdata;
  assign k_spm_wstrb = '1;

endmodule
