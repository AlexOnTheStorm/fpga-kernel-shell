`timescale 1ns / 1ps
// =====================================================================
//  cmd_seq_cov — functional coverage cmd_seq (bind; только VCOV=1).
//  docs/design.md §6, §7, §16.
//    cg_take (на взятии команды из FIFO): все коды команд, включая BAD_OPCODE.
//    cg_seq  (каждый такт): стоянка на WAIT (условие ложно); WAIT прерван
//            ошибкой (в S_WAIT пришёл error/halt — WAIT сбрасывается, §12.2).
//  Паттерн: синхронный снимок `_r` + явный sample() (docs/coverage.md tick_to_trade).
// =====================================================================
module cmd_seq_cov (
    input logic         clk,
    input logic         rst_n,
    input logic         take,
    input logic [127:0] cq_d,
    input logic [  1:0] state_q,
    input logic         error,
    input logic         halt_q,
    input logic         pf_wait
);
  // S_WAIT в cmd_seq.seq_state_e (S_IDLE = 0, S_DISP = 1, S_WAIT = 2)
  localparam logic [1:0] ST_WAIT = 2'd2;
  localparam int OP_LSB = 28;  // CMD0[31:28]

  logic take_r, stall_r, wabort_r;
  logic [3:0] op_r;
  always_ff @(posedge clk) begin
    take_r   <= take;
    op_r     <= cq_d[OP_LSB+:4];
    stall_r  <= pf_wait;
    wabort_r <= (state_q == ST_WAIT) && (error || halt_q);
  end

  covergroup cg_take;
    option.per_instance = 1;
    cp_op: coverpoint op_r {
      bins op_load = {4'd1};
      bins op_stream = {4'd2};
      bins op_store = {4'd3};
      bins op_wait = {4'd4};
      bins op_signal = {4'd5};
      bins op_run = {4'd6};
      bins op_bad = {4'd0, [4'd7 : 4'd15]};
    }
  endgroup

  covergroup cg_seq;
    option.per_instance = 1;
    cp_wait_stall: coverpoint stall_r {bins stall = {1'b1};}
    cp_wait_abort: coverpoint wabort_r {bins aborted = {1'b1};}
  endgroup

  cg_take cov_take = new();
  cg_seq  cov_seq = new();

  always @(posedge clk) begin
    if (rst_n) begin
      cov_seq.sample();
      if (take_r) cov_take.sample();
    end
  end

endmodule

bind cmd_seq cmd_seq_cov u_cov (.*);
