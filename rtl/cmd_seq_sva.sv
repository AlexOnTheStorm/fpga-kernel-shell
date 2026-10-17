`timescale 1ns / 1ps
// =====================================================================
//  cmd_seq_sva — ассершены cmd_seq (bind; синтез не читает). docs/design.md §6, §7, §12.2.
//    A. Выдача: не больше одной команды за такт; ничего не выдаётся при error/halt/enable=0;
//       при WAIT в работе выдачи нет; valid + поля держатся стабильно, пока очередь не готова.
//    B. Только верные команды: ушедшие в движки команды легальны (nb ≥ 1, адрес выровнен по
//       burst'у, spm + nb·BURST ≤ SPM_WORDS у LOAD/STORE) — гарантия для rd_dma/wr_dma.
//    C. error/halt: FIFO выкачивается (cq_pop = cq_v), cmd_no не растёт, halt липкий до soft_rst.
//    D. cq_pop только при cq_v; cmd_no +1 ровно на взятую команду; dec_err_v — импульс.
// =====================================================================
module cmd_seq_sva
  import kshell_pkg::*;
(
    input logic                   clk,
    input logic                   rst_n,
    input logic                   soft_rst,
    input logic                   enable,
    input logic                   error,
    input logic                   halt_q,
    input logic                   cq_v,
    input logic                   cq_pop,
    input logic                   rd_v,
    input logic                   rd_rdy,
    input logic                   rd_stream,
    input logic [SPMAW-1:0]       rd_spm,
    input logic [   AW-1:0]       rd_hbm,
    input logic [  NBW-1:0]       rd_nb,
    input logic [     31:0]       rd_no,
    input logic                   wr_v,
    input logic                   wr_rdy,
    input logic [SPMAW-1:0]       wr_spm,
    input logic [   AW-1:0]       wr_hbm,
    input logic [  NBW-1:0]       wr_nb,
    input logic [     31:0]       wr_no,
    input logic                   krn_v,
    input logic                   krn_rdy,
    input logic [      3:0][31:0] krn_arg,
    input logic [     31:0]       krn_no,
    input logic                   sig_v,
    input logic                   dec_err_v,
    input logic [     31:0]       cmd_no,
    input logic                   in_wait,
    input logic                   seq_busy,
    input logic                   take
);
  localparam int OFFW = $clog2(BURST_BYTES);

  default clocking cb @(posedge clk);
  endclocking
  default disable iff (!rst_n || soft_rst);

  logic any_v;
  assign any_v = rd_v || wr_v || krn_v || sig_v;

  // A. выдача
  a_one_hot :
  assert property ($onehot0({rd_v, wr_v, krn_v, sig_v}));
  a_quiet_on_stop :
  assert property ((error || halt_q || !enable) |-> !any_v);
  a_no_dispatch_in_wait :
  assert property (in_wait |-> !any_v);
  // команда держится, пока очередь не готова; уйти без handshake она может только по
  // error/halt (сброс) или enable = 0 (пауза: valid опускается, команда остаётся в cur_q)
  a_rd_held :
  assert property (rd_v && !rd_rdy |=> !enable || error || halt_q || (rd_v && $stable(
      rd_stream
  ) && $stable(
      rd_spm
  ) && $stable(
      rd_hbm
  ) && $stable(
      rd_nb
  ) && $stable(
      rd_no
  )));
  a_wr_held :
  assert property (wr_v && !wr_rdy |=> !enable || error || halt_q || (wr_v && $stable(
      wr_spm
  ) && $stable(
      wr_hbm
  ) && $stable(
      wr_nb
  ) && $stable(
      wr_no
  )));
  a_krn_held :
  assert property (krn_v && !krn_rdy |=> !enable || error || halt_q || (krn_v && $stable(
      krn_arg
  ) && $stable(
      krn_no
  )));

  // B. ушедшие команды легальны
  a_rd_legal :
  assert property (rd_v |-> rd_nb != '0 && rd_hbm[OFFW-1:0] == '0);
  a_rd_spm_range :
  assert property (rd_v && !rd_stream |-> 33'(rd_spm) + 33'(rd_nb) * 33'(BURST) <= 33'(SPM_WORDS));
  a_wr_legal :
  assert property (wr_v |-> wr_nb != '0 && wr_hbm[OFFW-1:0] == '0
                   && 33'(wr_spm) + 33'(wr_nb) * 33'(BURST) <= 33'(SPM_WORDS));

  a_wr_end_in_mem :
  assert property (wr_v |-> 41'(wr_hbm) + 41'(wr_nb) * 41'(BURST_BYTES) <= 41'(1) << AW);
  a_rd_end_in_mem :
  assert property (rd_v |-> 41'(rd_hbm) + 41'(rd_nb) * 41'(BURST_BYTES) <= 41'(1) << AW);

  // busy: команда в cur_q (кроме error/halt, где она сброшена) видна kshell для IDLE
  a_busy_when_disp :
  assert property (any_v |-> seq_busy);
  a_busy_when_wait :
  assert property (in_wait |-> seq_busy);
  a_busy_idle_is_empty :
  assert property (!seq_busy |-> !any_v && !in_wait);

  // C. error / halt
  a_flush :
  assert property ((error || halt_q) |-> cq_pop == cq_v);
  a_halt_sticky :
  assert property (halt_q |=> halt_q);
  a_halt_on_dec_err :
  assert property (dec_err_v |-> halt_q);
  a_no_take_on_stop :
  assert property ((error || halt_q) |-> !take);

  // D. FIFO / счётчик / импульс
  a_pop_needs_v :
  assert property (cq_pop |-> cq_v);
  a_cmd_no_inc :
  assert property (cmd_no == $past(cmd_no) + 32'($past(take)) || $past(soft_rst) || $past(!rst_n));
  a_dec_err_pulse :
  assert property (dec_err_v |=> !dec_err_v || $past(take));

endmodule

bind cmd_seq cmd_seq_sva u_sva (.*);
