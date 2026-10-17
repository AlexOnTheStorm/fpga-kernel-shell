`timescale 1ns / 1ps
// =====================================================================
//  rd_dma_sva — ассершены rd_dma (bind; синтез не читает). docs/design.md §8, §16.
//    A. AXI4 AR-мастер: VALID держится до handshake, payload стабилен, константы
//       burst'а; новый AR только при go && !abort.
//    B. Окно: R только для слота в полёте; RLAST ⇔ beat_cnt == BURST−1;
//       tail − head ≤ NTAG; выданных AR без RLAST ≤ занятых слотов.
//    C. AXIS к ядру: стабильность при tvalid && !tready; буфер не переполняется.
//    D. abort/soft_rst: нет sem_inc при abort; soft_rst только при axi_quiet.
//    E. Допущение о входе: команда с cmd_nb = 0 не приходит (len ≠ 0, §6).
// =====================================================================
/* verilator lint_off SYMRSVDWORD */
module rd_dma_sva
  import kshell_pkg::*;
#(
    parameter int NTAG = 16
) (
    input logic                                       clk,
    input logic                                       rst_n,
    input logic                                       soft_rst,
    input logic                                       go,
    input logic                                       abort,
    input logic                                       cmd_v,
    input logic                                       cmd_rdy,
    input logic [         NBW-1:0]                    cmd_nb,
    input logic                                       m_axi_arvalid,
    input logic                                       m_axi_arready,
    input logic [          AW-1:0]                    m_axi_araddr,
    input logic [$clog2(NTAG)-1:0]                    m_axi_arid,
    input logic [             7:0]                    m_axi_arlen,
    input logic [             2:0]                    m_axi_arsize,
    input logic [             1:0]                    m_axi_arburst,
    input logic                                       m_axi_rvalid,
    input logic [$clog2(NTAG)-1:0]                    m_axi_rid,
    input logic                                       m_axi_rlast,
    input logic                                       k_s_tvalid,
    input logic                                       k_s_tready,
    input logic [          DW-1:0]                    k_s_tdata,
    input logic                                       k_s_tlast,
    input logic                                       sem_inc_v,
    input logic                                       axi_quiet,
    input logic                                       ar_hold_q,
    input logic                                       rd_pend_q,
    input logic                                       ob_in_rdy,
    input logic [  $clog2(NTAG):0]                    head_q,
    input logic [  $clog2(NTAG):0]                    tail_q,
    input logic [  $clog2(NTAG):0]                    out_cnt_q,
    input logic [        NTAG-1:0][$clog2(BURST)-1:0] w_beat_q,
    input logic [        NTAG-1:0]                    w_done_q
);
  /* verilator lint_on SYMRSVDWORD */
  localparam int TAGW = $clog2(NTAG);
  localparam int PW = TAGW + 1;
  localparam logic [PW-1:0] WIN_FULL = PW'(NTAG);

  default clocking cb @(posedge clk);
  endclocking
  default disable iff (!rst_n);

  logic [  PW-1:0] occ;
  logic [TAGW-1:0] r_dist;  // расстояние ответа от головы окна (OoO)
  assign occ    = tail_q - head_q;
  assign r_dist = m_axi_rid - head_q[TAGW-1:0];

  // A. AR: VALID держится до READY, адрес и ID не меняются
  a_ar_valid_held :
  assert property (m_axi_arvalid && !m_axi_arready |=> m_axi_arvalid);
  a_ar_payload_stable :
  assert property (m_axi_arvalid && !m_axi_arready |=> $stable(
      m_axi_araddr
  ) && $stable(
      m_axi_arid
  ));
  a_ar_len :
  assert property (m_axi_arvalid |-> m_axi_arlen == 8'(BURST - 1));
  a_ar_size_burst :
  assert property (m_axi_arvalid |-> m_axi_arsize == 3'($clog2(BB)) && m_axi_arburst == 2'b01);
  a_ar_aligned :
  assert property (m_axi_arvalid |-> m_axi_araddr[$clog2(BURST_BYTES)-1:0] == '0);
  // A. новый (не удерживаемый) AR — только при go && !abort и свободном слоте
  a_ar_new_only_go :
  assert property (m_axi_arvalid && !ar_hold_q |-> go && !abort);
  a_ar_slot_free :
  assert property (m_axi_arvalid |-> occ < WIN_FULL);

  // B. R только для слота в полёте: занят ([head, tail)) и ещё без RLAST
  a_r_inflight :
  assert property (m_axi_rvalid |-> PW'(r_dist) < occ && !w_done_q[m_axi_rid]);
  a_rlast_iff_beat :
  assert property (m_axi_rvalid |-> m_axi_rlast == (w_beat_q[m_axi_rid] == $clog2(
      BURST
  )'(BURST - 1)));
  a_win_bound :
  assert property (occ <= WIN_FULL);
  a_out_le_occ :
  assert property (out_cnt_q <= occ);

  // C. AXIS к ядру (soft_rst сбрасывает поток вместе с ядром — исключение)
  a_axis_stable :
  assert property (disable iff (!rst_n || soft_rst)
      k_s_tvalid && !k_s_tready |=> k_s_tvalid && $stable(
      k_s_tdata
  ) && $stable(
      k_s_tlast
  ));
  a_obuf_no_overflow :
  assert property (rd_pend_q |-> ob_in_rdy);

  // D. abort: семафоры не растут; soft_rst — только в тишине AXI (§12.4)
  a_no_sem_on_abort :
  assert property (abort |-> !sem_inc_v);
  a_soft_rst_quiet :
  assert property (soft_rst |-> axi_quiet);
  a_no_cmd_on_abort :
  assert property (abort |-> !cmd_rdy);

  // E. допущение о секвенсоре
  a_cmd_nb_nonzero :
  assert property (cmd_v && cmd_rdy |-> cmd_nb != '0);

endmodule

bind rd_dma rd_dma_sva #(.NTAG(NTAG)) u_sva (.*);
