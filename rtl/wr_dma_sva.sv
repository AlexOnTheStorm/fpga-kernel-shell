`timescale 1ns / 1ps
// =====================================================================
//  wr_dma_sva — ассершены wr_dma (bind; синтез не читает). docs/design.md §9, §16.
//    A. AXI4 AW-мастер: VALID держится до handshake, payload стабилен, константы
//       burst'а (AWID=0, AWCACHE=0010, INCR); новый AW только при go && !abort,
//       полном burst'е в W-FIFO и месте в очереди полёта (≤ WR_OUT).
//    B. W: только после handshake AW; без пауз со стороны wr_dma (WVALID не падает
//       до WLAST и поднят, пока есть AW без дописанного W); WLAST на beat'е BURST;
//       данные стабильны при WREADY=0; WSTRB все единицы. Счётчики AW/W/B — локальные
//       (независимо от RTL).
//    C. B/sem: B только при наличии AW в полёте; sem не растёт при abort/ошибке.
//    D. abort/soft_rst: нет команд при abort; soft_rst только при axi_quiet;
//       W-FIFO не переполняется; axi_quiet совпадает с независимым счётом.
//    E. Допущение о входе: команда с cmd_nb = 0 не приходит (len ≠ 0, §6).
// =====================================================================
/* verilator lint_off SYMRSVDWORD */
module wr_dma_sva
  import kshell_pkg::*;
(
    input logic                         clk,
    input logic                         rst_n,
    input logic                         soft_rst,
    input logic                         go,
    input logic                         abort,
    input logic                         cmd_v,
    input logic                         cmd_rdy,
    input logic [              NBW-1:0] cmd_nb,
    input logic                         m_axi_awvalid,
    input logic                         m_axi_awready,
    input logic [               AW-1:0] m_axi_awaddr,
    input logic [                  0:0] m_axi_awid,
    input logic [                  7:0] m_axi_awlen,
    input logic [                  2:0] m_axi_awsize,
    input logic [                  1:0] m_axi_awburst,
    input logic [                  3:0] m_axi_awcache,
    input logic                         m_axi_wvalid,
    input logic                         m_axi_wready,
    input logic [               DW-1:0] m_axi_wdata,
    input logic [               BB-1:0] m_axi_wstrb,
    input logic                         m_axi_wlast,
    input logic                         m_axi_bvalid,
    input logic                         m_axi_bready,
    input logic                         sem_inc_v,
    input logic                         axi_quiet,
    input logic [                  7:0] inflight,
    input logic                         aw_hold_q,
    input logic                         rd_pend_q,
    input logic                         wf_in_rdy,
    input logic [$clog2(2*BURST+1)-1:0] rdy_bursts_q
);
  /* verilator lint_on SYMRSVDWORD */
  localparam int BEATW = $clog2(BURST);
  localparam int OFFW = $clog2(BURST_BYTES);

  default clocking cb @(posedge clk);
  endclocking
  default disable iff (!rst_n || soft_rst);

  // независимые счётчики: AW без B (out_q), AW с недописанным W (wp_q), beat W в burst'е
  logic [7:0] out_q, wp_q;
  logic [BEATW-1:0] beat_q;
  logic aw_hs, w_hs, b_hs;
  assign aw_hs = m_axi_awvalid && m_axi_awready;
  assign w_hs  = m_axi_wvalid && m_axi_wready;
  assign b_hs  = m_axi_bvalid && m_axi_bready;

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) out_q <= '0;
    else out_q <= out_q + 8'(aw_hs) - 8'(b_hs);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) wp_q <= '0;
    else wp_q <= wp_q + 8'(aw_hs) - 8'(w_hs && m_axi_wlast);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || soft_rst) beat_q <= '0;
    else if (w_hs) beat_q <= m_axi_wlast ? '0 : beat_q + BEATW'(1);
  end

  // A. AW: VALID держится до READY, адрес стабилен, константы
  a_aw_valid_held :
  assert property (m_axi_awvalid && !m_axi_awready |=> m_axi_awvalid);
  a_aw_payload_stable :
  assert property (m_axi_awvalid && !m_axi_awready |=> $stable(m_axi_awaddr));
  a_aw_len :
  assert property (m_axi_awvalid |-> m_axi_awlen == 8'(BURST - 1));
  a_aw_size_burst :
  assert property (m_axi_awvalid |-> m_axi_awsize == 3'($clog2(BB)) && m_axi_awburst == 2'b01);
  a_aw_id :
  assert property (m_axi_awvalid |-> m_axi_awid == 1'b0);
  a_aw_cache :
  assert property (m_axi_awvalid |-> m_axi_awcache == 4'b0010);
  a_aw_aligned :
  assert property (m_axi_awvalid |-> m_axi_awaddr[OFFW-1:0] == '0);
  // новый (не удерживаемый) AW — только при go && !abort
  a_aw_new_only_go :
  assert property (m_axi_awvalid && !aw_hold_q |-> go && !abort);
  // AW только когда полный burst лежит в W-FIFO (иначе W встал бы после AW)
  a_aw_full_burst :
  assert property (m_axi_awvalid |-> rdy_bursts_q != '0);
  a_inflight_bound :
  assert property (inflight <= 8'(WR_OUT) && out_q <= 8'(WR_OUT));
  a_inflight_matches :
  assert property (inflight == out_q);

  // B. W: только после AW; без пауз; WLAST на beat'е BURST
  a_w_after_aw :
  assert property (m_axi_wvalid |-> wp_q != '0);
  a_w_no_gap :
  assert property (wp_q != '0 |-> m_axi_wvalid);
  a_w_valid_held :
  assert property (m_axi_wvalid && !(m_axi_wready && m_axi_wlast) |=> m_axi_wvalid);
  a_w_data_stable :
  assert property (m_axi_wvalid && !m_axi_wready |=> $stable(m_axi_wdata) && $stable(m_axi_wlast));
  a_wlast_iff_beat :
  assert property (m_axi_wvalid |-> m_axi_wlast == (beat_q == BEATW'(BURST - 1)));
  a_wstrb_full :
  assert property (m_axi_wvalid |-> m_axi_wstrb == '1);

  // C. B и sem
  a_b_inflight :
  assert property (m_axi_bvalid |-> out_q != '0 && wp_q < out_q);
  a_bready_const :
  assert property (m_axi_bready);
  a_no_sem_on_abort :
  assert property (abort |-> !sem_inc_v);

  // D. abort / soft_rst / FIFO / quiet
  a_no_cmd_on_abort :
  assert property (abort |-> !cmd_rdy);
  a_soft_rst_quiet :
  assert property (disable iff (!rst_n) soft_rst |-> axi_quiet);
  a_wfifo_no_overflow :
  assert property (rd_pend_q |-> wf_in_rdy);
  a_quiet_matches :
  assert property (axi_quiet == (out_q == '0 && !m_axi_awvalid));

  // E. допущение о секвенсоре
  a_cmd_nb_nonzero :
  assert property (cmd_v && cmd_rdy |-> cmd_nb != '0);

endmodule

bind wr_dma wr_dma_sva u_sva (.*);
