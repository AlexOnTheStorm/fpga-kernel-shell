`timescale 1ns / 1ps
// =====================================================================
//  kshell_sink_top — kshell + sink_kernel (системные тесты T1, T2, T7, T9;
//  свип демо). Порты = внешние интерфейсы kshell_top (§4) + tb-вход k_stall
//  (случайный backpressure ядра). NTAG и WDOG_CYCLES пробрасываются в kshell
//  (тесты собираются с -GWDOG_CYCLES=2000, демо — с -GNTAG=...).
//  Сигналы контракта ядра выведены наружу только для наблюдения из cocotb.
// =====================================================================
module kshell_sink_top
  import kshell_pkg::*;
#(
    parameter int NTAG        = 16,
    parameter int WDOG_CYCLES = 1 << 20
) (
    input logic clk,
    input logic rst_n,
    input logic k_stall,

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

    // наблюдение контракта ядра (только выходы)
    output logic k_s_tvalid,
    output logic k_s_tready,
    output logic k_start,
    output logic k_done,
    output logic k_err
);

  logic k_rst_n;
  logic [3:0][31:0] k_arg;
  logic [DW-1:0] k_s_tdata;
  logic k_s_tlast;
  logic k_spm_en, k_spm_we;
  logic [SPMAW-1:0] k_spm_addr;
  logic [DW-1:0] k_spm_wdata, k_spm_rdata;
  logic [BB-1:0] k_spm_wstrb;

  kshell #(
      .NTAG       (NTAG),
      .WDOG_CYCLES(WDOG_CYCLES)
  ) u_shell (
      .*
  );

  sink_kernel u_krn (
      .clk        (clk),
      .k_rst_n    (k_rst_n),
      .stall      (k_stall),
      .k_start    (k_start),
      .k_arg      (k_arg),
      .k_done     (k_done),
      .k_err      (k_err),
      .k_s_tvalid (k_s_tvalid),
      .k_s_tready (k_s_tready),
      .k_s_tdata  (k_s_tdata),
      .k_s_tlast  (k_s_tlast),
      .k_spm_en   (k_spm_en),
      .k_spm_we   (k_spm_we),
      .k_spm_addr (k_spm_addr),
      .k_spm_wdata(k_spm_wdata),
      .k_spm_wstrb(k_spm_wstrb),
      .k_spm_rdata(k_spm_rdata)
  );

endmodule
