`timescale 1ns / 1ps
// =====================================================================
//  spm — scratchpad SPM_WORDS x DW, два порта (docs/design.md §10).
//  Порт A — DMA: запись rd_dma (абсолютный приоритет, слово целиком)
//  и чтение wr_dma (только в такт без a_wr_v; a_rd_gnt комбинаторный).
//  Порт B — ядро: побайтовая маска записи b_wstrb, чтение через 1 такт.
//  Чтение — задержка 1 такт (регистр данных). Одновременный доступ
//  портов A и B к одному адресу запрещён правилами и ловится SVA;
//  при записи в один адрес побеждает порт B (не определено контрактом).
//  Память и регистры чтения resetless (датапат).
// =====================================================================
module spm (
    input logic clk,
    // порт A: запись rd_dma (абсолютный приоритет)
    input logic a_wr_v,
    input logic [kshell_pkg::SPMAW-1:0] a_wr_addr,
    input logic [kshell_pkg::DW-1:0] a_wr_data,
    // порт A: чтение wr_dma — выполняется только в такт без a_wr_v
    input logic a_rd_req,
    input logic [kshell_pkg::SPMAW-1:0] a_rd_addr,
    output logic a_rd_gnt,  // = a_rd_req && !a_wr_v
    output logic [    kshell_pkg::DW-1:0] a_rd_data,   // данные через 1 такт после gnt
    // порт B: ядро
    input logic b_en,
    input logic b_we,
    input logic [kshell_pkg::SPMAW-1:0] b_addr,
    input logic [kshell_pkg::DW-1:0] b_wdata,
    input logic [kshell_pkg::BB-1:0] b_wstrb,
    output logic [kshell_pkg::DW-1:0] b_rdata  // через 1 такт после b_en && !b_we
);

  import kshell_pkg::*;

  logic [DW-1:0] mem_q[SPM_WORDS];

  assign a_rd_gnt = a_rd_req && !a_wr_v;

  // запись: порт A (слово целиком), затем порт B (по байтам)
  always_ff @(posedge clk) begin
    if (a_wr_v) mem_q[a_wr_addr] <= a_wr_data;
    if (b_en && b_we) begin
      for (int i = 0; i < BB; i++) begin
        if (b_wstrb[i]) mem_q[b_addr][8*i+:8] <= b_wdata[8*i+:8];
      end
    end
  end

  // чтение порта A
  always_ff @(posedge clk) begin
    if (a_rd_gnt) a_rd_data <= mem_q[a_rd_addr];
  end

  // чтение порта B
  always_ff @(posedge clk) begin
    if (b_en && !b_we) b_rdata <= mem_q[b_addr];
  end

endmodule
