`timescale 1ns / 1ps
// =====================================================================
//  spm — scratchpad SPM_WORDS x DW, два порта (docs/design.md §10).
//  Порт A — DMA: запись rd_dma (абсолютный приоритет, слово целиком)
//  и чтение wr_dma (только в такт без a_wr_v; a_rd_gnt комбинаторный).
//  Порт B — ядро: побайтовая маска записи b_wstrb, чтение через 1 такт.
//
//  Описано по шаблону вендора «true dual-port RAM с побайтовой записью»
//  (Xilinx UG901, режим NO_CHANGE), чтобы синтез положил массив в BRAM:
//  один адрес на порт, один always_ff на порт, у каждого порта — en/we/
//  маска байтов/регистр чтения. Порт A пишет полной маской.
//  NO_CHANGE: регистр чтения обновляется только чтением (en && !we),
//  запись его не трогает. Поэтому a_rd_data держится после gnt до
//  следующего ЧТЕНИЯ порта A, даже если в следующем такте rd_dma пишет
//  (wr_dma забирает данные ровно через такт, но удержание — запас).
//  Чтение и запись одного порта в одном такте не бывает по построению
//  (A: gnt = req && !wr; B: we выбирает одно из двух), так что выбор
//  read-first / write-first здесь не влияет на поведение.
//
//  Одновременный доступ портов A и B к одному адресу, если хоть одна
//  сторона пишет, запрещён правилами и ловится SVA (поведение BRAM при
//  такой коллизии не определено). Память и регистры чтения — resetless.
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

  // порт A: запись и чтение взаимоисключающие → один адрес порта
  logic a_en;
  logic [SPMAW-1:0] a_addr;

  assign a_rd_gnt = a_rd_req && !a_wr_v;
  assign a_en     = a_wr_v || a_rd_gnt;
  assign a_addr   = a_wr_v ? a_wr_addr : a_rd_addr;

  // порт A (we = a_wr_v, маска — все байты)
  always_ff @(posedge clk) begin
    if (a_en) begin
      if (a_wr_v) begin
        for (int i = 0; i < BB; i++) mem_q[a_addr][8*i+:8] <= a_wr_data[8*i+:8];
      end else begin
        a_rd_data <= mem_q[a_addr];
      end
    end
  end

  // порт B (we = b_we, маска b_wstrb)
  always_ff @(posedge clk) begin
    if (b_en) begin
      if (b_we) begin
        for (int i = 0; i < BB; i++) begin
          if (b_wstrb[i]) mem_q[b_addr][8*i+:8] <= b_wdata[8*i+:8];
        end
      end else begin
        b_rdata <= mem_q[b_addr];
      end
    end
  end

endmodule
