`timescale 1ns / 1ps
// =====================================================================
//  sem_file — NSEM 32-битных семафоров (docs/design.md §11).
//  Приращения от четырёх источников на один семафор в одном такте
//  СУММИРУЮТСЯ (rd/wr/k по +1, SIGNAL по +inc_sig_val). Запись хоста
//  задаёт значение и приоритетнее приращений; clr (синхронно) обнуляет
//  всё и приоритетнее всего. Новое значение видно со следующего такта.
//  Сравнение для WAIT — снаружи (знаковая разность), здесь только счётчики.
// =====================================================================
module sem_file (
    input logic clk,
    input logic rst_n,
    input logic clr,

    input logic                        inc_rd_v,
    input logic [kshell_pkg::SEMW-1:0] inc_rd_idx,
    input logic                        inc_wr_v,
    input logic [kshell_pkg::SEMW-1:0] inc_wr_idx,
    input logic                        inc_k_v,
    input logic [kshell_pkg::SEMW-1:0] inc_k_idx,
    input logic                        inc_sig_v,
    input logic [kshell_pkg::SEMW-1:0] inc_sig_idx,
    input logic [                31:0] inc_sig_val,

    input logic                        host_we,
    input logic [kshell_pkg::SEMW-1:0] host_idx,
    input logic [                31:0] host_val,

    output logic [kshell_pkg::NSEM-1:0][31:0] sem_val
);

  import kshell_pkg::*;

  for (genvar i = 0; i < NSEM; i++) begin : g_sem
    localparam logic [SEMW-1:0] IDX = SEMW'(i);

    logic [31:0] sem_q;
    logic [31:0] delta;
    logic        host_hit;

    // сумма приращений за такт (каждый источник попадает в свой семафор)
    assign delta = ((inc_rd_v && inc_rd_idx == IDX) ? 32'd1 : 32'd0)
                 + ((inc_wr_v && inc_wr_idx == IDX) ? 32'd1 : 32'd0)
                 + ((inc_k_v && inc_k_idx == IDX) ? 32'd1 : 32'd0)
                 + ((inc_sig_v && inc_sig_idx == IDX) ? inc_sig_val : 32'd0);
    assign host_hit = host_we && (host_idx == IDX);

    always_ff @(posedge clk) begin
      if (!rst_n || clr) sem_q <= 32'd0;
      else if (host_hit) sem_q <= host_val;
      else sem_q <= sem_q + delta;
    end

    assign sem_val[i] = sem_q;
  end

endmodule
