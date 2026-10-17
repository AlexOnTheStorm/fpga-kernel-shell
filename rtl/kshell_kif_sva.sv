`timescale 1ns / 1ps
// =====================================================================
//  kshell_kif_sva — исполняемый контракт ядра kshell (docs/design.md §5, §16).
//  Только модуль-проверщик: порты = контракт §5 со стороны ядра, все входы.
//  Подключается bind'ом к любому ядру с теми же именами портов:
//    rtl/gemv_kernel_sva.sv, tb/sink_kernel_sva.sv — `bind <ядро> kshell_kif_sva u_kif (.*);`
//  Синтез не читает.
//
//  busy_q — свой трекер RUN: ставится по k_start, снимается по k_done или
//  сбросу ядра (k_rst_n = 0). После k_err ядро может не выдать k_done (§5.5):
//  busy_q держится до сброса, обёртка новых k_start не выдаёт.
//  Правила (§16):
//    1. k_done только при busy_q (не в такте k_start, не второй раз за RUN);
//    2. ровно один k_done на k_start, если не было k_err: новый k_start при
//       busy_q = «предыдущий RUN не выдал k_done» (сюда же — k_start без
//       сброса после k_err); второй k_done ловит правило 1;
//    3. k_s_tready = 0 вне окна k_start..k_done, а внутри — только до
//       принятого tlast («один RUN — одна команда STREAM», §5.1);
//    4. в такте k_rst_n = 0 ядро не обращается к порту B (и не принимает поток);
//    5. k_done только после принятого tlast своего RUN (или после k_err):
//       ядро обязано дочитать поток (§5, правила 1–2), иначе остаток beat'ов
//       молча достанется следующему RUN.
//  Живучесть (k_done когда-нибудь будет) здесь не проверяется — её ловят
//  таймауты тестов и watchdog обёртки.
// =====================================================================
module kshell_kif_sva
  import kshell_pkg::*;
(
    input logic                   clk,
    input logic                   k_rst_n,
    input logic                   k_start,
    input logic [      3:0][31:0] k_arg,
    input logic                   k_done,
    input logic                   k_err,
    input logic                   k_s_tvalid,
    input logic                   k_s_tready,
    input logic [   DW-1:0]       k_s_tdata,
    input logic                   k_s_tlast,
    input logic                   k_spm_en,
    input logic                   k_spm_we,
    input logic [SPMAW-1:0]       k_spm_addr,
    input logic [   DW-1:0]       k_spm_wdata,
    input logic [   BB-1:0]       k_spm_wstrb,
    input logic [   DW-1:0]       k_spm_rdata
);

  logic busy_q;  // RUN выдан, k_done ещё нет
  logic err_q;  // в текущем RUN был k_err
  logic tl_q;  // в текущем RUN принят beat с tlast

  always_ff @(posedge clk) begin
    if (!k_rst_n) busy_q <= 1'b0;
    else if (k_start) busy_q <= 1'b1;
    else if (k_done) busy_q <= 1'b0;
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n || k_start) err_q <= 1'b0;
    else if (k_err) err_q <= 1'b1;
  end

  always_ff @(posedge clk) begin
    if (!k_rst_n || k_start) tl_q <= 1'b0;
    else if (k_s_tvalid && k_s_tready && k_s_tlast) tl_q <= 1'b1;
  end

  default clocking cb @(posedge clk);
  endclocking

  // правило 4 — в самом такте сброса (вне disable iff)
  a_kif_rst_no_spm :
  assert property (!k_rst_n |-> !k_spm_en);
  a_kif_rst_no_tready :
  assert property (!k_rst_n |-> !k_s_tready);

  // 1. k_done только у запущенного RUN
  a_kif_done_busy :
  assert property (disable iff (!k_rst_n) k_done |-> busy_q);
  // 2. новый RUN только после k_done предыдущего (иначе его k_done потерян)
  a_kif_start_free :
  assert property (disable iff (!k_rst_n) k_start |-> !busy_q);
  // 3. поток — только в окне своего RUN и до его tlast
  a_kif_tready_window :
  assert property (disable iff (!k_rst_n) k_s_tready |-> (busy_q || k_start) && !tl_q);
  // 5. k_done — только когда tlast уже принят (в этом или прошлом такте) или был k_err
  a_kif_done_after_tlast :
  assert property (disable iff (!k_rst_n)
      k_done |-> tl_q || (k_s_tvalid && k_s_tready && k_s_tlast) || err_q || k_err);

  // покрытие контракта: RUN завершился без ошибки; ошибка ядра встречалась
  c_kif_done_ok :
  cover property (disable iff (!k_rst_n) k_done && !err_q);
  c_kif_err :
  cover property (disable iff (!k_rst_n) k_err);

endmodule
