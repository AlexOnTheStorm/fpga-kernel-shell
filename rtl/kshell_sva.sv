`timescale 1ns / 1ps
// =====================================================================
//  kshell_sva — системные ассершены kshell (bind; синтез не читает).
//  docs/design.md §10, §11, §12, §16.
//    A. SOFT_RESET только в тишине AXI; k_rst_n = rst_n && !soft_rst.
//    B. Ошибка: липкая до soft_rst; первая (код и номер) не перезаписывается;
//       код ≠ NONE при ERROR; abort держится вместе с ошибкой (контракт wr_dma);
//       приоритет одновременных ошибок §12.2 (TIMEOUT > RRESP > BRESP > KERNEL > dec).
//    C. Семафоры: при ошибке приращений от rd/wr/ядра нет (§11); k_done в такте
//       сырого k_err (до регистра k_err_q) семафор RUN не увеличивает.
//    D. spm: порт A — один источник за такт (запись rd_dma ИЛИ чтение wr_dma);
//       запись одного порта не совпадает по адресу с доступом другого (§10);
//       чтение A и чтение B одного адреса — законно.
//    E. Ядро: k_start только при свободном ядре и без ошибки/паузы; krn_busy
//       снимается только k_done/soft_rst.
//    F. Предикаты: IDLE ⇒ AXI_QUIET и всё пусто; watchdog не считает в тишине.
//    G. cq_ovf — липкий, не ошибка.
// =====================================================================
module kshell_sva
  import kshell_pkg::*;
(
    input logic             clk,
    input logic             rst_n,
    input logic             soft_rst,
    input logic             axi_quiet,
    input logic             idle,
    input logic             k_rst_n,
    input logic             enable,
    input logic             error_q,
    input logic             err_any,
    input logic             go,
    input logic             abort,
    input err_e             err_code_q,
    input logic [     31:0] err_cmd_q,
    input logic             wdog_hit_q,
    input logic             rd_err_v,
    input logic             wr_err_v,
    input logic             k_err_q,
    input logic             dec_err_v,
    input logic             inc_rd_v,
    input logic             inc_wr_v,
    input logic             k_done_ok,
    input logic             sem_we,
    input logic [NSEM-1:0][31:0] sem_val,
    input logic             rd_spm_wr_v,
    input logic [SPMAW-1:0] rd_spm_wr_addr,
    input logic             wr_spm_rd_gnt,
    input logic [SPMAW-1:0] wr_spm_rd_addr,
    input logic             k_spm_en,
    input logic             k_spm_we,
    input logic [SPMAW-1:0] k_spm_addr,
    input logic             k_start,
    input logic             k_done,
    input logic             k_err,
    input logic             krn_busy_q,
    input logic             cq_empty,
    input logic             rq_empty,
    input logic             wq_empty,
    input logic             kq_empty,
    input logic             seq_busy,
    input logic             cq_ovf_q,
    input logic             wdog_cnt
);

  default clocking cb @(posedge clk);
  endclocking
  default disable iff (!rst_n);

  // A. SOFT_RESET
  a_soft_rst_quiet :
  assert property (soft_rst |-> axi_quiet);
  a_k_rst_n :
  assert property (k_rst_n == (rst_n && !soft_rst));

  // B. ошибка
  a_err_sticky :
  assert property (error_q && !soft_rst |=> error_q);
  a_err_first_kept :
  assert property (error_q && !soft_rst |=> $stable(err_code_q) && $stable(err_cmd_q));
  a_err_code_set :
  assert property (error_q |-> err_code_q != E_NONE);
  a_err_from_source :
  assert property (!error_q && !soft_rst |=> error_q == $past(err_any));
  a_abort_with_err :
  assert property (error_q |-> abort && !go);
  a_err_clear_on_soft_rst :
  assert property (soft_rst |=> !error_q && err_code_q == E_NONE);
  // приоритет §12.2 при совпадении в одном такте
  a_prio_timeout :
  assert property (!error_q && !soft_rst && wdog_hit_q |=> err_code_q == E_TIMEOUT);
  a_prio_rresp :
  assert property (!error_q && !soft_rst && !wdog_hit_q && rd_err_v |=> err_code_q == E_RRESP);
  a_prio_bresp :
  assert property (!error_q && !soft_rst && !wdog_hit_q && !rd_err_v && wr_err_v
                   |=> err_code_q == E_BRESP);
  a_prio_kernel :
  assert property (!error_q && !soft_rst && !wdog_hit_q && !rd_err_v && !wr_err_v && k_err_q
                   |=> err_code_q == E_KERNEL);

  // C. семафоры не растут от движков/ядра при ошибке
  a_no_inc_on_err :
  assert property ((error_q || err_any) |-> !inc_rd_v && !inc_wr_v && !k_done_ok);
  // k_done вместе с k_err ядра: приращения от ядра (inc_k_v = k_done_ok) нет в этом же такте
  a_no_kinc_on_raw_kerr :
  assert property (k_done && k_err |-> !k_done_ok);
  // под ошибкой (включая такт её импульса) семафор меняет только хост: SIGNAL секвенсор
  // тоже не выдаёт — независимая от гейтинга проверка R10
  a_sem_frozen_on_err :
  assert property ((error_q || err_any) && !soft_rst && !sem_we |=> $stable(sem_val));

  // D. scratchpad
  a_spm_a_one_src :
  assert property (!(rd_spm_wr_v && wr_spm_rd_gnt));
  a_spm_ab_wr_collide :
  assert property (!(rd_spm_wr_v && k_spm_en && rd_spm_wr_addr == k_spm_addr));
  a_spm_ab_rd_collide :
  assert property (!(wr_spm_rd_gnt && k_spm_en && k_spm_we && wr_spm_rd_addr == k_spm_addr));

  // E. ядро
  a_kstart_free :
  assert property (k_start |-> !krn_busy_q && go && enable);
  a_kbusy_set :
  assert property (k_start && !soft_rst |=> krn_busy_q);
  a_kbusy_hold :
  assert property (krn_busy_q && !k_done && !soft_rst |=> krn_busy_q);
  a_kbusy_rst :
  assert property (soft_rst |=> !krn_busy_q);

  // F. предикаты
  a_idle_quiet :
  assert property (idle |-> axi_quiet && cq_empty && rq_empty && wq_empty && kq_empty
                   && !krn_busy_q && !seq_busy);
  a_wdog_quiet :
  assert property (axi_quiet |-> !wdog_cnt);

  // G. cq_ovf
  a_ovf_sticky :
  assert property (cq_ovf_q && !soft_rst |=> cq_ovf_q);

endmodule

bind kshell kshell_sva u_sva (.*);
