`timescale 1ns / 1ps
// =====================================================================
//  gemv_kernel_sva — исполняемый контракт ядра (kshell_kif_sva) на gemv_kernel.
//  Bind; синтез не читает. Модуль kshell_kif_sva.sv компилируется отдельно
//  (SRCS или SVA_EXTRA).
// =====================================================================
bind gemv_kernel kshell_kif_sva u_kif (.*);
