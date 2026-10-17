#!/usr/bin/env bash
# =====================================================================
#  run.sh — обёртка над Makefile: активирует fpga-venv, чистит, гоняет
#  один модуль. `make clean` перед прогоном — иначе конфликт trace/no-trace
#  сборок между модулями (урок из gemm).
#
#  Использование:
#    ./run.sh <TOP> <MODULE> [SRCS...]
#  Примеры:
#    ./run.sh sem_file test_prims "../rtl/kshell_pkg.sv ../rtl/sem_file.sv"
# =====================================================================
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

# активируем venv c cocotb/verilator, если есть
[ -f /Users/alex/fpga-venv/bin/activate ] && source /Users/alex/fpga-venv/bin/activate

TOP="${1:?usage: run.sh <TOP> <MODULE> [SRCS]}"
MODULE="${2:?usage: run.sh <TOP> <MODULE> [SRCS]}"
SRCS="${3:-$HERE/../rtl/$TOP.sv}"

make clean >/dev/null 2>&1 || true
make TOP="$TOP" MODULE="$MODULE" SRCS="$SRCS"
