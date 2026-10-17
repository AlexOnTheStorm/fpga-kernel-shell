#!/usr/bin/env bash
# =====================================================================
#  run.sh — обёртка над Makefile: активирует fpga-venv, чистит, гоняет
#  один модуль. `make clean` перед прогоном — иначе конфликт trace/no-trace
#  сборок между модулями (урок из gemm).
#
#  Использование:
#    ./run.sh <TOP> <MODULE> [SRCS...]
#  Сторож: после SVA-ошибки Verilator-процесс может зависнуть (macOS без `timeout`),
#  поэтому make идёт под perl-alarm; по истечении SIM_TIMEOUT секунд (по умолчанию
#  600) убивается вся группа процессов, код выхода 124.
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

SIM_TIMEOUT="${SIM_TIMEOUT:-600}"
# потомок — лидер своей группы процессов, чтобы убить make вместе с симулятором
perl -e '
  my $limit = shift;
  my $pid = fork();
  die "fork: $!" unless defined $pid;
  if ($pid == 0) { setpgrp(0, 0); exec @ARGV or die "exec: $!"; }
  my $kill_group = sub { kill "TERM", -$pid; sleep 2; kill "KILL", -$pid; };
  $SIG{ALRM} = sub {
    print STDERR "run.sh: SIM_TIMEOUT=${limit}s истёк, убиваю симуляцию\n";
    $kill_group->();
    exit 124;
  };
  $SIG{INT} = $SIG{TERM} = sub { $kill_group->(); exit 130; };
  alarm $limit;
  waitpid($pid, 0);
  exit($? & 127 ? 128 + ($? & 127) : $? >> 8);
' "$SIM_TIMEOUT" make TOP="$TOP" MODULE="$MODULE" SRCS="$SRCS"
