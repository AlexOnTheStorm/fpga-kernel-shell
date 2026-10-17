#!/usr/bin/env bash
# =====================================================================
#  demo.sh — демо §16: свип полосы STREAM по NTAG × задержка памяти (каждая сборка —
#  один NTAG, все L в одном прогоне через DEMO_LAT), затем перекрытие vs последовательная
#  на kshell_top, затем results/demo.md. Любой упавший тест — ненулевой код выхода.
#  Переменные: DEMO_NTAGS ("4 8 16"), DEMO_LAT ("20,50,100,150,200,300"), SIM_TIMEOUT.
#  «Симуляция с моделью памяти, не кремний».
# =====================================================================
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

export SIM_TIMEOUT="${SIM_TIMEOUT:-900}"
export DEMO_LAT="${DEMO_LAT:-20,50,100,150,200,300}"
NTAGS="${DEMO_NTAGS:-4 8 16}"
RTL="../rtl"
COMMON="$RTL/kshell_pkg.sv $RTL/fifo_sync.sv $RTL/sem_file.sv $RTL/spm.sv $RTL/rd_dma.sv $RTL/wr_dma.sv $RTL/cmd_seq.sv $RTL/axil_regs.sv $RTL/kshell_perf.sv $RTL/kshell.sv"
SRCS_SINK="$COMMON ../tb/sink_kernel.sv ../tb/kshell_sink_top.sv"
SRCS_TOP="$COMMON $RTL/gemv_kernel.sv $RTL/kshell_top.sv"
SVA_COMMON="$RTL/rd_dma_sva.sv $RTL/wr_dma_sva.sv $RTL/cmd_seq_sva.sv $RTL/kshell_sva.sv $RTL/kshell_kif_sva.sv"

mkdir -p ../results
rm -f ../results/sweep.csv ../results/overlap.csv ../results/demo.md

# make/cocotb возвращает 0 и при упавшем тесте — проверяем results.xml
check_xml() {
  python3 - "$1" <<'PY'
import sys, xml.etree.ElementTree as ET
t = ET.parse("results.xml")
bad = [c.get("name") for c in t.iter("testcase") if c.find("failure") is not None or c.find("error") is not None]
n_run = sum(1 for c in t.iter("testcase") if c.find("skipped") is None)
if bad or n_run == 0:
    sys.exit(f"demo.sh: {sys.argv[1]}: упали {bad} (выполнено тестов: {n_run})")
PY
}

for N in $NTAGS; do
  echo "=== свип: NTAG=$N, L=$DEMO_LAT ==="
  SVA_EXTRA="$SVA_COMMON ../tb/sink_kernel_sva.sv" EXTRA_ARGS="-GNTAG=$N -GWDOG_CYCLES=2000" \
    ./run.sh kshell_sink_top test_demo "$SRCS_SINK"
  check_xml "demo_sweep NTAG=$N"
done

echo "=== перекрытие: kshell_top ==="
SVA_EXTRA="$SVA_COMMON $RTL/gemv_kernel_sva.sv" EXTRA_ARGS="-GWDOG_CYCLES=2000" \
  ./run.sh kshell_top test_demo "$SRCS_TOP"
check_xml "demo_overlap"

python3 demo_report.py
echo "demo OK → results/demo.md, results/sweep.csv"
