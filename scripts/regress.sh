#!/usr/bin/env bash
# =====================================================================
#  regress.sh — полная регрессия kshell одной командой (запуск из любого места):
#    lint → pytest (модели, генератор, чекер) → модульные тесты → обе системные
#    сборки (все SVA bind'ы, -GWDOG_CYCLES=2000); с аргументом `cov` — ещё две
#    сборки покрытия (VCOV=1) и сводка covergroup по каждому топу и по объединению.
#  Переменные: T9_SEEDS (по умолчанию 20), SIM_TIMEOUT (по умолчанию 900).
#  Любой упавший шаг — ненулевой код выхода.
#    ./regress.sh          # корректность
#    ./regress.sh cov      # + покрытие (sink.dat, top.dat, merged.dat в scripts/)
# =====================================================================
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
[ -f /Users/alex/fpga-venv/bin/activate ] && source /Users/alex/fpga-venv/bin/activate

export SIM_TIMEOUT="${SIM_TIMEOUT:-900}"
export T9_SEEDS="${T9_SEEDS:-20}"
RTL="../rtl"
COMMON="$RTL/kshell_pkg.sv $RTL/fifo_sync.sv $RTL/sem_file.sv $RTL/spm.sv $RTL/rd_dma.sv $RTL/wr_dma.sv $RTL/cmd_seq.sv $RTL/axil_regs.sv $RTL/kshell_perf.sv $RTL/kshell.sv"
SRCS_SINK="$COMMON ../tb/sink_kernel.sv ../tb/kshell_sink_top.sv"
SRCS_TOP="$COMMON $RTL/gemv_kernel.sv $RTL/kshell_top.sv"
SVA_COMMON="$RTL/rd_dma_sva.sv $RTL/wr_dma_sva.sv $RTL/cmd_seq_sva.sv $RTL/kshell_sva.sv $RTL/kshell_kif_sva.sv"
COV_FILES="$RTL/rd_dma_cov.sv $RTL/cmd_seq_cov.sv $RTL/kshell_cov.sv"

# make/cocotb возвращает 0 и при упавшем тесте — проверяем results.xml
check_xml() {
  python3 - "$1" <<'PY'
import sys, xml.etree.ElementTree as ET
t = ET.parse("results.xml")
cases = list(t.iter("testcase"))
bad = [c.get("name") for c in cases if c.find("failure") is not None or c.find("error") is not None]
n_run = sum(1 for c in cases if c.find("skipped") is None)
print(f"regress: {sys.argv[1]}: выполнено {n_run}, упало {len(bad)}")
if bad or n_run == 0:
    sys.exit(f"regress: {sys.argv[1]}: упали {bad}")
PY
}

run() {  # run <метка> <TOP> <MODULE> <SRCS>  (SVA_EXTRA/EXTRA_ARGS — из окружения)
  echo "=== $1 ==="
  ./run.sh "$2" "$3" "$4" >"regress_$1.log" 2>&1 || { tail -30 "regress_$1.log"; exit 1; }
  check_xml "$1"
}

echo "=== lint ==="
make lint
echo "=== pytest ==="
python -m pytest ../tb/test_isa.py ../tb/test_hbm_model.py -q

run prims_sem_file sem_file test_prims "$RTL/kshell_pkg.sv $RTL/sem_file.sv"
run prims_fifo_sync fifo_sync test_prims "$RTL/kshell_pkg.sv $RTL/fifo_sync.sv"
run prims_spm spm test_prims "$RTL/kshell_pkg.sv $RTL/spm.sv"
run rd_dma rd_dma test_rd_dma "$RTL/kshell_pkg.sv $RTL/fifo_sync.sv $RTL/rd_dma.sv"
run wr_dma wr_dma test_wr_dma "$RTL/kshell_pkg.sv $RTL/fifo_sync.sv $RTL/wr_dma.sv"
run cmd_seq cmd_seq test_cmd_seq "$RTL/kshell_pkg.sv $RTL/cmd_seq.sv"
run gemv gemv_kernel test_gemv "$RTL/kshell_pkg.sv $RTL/kshell_kif_sva.sv $RTL/gemv_kernel.sv"

SVA_EXTRA="$SVA_COMMON ../tb/sink_kernel_sva.sv" EXTRA_ARGS="-GWDOG_CYCLES=2000" \
  run sys_sink kshell_sink_top test_system "$SRCS_SINK"
SVA_EXTRA="$SVA_COMMON $RTL/gemv_kernel_sva.sv" EXTRA_ARGS="-GWDOG_CYCLES=2000" \
  run sys_top kshell_top test_system "$SRCS_TOP"

if [ "${1:-}" = "cov" ]; then
  # покрытие — отдельные сборки (--coverage без --assert, см. Makefile)
  for t in sink top; do
    if [ $t = sink ]; then top=kshell_sink_top; srcs="$SRCS_SINK"; else top=kshell_top; srcs="$SRCS_TOP"; fi
    echo "=== cov_$t ==="
    COV_EXTRA="$COV_FILES" EXTRA_ARGS="-GWDOG_CYCLES=2000" \
      make cov TOP=$top MODULE=test_system SRCS="$srcs" >"regress_cov_$t.log" 2>&1 \
      || { tail -30 "regress_cov_$t.log"; exit 1; }
    check_xml "cov_$t"
    cp coverage.dat $t.dat
  done
  verilator_coverage --write merged.dat top.dat sink.dat
  # сводка covergroup: бины с ненулевым счётом по каждому топу и по объединению
  python3 - <<'PY'
import re
def bins(path):
    out = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        if "v_covergroup" not in line:
            continue
        m = re.match(r"C '(.*)' (\d+)$", line.strip())
        if m:
            out[m.group(1)] = int(m.group(2))
    return out
for name in ("sink", "top", "merged"):
    b = bins(f"{name}.dat")
    hit = sum(1 for v in b.values() if v > 0)
    miss = [k.split("\x01h\x02")[-1] for k, v in b.items() if v == 0]
    print(f"covergroup {name}: {hit}/{len(b)} бинов" + (f", не покрыты: {miss}" if miss else ""))
PY
fi
echo "regress OK"
