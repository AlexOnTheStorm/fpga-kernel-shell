# Демо (docs/design.md §3, §6.2, §14, §16): «симуляция с моделью памяти, не кремний».
#   demo_sweep   (kshell_sink_top) — полоса STREAM в sink_kernel против NTAG × задержка памяти;
#                сборка на один NTAG, задержки L — по списку DEMO_LAT (через запятую), одна
#                сборка на все L. Строки дописываются в results/sweep.csv.
#   demo_overlap (kshell_top)      — программа §6.2 с перекрытием и последовательная, M=64,
#                K=512, lat=(40,160) (как T4); таблица perf → results/overlap.csv.
# Тесты утверждают качественный результат, чтобы демо не гнило молча:
#   полоса ≥ 0.9 при L ≪ NTAG×BURST, строго убывает с L за «коленом», перекрытие быстрее.
# results/demo.md собирает scripts/demo_report.py (см. scripts/demo.sh).
import os

import cocotb
import numpy as np
from cocotb.triggers import ReadOnly, RisingEdge

from hbm_model import HbmModel
from isa import Program, check_program
import shell
# только функции (не @cocotb.test): иначе cocotb подберёт чужие тесты из импортированного модуля
from test_system import bringup, gemv_case, overlapped, sequential, settle, spm_words

TOP = os.environ.get("COCOTB_TOPLEVEL") or os.environ.get("TOPLEVEL")
RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
BURST = 8                       # beat'ов в burst'е (kshell_pkg::BURST)
BEATS = 1024                    # = SPM_WORDS: 32 KB (ruling R6)
SWEEP_HDR = "ntag,lat,beats,cycles,bw,bw_ss,model"
OVL_HDR = "name,CYCLES,KRN_BUSY,RD_BURSTS,RD_LAT_SUM,STRM_BP,SEQ_WAIT,RD_WIN_FULL"


def model_bw(ntag, lat):
    """Закон Литтла (§3): в полёте NTAG×BURST beat'ов, время жизни burst'а ≈ L + BURST тактов
    (AR → последний beat), потолок — 1 beat за такт."""
    return min(1.0, ntag * BURST / (lat + BURST))


def _append(path, hdr, row):
    os.makedirs(RESULTS, exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a") as f:
        if new:
            f.write(hdr + "\n")
        f.write(row + "\n")


class StreamMon:
    """Номера тактов: первый AR-handshake и каждый handshake потока ядру (tvalid & tready)."""

    def __init__(self, dut):
        self.dut = dut
        self.t = 0
        self.first_ar = None
        self.beats = []
        self._task = cocotb.start_soon(self._run())

    async def _run(self):
        d = self.dut
        while True:
            await RisingEdge(d.clk)
            await ReadOnly()
            if self.first_ar is None and int(d.m_axi_arvalid.value) and int(d.m_axi_arready.value):
                self.first_ar = self.t
            if int(d.k_s_tvalid.value) and int(d.k_s_tready.value):
                self.beats.append(self.t)
            self.t += 1

    def stop(self):
        self._task.cancel()


@cocotb.test(skip=TOP != "kshell_sink_top")
async def demo_sweep(dut):
    ntag = 1 << len(dut.m_axi_arid)          # ширина ARID = clog2(NTAG) — NTAG собранного DUT
    lats = [int(x) for x in os.environ.get("DEMO_LAT", "20,50,100,150,200,300").split(",")]
    csv = os.path.join(RESULTS, "sweep.csv")
    ax, mem = await bringup(dut, lat=(lats[0], lats[0]), seed=11, p_stall=0.0, n_pc=8)
    src = bytes(np.random.default_rng(11).integers(0, 256, BEATS * 32, dtype=np.uint8))
    mem.write_bytes(0x300000, src)
    res = {}
    for lat in lats:
        mem.lat = (lat, lat)
        mon = StreamMon(dut)
        p = Program(); p.stream(0x300000, len(src)); p.run(0, 0, BEATS, 0, 0); p.wait(0, 1)
        p.signal(7, 1)
        check_program(p, ntag=ntag, run_regions=None)
        await shell.submit(ax, p)
        await shell.wait_sem(ax, 7, 1, timeout=60000)
        st = await settle(dut, ax)
        assert not st["ERROR"], st
        mon.stop()
        assert len(mon.beats) == BEATS, (lat, len(mon.beats))
        w = spm_words(dut, range(BEATS))
        assert all(w[i] == src[32 * i:32 * i + 32] for i in range(BEATS)), f"данные L={lat}"
        cycles = mon.beats[-1] - mon.first_ar + 1
        bw = BEATS / cycles
        # установившаяся полоса: средняя половина потока (без разгона на L и без хвоста)
        lo, hi = BEATS // 4, 3 * BEATS // 4
        bw_ss = (hi - lo) / (mon.beats[hi] - mon.beats[lo])
        mdl = model_bw(ntag, lat)
        res[lat] = (bw, bw_ss, mdl)
        _append(csv, SWEEP_HDR, f"{ntag},{lat},{BEATS},{cycles},{bw:.4f},{bw_ss:.4f},{mdl:.4f}")
        dut._log.info("NTAG=%d L=%d: cycles=%d bw=%.3f bw_ss=%.3f model=%.3f",
                      ntag, lat, cycles, bw, bw_ss, mdl)
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)

    # --- качественные утверждения (§3)
    for lat, (bw, bw_ss, mdl) in res.items():
        assert abs(bw_ss - mdl) <= 0.10 * mdl + 0.02, f"NTAG={ntag} L={lat}: ss={bw_ss:.3f} модель={mdl:.3f}"
        if 2 * (lat + BURST) <= ntag * BURST:           # L ≪ NTAG×BURST: окно закрывает задержку
            assert bw_ss >= 0.95 and bw >= 0.9, f"NTAG={ntag} L={lat}: bw={bw:.3f} ss={bw_ss:.3f}"
    beyond = [l for l in sorted(res) if model_bw(ntag, l) < 1.0]      # за «коленом»
    for a, b in zip(beyond, beyond[1:]):
        assert res[b][1] < res[a][1], f"NTAG={ntag}: полоса не убывает {a}→{b}: {res[a][1]:.3f} {res[b][1]:.3f}"


@cocotb.test(skip=TOP != "kshell_top")
async def demo_overlap(dut):
    ax, mem = await bringup(dut, lat=(40, 160), seed=4)
    rng = np.random.default_rng(4); M, K = 64, 512
    W, x0, y0 = gemv_case(rng, M, K); _, x1, _ = gemv_case(rng, M, K)
    y1 = (W.astype(np.int64) @ x1.astype(np.int64)).astype(np.int32)
    mem.write_bytes(0x1000000, W.tobytes()); mem.write_bytes(0x2000000, x0.tobytes())
    mem.write_bytes(0x2100000, x1.tobytes())
    res = {}
    for name, fn in (("sequential", sequential), ("overlapped", overlapped)):
        for a in (0x3000000, 0x3100000):
            mem.write_bytes(a, b"\xAA" * (4 * M))
        p = fn(M, K); check_program(p)
        await shell.submit(ax, p); await shell.wait_sem(ax, 7, 1)
        res[name] = await shell.perf(ax)
        assert (np.frombuffer(mem.read_bytes(0x3000000, 4 * M), np.int32) == y0).all()
        assert (np.frombuffer(mem.read_bytes(0x3100000, 4 * M), np.int32) == y1).all()
        await shell.soft_reset(ax); await ax.write(shell.REG["CTRL"], 1)
    csv = os.path.join(RESULTS, "overlap.csv")
    if os.path.exists(csv):
        os.remove(csv)
    for name, r in res.items():
        _append(csv, OVL_HDR, f"{name},{r['CYCLES']},{r['KRN_BUSY']},{r['RD_BURSTS']},"
                f"{r['RD_LAT_SUM']},{r['STRM_BP']},{r['SEQ_WAIT']},{r['RD_WIN_FULL']}")
    s, o = res["sequential"], res["overlapped"]
    assert o["CYCLES"] < s["CYCLES"], (s, o)
    assert o["KRN_BUSY"] / o["CYCLES"] > s["KRN_BUSY"] / s["CYCLES"], (s, o)
    dut._log.info("speedup %.2fx", s["CYCLES"] / o["CYCLES"])
