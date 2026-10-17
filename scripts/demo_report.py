#!/usr/bin/env python3
"""Собирает results/demo.md из results/sweep.csv и results/overlap.csv (пишет tb/test_demo.py)."""
import csv
import os
import sys

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")


def rows(name):
    with open(os.path.join(R, name)) as f:
        return list(csv.DictReader(f))


def main():
    sw, ov = rows("sweep.csv"), rows("overlap.csv")
    ntags = sorted({int(r["ntag"]) for r in sw})
    lats = sorted({int(r["lat"]) for r in sw})
    by = {(int(r["ntag"]), int(r["lat"])): r for r in sw}
    assert len(by) == len(ntags) * len(lats) == len(sw), "в sweep.csv дубли или дыры"
    o = {r["name"]: {k: (float(v) if k == "p_stall" else int(v)) for k, v in r.items() if k != "name"}
         for r in ov}
    s, v = o["sequential"], o["overlapped"]

    out = ["# Демо: перекрытие команд и полоса STREAM против задержки памяти", "",
           "**Симуляция с моделью памяти, не кремний** (Verilator + cocotb; AXI-модель `tb/hbm_model.py`, "
           "8 псевдоканалов, OoO по ID; такт = 1 ед., частота не задана). Числа ниже — тактовые "
           "счётчики симуляции и описывают структуру обёртки, а не измеренную скорость на плате.", "",
           f"## 1. Перекрытие (§6.2): GEMV M=64, K=512, две порции, `lat=({s['lat_lo']},{s['lat_hi']})`, "
           f"`p_stall={s['p_stall']}`", "",
           "| программа | CYCLES | KRN_BUSY, % | средняя задержка чтения (RD_LAT_SUM/RD_BURSTS), тактов "
           "| STRM_BP | SEQ_WAIT | RD_WIN_FULL |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name, label in (("sequential", "последовательная (WAIT после каждой команды)"),
                        ("overlapped", "с перекрытием (§6.2)")):
        r = o[name]
        out.append(f"| {label} | {r['CYCLES']} | {100 * r['KRN_BUSY'] / r['CYCLES']:.1f} | "
                   f"{r['RD_LAT_SUM'] / r['RD_BURSTS']:.1f} | {r['STRM_BP']} | {r['SEQ_WAIT']} | "
                   f"{r['RD_WIN_FULL']} |")
    out += ["", f"Ускорение (CYCLES последовательной / с перекрытием): **{s['CYCLES'] / v['CYCLES']:.2f}×**. "
            "Перекрытие прячет загрузку x и часть задержки памяти за счётом ядра: растёт доля "
            "KRN_BUSY, SEQ_WAIT (команда в голове ждёт семафор) падает. Выигрыш скромный: GEMV 64×512 "
            "ограничен потоком матрицы W (STREAM 32 KB на порцию), перекрывается только загрузка x "
            "и STORE; ядро и так занято большую часть времени.", "",
            "## 2. Полоса STREAM против NTAG × задержка памяти (закон Литтла, §3)", "",
            "Одна команда STREAM на 32 KB (1024 beat'а = SPM_WORDS) в `sink_kernel` с `tready = 1`; "
            "модель памяти с **фиксированной** задержкой `L` (`lat=(L,L)`) и `p_stall=0`; "
            "полоса — в beat'ах за такт, потолок 1.0.", "",
            "- **Измерено, `bw`** = 1024 / (такты от первого AR-handshake до последнего beat'а потока). "
            "Включает разгон: первый beat приходит не раньше чем через ≈ L тактов, поэтому даже "
            "без колена `bw` ≈ 1024 / (1024 + L + const).",
            "- **Измерено, `bw_ss`** = установившаяся полоса: 512 beat'ов средней половины потока / "
            "их такты (без разгона и хвоста). Это то, что сравнивается с моделью.",
            "- **Модель** = `min(1, NTAG·BURST / (L + BURST))`, BURST = 8. Это закон Литтла: в окне "
            "NTAG·BURST beat'ов, слот занят от AR до последнего beat'а (≈ L + BURST тактов). "
            "Колено — при `L ≈ NTAG·BURST − BURST` (NTAG=4: 24, 8: 56, 16: 120 тактов). "
            "Измерено немного ниже модели за коленом: данные хорошо ложатся на эмпирическую подгонку "
            "`min(1, NTAG·BURST / (L + 2·BURST))` (константное смещение ≈ BURST = 8 тактов, ≈ 16 тактов "
            "против L, одинаковое во всех точках). Возможное объяснение — слот освобождается только после "
            "того, как его beat'ы ушли в поток (коммит по порядку), — это вывод из подгонки, "
            "**трассой не проверено**. Тесты проверяют: `bw_ss` не выше модели закона Литтла (потолок), "
            "`bw_ss` в пределах 3 % от подгонки, насыщение ≥ 0.97 там, где подгонка даёт 1.0.", ""]

    def matrix(title, key, fmt="{:.3f}"):
        out.extend([f"### {title}", "", "| NTAG \\ L, тактов | " + " | ".join(str(l) for l in lats) + " |",
                    "|---|" + "---:|" * len(lats)])
        for n in ntags:
            out.append(f"| {n} (окно {n * 8} beat'ов) | " +
                       " | ".join(fmt.format(float(by[(n, l)][key])) for l in lats) + " |")
        out.append("")

    matrix("Измерено: установившаяся полоса `bw_ss`, beat/такт", "bw_ss")
    matrix("Измерено: полоса `bw` от первого AR до последнего beat'а", "bw")
    matrix("Модель: `min(1, NTAG·BURST / (L + BURST))`", "model")
    out += ["### Сырые строки (`results/sweep.csv`)", "", "| NTAG | L | beat'ов | тактов | bw | bw_ss | модель |",
            "|---:|---:|---:|---:|---:|---:|---:|"]
    for n in ntags:
        for l in lats:
            r = by[(n, l)]
            out.append(f"| {n} | {l} | {r['beats']} | {r['cycles']} | {float(r['bw']):.3f} | "
                       f"{float(r['bw_ss']):.3f} | {float(r['model']):.3f} |")
    out += ["", "Воспроизвести: `scripts/demo.sh` (≈ минуты; пересборка на каждый NTAG).", ""]
    with open(os.path.join(R, "demo.md"), "w") as f:
        f.write("\n".join(out))
    print("results/demo.md собран")


if __name__ == "__main__":
    sys.exit(main())
