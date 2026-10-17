# Тест демо-ядра gemv_kernel отдельно (docs/design.md §15): тестбенч играет роль обёртки —
# гонит поток W и моделирует scratchpad на порту B (чтение с задержкой 1 такт).
import random, cocotb, numpy as np
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

def pack_i8(v):  return int.from_bytes(np.asarray(v, np.int8).tobytes(), "little")

@cocotb.test()
async def test_gemv_vs_numpy(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    rng = np.random.default_rng(5)
    for M, K in ((8, 32), (16, 64), (64, 256)):
        W = rng.integers(-128, 128, (M, K), dtype=np.int8)
        x = rng.integers(-128, 128, K, dtype=np.int8)
        spm = {i: pack_i8(x[32*i:32*(i+1)]) for i in range(K // 32)}
        dut.k_rst_n.value = 0; dut.k_start.value = 0; dut.k_s_tvalid.value = 0
        await RisingEdge(dut.clk); dut.k_rst_n.value = 1
        X_AT, Y_AT = 0, 100
        dut.k_arg.value = (K << 96) | (M << 64) | (Y_AT << 32) | X_AT
        dut.k_start.value = 1; await RisingEdge(dut.clk); dut.k_start.value = 0
        beats = [pack_i8(W[m, 32*j:32*(j+1)]) for m in range(M) for j in range(K // 32)]
        prng = random.Random(M); i = 0; rd_pending = None; done = False
        while not done:
            dut.k_s_tvalid.value = 1 if (i < len(beats) and prng.random() < 0.8) else 0
            dut.k_s_tdata.value = beats[i] if i < len(beats) else 0
            dut.k_s_tlast.value = 1 if i == len(beats) - 1 else 0
            dut.k_spm_rdata.value = spm.get(rd_pending, 0) if rd_pending is not None else 0
            await ReadOnly()
            acc = int(dut.k_s_tvalid.value) and int(dut.k_s_tready.value)
            rd_pending = None
            if int(dut.k_spm_en.value):
                a = int(dut.k_spm_addr.value)
                if int(dut.k_spm_we.value):
                    old = spm.get(a, 0); st = int(dut.k_spm_wstrb.value); wd = int(dut.k_spm_wdata.value)
                    for b in range(32):
                        if st >> b & 1: old = (old & ~(0xFF << 8*b)) | (wd & (0xFF << 8*b))
                    spm[a] = old
                else:
                    rd_pending = a
            done = int(dut.k_done.value) == 1
            assert int(dut.k_err.value) == 0
            await RisingEdge(dut.clk)
            if acc: i += 1
        y = (W.astype(np.int64) @ x.astype(np.int64)).astype(np.int32)
        got = [np.int32(np.uint32((spm.get(Y_AT + m // 8, 0) >> (32 * (m % 8))) & 0xFFFFFFFF))
               for m in range(M)]
        assert list(got) == list(y), f"M={M} K={K}"


# ============================================================================
#  Дополнительно к брифу (Task 7): модель обёртки с проверкой spm целиком, паттерны
#  tvalid, темп (beat/такт), k_err по tlast и аргументам, RUN подряд, переполнение INT32.
# ============================================================================
SPM_WORDS = 1024


def gemv_ref(W, x):
    return (W.astype(np.int64) @ x.astype(np.int64)).astype(np.int32)


def w_beats(W):
    M, K = W.shape
    return [pack_i8(W[m, 32*j:32*(j+1)]) for m in range(M) for j in range(K // 32)]


def put_y(spm, y_at, y):
    """Ожидаемый spm после записи y: дорожка m%8 слова y_at + m/8, остальные байты не трогаются."""
    for m, v in enumerate(y):
        a = (y_at + m // 8) % SPM_WORDS; sh = 32 * (m % 8)
        spm[a] = (spm.get(a, 0) & ~(0xFFFFFFFF << sh)) | ((int(v) & 0xFFFFFFFF) << sh)


class Shell:
    """Тестбенч в роли обёртки: поток W на AXIS + scratchpad на порту B (чтение через 1 такт)."""

    def __init__(self, dut, spm=None):
        self.dut, self.spm, self.rd_pending, self.cycle = dut, dict(spm or {}), None, 0
        self.reads, self.writes = [], []                       # (такт, адрес)

    async def tick(self, tvalid=0, tdata=0, tlast=0, start=0, arg=None, rst_n=1):
        d = self.dut
        d.k_rst_n.value = rst_n; d.k_start.value = start
        if arg is not None:
            d.k_arg.value = arg
        d.k_s_tvalid.value = tvalid; d.k_s_tdata.value = tdata; d.k_s_tlast.value = tlast
        d.k_spm_rdata.value = self.spm.get(self.rd_pending, 0) if self.rd_pending is not None else 0
        await ReadOnly()
        s = {k: int(getattr(d, n).value) for k, n in (
            ("tready", "k_s_tready"), ("done", "k_done"), ("err", "k_err"), ("en", "k_spm_en"),
            ("we", "k_spm_we"), ("addr", "k_spm_addr"))}
        s["hs"] = int(tvalid and s["tready"]); s["cycle"] = self.cycle
        self.rd_pending = None
        if s["en"]:
            a = s["addr"]
            if s["we"]:
                old = self.spm.get(a, 0); st = int(d.k_spm_wstrb.value); wd = int(d.k_spm_wdata.value)
                for b in range(32):
                    if st >> b & 1:
                        old = (old & ~(0xFF << 8*b)) | (wd & (0xFF << 8*b))
                self.spm[a] = old; self.writes.append((self.cycle, a))
            else:
                self.rd_pending = a; self.reads.append((self.cycle, a))
        assert not (s["done"] and s["err"]), "k_done и k_err в одном такте"
        await RisingEdge(d.clk)
        self.cycle += 1
        return s

    async def reset(self):
        await self.tick(rst_n=0)

    async def run(self, beats, M, K, x_at, y_at, vfn=lambda c: True, tlast_idx="last", max_cycles=100000):
        """k_start (такт 0) → поток beats с tvalid по vfn(такт) → k_done/k_err. Возвращает лог."""
        if tlast_idx == "last":
            tlast_idx = len(beats) - 1
        arg = (K << 96) | (M << 64) | (y_at << 32) | x_at
        t0 = self.cycle; nw0 = len(self.writes); nr0 = len(self.reads)
        s = await self.tick(start=1, arg=arg)
        assert not s["tready"] and not s["en"], "до k_start ядро не принимает поток и молчит на порту B"
        r = {"t0": t0, "hs": [], "done": None, "err": None, "trace": []}
        i = 0
        for c in range(max_cycles):
            v = int(i < len(beats) and vfn(c))
            s = await self.tick(tvalid=v, tdata=beats[i] if i < len(beats) else 0,
                                tlast=int(i == tlast_idx))
            r["trace"].append((s["cycle"], v, s["tready"], s["we"] and s["en"]))
            if s["hs"]:
                r["hs"].append(s["cycle"]); i += 1
            if s["done"]:
                r["done"] = s["cycle"]; break
            if s["err"]:
                r["err"] = s["cycle"]; break
        else:
            raise AssertionError(f"нет k_done/k_err за {max_cycles} тактов (принято {i})")
        r["accepted"] = i
        r["reads"] = self.reads[nr0:]; r["writes"] = self.writes[nw0:]
        return r

    async def quiet(self, n=20, beat=0):
        """После k_done/k_err: поток предлагается, но ядро его не берёт и не трогает spm."""
        for _ in range(n):
            s = await self.tick(tvalid=1, tdata=beat, tlast=0)
            assert not (s["tready"] or s["en"] or s["done"] or s["err"]), f"ядро активно вне RUN: {s}"


def check_run(r, sh, W, x, x_at, y_at, exp_spm):
    """Результат + spm целиком + чтения x ровно в такты handshake по адресам x_at + j."""
    M, K = W.shape; KW = K // 32
    assert r["done"] is not None and r["err"] is None, f"нет k_done: {r['err']=}"
    assert r["accepted"] == M * KW
    assert [c for c, _ in r["reads"]] == r["hs"], "чтение x — в такте handshake, по одному на beat"
    assert [a for _, a in r["reads"]] == [(x_at + n % KW) % SPM_WORDS for n in range(M * KW)]
    assert [a for _, a in r["writes"]] == [(y_at + m // 8) % SPM_WORDS for m in range(M)]
    assert r["done"] == r["writes"][-1][0] + 1, "k_done в такте после записи последнего y"
    put_y(exp_spm, y_at, gemv_ref(W, x))
    keys = set(exp_spm) | set(sh.spm)
    bad = [a for a in sorted(keys) if exp_spm.get(a, 0) != sh.spm.get(a, 0)]
    assert not bad, f"spm расходится в словах {bad[:8]} (M={M} K={K})"


def x_words(x, x_at):
    return {(x_at + i) % SPM_WORDS: pack_i8(x[32*i:32*(i+1)]) for i in range(len(x) // 32)}


async def fresh(dut, spm=None):
    # cocotb гасит задачи в конце каждого теста → свой Clock на тест (fresh — один раз на тест)
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    sh = Shell(dut, spm); await sh.reset(); await sh.reset()
    return sh


def kif_busy(dut):
    """busy_q привязанного kshell_kif_sva (RUN без k_done); None, если иерархия недоступна."""
    try:
        return int(dut.u_kif.busy_q.value)
    except AttributeError:
        return None


@cocotb.test()
async def test_gemv_throughput(dut):
    """tvalid = 1 всегда: каждый такт без приёма внутри окна потока — такт записи y (пауза
    строки), т.е. темп = 1 beat/такт за вычетом пауз; k_done через 6 тактов после последнего beat'а."""
    rng = np.random.default_rng(11)
    sh = await fresh(dut)
    for M, K in ((64, 256), (16, 1024), (32, 32), (9, 64)):
        W = rng.integers(-128, 128, (M, K), dtype=np.int8); x = rng.integers(-128, 128, K, dtype=np.int8)
        x_at, y_at = 3, 700
        sh.spm = x_words(x, x_at); exp = dict(sh.spm)
        r = await sh.run(w_beats(W), M, K, x_at, y_at)
        check_run(r, sh, W, x, x_at, y_at, exp)
        first, last = r["hs"][0], r["hs"][-1]
        win = [t for t in r["trace"] if first <= t[0] <= last]
        idle = [t for t in win if not t[2]]
        assert all(t[3] for t in idle), "tready = 0 внутри потока только в такт записи y"
        assert first == r["t0"] + 1, "первый beat принимается в такте сразу после k_start"
        bpc = len(r["hs"]) / (last - first + 1); KW = K // 32
        dut._log.info(f"M={M} K={K}: {len(r['hs'])} beat / {last - first + 1} тактов = "
                      f"{bpc:.3f} beat/такт (пауз {len(idle)}, KW/(KW+1)={KW/(KW+1):.3f}); "
                      f"k_done через {r['done'] - last} тактов после последнего beat'а")
        assert r["done"] - last == 6
        assert bpc >= KW / (KW + 1) - 1e-9  # не хуже одной паузы на строку
        await sh.quiet(5)


@cocotb.test()
async def test_gemv_stall_patterns(dut):
    """Паттерны tvalid; M не кратно 8 (неполное последнее слово y), ненулевые адреса,
    часовые слова вокруг y: соседние дорожки и слова не тронуты, x не испорчен."""
    rng = np.random.default_rng(23)
    pats = {
        "bursty7/3": lambda c: c % 10 < 7,
        "alt": lambda c: c % 2 == 0,
        "sparse10": (lambda p: (lambda c: p.random() < 0.1))(random.Random(1)),
        "rand50": (lambda p: (lambda c: p.random() < 0.5))(random.Random(2)),
        "late": lambda c: c > 40,
    }
    sh = await fresh(dut)
    for n, (name, vfn) in enumerate(pats.items()):
        M, K = (13, 96) if n % 2 == 0 else (5, 32)
        W = rng.integers(-128, 128, (M, K), dtype=np.int8); x = rng.integers(-128, 128, K, dtype=np.int8)
        x_at, y_at = 1020, 37 + n        # x заворачивается через конец spm
        sh.spm = {a: int(rng.integers(0, 2**63)) << 192 | int(rng.integers(0, 2**63))
                  for a in range(30, 60)}  # часовые слова (в т.ч. дорожки y-слов)
        sh.spm.update(x_words(x, x_at)); exp = dict(sh.spm)
        r = await sh.run(w_beats(W), M, K, x_at, y_at, vfn=vfn)
        check_run(r, sh, W, x, x_at, y_at, exp)
        dut._log.info(f"{name}: M={M} K={K} ok, {r['done'] - r['t0']} тактов")
        await sh.quiet(3)
    assert kif_busy(dut) in (0, None)


@cocotb.test()
async def test_gemv_tlast_errors(dut):
    """k_err: tlast раньше beat'а M·KW (в середине строки и на границе строки), нет tlast на
    последнем beat'е. Импульс в такте после beat'а-нарушителя, без k_done; дальше поток не
    берётся и порт B молчит; после сброса ядра следующий RUN верен."""
    rng = np.random.default_rng(31)
    M, K = 6, 128; KW = K // 32; N = M * KW
    W = rng.integers(-128, 128, (M, K), dtype=np.int8); x = rng.integers(-128, 128, K, dtype=np.int8)
    sh = await fresh(dut)
    paced, dense = (lambda c: c % 4 != 1), (lambda c: True)
    # dense/early_row_end: нарушитель приходит ровно когда конец строки 0 на ступени аккумулятора
    # (t+4) — запись её y гасится вместе с конвейером (порт B молчит с такта k_err)
    cases = (("early_mid_row", N - 3, 0, paced), ("early_row_end", 2 * KW - 1, 0, paced),
             ("dense/early_row_end", 2 * KW - 1, 0, dense),
             ("early_first_beat", 0, 0, paced), ("missing", None, 3, paced))
    for name, tl, extra, vfn in cases:
        sh.spm = x_words(x, 0)
        beats = w_beats(W) + [0] * extra
        r = await sh.run(beats, M, K, 0, 200, vfn=vfn, tlast_idx=tl)
        bad_beat = tl if tl is not None else N - 1
        assert r["done"] is None and r["err"] is not None, f"{name}: нет k_err"
        assert r["accepted"] == bad_beat + 1, f"{name}: принято {r['accepted']}"
        assert r["err"] == r["hs"][-1] + 1, f"{name}: k_err не в такте после нарушителя"
        assert all(c < r["err"] for c, _ in r["writes"]), f"{name}: запись y после k_err"
        await sh.quiet(20, beat=beats[min(r['accepted'], len(beats) - 1)])
        dut._log.info(f"{name}: k_err после beat'а {bad_beat}, без k_done")
        if kif_busy(dut) is not None:
            assert kif_busy(dut) == 1, "после k_err RUN висит до сброса ядра (§5.5)"
        await sh.reset()
    # после сброса — верный RUN
    sh.spm = x_words(x, 0); exp = dict(sh.spm)
    r = await sh.run(w_beats(W), M, K, 0, 200)
    check_run(r, sh, W, x, 0, 200, exp)


@cocotb.test()
async def test_gemv_bad_args(dut):
    """M = 0, K = 0, K не кратно 32 → k_err в такте после k_start, поток не принимается."""
    sh = await fresh(dut)
    for M, K in ((0, 64), (4, 0), (4, 48), (4, 31)):
        r = await sh.run([1, 2, 3, 4], M, K, 0, 10)
        assert r["err"] == r["t0"] + 1 and r["accepted"] == 0 and not r["reads"] and not r["writes"], \
            f"M={M} K={K}: {r['err']=} {r['accepted']=}"
        await sh.quiet(5); await sh.reset()


@cocotb.test()
async def test_gemv_back_to_back(dut):
    """RUN подряд без сброса: k_start в такте сразу после k_done (раньше обёртка не может, §5.4).
    Каждый RUN со своими x/y; результаты прошлых RUN в spm не портятся."""
    rng = np.random.default_rng(41)
    sh = await fresh(dut); exp = {}
    shapes = ((8, 32), (3, 64), (17, 160), (1, 32), (12, 288))
    prng = random.Random(7)
    for n, (M, K) in enumerate(shapes):
        W = rng.integers(-128, 128, (M, K), dtype=np.int8); x = rng.integers(-128, 128, K, dtype=np.int8)
        x_at, y_at = 100 * n, 900 + 3 * n
        xs = x_words(x, x_at); sh.spm.update(xs); exp.update(xs)
        vfn = (lambda c: True) if n % 2 == 0 else (lambda c: prng.random() < 0.7)
        r = await sh.run(w_beats(W), M, K, x_at, y_at, vfn=vfn)
        check_run(r, sh, W, x, x_at, y_at, exp)
        if n:
            assert r["t0"] == prev_done + 1, "k_start в такте сразу после k_done"
        prev_done = r["done"]
    await sh.quiet(5)
    b = kif_busy(dut)
    dut._log.info(f"kshell_kif_sva.busy_q после серии = {b}")
    assert b in (0, None)


@cocotb.test()
async def test_gemv_int32_wrap(dut):
    """Накопление по модулю 2^32: W = x = −128, K = 196608 → Σ = 3·2^30 → INT32 = −2^30.
    x занимает K/32 = 6144 слов > SPM_WORDS: адреса x заворачиваются (все слова одинаковы)."""
    M, K = 1, 196608
    W = np.full((M, K), -128, np.int8); x = np.full(K, -128, np.int8)
    sh = await fresh(dut, {a: pack_i8(x[:32]) for a in range(SPM_WORDS)})
    exp = dict(sh.spm)
    r = await sh.run(w_beats(W), M, K, 0, 512)
    check_run(r, sh, W, x, 0, 512, exp)
    got = np.int32(np.uint32(sh.spm[512] & 0xFFFFFFFF))
    assert got == np.int32(-(1 << 30)), got


@cocotb.test()
async def test_gemv_reset_mid_run(dut):
    """Сброс ядра (k_rst_n = 0, как SOFT_RESET обёртки) посреди RUN, когда в конвейере едут
    концы строк: в такте сброса ни tready, ни порта B; после — тишина, следующий RUN верен."""
    rng = np.random.default_rng(53)
    M, K = 8, 32
    W = rng.integers(-128, 128, (M, K), dtype=np.int8); x = rng.integers(-128, 128, K, dtype=np.int8)
    sh = await fresh(dut, x_words(x, 0))
    beats = w_beats(W)
    for at in (1, 4, 5, 6):  # сброс через `at` тактов приёма: запись y впереди / в этом такте
        await sh.tick(start=1, arg=(K << 96) | (M << 64) | (64 << 32))
        for c in range(at):
            await sh.tick(tvalid=1, tdata=beats[c % M])
        s = await sh.tick(tvalid=1, tdata=beats[0], rst_n=0)
        assert not (s["tready"] or s["en"]), f"at={at}: ядро активно в такте сброса: {s}"
        await sh.quiet(10)
        if kif_busy(dut) is not None:
            assert kif_busy(dut) == 0
    sh.spm = x_words(x, 0); exp = dict(sh.spm)
    r = await sh.run(beats, M, K, 0, 64)
    check_run(r, sh, W, x, 0, 64, exp)
