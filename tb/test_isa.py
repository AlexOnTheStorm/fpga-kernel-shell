# Тесты кодировщика команд и статического проверщика программ (pytest, без симулятора).
import pytest
from isa import Program, check_program, ProgramError, encode


def test_encode_load():
    c0, c1, c2, c3 = encode("LOAD", sem=3, spm=0x40, hbm=0x12_3456_7800, length=512)
    assert c0 == (1 << 28) | (3 << 24) | 0x40
    assert c1 == 0x34567800 and c2 == 0x12 and c3 == 512


def demo_program(M=64, K=256):
    p = Program()
    p.load(0, 0, 0x1000, K); p.stream(0x10000, M * K); p.wait(0, 1)
    p.run(1, 0, 64, M, K)
    p.load(0, 16, 0x2000, K); p.stream(0x10000, M * K); p.wait(0, 2)
    p.run(1, 16, 72, M, K)
    p.wait(1, 1); p.store(2, 64, 0x80000, 4 * M)
    p.wait(1, 2); p.store(2, 72, 0x90000, 4 * M)
    p.wait(2, 2); p.signal(7, 1)
    return p


def test_demo_program_ok():
    check_program(demo_program())


def test_hol_deadlock_rejected():
    p = Program()
    p.stream(0x10000, 64 * 256 * 64)            # больше NTAG burst'ов
    p.load(0, 0, 0x1000, 256)
    p.wait(0, 1)                                # ждём LOAD за STREAM, RUN ещё не выдан
    p.run(1, 0, 64, 64, 256)
    with pytest.raises(ProgramError, match="head-of-line"):
        check_program(p)


def test_missing_wait_rejected():
    p = Program()
    p.load(0, 0, 0x1000, 256)
    p.run(1, 0, 64, 1, 256)                     # RUN читает spm[0..] без WAIT на LOAD
    p.stream(0x10000, 256)
    with pytest.raises(ProgramError):
        check_program(p)


# ---------------------------------------------------------------- дополнительные


def test_encode_wait_signal_run():
    assert encode("WAIT", sem=1, w1=5) == ((4 << 28) | (1 << 24), 5, 0, 0)
    assert encode("SIGNAL", sem=7, w1=1) == ((5 << 28) | (7 << 24), 1, 0, 0)
    p = Program()
    p.run(2, 0x11, 0x22, 0x33, 0x44)
    assert p.encoded() == [((6 << 28) | (2 << 24) | 0x44, 0x11, 0x22, 0x33)]
    with pytest.raises(ValueError):
        p.run(0, 0, 0, 0, 0x10000)              # arg3 не влезает в CMD0[15:0]


def test_raw_and_increments():
    p = Program()
    p.raw(0, 1, 2, 3)
    p.load(3, 0, 0, 256); p.store(3, 0, 0x1000, 256); p.signal(3, 5); p.run(4, 0, 0, 0, 0)
    assert p.encoded()[0] == (0, 1, 2, 3)
    assert p.increments(3) == 7 and p.increments(4) == 1 and p.increments(0) == 0
    with pytest.raises(ProgramError, match="BAD_OPCODE"):
        check_program(p)


def test_decode_errors_rejected():
    for build in (lambda p: p.load(0, 0, 0x1001, 256),        # ALIGN (адрес)
                  lambda p: p.load(0, 0, 0x1000, 100),        # ALIGN (длина)
                  lambda p: p.load(0, 1020, 0x1000, 512)):    # RANGE
        p = Program(); build(p)
        with pytest.raises(ProgramError):
            check_program(p, run_regions=None)


def test_rule1_pairing():
    p = Program(); p.run(0, 0, 0, 0, 0)                      # RUN без STREAM
    with pytest.raises(ProgramError, match="rule 1"):
        check_program(p, run_regions=None)
    p = Program(); p.stream(0x1000, 256)                     # STREAM без RUN
    with pytest.raises(ProgramError, match="rule 1"):
        check_program(p, run_regions=None)
    p = Program(); p.run(0, 0, 0, 0, 0); p.stream(0x1000, 256)   # STREAM после своего RUN
    with pytest.raises(ProgramError, match="rule 1"):
        check_program(p, run_regions=None)


def test_memcpy_and_sink_ok():
    p = Program(); p.load(0, 0, 0x100000, 6144); p.wait(0, 1)
    p.store(1, 0, 0x200000, 6144); p.wait(1, 1); p.signal(7, 1)
    check_program(p, run_regions=None)
    p = Program(); p.stream(0x300000, 5120); p.run(0, 0, 160, 0, 0)
    p.wait(0, 1); p.store(1, 0, 0x400000, 5120); p.wait(1, 1); p.signal(7, 1)
    # sink-ядро пишет spm[arg0, arg0+arg1): своя функция областей
    check_program(p, run_regions=lambda a: ([], [(a[0], a[0] + a[1])]))


def test_sink_missing_wait_rejected():
    p = Program(); p.stream(0x300000, 5120); p.run(0, 0, 160, 0, 0)
    p.store(1, 0, 0x400000, 5120); p.wait(1, 1)              # STORE читает spm, пока RUN пишет
    with pytest.raises(ProgramError, match="rule 3"):
        check_program(p, run_regions=lambda a: ([], [(a[0], a[0] + a[1])]))
    check_program(p, run_regions=None)                       # без областей RUN — конфликта нет


def test_wait_on_shared_sem_needs_all_increments():
    # два LOAD на sem0 в разные области spm, затем STORE из области второго:
    # WAIT sem0>=1 не гарантирует второй LOAD (коммит LOAD по порядку → гарантирует первый)
    p = Program(); p.load(0, 0, 0x1000, 256); p.load(0, 8, 0x2000, 256); p.wait(0, 1)
    p.store(1, 8, 0x3000, 256)
    with pytest.raises(ProgramError, match="rule 3"):
        check_program(p, run_regions=None)
    p = Program(); p.load(0, 0, 0x1000, 256); p.load(0, 8, 0x2000, 256); p.wait(0, 1)
    p.store(1, 0, 0x3000, 256)                               # первый LOAD гарантирован
    check_program(p, run_regions=None)
    # LOAD и STORE на один семафор — разные движки, порядок завершения любой
    p = Program(); p.store(0, 100, 0x5000, 256); p.load(0, 0, 0x1000, 256); p.wait(0, 1)
    p.store(1, 0, 0x3000, 256)
    with pytest.raises(ProgramError, match="rule 3"):
        check_program(p, run_regions=None)


def test_memory_conflicts():
    # STORE в память, затем LOAD оттуда без WAIT — RAW по памяти
    p = Program(); p.store(0, 0, 0x8000, 256); p.load(1, 8, 0x8000, 256)
    with pytest.raises(ProgramError, match="rule 3"):
        check_program(p, run_regions=None)
    p = Program(); p.store(0, 0, 0x8000, 256); p.wait(0, 1); p.load(1, 8, 0x8000, 256)
    check_program(p, run_regions=None)
    # STORE поверх области, которую ещё читает STREAM (WAR): нужен WAIT на RUN
    p = Program(); p.stream(0x8000, 512); p.run(0, 0, 0, 0, 0); p.store(1, 0, 0x8100, 256)
    with pytest.raises(ProgramError, match="rule 3"):
        check_program(p, run_regions=None)
    p = Program(); p.stream(0x8000, 512); p.run(0, 0, 0, 0, 0); p.wait(0, 1)
    p.store(1, 0, 0x8100, 256)
    check_program(p, run_regions=None)


def test_wait_never_satisfied_rejected():
    p = Program(); p.load(0, 0, 0x1000, 256); p.wait(0, 2)
    with pytest.raises(ProgramError, match="deadlock"):
        check_program(p, run_regions=None)
    # STREAM застряла, но WAIT ждёт семафор, который никто не увеличивает, — не head-of-line
    p = Program(); p.stream(0x1000, 256); p.wait(5, 1); p.run(0, 0, 0, 0, 0)
    with pytest.raises(ProgramError, match="никогда"):
        check_program(p, run_regions=None)


def test_hol_by_full_rd_queue():
    # STREAM длиннее окна держит резчик; LOAD'ы за ним забивают очередь rd —
    # секвенсор встаёт на 6-м rd-командe и не доходит до RUN
    p = Program(); p.stream(0x10000, 256 * 64)
    for i in range(5):
        p.load(0, 8 * i, 0x1000, 256)
    p.run(1, 0, 0, 0, 0)
    with pytest.raises(ProgramError, match="head-of-line"):
        check_program(p, run_regions=None)
    check_program(p, run_regions=None, eq_depth=None)        # без модели очередей — проходит
    p = Program(); p.stream(0x10000, 256 * 64)               # 4 LOAD влезают в очередь
    for i in range(4):
        p.load(0, 8 * i, 0x1000, 256)
    p.run(1, 0, 0, 0, 0); p.wait(0, 4)
    check_program(p, run_regions=None)


def test_overlapped_and_sequential_gemv_ok():
    for M, K in ((64, 256), (64, 512)):
        p = Program()
        p.load(0, 0, 0x2000000, K); p.wait(0, 1); p.stream(0x1000000, M * K)
        p.run(1, 0, 64, M, K); p.wait(1, 1)
        p.load(0, 16, 0x2100000, K); p.wait(0, 2); p.stream(0x1000000, M * K)
        p.run(1, 16, 72, M, K); p.wait(1, 2)
        p.store(2, 64, 0x3000000, 4 * M); p.wait(2, 1); p.store(2, 72, 0x3100000, 4 * M)
        p.wait(2, 2); p.signal(7, 1)
        check_program(p)
        check_program(demo_program(M, K))


# --------------------------------------------------------------------------- T9: генератор и эталон

import random                                                    # noqa: E402
from isa import random_program, execute_reference                # noqa: E402


def test_random_program_passes_checker():
    for seed in range(300):
        rng = random.Random(seed)
        p = random_program(rng, n_cmds=40, spm_words=1024, hbm_base=0x500000, hbm_size=1 << 16)
        check_program(p, run_regions=None)
        ops = {c["op"] for c in p.cmds}
        assert ops <= {"LOAD", "STORE", "SIGNAL", "WAIT"}, ops
        assert p.cmds[-1]["op"] == "SIGNAL" and p.cmds[-1]["sem"] == 7 and p.cmds[-1]["inc"] == 1
        assert all(c["sem"] < 7 for c in p.cmds[:-1])
        for c in p.cmds:
            if c["op"] in ("LOAD", "STORE"):
                assert c["len"] % 256 == 0 and c["hbm"] % 256 == 0
                assert 0x500000 <= c["hbm"] and c["hbm"] + c["len"] <= 0x500000 + (1 << 16)
                assert c["spm"] + c["len"] // 32 <= 1024


def test_random_program_wait_reachable_and_barriers():
    """WAIT sem ≥ N только при N ≤ уже выданных приращений; хвост — общий барьер."""
    for seed in range(100):
        p = random_program(random.Random(seed), n_cmds=40, hbm_base=0, hbm_size=1 << 16)
        for i, c in enumerate(p.cmds):
            if c["op"] == "WAIT":
                before = Program(); before.cmds = p.cmds[:i]
                assert c["thr"] <= before.increments(c["sem"]), (seed, i)
        tail = [c for c in p.cmds[:-1][::-1]]
        totals = {s: p.increments(s) for s in range(7) if p.increments(s)}
        barrier = {c["sem"]: c["thr"] for c in tail[:len(totals)]}
        assert all(c["op"] == "WAIT" for c in tail[:len(totals)]) and barrier == totals, seed


def test_random_program_variety():
    """Генератор реально даёт пересечения (вставленные барьеры), все 4 кода и STORE."""
    seen, barriers = set(), 0
    for seed in range(50):
        p = random_program(random.Random(seed), n_cmds=40, hbm_base=0, hbm_size=1 << 16)
        seen |= {c["op"] for c in p.cmds}
        barriers += sum(1 for c in p.cmds if c["op"] == "WAIT" and c["thr"] == p.increments(c["sem"]))
    assert seen == {"LOAD", "STORE", "SIGNAL", "WAIT"} and barriers > 50


def test_reference_hand_written():
    base = 0x1000
    init = bytes(range(256)) * 16                                # 4 KB
    p = Program()
    p.load(0, 10, base + 0x100, 512)                             # spm[10..26) ← mem[0x100..0x300)
    p.signal(2, 3)
    p.wait(0, 1)
    p.store(1, 18, base + 0x800, 256)                            # mem[0x800..0x900) ← spm[18..26)
    p.wait(1, 1)
    p.load(0, 0, base + 0x800, 256)                              # spm[0..8) ← mem[0x800..)
    p.wait(0, 2)
    p.store(1, 0, base + 0x000, 256)                             # mem[0..0x100) ← spm[0..8)
    p.wait(1, 2); p.signal(7, 1)
    mem, sem, spm = execute_reference(p, init, base, with_spm=True)
    exp = bytearray(init)
    exp[0x800:0x900] = init[0x200:0x300]
    exp[0x000:0x100] = init[0x200:0x300]
    assert mem == bytes(exp)
    assert sem == [2, 2, 3, 0, 0, 0, 0, 1]
    assert spm[10] == init[0x100:0x120] and spm[0] == init[0x200:0x220] and spm[8] is None
    assert execute_reference(p, init, base)[0] == mem             # без spm — 2 элемента


def test_reference_rejects_bad_programs():
    init = bytes(4096)
    p = Program(); p.store(0, 0, 0, 256)                         # spm не записан
    with pytest.raises(ValueError, match="незаписанное"):
        execute_reference(p, init, 0)
    p = Program(); p.wait(0, 1)                                  # порог недостижим
    with pytest.raises(ValueError, match="недостижим"):
        execute_reference(p, init, 0)
    p = Program(); p.load(0, 0, 0x2000, 256)                     # вне памяти
    with pytest.raises(ValueError, match="вне"):
        execute_reference(p, init, 0)


def test_reference_matches_random_programs():
    """Эталон исполняет каждую сгенерированную программу без ошибок; sem = выданные."""
    for seed in range(50):
        rng = random.Random(seed)
        init = bytes(rng.randrange(256) for _ in range(1 << 16))
        p = random_program(rng, n_cmds=40, hbm_base=0x500000, hbm_size=1 << 16)
        mem, sem = execute_reference(p, init, 0x500000)
        assert len(mem) == len(init)
        assert sem == [p.increments(s) for s in range(8)]
