# Помощники хоста kshell поверх AxilMaster (docs/design.md §12, §13, §14):
# сброс и тактирование, подача программы через DOORBELL с проверкой cq_free,
# ожидание семафора, разбор STATUS, снимок perf, SOFT_RESET в тишине AXI.
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb.utils import get_sim_time

from axil import AxilMaster  # noqa: F401  (тип для подсказок; тесты создают его сами)

REG = dict(CTRL=0x000, STATUS=0x004, ERR_CODE=0x008, ERR_CMD=0x00C, CMD0=0x010,
           DOORBELL=0x020, CMD_COUNT=0x024, SEM=0x040, PERF=0x080)
PERF_NAMES = ["CYCLES", "SEQ_WAIT", "RD_BURSTS", "RD_LAT_SUM", "RD_WIN_FULL",
              "STRM_BP", "WR_BURSTS", "KRN_BUSY"]
CTRL_ENABLE, CTRL_SOFT_RESET, CTRL_PERF_SNAP = 1, 2, 4
CLK_NS = 10
ERR_NAMES = {0: "NONE", 1: "RRESP_ERR", 2: "BRESP_ERR", 3: "ALIGN", 4: "RANGE",
             5: "BAD_OPCODE", 6: "KERNEL_ERR", 7: "TIMEOUT"}

# задача Clock текущего теста: cocotb убивает задачи теста по его окончании, поэтому
# флаг — не «запущен когда-то», а «жива ли задача» (повторный reset в одном тесте)
_clk = {}


def _cycles():
    return int(get_sim_time(unit="ns")) // CLK_NS


async def reset(dut, cycles=5):
    """Синхронный сброс rst_n; Clock запускается один раз на тест."""
    t = _clk.get(id(dut))
    if t is None or t.done():
        _clk[id(dut)] = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    if hasattr(dut, "k_stall"):            # kshell_sink_top: tb-вход backpressure ядра
        dut.k_stall.value = 0
    dut.rst_n.value = 0
    for _ in range(cycles):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def _decode_status(v):
    return dict(IDLE=v & 1, AXI_QUIET=(v >> 1) & 1, ERROR=(v >> 2) & 1, cq_ovf=(v >> 3) & 1,
                krn_busy=(v >> 4) & 1, cq_free=(v >> 8) & 0xFF, rd_inflight=(v >> 16) & 0xFF,
                wr_inflight=(v >> 24) & 0xFF, raw=v)


async def status(axil) -> dict:
    return _decode_status(await axil.read(REG["STATUS"]))


async def submit(axil, prog, timeout=100000) -> None:
    """CMD0..CMD3 + DOORBELL на каждую команду. Хост обязан проверять cq_free (§6): место
    берётся из STATUS и расходуется локально, STATUS перечитывается, когда оно кончилось."""
    free = 0
    t0 = _cycles()
    for c in prog.encoded():
        while free == 0:
            st = await status(axil)
            if st["ERROR"]:
                raise AssertionError(f"submit: ERROR {_err(st)} — программа не подана целиком")
            free = st["cq_free"]
            if free == 0 and _cycles() - t0 > timeout:
                raise TimeoutError(f"submit: командная FIFO не освобождается {timeout} тактов")
        for k in range(4):
            await axil.write(REG["CMD0"] + 4 * k, c[k])
        await axil.write(REG["DOORBELL"], 1)
        free -= 1


def _err(st):
    return f"STATUS={st['raw']:#010x}"


async def wait_sem(axil, idx, thr, timeout=100000) -> None:
    """Опрос SEM[idx] до $signed(sem − thr) ≥ 0 (§11); ERROR — сразу провал с кодом."""
    t0 = _cycles()
    while True:
        v = await axil.read(REG["SEM"] + 4 * idx)
        if ((v - thr) & 0xFFFFFFFF) < (1 << 31):
            return
        st = await status(axil)
        if st["ERROR"]:
            code = await axil.read(REG["ERR_CODE"])
            cmd = await axil.read(REG["ERR_CMD"])
            raise AssertionError(f"wait_sem({idx}, {thr}): ERROR {ERR_NAMES.get(code, code)} "
                                 f"на команде #{cmd}, {_err(st)}")
        if _cycles() - t0 > timeout:
            raise TimeoutError(f"wait_sem({idx}, {thr}): SEM={v} за {timeout} тактов, "
                               f"{_err(st)}")
        for _ in range(8):
            await RisingEdge(axil.clk)


async def perf(axil) -> dict:
    """PERF_SNAP (ENABLE сохраняется) и чтение 8 снимков в порядке §14."""
    en = (await axil.read(REG["CTRL"])) & CTRL_ENABLE
    await axil.write(REG["CTRL"], en | CTRL_PERF_SNAP)
    return {n: await axil.read(REG["PERF"] + 4 * i) for i, n in enumerate(PERF_NAMES)}


async def soft_reset(axil, timeout=100000) -> None:
    """Пауза (ENABLE = 0: новые AR/AW не стартуют, тишина после наступления держится),
    ожидание AXI_QUIET, затем SOFT_RESET. ENABLE остаётся 0 — хост включает явно."""
    await axil.write(REG["CTRL"], 0)
    t0 = _cycles()
    while not (await status(axil))["AXI_QUIET"]:
        if _cycles() - t0 > timeout:
            raise TimeoutError(f"soft_reset: нет AXI_QUIET за {timeout} тактов")
        for _ in range(8):
            await RisingEdge(axil.clk)
    await axil.write(REG["CTRL"], CTRL_SOFT_RESET)
