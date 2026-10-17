# Тесты примитивов kshell: sem_file, fifo_sync, spm.
# Один модуль на прогон: тест выбирается по имени верхнего уровня.
import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly


def top_name():
    # cocotb 2.x выставляет COCOTB_TOPLEVEL; TOPLEVEL — имя из Makefile (запасной вариант)
    return os.environ.get("COCOTB_TOPLEVEL") or os.environ.get("TOPLEVEL") or ""


async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.rst_n.value = 0
    for s in ("clr",):
        if hasattr(dut, s):
            getattr(dut, s).value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1


@cocotb.test(skip=top_name() != "sem_file")
async def test_sem_sum_and_host_priority(dut):
    await reset(dut)
    for n in ("inc_rd_v", "inc_wr_v", "inc_k_v", "inc_sig_v", "host_we"):
        getattr(dut, n).value = 0
    # три источника на семафор 2 и SIGNAL +5 на него же в одном такте → +8
    dut.inc_rd_v.value = 1
    dut.inc_rd_idx.value = 2
    dut.inc_wr_v.value = 1
    dut.inc_wr_idx.value = 2
    dut.inc_k_v.value = 1
    dut.inc_k_idx.value = 2
    dut.inc_sig_v.value = 1
    dut.inc_sig_idx.value = 2
    dut.inc_sig_val.value = 5
    await RisingEdge(dut.clk)
    for n in ("inc_rd_v", "inc_wr_v", "inc_k_v", "inc_sig_v"):
        getattr(dut, n).value = 0
    await ReadOnly()
    assert int(dut.sem_val.value) >> (32 * 2) & 0xFFFFFFFF == 8
    await RisingEdge(dut.clk)
    # запись хоста побеждает приращение в том же такте
    dut.host_we.value = 1
    dut.host_idx.value = 2
    dut.host_val.value = 100
    dut.inc_rd_v.value = 1
    dut.inc_rd_idx.value = 2
    await RisingEdge(dut.clk)
    dut.host_we.value = 0
    dut.inc_rd_v.value = 0
    await ReadOnly()
    assert int(dut.sem_val.value) >> (32 * 2) & 0xFFFFFFFF == 100


@cocotb.test(skip=top_name() != "fifo_sync")
async def test_fifo_random(dut):
    await reset(dut)
    rng = random.Random(1)
    src = [rng.randrange(256) for _ in range(500)]
    got = []
    i = 0
    for _ in range(3000):
        dut.in_v.value = 1 if (i < len(src) and rng.random() < 0.6) else 0
        dut.in_d.value = src[i] if i < len(src) else 0
        dut.out_rdy.value = 1 if rng.random() < 0.5 else 0
        await ReadOnly()
        push = int(dut.in_v.value) and int(dut.in_rdy.value)
        pop = int(dut.out_v.value) and int(dut.out_rdy.value)
        if pop:
            got.append(int(dut.out_d.value))
        await RisingEdge(dut.clk)
        if push:
            i += 1
    assert got == src[:len(got)] and len(got) > 400


@cocotb.test(skip=top_name() != "spm")
async def test_spm_port_a_priority(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.b_en.value = 0
    dut.a_rd_req.value = 0
    dut.a_wr_v.value = 1
    dut.a_wr_addr.value = 5
    dut.a_wr_data.value = 0xABC
    dut.a_rd_req.value = 1
    dut.a_rd_addr.value = 5
    await ReadOnly()
    assert int(dut.a_rd_gnt.value) == 0      # запись R побеждает
    await RisingEdge(dut.clk)
    dut.a_wr_v.value = 0
    await ReadOnly()
    assert int(dut.a_rd_gnt.value) == 1
    await RisingEdge(dut.clk)
    dut.a_rd_req.value = 0
    await ReadOnly()
    assert int(dut.a_rd_data.value) == 0xABC


@cocotb.test(skip=top_name() != "spm")
async def test_spm_tdp_template(dut):
    """BRAM-шаблон (NO_CHANGE): a_rd_data держится через запись порта A; маска байтов порта B."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    dut.b_en.value = 0
    dut.b_we.value = 0
    dut.a_rd_req.value = 0
    dut.a_wr_v.value = 0

    async def a_write(addr, data):
        dut.a_wr_v.value = 1
        dut.a_wr_addr.value = addr
        dut.a_wr_data.value = data
        await RisingEdge(dut.clk)
        dut.a_wr_v.value = 0

    await a_write(7, 0x1111)
    await a_write(8, 0x2222)
    # чтение A по адресу 7; в следующем такте порт A пишет адрес 8 (как rd_dma после gnt wr_dma)
    dut.a_rd_req.value = 1
    dut.a_rd_addr.value = 7
    await ReadOnly()
    assert int(dut.a_rd_gnt.value) == 1
    await RisingEdge(dut.clk)
    dut.a_rd_req.value = 0
    dut.a_wr_v.value = 1
    dut.a_wr_addr.value = 8
    dut.a_wr_data.value = 0x3333
    await ReadOnly()
    assert int(dut.a_rd_data.value) == 0x1111          # такт после gnt — данные готовы
    await RisingEdge(dut.clk)
    dut.a_wr_v.value = 0
    await ReadOnly()
    assert int(dut.a_rd_data.value) == 0x1111          # запись порта A не портит регистр чтения
    await RisingEdge(dut.clk)

    # порт B: запись только байтов 0 и 2 поверх 0x3333 в адресе 8, затем чтение
    dut.b_en.value = 1
    dut.b_we.value = 1
    dut.b_addr.value = 8
    dut.b_wdata.value = 0xAABBCCDD
    dut.b_wstrb.value = 0b101
    await RisingEdge(dut.clk)
    dut.b_we.value = 0
    await RisingEdge(dut.clk)
    dut.b_en.value = 0
    await ReadOnly()
    assert int(dut.b_rdata.value) == 0x00BB33DD
