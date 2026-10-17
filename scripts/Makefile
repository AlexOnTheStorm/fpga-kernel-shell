# =====================================================================
#  Симуляция kshell модулей в cocotb + Verilator.
#  Параметризовано (много модулей): указываешь TOP, MODULE и SRCS.
#    ./run.sh sem_file test_prims "../rtl/kshell_pkg.sv ../rtl/sem_file.sv"
#  Пакет kshell_pkg.sv НЕ добавляется автоматически: каждый SRCS начинается с него.
#    make clean
#  Запускать ИЗ scripts/. По умолчанию SRCS = rtl/$(TOP).sv (один файл).
#  Идиома — как в ../gemm_kernelgen/scripts/Makefile.
# =====================================================================
SIM           ?= verilator
TOPLEVEL_LANG ?= verilog

TOP    ?= fifo_sync
MODULE ?= test_prims
SRCS   ?= $(CURDIR)/../rtl/$(TOP).sv

# Соседние bind-файлы TOP-модуля подключаем автоматически, если существуют:
#   <TOP>_sva.sv — ассершены (нужен --assert), <TOP>_cov.sv — covergroup (нужен --coverage).
SVA := $(wildcard $(CURDIR)/../rtl/$(TOP)_sva.sv)
COV := $(wildcard $(CURDIR)/../rtl/$(TOP)_cov.sv)
# дополнительные bind-файлы для --assert (например ../rtl/kshell_kif_sva.sv)
SVA_EXTRA ?=
# дополнительные bind-файлы для --coverage (системные топы: ../rtl/rd_dma_cov.sv …)
COV_EXTRA ?=

VERILOG_SOURCES += $(SRCS)
TOPLEVEL = $(TOP)

# EXTRA_ARGS из окружения (например -GWDOG_CYCLES=2000) — до наших `+=`: make экспортирует
# переменную окружения дальше уже ДОПОЛНЕННОЙ, и `make cov` (здесь VCOV=0 → --assert)
# передал бы --assert в VCOV=1-сборку → Internal Error Verilator. cov передаёт исходное.
EXTRA_ARGS_USER := $(EXTRA_ARGS)

# где cocotb ищет test-модуль (tb/)
export PYTHONPATH := $(CURDIR)/../tb:$(PYTHONPATH)

EXTRA_ARGS += --trace --trace-structs   # VCD для разбора волн
EXTRA_ARGS += -Werror-UNOPTFLAT         # комбинаторная петля → фатально (ловим сразу)

# Ассершены и coverage — РАЗДЕЛЬНЫЕ прогоны: Verilator 5.x падает Internal Error
# (V3Localize «AstVarRef not under function») на --coverage + --assert одновременно.
#   make ...        → --assert + <TOP>_sva.sv   (корректность + ассершены)
#   make ... VCOV=1 → --coverage + <TOP>_cov.sv (line/toggle/user + covergroup)
# NB: имя НЕ COVERAGE — оно занято cocotb (=COCOTB_USER_COVERAGE, Python-покрытие).
VCOV ?= 0
ifeq ($(VCOV),1)
VERILOG_SOURCES += $(COV) $(COV_EXTRA)
# line-покрытие вставляет счётчики в always_comb FSM ядер (k_s_tready и state_d в одном
# блоке) → Verilator не делит блок и видит ЛОЖНУЮ петлю tready→hs→блок. Реальные петли
# ловит обычный (--assert) прогон с -Werror-UNOPTFLAT; здесь — только предупреждение.
EXTRA_ARGS += --coverage -Wno-UNOPTFLAT
else
VERILOG_SOURCES += $(SVA) $(SVA_EXTRA)
EXTRA_ARGS += --assert
endif

# cocotb-часть подключаем только если cocotb доступен (lint/fmt от него не зависят)
COCOTB_MK := $(shell cocotb-config --makefiles 2>/dev/null)
ifneq ($(COCOTB_MK),)
include $(COCOTB_MK)/Makefile.sim
endif

# ---- lint/format гейт (обязателен перед показом кода, см. docs/style_notes.md) ----
RTL_DIR    := $(CURDIR)/../rtl
RTL_ALL    := $(wildcard $(RTL_DIR)/*.sv)
# design = без bind-файлов проверок/покрытия (их синтез/чистый lint не читают)
# пакет первым (порядок компиляции)
PKG        := $(RTL_DIR)/kshell_pkg.sv
RTL_DESIGN := $(PKG) $(filter-out $(PKG) %_sva.sv %_cov.sv,$(RTL_ALL))
VERIBLE_RULES := $(CURDIR)/../.rules.verible
# UNUSEDPARAM: пакет — общий словарь, не каждый модуль берёт все параметры
VLINT_WAIVE := -Wall -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-MULTITOP

.PHONY: lint fmt cov
lint:
	@echo "== verible-verilog-lint =="
	@verible-verilog-lint --rules_config=$(VERIBLE_RULES) $(RTL_ALL)
	@echo "== verilator --lint-only (design) =="
	@verilator --lint-only $(VLINT_WAIVE) $(RTL_DESIGN)
	@echo "lint OK"

fmt:
	@verible-verilog-format --inplace $(RTL_ALL)
	@echo "formatted"

# code + functional coverage: прогон под --coverage → аннотация исходников.
# verilator_coverage печатает сводку (line/toggle/branch/expr/covergroup);
# строки с '%00' в annot/ — непокрытые точки. Ловушки тулчейна — docs/style_notes.md §8.
cov:
	@$(MAKE) --no-print-directory clean
	@EXTRA_ARGS='$(EXTRA_ARGS_USER)' $(MAKE) --no-print-directory VCOV=1 TOP=$(TOP) MODULE=$(MODULE)
	@verilator_coverage --annotate $(CURDIR)/annot --annotate-min 1 coverage.dat
	@echo "покрытие аннотировано → annot/ (ищи '%00' — непокрытое)"
