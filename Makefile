# =========================================================================
# Makefile for STM32 Automotive UDS & ISO-TP Diagnostic Node
# Target MCU: STM32F446RET6 (ARM Cortex-M4 @ 180 MHz)
# Host Target: Linux / macOS / Windows MinGW GCC (Desktop Simulation)
# =========================================================================

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -Iinclude
TEST_CFLAGS ?= $(CFLAGS) -Itests/unity

# ARM Toolchain for Target Hardware Compilation
ARM_CC ?= arm-none-eabi-gcc
ARM_CFLAGS ?= -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
              -DSTM32F446xx -DTARGET_STM32F4 -Iinclude -O2 -Wall -Wextra

SRC = src/can_driver.c src/isotp.c src/uds_server.c src/diagnostic_app.c
MAIN_SRC = $(SRC) src/main.c
TEST_SRC = $(SRC) tests/unity/unity.c tests/test_uds_isotp_suite.c

BIN_DEMO = build_demo
BIN_TEST = run_tests

.PHONY: all host test arm clean

all: host test

# Build Host Desktop Demo
host:
	@echo "==> Building Host Diagnostic Demonstration..."
	$(CC) $(CFLAGS) $(MAIN_SRC) -o $(BIN_DEMO)
	@echo "==> Build successful: $(BIN_DEMO)"

# Build and Execute Unit Test Suite
test:
	@echo "==> Building Unity Unit Test Suite..."
	$(CC) $(TEST_CFLAGS) $(TEST_SRC) -o $(BIN_TEST)
	@echo "==> Executing Unit Tests..."
	./$(BIN_TEST)

# Validate Compilation with ARM Toolchain (Object Files Syntax Check)
arm:
	@echo "==> Cross-compiling for STM32 Cortex-M4 (arm-none-eabi-gcc)..."
	$(ARM_CC) $(ARM_CFLAGS) -c $(MAIN_SRC)
	@echo "==> Cortex-M4 objects compiled successfully."

clean:
	@echo "==> Cleaning build artifacts..."
	rm -f $(BIN_DEMO) $(BIN_TEST) $(BIN_DEMO).exe $(BIN_TEST).exe *.o
