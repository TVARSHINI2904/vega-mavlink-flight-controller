# vega-fc-v2 Makefile
# RISC-V Vega flight controller firmware (PowerShell)

TOOLCHAIN  := C:/Users/tvars/vega-tools-windows/bin/riscv64-vega-elf
CC         := $(TOOLCHAIN)-gcc
OBJCOPY    := $(TOOLCHAIN)-objcopy
SIZE       := $(TOOLCHAIN)-size

ARCH       := -march=rv32im -mabi=ilp32
CFLAGS     := $(ARCH) -nostdlib -nostartfiles -ffreestanding -Os -g
CFLAGS     += -I. -Ic_library_v2 -Ic_library_v2/common
LDFLAGS    := $(ARCH) -nostartfiles -nostdlib -T link.ld -Wl,-Map=vega-fc-v2.map

TARGET     := vega-fc-v2.elf
BIN        := vega-fc-v2.bin

SRC_C      := main.c uart.c scheduler.c mavlink_tx.c mavlink_rx.c mission.c
SRC_ASM    := start.s
OBJS       := $(SRC_C:.c=.o) $(SRC_ASM:.s=.o)

# Default target
all: $(TARGET)

# Link - NOTE: -lgcc must come AFTER object files
$(TARGET): $(OBJS) link.ld
	$(CC) $(LDFLAGS) -o $@ $(OBJS) -lgcc
	$(SIZE) $@

# Compile C files
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Assemble .s files
%.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

# Generate raw binary
bin: $(TARGET)
	$(OBJCOPY) -O binary $(TARGET) $(BIN)

# Clean (Windows cmd compatible)
clean:
	-del /f $(subst /,\,$(OBJS)) $(TARGET) $(BIN) vega-fc-v2.map 2>nul

# Show build errors only (no warnings)
errors:
	$(MAKE) 2>&1 | grep -E "error:|Error:"

.PHONY: all clean bin errors