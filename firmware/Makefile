RISCV_GNU_TOOLCHAIN_INSTALL_PREFIX = /home/autumn/work/tool
TOOLCHAIN_PREFIX = $(RISCV_GNU_TOOLCHAIN_INSTALL_PREFIX)/riscv/bin/
# ---- iCE40 HX8K Breakout Board ----
DEPENDENCIES := -lpthread
UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
	DEPENDENCIES += -lobjc -framework IOKit -framework CoreFoundation
else
	DEPENDENCIES += -lrt
endif

CFLAGS = -Wall -Wextra

# add:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/add.elf tests/add.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/add.elf tests/add.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/add.elf > tests/add.dump
# 	./bin2coe.py 32 < tests/add.bin > tests/add.coe
# mul:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32im -mabi=ilp32 -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/mul.elf tests/mul.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/mul.elf tests/mul.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/mul.elf > tests/mul.dump
# 	./bin2coe.py 32 < tests/mul.bin > tests/mul.coe
# and:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/and.elf tests/and.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/and.elf tests/and.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/and.elf > tests/and.dump
# 	./bin2coe.py 32 < tests/and.bin > tests/and.coe

# slli:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/slli.elf tests/slli.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/slli.elf tests/slli.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/slli.elf > tests/slli.dump
# 	./bin2coe.py 32 < tests/slli.bin > tests/slli.coe

# sll:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/sll.elf tests/sll.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/sll.elf tests/sll.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/sll.elf > tests/sll.dump
# 	./bin2coe.py 32 < tests/sll.bin > tests/sll.coe

# srli:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/srli.elf tests/srli.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/srli.elf tests/srli.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/srli.elf > tests/srli.dump
# 	./bin2coe.py 32 < tests/srli.bin > tests/srli.coe

# xor:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/xor.elf tests/xor.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/xor.elf tests/xor.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/xor.elf > tests/xor.dump
# 	./bin2coe.py 32 < tests/xor.bin > tests/xor.coe

# xori:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/xori.elf tests/xori.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/xori.elf tests/xori.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/xori.elf > tests/xori.dump
# 	./bin2coe.py 32 < tests/xori.bin > tests/xori.coe

# sub:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/sub.elf tests/sub.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/sub.elf tests/sub.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/sub.elf > tests/sub.dump
# 	./bin2coe.py 32 < tests/sub.bin > tests/sub.coe

# sra:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/sra.elf tests/sra.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/sra.elf tests/sra.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/sra.elf > tests/sra.dump
# 	./bin2coe.py 32 < tests/sra.bin > tests/sra.coe

# srai:
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/srai.elf tests/srai.S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/srai.elf tests/srai.bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/srai.elf > tests/srai.dump
# 	./bin2coe.py 32 < tests/srai.bin > tests/srai.coe

# $(inst):
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostartfiles -nostdlib -o tests/$(inst).elf tests/$(inst).S -I tests/
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary tests/$(inst).elf tests/$(inst).bin
# 	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xD tests/$(inst).elf > tests/$(inst).dump
# 	./bin2coe.py 32 < tests/$(inst).bin > tests/$(inst).coe

.PHONY: firmware.hex

firmware.hex: firmware.coe
	./coe2hex

.PHONY: firmware.coe

firmware.coe: firmware.bin
	./bin2coe.py 32 < firmware.bin > firmware.coe

.PHONY: firmware.bin

firmware.bin: firmware.elf
	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objcopy -O binary firmware.elf firmware.bin

.PHONY: firmware.elf

firmware.elf: sections.ld start.s firmware.c
	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-gcc -march=rv32i -Wl,-Bstatic,-T,sections.ld,--strip-debug -ffreestanding -nostdlib -o firmware.elf start.s firmware.c

.PHONY: firmware.dump
firmware.dump: firmware.elf
	$(TOOLCHAIN_PREFIX)riscv32-unknown-elf-objdump -xSD firmware.elf > firmware.dump

.PHONY: coe2hex
coe2hex: coe2hex.c
	g++  coe2hex.c -std=c++11 -o coe2hex $(CFLAGS) $(DEPENDENCIES) -fpermissive -L/lib64
# ---- Clean ----

clean:
	rm -f firmware.elf firmware.hex firmware.bin firmware.coe

.PHONY: clean

