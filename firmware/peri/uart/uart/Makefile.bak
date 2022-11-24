
user_uart = clean user_uart.elf user_uart.dump user_uart.bin 

GCC?=riscv64-unknown-elf-gcc
OBJCOPY?=riscv64-unknown-elf-objcopy
OBJDUMP?=riscv64-unknown-elf-objdump

all: $(user_uart)

%.img: %.bin
	dd if=$< of=$@ bs=256 count=1

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.elf: start.s %.c
	$(GCC) start.s user_uart.c -I$(RISCV)/include -T link.ld -march=rv32i -mabi=ilp32 -nostdlib -static -Wl,--no-gc-sections -o $@

%.dump: %.elf
	$(OBJDUMP) -xSD $< --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data  > $@

clean:
	rm -f *.img *.dump *.bin *.elf
