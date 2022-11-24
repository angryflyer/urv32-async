##TARGET=riscv32-rivai-elf
# TARGET=/work/home/zhaowei/work/software/gcc/bin/riscv32-rivai-elf
# TARGET=/work/home/zhaowei/tian/pygmy/tool/riscv/bin/riscv32-unknown-elf
RISCV =/work/home/zhaowei/work/software/tool/riscv
TARGET=$(RISCV)/bin/riscv32-unknown-elf
PROJ_ROOT=/work/home/zhaowei/work/software/urv32_work/firmware/dhrystone

ifeq ($(APP_NAME),)
	APP_NAME = dhrystone
endif

GCC     = $(TARGET)-gcc
OBJCOPY = $(TARGET)-objcopy
OBJDUMP = $(TARGET)-objdump
STRIP   = $(TARGET)-strip
ELF2HEX = elf2hex
AR      = $(TARGET)-ar
RANLIB  = $(TARGET)-ranlib
SIZE    = $(TARGET)-size

OUTPUT_BIN_DIR = .

BASE_DIR    = .
SRC_DIR     = $(BASE_DIR)

PROG    = $(OUTPUT_BIN_DIR)/$(APP_NAME)

#-----------------------------------------------------------
WARNINGS= -Wall -Wextra -Wshadow -Wpointer-arith -Wbad-function-cast -Wcast-align -Wsign-compare \
          -Waggregate-return -Wstrict-prototypes -Wmissing-prototypes -Wmissing-declarations -Wunused

# INCLUDES = \
# 	-I/work/home/zhaowei/tian/pygmy/tool/riscv/riscv32-unknown-elf/include \
# 	-I/work/home/zhaowei/tian/pygmy/tool/riscv/lib/gcc/riscv32-unknown-elf/8.3.0/include \
# 	$(addprefix -I,$(SRC_DIR))

# INCLUDES = \
# 	-I$(PROJ_ROOT)/gcc/riscv32-rivai-elf/include \
# 	-I$(PROJ_ROOT)/gcc/lib/gcc/riscv32-rivai-elf/10.2.1/include \
# 	$(addprefix -I,$(SRC_DIR))

INCLUDES = \
	-I$(RISCV)/riscv32-unknown-elf/include \
	-I$(RISCV)/lib/gcc/riscv32-unknown-elf/10.2.0/include \
	$(addprefix -I,$(SRC_DIR))

ASM_SRC = $(shell find $(SRC_DIR) -maxdepth 1 -name "*.S")
C_SRC   = $(shell find $(SRC_DIR) -maxdepth 1 -name "*.c")

CFLAGS := -O3 -g
CFLAGS += \
	$(WARNINGS) $(INCLUDES) \
	-fno-strict-aliasing -fno-builtin \
	-mcmodel=medany -msmall-data-limit=8 -fsigned-char -ffunction-sections -fdata-sections \
	-std=c++11  -march=rv32im -mabi=ilp32 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" \

# MACROS += -U __riscv_atomic

# LOG_ERROR_LEVEL     (0)
# LOG_WARN_LEVEL      (1)
# LOG_INFO_LEVEL      (2)
# LOG_DBUG_LEVEL      (3)
# Each log level which less or equal LOG_LELVEL will be open, see driver/log.h
MACROS = -DLOG_LEVEL=2
MACROS += -DBOOT_FROM_ROM
# MACROS += -DREG
#-----------------------------------------------------------
# Define all object files.
#-----------------------------------------------------------
OBJS  = $(ASM_SRC:.S=.o)
OBJS += $(C_SRC:.c=.o)
LDFLAGS	 = -T../sections.ld -nostartfiles -static \
           -msmall-data-limit=8 -fsigned-char -Xlinker --gc-sections --specs=nano.specs --enable-multilib -v

# dont need -L path due to search them by default actually. Anyway, it is fine to mark here
# -lm should be the first one due to some APIs linking, e.g. floor()
# LIBS	 = -L/work/home/zhaowei/tian/pygmy/tool/riscv/lib/gcc/riscv32-unknown-elf/8.3.0 \
# 		   -L/work/home/zhaowei/tian/pygmy/tool/riscv/riscv32-unknown-elf/lib \
# 		   -lm -lc -lgcc

# LIBS	 = -L$(PROJ_ROOT)/gcc/lib/gcc/riscv32-rivai-elf/10.2.1 \
# 		   -L$(PROJ_ROOT)/gcc/riscv32-rivai-elf/lib \
# 		   -lm -lc -lgcc

LIBS	 = -L$(RISCV)/lib/gcc/riscv32-unknown-elf/10.2.0 \
		   -L$(RISCV)/riscv32-unknown-elf/lib \
		   -lm -lc -lgcc

%.o: %.c
	@echo "    CC $<"
	$(GCC) $(MACROS) -c $(CFLAGS) -o $@ $<

%.o: %.S
	@echo "    CC $<"
	$(GCC) $(MACROS) -c $(CFLAGS) -o $@ $<

all: clean $(PROG).elf dhrystone

$(PROG).elf: $(OBJS)
	@echo Linking....
	$(GCC) -o $@ $(LDFLAGS) $(OBJS) $(LIBS)
	@$(OBJDUMP) -xSD $@ > $(@:elf=dump)
	@echo Completed $@
	@echo ' '
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	$(OBJCOPY) -O ihex $@ $(@:elf=hex)
	@echo ' '
	@$(OBJCOPY) -O binary -j .spinortext $(PROG).elf $(PROG).bin
	@$(OBJCOPY) -O binary -j .text -j .rodata -j .eh_frame -j .sbss -j .sdata -j .data -j .bss -j .heap -j .stack $(PROG).elf $(PROG)-data.bin
	@cat $(PROG)-data.bin >> $(PROG).bin
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	$(SIZE) --format=berkeley $@
	@echo ' '

clean:
	rm -rf $(OBJS) $(OBJS:.o=.d) $(PROG).elf $(PROG).hex $(PROG).map $(PROG).dump $(PROG).bin $(PROG).coe *.bin

run:
	spike --dv --isa=rv32I  ++set_orv32_rst_pc=80000000 ++load_pk ++$(PROG).elf

debug:
	spike --isa=rv32I --debug-mode -d ++set_orv32_rst_pc=80000000 ++load_pk ++$(PROG).elf

dhrystone:
	../bin2coe.py 32 < dhrystone.bin > dhrystone.coe

.PHONY: all clean run debug
