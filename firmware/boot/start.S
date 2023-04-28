# define STORE    sw
# define LOAD     lw
# define REGBYTES 4

#define REGBLK_SUBSYS_UART_ADDR 0x10000000
##define REGBLK_SUBSYS_UART_ADDR 0xFF027000
#define UART_DATA_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0x0)
#define UART_STATE_ADDR (REGBLK_SUBSYS_UART_ADDR + 0x4)
#define UART_CTRL_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0x8)
#define UART_INT_ADDR   (REGBLK_SUBSYS_UART_ADDR + 0xC)
#define UART_BAUD_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0x10)
#define UART_PID4_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0xFD0) #0x04
#define PRINT_END_ADDR  (0x80000000 + 0x30)
#define PRINT_BAUT      (50000000l / 115200)

.section .text
.global  _start
#.set     test, 0x100
#test: .ascii "test"
.set     code_spi  , 0x30000000
.set     code_store, 0x30010000
.set     code_start, 0x80000000
.set     code_end,   0x80080000

_start:

# zero-initialize register file
    # li t6, 0x1800
    addi x1, zero, 0
    addi x2, zero, 0
    addi x3, zero, 0
    addi x4, zero, 0
    addi x5, zero, 0
    addi x6, zero, 0
    addi x7, zero, 0
    addi x8, zero, 0
    addi x9, zero, 0
    addi x10, zero, 0
    addi x11, zero, 0
    addi x12, zero, 0
    addi x13, zero, 0
    addi x14, zero, 0
    addi x15, zero, 0
    addi x16, zero, 0
    addi x17, zero, 0
    addi x18, zero, 0
    addi x19, zero, 0
    addi x20, zero, 0
    addi x21, zero, 0
    addi x22, zero, 0
    addi x23, zero, 0
    addi x24, zero, 0
    addi x25, zero, 0
    addi x26, zero, 0
    addi x27, zero, 0
    addi x28, zero, 0
    addi x29, zero, 0
    addi x30, zero, 0
    addi x31, zero, 0

    # uart tx
    li gp, UART_DATA_ADDR
    li a0, UART_CTRL_ADDR
    li a1, UART_PID4_ADDR
    li a2, UART_BAUD_ADDR
    li a3, UART_STATE_ADDR
    li a4, code_start
    # li a5, PRINT_END_ADDR
    li t0, 0x03
    # li t1, 0x104
    li t1, PRINT_BAUT
    # booting...\r\n
repeat:
    lui  a4, %hi(string1)
    addi a4, a4, %lo(string1)
    addi a5, a4, 13

    # init uart
    sw t0, 0(a0)
    sw t1, 0(a2)
print:
    lw t2, 0(a3)
    andi t2, t2, 0x01
    bnez t2, print
    lb t2, 0(a4)
    sw t2, 0(gp)
    addi a4, a4, 1
    blt a4, a5, print

# copy data section
	la a0, code_store
	la a1, code_start
	la a2, code_end
	bge a1, a2, end_init_data

loop_init_data:
	lw a3, 0(a0)
	sw a3, 0(a1)
	addi a0, a0, 4
	addi a1, a1, 4
	blt a1, a2, loop_init_data
end_init_data:

call code_start
# call code_spi

loop:
j loop

# j repeat

.section .rodata
.balign 4

string1:
.string "booting...\r\n"