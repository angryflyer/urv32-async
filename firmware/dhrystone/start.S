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
#define PRINT_BAUT      (50000000l / 115200)

.section .text
.global  _start
#.set     test, 0x100
#test: .ascii "test"
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

# # init uart
# li a0, UART_CTRL_ADDR
# li a1, UART_BAUD_ADDR

# li t0, 0x03
# li t1, PRINT_BAUT

# sw t0, 0(a0)
# sw t1, 0(a1)

# # div
# li a0, -15
# li a1, -2
# li a3, 0x5a
# li a4, 0x20000
# sw a3, 0(a4)
# lw a4, 0(a4)
# div a3, a0, a1
# rem a3, a0, a1

# li a0, 32
# li a1, 10
# li a3, 0x0
# li a4, 0x20000
# lw a4, 0(a4)
# div a3, a0, a1
# rem a3, a0, a1

# # mul
# li a0, 0xffffffff
# li a1, 0xffffffff
# li a3, 0x0
# # sw a0, 0(a3)
# # lw a1, 0(a3)
# mul a2, a0, a1
# mulh a2, a0, a1
# mulhsu a2, a0, a1
# mulhu a2, a0, a1
# # mul a2, a0, a1

# li a0, 0x0
# li a1, 0x1
# li a2, 0x2
# li a3, 0x3
# li a4, 0x4
# li a5, 0x5
# li a6, 0x20000
# sw a0, 0(a6)
# sw a1, 4(a6)
# sw a2, 8(a6)
# sw a3, 12(a6)
# sw a4, 16(a6)
# sw a5, 20(a6)

# lw a0, 0(a6)
# lw a1, 4(a6)
# lw a2, 8(a6)
# lw a3, 12(a6)
# lw a4, 16(a6)
# lw a5, 20(a6)

init_stack:
    /* set stack pointer */
    # li  a0, 0x4
    # lw  sp, 0(a0)
    # lw  sp, 4(a0)
    la	sp, _stack

#if defined (BOOT_FROM_ROM)
# copy data section
	la a0, _sidata
	la a1, _sdata
	la a2, _edata
	bge a1, a2, end_init_data

loop_init_data:
	lw a3, 0(a0)
	sw a3, 0(a1)
	addi a0, a0, 4
	addi a1, a1, 4
	blt a1, a2, loop_init_data
end_init_data:
#endif

# zero-init bss section
	la a0, _sbss
	la a1, _ebss
	bge a0, a1, end_init_bss
loop_init_bss:
	sw zero, 0(a0)
	addi a0, a0, 4
	blt a0, a1, loop_init_bss
	end_init_bss:

# zero-init stack section
	la a0, _stack_end
	la a1, __stack
	bge a0, a1, end_init_stack
loop_init_stack:
    /* init stack section */
 	sw zero, 0(a0)
	addi a0, a0, 4
	blt a0, a1, loop_init_stack
end_init_stack:

# enable timer
# li a0, 0x50010
li a0, 0x50000
li a1, 0x50014

# init time zero
li a3, 0x0
sw a3, 1*4(a0)
sw a3, 2*4(a0)
sw a3, 3*4(a0)
sw a3, 4*4(a0)

# init time cmp gap
# li a3, 0x100
# li a3, -1
li a3, 0x2FAF080
sw a3, 3*4(a0)
# sw a3, 4*4(a0)

# init mtvec
la t0, early_trap_entry 
# li t0, 0x0
csrw mtvec, t0
# enable mtie
# li t1, 0x80
# enable meie/mtie/msie
li t1, 0x888
li t2, 0x8
csrw mie, t1
csrw mstatus, t2

# enable timer
li a1, 0x50014
li a3, 0x1
sw a3, 0(a1)

# sys init
call sys_init

# call main
call main
loop:
j loop

/* For when a trap is fired */
early_trap_entry:
#if 0
#if 0
	csrr a0, mepc
	csrr a1, mcause
	csrr a2, mtval
	csrr a3, mstatus
	csrr a4, time
	csrr a5, timeh
	jal trap_handle
#endif
    j early_trap_entry

#else
	/* System call and other traps */
	addi sp, sp, -REGBYTES*16
	STORE x1,  0*REGBYTES(sp)
	STORE x5,  1*REGBYTES(sp)
	STORE x6,  2*REGBYTES(sp)
	STORE x7,  3*REGBYTES(sp)
	STORE x10, 4*REGBYTES(sp)
	STORE x11, 5*REGBYTES(sp)
	STORE x12, 6*REGBYTES(sp)
	STORE x13, 7*REGBYTES(sp)
	STORE x14, 8*REGBYTES(sp)
	STORE x15, 9*REGBYTES(sp)
	STORE x16, 10*REGBYTES(sp)
	STORE x17, 11*REGBYTES(sp)
	STORE x28, 12*REGBYTES(sp)
	STORE x29, 13*REGBYTES(sp)
	STORE x30, 14*REGBYTES(sp)
	STORE x31, 15*REGBYTES(sp)

	csrr a0, mepc
	csrr a1, mcause
	csrr a2, mtval
	csrr a3, mstatus
	csrr a4, time
	csrr a5, timeh
	# mv   a4, sp
	jal trap_handle

    # mtime count reg
    # li a0, 0x50000
    # # mtime cmp reg
    # li a2, 0x500
	# # li a2, 0x40000000
	# # li a2, -1;
    # lw a3, 1*4(a0)
    # add a3, a3, a2
    # sw a3, 3*4(a0)
    # # sw a2, 4*4(a0)

	LOAD x1,  0*REGBYTES(sp)
	LOAD x5,  1*REGBYTES(sp)
	LOAD x6,  2*REGBYTES(sp)
	LOAD x7,  3*REGBYTES(sp)
	LOAD x10, 4*REGBYTES(sp)
	LOAD x11, 5*REGBYTES(sp)
	LOAD x12, 6*REGBYTES(sp)
	LOAD x13, 7*REGBYTES(sp)
	LOAD x14, 8*REGBYTES(sp)
	LOAD x15, 9*REGBYTES(sp)
	LOAD x16, 10*REGBYTES(sp)
	LOAD x17, 11*REGBYTES(sp)
	LOAD x28, 12*REGBYTES(sp)
	LOAD x29, 13*REGBYTES(sp)
	LOAD x30, 14*REGBYTES(sp)
	LOAD x31, 15*REGBYTES(sp)

	addi sp, sp, REGBYTES*16
#endif
	mret

# again:
# rdcycleh x6
# rdcycle x5
# rdcycleh x7
# bne x6, x7, again
# li t6, 's'
# csrw mstatus, t6
# csrw mtvec, t0
# csrr a0, mepc
# csrr a1, mcause
# csrr a2, mtval
# csrr a3, mstatus
# csrw mepc, a0