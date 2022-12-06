# define STORE    sw
# define LOAD     lw
# define REGBYTES 4

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

# div
li a0, -15
li a1, -2
li a3, 0x5a
li a4, 0x20000
sw a3, 0(a4)
lw a4, 0(a4)
div a3, a0, a1
rem a3, a0, a1

li a0, 32
li a1, 10
li a3, 0x0
li a4, 0x20000
lw a4, 0(a4)
div a3, a0, a1
rem a3, a0, a1

# mul
li a0, 0xffffffff
li a1, 0xffffffff
li a3, 0x0
# sw a0, 0(a3)
# lw a1, 0(a3)
mul a2, a0, a1
mulh a2, a0, a1
mulhsu a2, a0, a1
mulhu a2, a0, a1
# mul a2, a0, a1

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
li a0, 0x5000c
li a1, 0x50014

# init time cmp
# li a3, 0x100
li a3, -1
sw a3, 0(a0)

# init mtvec
la t0, early_trap_entry 
# li t0, 0x0
csrw mtvec, t0
# enable mie
li t1, 0x80
li t2, 0x8
csrw mie, t1
csrw mstatus, t2

# li a0, 0x51004
# li a1, 0x51008
# li a2, 0x51084

# li a3, 0x2
# sw a3, 0(a0)
# sw a3, 0(a1)
# lw t1, 0(a1)
# li a3, 0xf
# sw a3, 0(a2)

# enable timer
li a1, 0x50014
li a3, 0x1
sw a3, 0(a1)

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
	jal trap_print
#endif
    j early_trap_entry

#else
	/* System call and other traps */
	addi sp, sp, -REGBYTES*32
	STORE x1, 1*REGBYTES(sp)
	STORE x2, 2*REGBYTES(sp)
	STORE x3, 3*REGBYTES(sp)
	STORE x4, 4*REGBYTES(sp)
	STORE x5, 5*REGBYTES(sp)
	STORE x6, 6*REGBYTES(sp)
	STORE x7, 7*REGBYTES(sp)
	STORE x8, 8*REGBYTES(sp)
	STORE x9, 9*REGBYTES(sp)
	STORE x10, 10*REGBYTES(sp)
	STORE x11, 11*REGBYTES(sp)
	STORE x12, 12*REGBYTES(sp)
	STORE x13, 13*REGBYTES(sp)
	STORE x14, 14*REGBYTES(sp)
	STORE x15, 15*REGBYTES(sp)
	STORE x16, 16*REGBYTES(sp)
	STORE x17, 17*REGBYTES(sp)
	STORE x18, 18*REGBYTES(sp)
	STORE x19, 19*REGBYTES(sp)
	STORE x20, 20*REGBYTES(sp)
	STORE x21, 21*REGBYTES(sp)
	STORE x22, 22*REGBYTES(sp)
	STORE x23, 23*REGBYTES(sp)
	STORE x24, 24*REGBYTES(sp)
	STORE x25, 25*REGBYTES(sp)
	STORE x26, 26*REGBYTES(sp)
	STORE x27, 27*REGBYTES(sp)
	STORE x28, 28*REGBYTES(sp)
	STORE x29, 29*REGBYTES(sp)
	STORE x30, 30*REGBYTES(sp)
	STORE x31, 31*REGBYTES(sp)

	csrr a0, mcause
	csrr a1, mepc
	mv a2, sp
	jal trap_handle

    # mtime count reg
    li a0, 0x50004
    # mtime cmp reg
    li a1, 0x5000c
    li a2, 0x500
    lw a3, 0(a0)
    add a3, a3, a2
    sw a3, 0(a1)

    # li a0, 0x5108c
    # lw t1, 0(a0)
    # sw t1, 0(a0) 

	LOAD x1, 1*REGBYTES(sp)
	LOAD x2, 2*REGBYTES(sp)
	LOAD x3, 3*REGBYTES(sp)
	LOAD x4, 4*REGBYTES(sp)
	LOAD x5, 5*REGBYTES(sp)
	LOAD x6, 6*REGBYTES(sp)
	LOAD x7, 7*REGBYTES(sp)
	LOAD x8, 8*REGBYTES(sp)
	LOAD x9, 9*REGBYTES(sp)
	LOAD x10, 10*REGBYTES(sp)
	LOAD x11, 11*REGBYTES(sp)
	LOAD x12, 12*REGBYTES(sp)
	LOAD x13, 13*REGBYTES(sp)
	LOAD x14, 14*REGBYTES(sp)
	LOAD x15, 15*REGBYTES(sp)
	LOAD x16, 16*REGBYTES(sp)
	LOAD x17, 17*REGBYTES(sp)
	LOAD x18, 18*REGBYTES(sp)
	LOAD x19, 19*REGBYTES(sp)
	LOAD x20, 20*REGBYTES(sp)
	LOAD x21, 21*REGBYTES(sp)
	LOAD x22, 22*REGBYTES(sp)
	LOAD x23, 23*REGBYTES(sp)
	LOAD x24, 24*REGBYTES(sp)
	LOAD x25, 25*REGBYTES(sp)
	LOAD x26, 26*REGBYTES(sp)
	LOAD x27, 27*REGBYTES(sp)
	LOAD x28, 28*REGBYTES(sp)
	LOAD x29, 29*REGBYTES(sp)
	LOAD x30, 30*REGBYTES(sp)
	LOAD x31, 31*REGBYTES(sp)

	addi sp, sp, REGBYTES*32
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