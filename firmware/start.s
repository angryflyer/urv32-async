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

# #if defined BOOT_FROM_ROM
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
# #endif
    li a0, 0x0
	lw a3, 0(a0)
	lw a3, 0(a0)
	lw a3, 0(a0)
	lw a3, 0(a0)
	lw a3, 0(a0)
	lw a3, 0(a0)
# la   gp, _gp 
# init_bss:
#     /* init bss section */
#     la	a0, _sbss
#     la	a1, _ebss
#     li	a2, 0x0
#     jal	fill_block

# write_stack_pattern:
#     /* init bss section */
#     la	a0, _stack_end  /* note the stack grows from top to bottom */
#     la	a1, __stack
#     li	a2, 0x0
#     jal	fill_block

init_stack:
    /* set stack pointer */
    la	sp, _stack

# init_trap:
#     la t0, main
#     csrrw a0, mtvec, t0
# 	addi a0, a0, 2
# call main
call main
loop:
j loop

# /* Fills memory blocks */
# fill_block:
#     bgeu    a0, a1, fb_end
#     sw      a2, 0(a0)
#     addi    a0, a0, 4
#     j       fill_block
# fb_end:
#     ret
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
