// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#include <stdarg.h>
#include <stdint.h>
#include "reg_common.h"

#define REGBLK_SUBSYS_UART_ADDR 0x10000000
//#define REGBLK_SUBSYS_UART_ADDR 0xFF027000
#define UART_DATA_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0x0)
#define UART_STATE_ADDR (REGBLK_SUBSYS_UART_ADDR + 0x4)
#define UART_CTRL_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0x8)
#define UART_INT_ADDR   (REGBLK_SUBSYS_UART_ADDR + 0xC)
#define UART_BAUD_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0x10)
#define UART_PID4_ADDR  (REGBLK_SUBSYS_UART_ADDR + 0xFD0) #0x04
#define UART_BAUT_COE   (50000000l / 115200)

#define REGBLK_SUBSYS_PLIC_ADDR 0x51000
#define PLIC_PRI_BASE_ADDR      (REGBLK_SUBSYS_PLIC_ADDR + 0x0)
#define PLIC_PRI_SRC1           (REGBLK_SUBSYS_PLIC_ADDR + 0x4)
#define PLIC_PRI_SRC2           (REGBLK_SUBSYS_PLIC_ADDR + 0x8)
#define PLIC_PRI_SRC31          (REGBLK_SUBSYS_PLIC_ADDR + 0x7c)

#define PLIC_IP                 (REGBLK_SUBSYS_PLIC_ADDR + 0x80)
#define PLIC_IE                 (REGBLK_SUBSYS_PLIC_ADDR + 0x84)
#define PLIC_PRI_THRES          (REGBLK_SUBSYS_PLIC_ADDR + 0x88)
#define PLIC_CLAIM              (REGBLK_SUBSYS_PLIC_ADDR + 0x8c)

#define USE_MYSTDLIB

extern long time();
extern long insn();

#ifdef USE_MYSTDLIB
extern char *malloc();
extern int printf(const char *format, ...);

extern void *memcpy(void *dest, const void *src, long n);
extern char *strcpy(char *dest, const char *src);
extern int strcmp(const char *s1, const char *s2);

char heap_memory[1024];
int heap_memory_used = 0;
#endif

long time()
{
	int cycles;
	asm volatile ("rdcycle %0" : "=r"(cycles));
	// printf("[time() -> %d]", cycles);
	return cycles;
}

long insn()
{
	int insns;
	asm volatile ("rdinstret %0" : "=r"(insns));
	// printf("[insn() -> %d]", insns);
	return insns;
}

void plic_init()
{
    // plic_init
	// set plic ip
	// set plic ie-> enable bit[1] for uart intr
    write32(PLIC_IE, 0x02);
	// set plic pri thres
    write32(PLIC_PRI_THRES, 0x0);
	// set plic pri src1 and src2
	write32(PLIC_PRI_SRC1, 0x02);
	write32(PLIC_PRI_SRC2, 0x02);
}

void uart_init()
{
    // uart init
	uint8_t uart_test_mode         = 0;
	uint8_t uart_rx_overrun_int_en = 0;
	uint8_t uart_tx_overrun_int_en = 0;
	uint8_t uart_rx_int_en         = 1;
	uint8_t uart_tx_int_en         = 0;
	uint8_t uart_rx_en             = 1;
	uint8_t uart_tx_en             = 1;
	uint8_t uart_ctrl              = 0;

	// init uart_ctrl
	uart_ctrl = (uart_test_mode         << 6)
	          | (uart_rx_overrun_int_en << 5)
			  | (uart_tx_overrun_int_en << 4)
	          | (uart_rx_int_en         << 3)
			  | (uart_tx_int_en         << 2)
			  | (uart_rx_en             << 1)
			  | (uart_tx_en             << 0);
    write32(UART_CTRL_ADDR, uart_ctrl);
	// set bautrate
    write32(UART_BAUD_ADDR, UART_BAUT_COE);
}

void sys_init()
{
	plic_init();
	uart_init();
}

void trap_handle(uint32_t mepc, uint32_t mcause, uint32_t mtval, uint32_t mtsatus, uint32_t timel, uint32_t timeh)
{
	unsigned long long time_val;
	unsigned long long timel_val, timeh_val;
	// printf("Enter trap_handle!\r\n");
	printf("Enter trap_handle! mepc=%x mcause=%x mtval=%x mtsatus=%x timel=%u timeh=%u\r\n", mepc, mcause, mtval, mtsatus, timel, timeh);
	timel_val = *(volatile uint32_t *)0x50004;
	timeh_val = *(volatile uint32_t *)0x50008;
	time_val  = ((timeh_val << 32) | timel_val) + 0x2FAF080;
	*(volatile uint32_t *)0x5000c = time_val;
	*(volatile uint32_t *)0x50010 = time_val >> 32;

	// clear uart int
	write32(UART_INT_ADDR, 0xf);
	// clear plic id
    write32(PLIC_CLAIM, read32(PLIC_CLAIM));
}

#ifdef USE_MYSTDLIB
char *malloc(int size)
{
	char *p = heap_memory + heap_memory_used;
	// printf("[malloc(%d) -> %d (%d..%d)]", size, (int)p, heap_memory_used, heap_memory_used + size);
	heap_memory_used += size;
	if (heap_memory_used > 1024)
		asm volatile ("ebreak");
	return p;
}

// static void printf_c(int c)
// {
// 	*((volatile int*)0x10010000) = c;
// }

static void printf_c(int c)
{
	// *((volatile int*)0x10000008) = 0x43;
    // // *((volatile int*)0x10000010) = 0x104;
	// *((volatile int*)0x10000010) = (50000000 / 115200);
    while((*((volatile int*)0x10000004) & 0x01) != 0x0);
    *((volatile int*)0x10000000) = c;
}

static void printf_s(char *p)
{
	while (*p)
		*((volatile int*)0x10010000) = *(p++);
}

static void printf_d(int val)
{
	char buffer[32];
	char *p = buffer;
	if (val < 0) {
		printf_c('-');
		val = -val;
	}
	while (val || p == buffer) {
		*(p++) = '0' + val % 10;
		val = val / 10;
	}
	while (p != buffer)
		printf_c(*(--p));
}

// int printf(const char *format, ...)
// {
// 	int i;
// 	va_list ap;

// 	va_start(ap, format);

// 	for (i = 0; format[i]; i++)
// 		if (format[i] == '%') {
// 			while (format[++i]) {
// 				if (format[i] == 'c') {
// 					printf_c(va_arg(ap,int));
// 					break;
// 				}
// 				if (format[i] == 's') {
// 					printf_s(va_arg(ap,char*));
// 					break;
// 				}
// 				if (format[i] == 'd') {
// 					printf_d(va_arg(ap,int));
// 					break;
// 				}
// 			}
// 		} else
// 			printf_c(format[i]);

// 	va_end(ap);
// }

int printf(const char *fmt, ...)
{
    char    buf[2048], *p;
    va_list args;
    int     n = 0;

    va_start(args, fmt);
    vsprintf(buf, fmt, args);
    va_end(args);
    p = buf;
    while (*p)
    {
        printf_c(*p);
        n++;
        p++;
    }

    return n;
}

void *memcpy(void *aa, const void *bb, long n)
{
	// printf("**MEMCPY**\n");
	char *a = aa;
	const char *b = bb;
	while (n--) *(a++) = *(b++);
	return aa;
}

char *strcpy(char* dst, const char* src)
{
	char *r = dst;

	while ((((uint32_t)dst | (uint32_t)src) & 3) != 0)
	{
		char c = *(src++);
		*(dst++) = c;
		if (!c) return r;
	}

	while (1)
	{
		uint32_t v = *(uint32_t*)src;

		if (__builtin_expect((((v) - 0x01010101UL) & ~(v) & 0x80808080UL), 0))
		{
			dst[0] = v & 0xff;
			if ((v & 0xff) == 0)
				return r;
			v = v >> 8;

			dst[1] = v & 0xff;
			if ((v & 0xff) == 0)
				return r;
			v = v >> 8;

			dst[2] = v & 0xff;
			if ((v & 0xff) == 0)
				return r;
			v = v >> 8;

			dst[3] = v & 0xff;
			return r;
		}

		*(uint32_t*)dst = v;
		src += 4;
		dst += 4;
	}
}

int strcmp(const char *s1, const char *s2)
{
	while ((((uint32_t)s1 | (uint32_t)s2) & 3) != 0)
	{
		char c1 = *(s1++);
		char c2 = *(s2++);

		if (c1 != c2)
			return c1 < c2 ? -1 : +1;
		else if (!c1)
			return 0;
	}

	while (1)
	{
		uint32_t v1 = *(uint32_t*)s1;
		uint32_t v2 = *(uint32_t*)s2;

		if (__builtin_expect(v1 != v2, 0))
		{
			char c1, c2;

			c1 = v1 & 0xff, c2 = v2 & 0xff;
			if (c1 != c2) return c1 < c2 ? -1 : +1;
			if (!c1) return 0;
			v1 = v1 >> 8, v2 = v2 >> 8;

			c1 = v1 & 0xff, c2 = v2 & 0xff;
			if (c1 != c2) return c1 < c2 ? -1 : +1;
			if (!c1) return 0;
			v1 = v1 >> 8, v2 = v2 >> 8;

			c1 = v1 & 0xff, c2 = v2 & 0xff;
			if (c1 != c2) return c1 < c2 ? -1 : +1;
			if (!c1) return 0;
			v1 = v1 >> 8, v2 = v2 >> 8;

			c1 = v1 & 0xff, c2 = v2 & 0xff;
			if (c1 != c2) return c1 < c2 ? -1 : +1;
			return 0;
		}

		if (__builtin_expect((((v1) - 0x01010101UL) & ~(v1) & 0x80808080UL), 0))
			return 0;

		s1 += 4;
		s2 += 4;
	}
}
#endif

