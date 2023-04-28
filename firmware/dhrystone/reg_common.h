#ifndef __REG_COMMON_H__
#define __REG_COMMON_H__

#define WRITE_MEM(addr, val)            (*((volatile uint32_t *)(addr))) = (val)
#define READ_MEM(addr)                  (*((volatile uint32_t *)(addr)))

#define WRITE_REG(reg, val)             WRITE_MEM(reg, val)
#define READ_REG(reg)                   READ_MEM(reg)

#define write32(addr, val)              WRITE_MEM(addr, val)
#define read32(addr)                    READ_MEM(addr)

#endif  /* __REG_COMMON_H__ */