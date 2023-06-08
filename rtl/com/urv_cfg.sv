

//-------------------------------------------------------------------------------
// XXX
//-------------------------------------------------------------------------------
package urv_cfg;
parameter MEM_ADDR_W  = 32;
parameter MEM_DATA_W  = 32;
parameter MEM_MASK_W  = MEM_DATA_W / 8;
parameter MEM_FETCH_W = 32;
parameter CSR_TIME_W  = 64;
parameter CSR_TIME_CMP_W = 64; 

parameter MEM_REG_ADDR_W_IN_BYTE = 32'h1000;
parameter MEM_BASE_ADDR_DEBUG    = 32'h0005_0000;
parameter MEM_BASE_ADDR_CLINT    = MEM_BASE_ADDR_DEBUG;
parameter MEM_BASE_ADDR_PLIC     = MEM_BASE_ADDR_DEBUG + MEM_REG_ADDR_W_IN_BYTE;       // 32'h0005_1000
parameter MEM_BASE_ADDR_DM       = MEM_BASE_ADDR_DEBUG + (2 * MEM_REG_ADDR_W_IN_BYTE); // 32'h0005_2000
// peripheral addr space
parameter MEM_BASE_ADDR_UART0    = 32'h1000_0000;
parameter MEM_BASE_ADDR_UART1    = 32'h1000_1000;
// external boot rom
parameter MEM_BASE_ADDR_BOOT     = 32'h2000_0000;
// icache can config to internel sram
// parameter MEM_BASE_ADDR_ICACHE   = 32'h2009_0000
parameter MEM_BASE_ADDR_QSPI     = 32'h3000_0000;
// xilinx zcu104 ddr4 0x0000_0000 ~ 0x7FFF_FFFF(better 0x4000_0000 ~ 0x0FF0_0000)
parameter MEM_BASE_ADDR_DDR      = 32'h4000_0000;
// itcm or sram
// itcm 512KB, 0x8000_0000 ~ 0x8000_7FFF
parameter MEM_BASE_ADDR_ITCM     = 32'h8000_0000;
parameter MEM_BASE_ADDR_DTCM     = 32'h8008_0000;

parameter MEM_BASE_ADDR_ROM      = 32'h0000_0000;
parameter MEM_BASE_ADDR_RAM      = 32'h0002_0000;
parameter DTCM_WIDTH_IN_BYTE     = 32'h0002_0000;
parameter ITCM_WIDTH_IN_BYTE     = 32'h0008_0000; 

parameter RST_PC                 = MEM_BASE_ADDR_DDR;

// plic source num
parameter PLIC_IRQ_N        = 32;

// cache parameter
parameter CACHE_WAY_NUM     = 4;
parameter CACHE_WAY_W       = $clog2(CACHE_WAY_NUM);

parameter CACHE_DATA_BYTE   = 16;
parameter CACHE_DATA_LSB    = 0;
parameter CACHE_DATA_MSB    = 8 * CACHE_DATA_BYTE - 1; // 16BYTES*8 = 128bits, 128 / 32 = 4
parameter CACHE_DATA_W      = CACHE_DATA_MSB - CACHE_DATA_LSB + 1;
parameter CACHE_DATA_MASK_W = CACHE_DATA_W / 8;
parameter CACHE_DATA_DP_W   = CACHE_DATA_W / MEM_FETCH_W;

parameter CACHE_OFFSET_W    = $clog2(CACHE_DATA_BYTE);
parameter CACHE_OFFSET_LSB  = 0;
parameter CACHE_OFFSET_MSB  = CACHE_OFFSET_LSB + CACHE_OFFSET_W - 1;

parameter CACHE_INDEX_N     = 128;
parameter CACHE_INDEX_W     = $clog2(CACHE_INDEX_N);
parameter CACHE_INDEX_LSB   = CACHE_OFFSET_MSB + 1;
parameter CACHE_INDEX_MSB   = CACHE_INDEX_LSB + CACHE_INDEX_W - 1;

parameter CACHE_TAG_MSB     = 31;
parameter CACHE_TAG_LSB     = CACHE_INDEX_MSB + 1;
parameter CACHE_TAG_VALID   = 1;
parameter CACHE_TAG_DIRTY   = 1;
parameter CACHE_TAG_W       = CACHE_TAG_MSB - CACHE_TAG_LSB + 1 + CACHE_TAG_VALID + CACHE_TAG_DIRTY;
parameter CACHE_TAG_MASK_W  = (CACHE_TAG_W > 24) ? 4 : 3;

// busrt width per way 
parameter CACHE_REFILL_W    = CACHE_INDEX_W + $clog2(CACHE_DATA_DP_W) + 1;
// busrt width totol(all ways) 
parameter CACHE_PREFILL_W   = CACHE_WAY_W + CACHE_REFILL_W;
parameter MEM_BURST_W       = CACHE_PREFILL_W;
endpackage
 