

//-------------------------------------------------------------------------------
// XXX
//-------------------------------------------------------------------------------
package urv_cfg;
parameter MEM_ADDR_W  = 32;
parameter MEM_DATA_W  = 32;
parameter MEM_MASK_W  = MEM_DATA_W / 8;
parameter CSR_TIME_W  = 64;
parameter CSR_TIME_CMP_W = 64; 

parameter MEM_BASE_ADDR_CLINT    = 32'h50000;
parameter MEM_BASE_ADDR_PLIC     = 32'h51000;
parameter MEM_BASE_ADDR_DM       = 32'h52000;
parameter MEM_REG_ADDR_W_IN_BYTE = 32'h1000;

endpackage
 