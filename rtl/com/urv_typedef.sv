
// `include "urv_cfg.sv"
//-------------------------------------------------------------------------------
// Memory command enum
//-------------------------------------------------------------------------------
package urv_typedef;
import urv_cfg::*;

typedef enum logic {
    MEM_READ      = 1'b0,
    MEM_WRITE     = 1'b1
} mem_req_type_t;

typedef struct packed {
    mem_req_type_t               req_type;
    logic [MEM_ADDR_W-1:0]       req_addr;
    logic [MEM_MASK_W-1:0]       req_mask;
    logic [MEM_DATA_W-1:0]       req_data;
} mem_req_t;

typedef struct packed {
    mem_req_type_t               resp_type;
    logic [MEM_DATA_W-1:0]       resp_data;
} mem_resp_t;

// typedef struct packed {
//     logic                        soft_irq;
//     logic                        time_irq;
//     logic [CSR_TIME_W-1:0]       time_val;
// } clint_resp_t;

endpackage