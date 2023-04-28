
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
    logic [MEM_BURST_W-1:0]      req_burst;
} mem_req_t;

typedef struct packed {
    mem_req_type_t               resp_type;
    logic [MEM_DATA_W-1:0]       resp_data;
    logic                        resp_last;
} mem_resp_t;

// cache struct typedef
typedef struct packed {
    logic                                    val; // valid
    logic                                    dir; // dirty
    logic [CACHE_TAG_MSB-CACHE_TAG_LSB:0]    tag;
} cache_tag_t;

typedef struct packed {
    logic [CACHE_TAG_MSB-CACHE_TAG_LSB:0]    tag;
    logic [CACHE_INDEX_MSB-CACHE_INDEX_LSB:0]index;
    logic [CACHE_OFFSET_MSB:0]               offset;
} cache_line_t;

typedef struct packed {
    logic [CACHE_DATA_W-1:0]                 data;
} cache_data_t;

typedef struct packed {
    logic [CACHE_INDEX_W-1:0]                addr;
} cache_addr_t;

// typedef struct packed {
//     logic                        soft_irq;
//     logic                        time_irq;
//     logic [CSR_TIME_W-1:0]       time_val;
// } clint_resp_t;

endpackage