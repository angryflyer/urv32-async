//===============================================
//Name          : urv_sram_burst
//Author        : angrybird
//Email         : 
//Date          : 2023-03-04
//Description   : mem_if to sram/fast_sram_sp bridge
//===============================================
module urv_sram_burst 
    import urv_cfg::*;
    import urv_typedef::*; 
#(
    parameter integer WORDS = 2048//16K
) (
    input             clk,
    input             rstn,
    input             mem_req_valid,
    output            mem_req_ready,
    input  mem_req_t  mem_req, 

    output            mem_resp_valid,
    input             mem_resp_ready,
    output mem_resp_t mem_resp

);

localparam RAM_ADDR_W = $clog2(WORDS << 2);

// state enum
typedef enum logic [1:0] {
    IDLE      = 2'b00,
    FETCH     = 2'b01
    // FETCH_REQ = 2'b01,
    // FETCH_RESP= 2'b10
} state_t;

logic req_en;
logic is_idle;
logic is_fetch;
logic is_fetch_req;
// logic is_fetch_resp;
logic fetch_done;

logic csn;
logic wen;
logic [RAM_ADDR_W-1-2:0] addr;
logic [MEM_DATA_W-1:0]   din;
logic [MEM_DATA_W-1:0]   dout;
logic [MEM_MASK_W-1:0]   web;

// state change
logic [1:0]         curt_state;
logic [1:0]         next_state;

stdffrv #($bits(state_t)) rff_state_u (
    .clk(clk),
    .rstn(rstn),
    .rst_val(IDLE),
    .d(next_state),
    .q(curt_state) 
);

assign is_idle          = curt_state == IDLE;
assign is_fetch         = curt_state == FETCH;
// assign is_fetch_req     = curt_state == FETCH_REQ;
// assign is_fetch_resp    = curt_state == FETCH_RESP;

always_comb  begin
    next_state = curt_state;
    case(curt_state)
        IDLE  : begin
            next_state = mem_req_valid ? FETCH : IDLE;
        end
        FETCH : begin
            next_state = fetch_done ? (mem_req_valid ? FETCH : IDLE) : FETCH;   
        end
    endcase
end

mem_req_t mem_req_q;
mem_req_t mem_req_d;
logic  mem_req_flush;
logic  mem_req_en;
assign mem_req_flush        = (is_idle & mem_req_valid) | (is_fetch & fetch_done & mem_req_valid);
assign mem_req_en           = is_fetch;
assign mem_req_d.req_type   = mem_req_q.req_type;
assign mem_req_d.req_data   = mem_req_q.req_data;
assign mem_req_d.req_mask   = mem_req_q.req_mask;
assign mem_req_d.req_addr   = mem_req_q.req_addr  + 4;
assign mem_req_d.req_burst  = mem_req_q.req_burst - 1; 
assign fetch_done           = mem_req_q.req_burst == 'h1;
stdffref #($bits(mem_req_t)) rff_mem_req_u (
    .clk(clk),
    .rstn(rstn),
    .flush(mem_req_flush),
    .flush_val(mem_req),
    .en(mem_req_en),
    .d(mem_req_d),
    .q(mem_req_q)
);

assign is_fetch_req       = (is_idle  | (is_fetch & fetch_done)) & mem_req_valid;

assign csn                = ~((is_idle & mem_req_valid) | is_fetch);
assign wen                = is_fetch_req ? ~(mem_req.req_type   == MEM_WRITE) 
                          : (is_fetch & ~fetch_done) ? ~(mem_req_d.req_type == MEM_WRITE)
                          : 1'b1;
assign addr               = is_fetch_req ? mem_req.req_addr[RAM_ADDR_W-1:2] : mem_req_d.req_addr[RAM_ADDR_W-1:2];
assign din                = is_fetch_req ? mem_req.req_data                 : mem_req_d.req_data;
assign web                = is_fetch_req ? ~mem_req.req_mask                : mem_req_d.req_mask;
assign mem_req_ready      = 1'b1;
assign mem_resp_valid     = is_fetch;
assign mem_resp.resp_last = fetch_done;
assign mem_resp.resp_data = dout;
assign mem_resp.resp_type = mem_req_q.req_type;

fast_sram_sp #(
    .N_DW(MEM_DATA_W), 
    .N_DP(WORDS)
) fast_sram_sp_u (
    .clk(clk),
    .csn(csn), // chip select/enable, active low
    .wen(wen), // write enable, active low
    .web(web), // byte select, active low
    .addr(addr),
    .din(din),
    .dout(dout)        
);

endmodule