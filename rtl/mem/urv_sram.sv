//===============================================
//Name          : urv_sram
//Author        : angrybird
//Email         : 
//Date          : 2023-02-25
//Description   : mem_if to sram/fast_sram_sp bridge
//===============================================
module urv_sram 
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

logic csn;
logic wen;
logic [RAM_ADDR_W-1-2:0] addr;
logic [MEM_DATA_W-1:0]   din;
logic [MEM_DATA_W-1:0]   dout;
logic [MEM_MASK_W-1:0]   web;
assign csn                = ~(mem_req_valid | mem_resp_valid);
assign wen                = ~(mem_req.req_type == MEM_WRITE);
assign addr               = mem_req.req_addr[RAM_ADDR_W-1:2];
assign din                = mem_req.req_data;
assign web                = ~mem_req.req_mask;
assign mem_req_ready      = 1'b1;
assign mem_resp.resp_data = dout;
assign mem_resp.resp_last = mem_resp_valid;

stdffr #(1) rff_resp_valid_u (
    .clk(clk),
    .rstn(rstn),
    .d(mem_req_valid),
    .q(mem_resp_valid) 
);

stdffr #($bits(mem_req.req_type)) rff_resp_type_u (
    .clk(clk),
    .rstn(rstn),
    .d(mem_req.req_type),
    .q(mem_resp.resp_type) 
);

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