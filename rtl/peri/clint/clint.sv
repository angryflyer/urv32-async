///////////////////////
// description:
///////////////////////
`include "inst_index.v"
`include "macro.v"
// `include "cfg.sv"
// `include "typedef.sv"

module clint 
  import urv_cfg::*;
  import urv_typedef::*; 
(
    input                   clk,
    input                   rstn,

    // mem_if interface
    input                   mem_req_valid,
    output                  mem_req_ready,
    input  mem_req_t        mem_req, 

    output                  mem_resp_valid,
    input                   mem_resp_ready,
    output mem_resp_t       mem_resp, 

    output                  soft_irq,
    output                  time_irq,
    output [CSR_TIME_W-1:0] time_val   
);

localparam CLINT_SOFTINT_ADDR_OFFSET    = 3'h0; // 0 soft int
localparam CLINT_MTIMEL_ADDR_OFFSET     = 3'h1; // 4
localparam CLINT_MTIMEH_ADDR_OFFSET     = 3'h2; // 8
localparam CLINT_MTIMEL_CMP_ADDR_OFFSET = 3'h3; // c
localparam CLINT_MTIMEH_CMP_ADDR_OFFSET = 3'h4; // 10
localparam CLINT_MTIME_EN_ADDR_OFFSET   = 3'h5; // 14

logic ff_busy;
logic ff_ready;
logic ff_taken;
logic mem_req_handshaked;
logic mem_resp_handshaked;
logic ff_wr_en;
logic soft_int_wr_en;
logic mtimel_wr_en;
logic mtimeh_wr_en;
logic mtimel_cmp_wr_en;
logic mtimeh_cmp_wr_en;
logic mtime_en_wr_en;
logic mtime_count_en;

logic [2:0] ff_addr;
logic [MEM_DATA_W-1:0] ff_wdata;;
logic [MEM_MASK_W-1:0] ff_wmask;;
logic [MEM_DATA_W-1:0] ff_bit_mask;

logic [MEM_DATA_W-1:0] ff_out_0;
logic [MEM_DATA_W-1:0] ff_out_1;
logic [MEM_DATA_W-1:0] ff_out_2;
logic [MEM_DATA_W-1:0] ff_out_3;
logic [MEM_DATA_W-1:0] ff_out_4;
logic [MEM_DATA_W-1:0] ff_out_5;
logic [MEM_DATA_W-1:0] ff_out;
logic [MEM_DATA_W-1:0] resp_data;
logic [CSR_TIME_W-1:0] mtime_count_d, mtime_count_q;
logic [CSR_TIME_W-1:0] mtime_cmp_count_q;

assign ff_wdata         = mem_req.req_data;
assign ff_wmask         = mem_req.req_mask;
assign ff_addr          = mem_req.req_addr[4:2];
assign ff_ready         = ~ff_busy;
assign ff_taken         = mem_req_handshaked;
assign ff_wr_en         = mem_req_handshaked && (mem_req.req_type == MEM_WRITE);
assign mtime_count_en   = ff_out_5[0];
assign mtime_count_q    = {ff_out_2, ff_out_1};
assign mtime_cmp_count_q= {ff_out_4, ff_out_3};
assign mtime_count_d    = mtime_count_q + 1'b1;

assign mem_req_handshaked = mem_req_valid && mem_req_ready;
assign mem_resp_handshaked= mem_resp_valid && mem_resp_ready;

assign soft_int_wr_en   = ff_wr_en && (ff_addr == CLINT_SOFTINT_ADDR_OFFSET);
assign mtimel_wr_en     = ff_wr_en && (ff_addr == CLINT_MTIMEL_ADDR_OFFSET);
assign mtimeh_wr_en     = ff_wr_en && (ff_addr == CLINT_MTIMEH_ADDR_OFFSET);
assign mtimel_cmp_wr_en = ff_wr_en && (ff_addr == CLINT_MTIMEL_CMP_ADDR_OFFSET);
assign mtimeh_cmp_wr_en = ff_wr_en && (ff_addr == CLINT_MTIMEH_CMP_ADDR_OFFSET);
assign mtime_en_wr_en   = ff_wr_en && (ff_addr == CLINT_MTIME_EN_ADDR_OFFSET);

stdffref #(1) ff_busy_u (
    .clk(clk),
    .rstn(rstn),
    .flush(mem_req_handshaked),
    .flush_val(1'b1),
    .en(mem_resp_handshaked),
    .d(1'b0),
    .q(ff_busy) 
);

stdffre #(MEM_DATA_W) ff_resp_data_u (
    .clk(clk),
    .rstn(rstn),
    .en(ff_taken),
    .d(ff_out),
    .q(resp_data) 
);

genvar i;
generate
for(i=0;i<(MEM_DATA_W/8);i++)begin : GEN_FF_BIT_MASK
    assign  ff_bit_mask[8*i+:8]  = {8{ff_wmask[i]}};
end
endgenerate

stdffrem #(1) ff_soft_int_u (
    .clk(clk),
    .rstn(rstn),
    .en(soft_int_wr_en),
    .mask(ff_bit_mask[0]),
    .d(ff_wdata[0]),
    .q(ff_out_0[0])    
);
assign ff_out_0[MEM_DATA_W-1:1] = {(MEM_DATA_W-1){1'b0}};

stdffrefm #(MEM_DATA_W) ff_mtimel_u (
    .clk(clk),
    .rstn(rstn),
    .flush(mtimel_wr_en),
    .flush_val(ff_wdata),
    .mask(ff_bit_mask),
    .en(mtime_count_en),
    .d(mtime_count_d[MEM_DATA_W-1:0]),
    .q(ff_out_1)    
);

stdffrefm #(MEM_DATA_W) ff_mtimeh_u (
    .clk(clk),
    .rstn(rstn),
    .flush(mtimel_wr_en),
    .flush_val(ff_wdata),
    .mask(ff_bit_mask),
    .en(mtime_count_en),
    .d(mtime_count_d[CSR_TIME_W-1:MEM_DATA_W]),
    .q(ff_out_2)    
);

stdffrem #(MEM_DATA_W) ff_mtimel_cmp_u (
    .clk(clk),
    .rstn(rstn),
    .en(mtimel_cmp_wr_en),
    .mask(ff_bit_mask),
    .d(ff_wdata),
    .q(ff_out_3)   
);

stdffrem #(MEM_DATA_W) ff_mtimeh_cmp_u (
    .clk(clk),
    .rstn(rstn),
    .en(mtimeh_cmp_wr_en),
    .mask(ff_bit_mask),
    .d(ff_wdata),
    .q(ff_out_4)    
);

stdffrem #(1) ff_mtime_en_u (
    .clk(clk),
    .rstn(rstn),
    .en(mtime_en_wr_en),
    .mask(ff_bit_mask[0]),
    .d(ff_wdata[0]),
    .q(ff_out_5[0])    
);
assign ff_out_5[MEM_DATA_W-1:1] = {(MEM_DATA_W-1){1'b0}};

always_comb
begin 
  ff_out = {MEM_DATA_W{1'b0}};
  case (ff_addr)
    3'b000 : ff_out = ff_out_0;     
    3'b001 : ff_out = ff_out_1;    
    3'b010 : ff_out = ff_out_2;     
    3'b011 : ff_out = ff_out_3;    
    3'b100 : ff_out = ff_out_4;    
    3'b101 : ff_out = ff_out_5;      
    default  : ff_out = {MEM_DATA_W{1'b0}};
  endcase
end

// assign mem_req_ready       = mem_req_valid;
assign mem_req_ready       = ff_ready;
assign mem_resp_valid      = ff_busy;
assign mem_resp.resp_data  = resp_data;
assign mem_resp.resp_last  = mem_resp_valid;

assign soft_irq = ff_out_0[0];
assign time_irq = mtime_count_en && (unsigned'(mtime_count_q) >= unsigned'(mtime_cmp_count_q));
assign time_val = mtime_count_q;      

endmodule