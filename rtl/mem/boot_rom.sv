//存储器模块
module boot_rom
    import urv_cfg::*;
    import urv_typedef::*; 
#(
    parameter integer WORDS = 2048//16K
) (
    input  logic            clk,
    input  logic            rstn,
    input  logic            mem_req_valid,
    output logic            mem_req_ready,
    input  mem_req_t        mem_req,

    output logic            mem_resp_valid,
    input  logic            mem_resp_ready,
    output mem_resp_t       mem_resp
);
localparam RAM_ADDR_W = $clog2(WORDS << 2); 

logic req_fire;
assign req_fire = mem_req_valid & mem_req_ready; 
logic req_rd;
assign req_rd = mem_req.req_type == MEM_READ;
logic req_wr;
assign req_wr = mem_req.req_type == MEM_WRITE;

logic ff_wr_en;
logic ff_rd_en;
logic [RAM_ADDR_W-1-2:0] ff_addr;
logic [MEM_DATA_W-1:0] ff_wdata;
logic [MEM_ADDR_W-1:0] ff_rdata;
logic [MEM_MASK_W-1:0] ff_wmask;
assign ff_wr_en         = req_fire && req_wr;
assign ff_rd_en         = req_fire && req_rd; 
assign ff_wdata         = mem_req.req_data;
assign ff_wmask         = {MEM_MASK_W{ff_wr_en}} & mem_req.req_mask;
assign ff_addr          = mem_req.req_addr[RAM_ADDR_W-1:2];

logic [31:0] mem [WORDS-1:0];

//rom write
always @(posedge clk) begin
    if (ff_wmask[0]) mem[ff_addr][ 7: 0] <= ff_wdata[ 7: 0];
    if (ff_wmask[1]) mem[ff_addr][15: 8] <= ff_wdata[15: 8];
    if (ff_wmask[2]) mem[ff_addr][23:16] <= ff_wdata[23:16];
    if (ff_wmask[3]) mem[ff_addr][31:24] <= ff_wdata[31:24];
end

//rom read
logic[31:0] rom_rd_data_s1;
    always @(posedge clk) begin
    if (ff_rd_en) begin
        rom_rd_data_s1 <= mem[ff_addr];
    end
end


logic req_fire_s1;
stdffr #(1) req_fire_s1_dff(
    .clk(clk),
    .rstn(rstn),
    .d(req_fire),
    .q(req_fire_s1)
);
mem_req_t req_s1;
stdffr #($bits(mem_req_t)) req_s1_dff(
    .clk(clk),
    .rstn(rstn),
    .d(mem_req),
    .q(req_s1)
);

//resp_fifo
logic               resp_fifo_push      ; 
mem_resp_t          resp_fifo_in_data   ; 
logic               resp_fifo_full      ; 
logic               resp_fifo_near_full ; 
logic               resp_fifo_pop       ; 
logic               resp_fifo_empty     ; 
mem_resp_t          resp_fifo_out_data  ; 
ot_flop_fifo #(
    .width($bits(mem_resp_t)),
    .depth(2),
    .near_full_thshd(1)
) resp_fifo_u (
    .clk       (clk), 
    .rstn      (rstn), 

    .push      (resp_fifo_push      ), 
    .in_data   (resp_fifo_in_data   ), 
    .full      (resp_fifo_full      ), 
    .near_full (resp_fifo_near_full ), 

    .pop       (resp_fifo_pop       ), 
    .empty     (resp_fifo_empty     ), 
    .out_data  (resp_fifo_out_data  ), 

    .overflow  (), 
    .usage     ()  
);
assign resp_fifo_push = req_fire_s1;
assign resp_fifo_in_data.resp_data = rom_rd_data_s1;

assign mem_resp_valid = ~resp_fifo_empty;
assign mem_resp = resp_fifo_out_data;
assign resp_fifo_pop = ~resp_fifo_empty & mem_resp_ready;

assign mem_req_ready = ~resp_fifo_near_full;

endmodule