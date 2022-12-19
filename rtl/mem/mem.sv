//存储器模块
module ram 
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

	reg [MEM_DATA_W-1:0] mem [0:WORDS-1];

    logic ff_busy;
    logic ff_ready;
    logic ff_taken;
    logic ff_wr_en;
    logic [RAM_ADDR_W-1-2:0] ff_addr;
    logic [MEM_DATA_W-1:0] ff_wdata;
    logic [MEM_ADDR_W-1:0] ff_rdata;
    logic [MEM_MASK_W-1:0] ff_wmask;
    assign ff_wdata         = mem_req.req_data;
    assign ff_wmask         = {MEM_MASK_W{ff_wr_en}} & mem_req.req_mask;
    assign ff_addr          = mem_req.req_addr[RAM_ADDR_W-1:2];
    assign ff_taken         = mem_req_valid && mem_req_ready;
    assign ff_wr_en         = ff_taken && (mem_req.req_type == MEM_WRITE);
    assign mem_req_ready    = 1'b1;
    assign mem_resp.resp_data = ff_rdata;

    stdffr #(1) ff_busy_u (
        .clk(clk),
        .rstn(rstn),
        .d(mem_req_valid),
        .q(mem_resp_valid) 
    );

    stdffr #($bits(mem_req.req_type)) ff_type_u (
        .clk(clk),
        .rstn(rstn),
        .d(mem_req.req_type),
        .q(mem_resp.resp_type) 
    );

	always @(posedge clk) begin
		ff_rdata <= mem[ff_addr];
		if (ff_wmask[0]) mem[ff_addr][ 7: 0] <= ff_wdata[ 7: 0];
		if (ff_wmask[1]) mem[ff_addr][15: 8] <= ff_wdata[15: 8];
		if (ff_wmask[2]) mem[ff_addr][23:16] <= ff_wdata[23:16];
		if (ff_wmask[3]) mem[ff_addr][31:24] <= ff_wdata[31:24];
	end

endmodule

module rom 
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
    localparam ROM_ADDR_W = $clog2(WORDS << 2);
	reg [MEM_DATA_W-1:0] mem [0:WORDS-1];

    logic ff_busy;
    logic ff_ready;
    logic ff_taken;
    logic ff_wr_en;
    logic [ROM_ADDR_W-1-2:0] ff_addr;
    logic [MEM_DATA_W-1:0] ff_wdata;
    logic [MEM_ADDR_W-1:0] ff_rdata;
    logic [MEM_MASK_W-1:0] ff_wmask;
    assign ff_wdata         = mem_req.req_data;
    assign ff_wmask         = {MEM_MASK_W{ff_wr_en}} & mem_req.req_mask;
    assign ff_addr          = mem_req.req_addr[ROM_ADDR_W-1:2];
    assign ff_taken         = mem_req_valid && mem_req_ready;
    assign ff_wr_en         = ff_taken && (mem_req.req_type == MEM_WRITE);
    assign mem_req_ready    = 1'b1;
    assign mem_resp.resp_data = ff_rdata;

    stdffr #(1) ff_busy_u (
        .clk(clk),
        .rstn(rstn),
        .d(mem_req_valid),
        .q(mem_resp_valid) 
    );
    always @(posedge clk) begin
        ff_rdata <= mem[ff_addr];
    end
endmodule