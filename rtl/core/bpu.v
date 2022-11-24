//branch pre unit
`include "macro.v"

module bpu #(
    parameter  N_ADDR_BITS = `BP_ADDR_BITS,
    parameter  N_DATA_BITS = 32,
    parameter  N_STAT_BITS = 2,
    parameter  N_DEPTH_BITS= `BP_ADDR_DEPTH,
    parameter  N_ADDR_W    = $clog2(N_DEPTH_BITS)
) (
	input  clk,
	input  rstn,

	// ifu <-> bpu
	input  req_valid,
	output req_ready,
	input  [N_ADDR_BITS-1:0] req_pc,
	output resp_valid,
	output resp_match,
	output [N_ADDR_W-1:0] resp_addr,
	output [N_DATA_BITS-1:0] resp_pc,

	// exu <-> bpu
	input  flush_valid,
	input  flush_new_pc,
	input  flush_type,        // 1'b0 -> njump, 1'b1 -> jump
	input  [N_ADDR_W-1:0]  flush_addr, // record 8 pc max
	input  [N_ADDR_BITS-1:0] flush_bp_pc,
	input  [N_DATA_BITS-1:0] flush_pc
);

localparam STRG_NTAKEN  = 2'b00; // strong not taken
localparam WEAK_NTAKEN  = 2'b01;
localparam WEAK_TAKEN   = 2'b10; 
localparam STRG_TAKEN   = 2'b11; // strong taken

wire wrptr_wr_en;

wire rd_en;

wire bp_taken;

reg [N_ADDR_W-1:0] wrptr;
wire[N_ADDR_W-1:0] next_wrptr;

`ifdef BPU_FUNC0

reg  [N_ADDR_BITS-1:0] addr_mem  [N_DEPTH_BITS-1:0];
reg  [N_DATA_BITS-1:0] data_mem  [N_DEPTH_BITS-1:0];
reg  [N_STAT_BITS-1:0] state_mem [N_DEPTH_BITS-1:0];
reg  [N_ADDR_W-1:0]    resp_addr;
wire [N_ADDR_W-1:0]    state_addr;
wire [N_STAT_BITS-1:0] state, next_state;

wire [N_STAT_BITS-1:0] taken_state;
wire [N_DEPTH_BITS-1:0] addr_rd_en;
wire pc_wr_en;

// addr mem update
assign pc_wr_en = flush_valid && flush_new_pc;
always @(posedge clk or negedge rstn) begin
	if(!rstn) begin
		for(integer i = 0; i < N_DEPTH_BITS; i = i + 1) begin : ADDR_MEM_RESET
			addr_mem[i] <= {N_ADDR_BITS{1'b0}};
		end
	end else if(pc_wr_en) begin
		addr_mem[wrptr] <= flush_bp_pc;
	end
end

// data mem update
always @(posedge clk or negedge rstn) begin
	if(!rstn) begin
		for(integer i = 0; i < N_DEPTH_BITS; i = i + 1) begin : DATA_MEM_RESET
			data_mem[i] <= {N_DATA_BITS{1'b0}};
		end
	end else if(pc_wr_en) begin
		data_mem[wrptr] <= flush_pc;
	end
end

generate
	for(genvar i = 0; i < N_DEPTH_BITS; i = i + 1) begin : ADDR_RD_EN_GEN
		assign addr_rd_en[i] = req_valid && (req_pc == addr_mem[i]);
	end
endgenerate

// addr encode
always @(*) begin
    resp_addr = wrptr;
    for(integer i = 0; i < N_DEPTH_BITS; i = i + 1) begin : RESP_ADDR_GEN
        if(addr_rd_en[i]) begin
            resp_addr = i;
        end 
    end
end

// wrptr point which should be update
assign wrptr_wr_en = flush_valid && flush_new_pc;
assign next_wrptr = wrptr + 1'b1; 

stdffre #(N_ADDR_W) wrptr_u (
	.clk(clk),
	.rstn(rstn),
	.en(wrptr_wr_en),
	.d(next_wrptr),
	.q(wrptr)	
);

// state change
assign state_addr = flush_new_pc ? wrptr : flush_addr;
always @(posedge clk or negedge rstn) begin
	if(!rstn) begin
		for(integer i = 0; i < N_DEPTH_BITS; i = i + 1) begin : STATE_RESET
			state_mem[i] <= {N_STAT_BITS{1'b0}};
		end
	end else if(flush_valid) begin
		state_mem[state_addr] <= next_state;
	end
end
// state switch
assign state = state_mem[state_addr];
assign next_state = ((state == STRG_NTAKEN) && ~flush_type) ? STRG_NTAKEN
				  : ((state == STRG_TAKEN)  &&  flush_type) ? STRG_TAKEN
				  : state + (flush_type ? 2'b01 : 2'b11); // +1 -> 2'b01, -1 -> 2'b11 

// bp req
assign taken_state= state_mem[resp_addr];
assign bp_taken   = taken_state[1];

assign rd_en      = (addr_rd_en != {N_DEPTH_BITS{1'b0}});

assign req_ready  = req_valid;
assign resp_valid = rd_en && bp_taken;
assign resp_match = rd_en;

assign resp_pc    = ~rd_en ? {N_DATA_BITS{1'b0}} : data_mem[resp_addr];
`endif

`ifdef BPU_FUNC1
// fixme
// taken or ntaken state record
reg  [N_STAT_BITS-1:0] state_mem [N_DEPTH_BITS-1:0]; 
wire [N_STAT_BITS-1:0] state, next_state;

wire [N_STAT_BITS-1:0] taken_state;

// state change
always @(posedge clk or negedge rstn) begin
	if(!rstn) begin
		for(integer i = 0; i < N_DEPTH_BITS; i = i + 1) begin : STATE_RESET
			state_mem[i] <= {N_STAT_BITS{1'b0}};
		end
	end else if(flush_valid) begin
		state_mem[flush_pc] <= next_state;
	end
end

// state switch
assign state = state_mem[flush_pc];
assign next_state = ((state == STRG_NTAKEN) && ~flush_type) ? STRG_NTAKEN
				  : ((state == STRG_TAKEN)  &&  flush_type) ? STRG_TAKEN
				  : state + (flush_type ? 2'b01 : 2'b11); // +1 -> 2'b01, -1 -> 2'b11 

// bp req
assign taken_state= state_mem[req_pc];
assign bp_taken   = taken_state[1];

assign req_ready  = req_valid;
assign resp_valid = req_valid && bp_taken;
assign resp_match = req_valid;
assign resp_addr  = {N_ADDR_W{1'b0}};
assign resp_pc    = {N_DATA_BITS{1'b0}};

`endif
endmodule