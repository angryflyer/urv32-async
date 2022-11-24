//pc指针产生模块
`include "macro.v"
//`include "component.v"

module pc #(
) (
    input  clk,
    input  rstn,
    input  [31:0] rst_pc,
    input  flush,
    input  [31:0] flush_pc,
    // input  stall,
    input  bp_valid,
    input  [31:0] bp_pc,
    input  if_ready,
    output pc_valid,
    output [31:0] pc_plus,
    output [31:0] pc_value,
    output [31:0] pc_next
);

wire        rff_pc_valid;
wire [31:0] current_pc;
wire [31:0] current_pc_plus;
wire [31:0] next_pc;
wire [31:0] pc_adder_in1;

stdffr #(1) reg_pc_valid_u (
    .clk(clk),
    .rstn(rstn),
    .d(`ASSERT),
    .q(rff_pc_valid)
);

stdffrv pc_state_u (
    .clk(clk),
    .rstn(rstn),
    .rst_val(rst_pc),
    .d(next_pc),
    .q(current_pc)
);

//current_pc_plus_4
assign pc_adder_in1 = `PC_ILEN32;

adder pc_adder_u (
    .in0(current_pc), 
    .in1(pc_adder_in1), 
    .out(current_pc_plus)
);

// wire ready = ~stall && rff_pc_valid && if_ready;
wire ready = if_ready;
assign next_pc  = flush ? flush_pc 
                : bp_valid ? bp_pc
                : ready ? current_pc_plus 
                : current_pc;

assign pc_plus  = current_pc_plus;
assign pc_valid = rff_pc_valid;
assign pc_value = current_pc;
assign pc_next  = next_pc;

endmodule
