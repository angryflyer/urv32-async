//逻辑块or器件模块
`include "macro.v"

module mux2to1 #(
    parameter W = 32
) (
    input sel,

    input [W-1:0] in0,
    input [W-1:0] in1,

    output [W-1:0] out
);
assign out = sel ? in1 : in0;
// reg [W-1:0] reg_out;
// always @(*) begin
//     case(sel)
//         `ASSERT : reg_out = in1;
//         default : reg_out = in0;
//     endcase
// end
// assign out = reg_out;

endmodule

`define IN0_4TO1 2'b00
`define IN1_4TO1 2'b01
`define IN2_4TO1 2'b10
`define IN3_4TO1 2'b11
module mux4to1 #(
    parameter W = 32
) (
    input [1:0] sel,

    input [W-1:0] in0,
    input [W-1:0] in1,
    input [W-1:0] in2,
    input [W-1:0] in3,

    output [W-1:0] out
);
wire s0, s1;
wire [W-1:0] out0, out1;

assign s0 = sel[0];
assign s1 = sel[1];

assign out0 = s0 ? in1 : in0;
assign out1 = s0 ? in3 : in2;
assign out  = s1 ? out1 : out0;
// reg [W-1:0] reg_out;

// always @(*) begin
//     (*parallel_case*)
//     case(sel)// synopsys parallel_case
//         `IN0_4TO1 : reg_out = in0;
//         `IN1_4TO1 : reg_out = in1;
//         `IN2_4TO1 : reg_out = in2;
//         `IN3_4TO1 : reg_out = in3;
//         default : reg_out = in0;
//     endcase
// end

// assign out = reg_out;

endmodule

`define IN0_8TO1 3'b000
`define IN1_8TO1 3'b001
`define IN2_8TO1 3'b010
`define IN3_8TO1 3'b011
`define IN4_8TO1 3'b100
`define IN5_8TO1 3'b101
`define IN6_8TO1 3'b110
`define IN7_8TO1 3'b111
module mux8to1 #(
    parameter W = 32
) (
    input [3:0] sel,

    input [W-1:0] in0,
    input [W-1:0] in1,
    input [W-1:0] in2,
    input [W-1:0] in3,
    input [W-1:0] in4,
    input [W-1:0] in5,
    input [W-1:0] in6,
    input [W-1:0] in7,

    output [W-1:0] out
);

wire [1:0] s0;
wire s1;
wire [W-1:0] out0, out1;

assign s0 = sel[1:0];
assign s1 = sel[2];

mux4to1 mux4to1_u0(.sel(s0), .in0(in0), .in1(in1), .in2(in2), .in3(in3), .out(out0));
mux4to1 mux4to1_u1(.sel(s0), .in0(in4), .in1(in5), .in2(in6), .in3(in7), .out(out1));
assign  out = s1 ? out1 : out0;

// reg [W-1:0] reg_out;

// always @(*) begin
//     (*parallel_case*)
//     case(sel)// synopsys parallel_case
//         `IN0_8TO1 : reg_out = in0;
//         `IN1_8TO1 : reg_out = in1;
//         `IN2_8TO1 : reg_out = in2;
//         `IN3_8TO1 : reg_out = in3;
//         `IN4_8TO1 : reg_out = in4;
//         `IN5_8TO1 : reg_out = in5;
//         `IN6_8TO1 : reg_out = in6;
//         `IN7_8TO1 : reg_out = in7;

//         default : reg_out = in0;
//     endcase
// end

// assign out = reg_out;

endmodule

// delay 2cycle
module delay_2cycle
(
    input  clk,
    input  rstn,
    input  d,
    output q    
);
reg q1, q2;

always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        q1 <= 1'b0;
        q2 <= 1'b0;
    end else begin
        q1 <= d;
        q2 <= q1;
    end
end
assign q = q2;

endmodule

// sync_async_reset 2cycles
module sync_async_reset
(
    input  clk,
    input  arstn,
    output srstn
);
reg q1, q2;

always @(posedge clk or negedge arstn) begin
    if(!arstn) begin
        q1 <= 1'b0;
        q2 <= 1'b0;
    end else begin
        q1 <= arstn;
        q2 <= q1;
    end
end
assign srstn = q2;

endmodule

module stdffr #(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input  [W-1:0] d,
    output [W-1:0] q
);

reg [W-1:0] rff_q;
always @(posedge clk or negedge rstn)begin
    if(!rstn) begin
        rff_q <= {W{1'b0}};
    end else begin
        rff_q <= d;
    end
end

assign q = rff_q;

endmodule

module stdffre #(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input          en,
    input  [W-1:0] d,
    output [W-1:0] q
);

reg [W-1:0] rff_q;
always @(posedge clk or negedge rstn)begin
    if(!rstn) begin
        rff_q <= {W{1'b0}};
    end else if(en) begin
        rff_q <= d;
    end
end

assign q = rff_q;

endmodule

module stdffrv #(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input  [W-1:0] rst_val,
    input  [W-1:0] d,
    output [W-1:0] q
);

reg [W-1:0] rff_q;
always @(posedge clk or negedge rstn)begin
    if(!rstn) begin
        rff_q <= rst_val;
    end else begin
        rff_q <= d;
    end
end

assign q = rff_q;

endmodule

module stdffrve #(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input  [W-1:0] rst_val,
    input          en,
    input  [W-1:0] d,
    output [W-1:0] q
);

reg [W-1:0] rff_q;
always @(posedge clk or negedge rstn)begin
    if(!rstn) begin
        rff_q <= rst_val;
    end else if(en) begin
        rff_q <= d;
    end
end

assign q = rff_q;

endmodule

module stdffrem
#(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input          en,
    input  [W-1:0] mask,
    input  [W-1:0] d,
    output [W-1:0] q
);

logic    [W-1:0]     rff_q;

always_ff @(posedge clk or negedge rstn) begin
    if(~rstn)begin
        rff_q <= {W{1'b0}};
    end
    else if(en)begin
        rff_q <= (rff_q & ~mask) | (d & mask);
    end
end

assign  q = rff_q;

endmodule

module stdffrvem
#(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input  [W-1:0] rst_val,
    input          en,
    input  [W-1:0] mask,
    input  [W-1:0] d,
    output [W-1:0] q
);

logic    [W-1:0]     rff_q;

always_ff @(posedge clk or negedge rstn) begin
    if(~rstn)begin
        rff_q <= rst_val;
    end
    else if(en)begin
        rff_q <= (rff_q & ~mask) | (d & mask);
    end
end

assign  q = rff_q;

endmodule

module stdffref #(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input          flush,
    input  [W-1:0] flush_val,
    input          en,
    input  [W-1:0] d,
    output [W-1:0] q
);

reg [W-1:0] rff_q;
always @(posedge clk or negedge rstn)begin
    if(!rstn) begin
        rff_q <= {W{1'b0}};
    end else if(flush) begin
        rff_q <= flush_val;
    end else if(en) begin
        rff_q <= d;
    end
end

assign q = rff_q;

endmodule

module stdffrefm #(
    parameter W = 32
)
(
    input          clk,
    input          rstn,
    input          flush,
    input  [W-1:0] flush_val,
    input  [W-1:0] mask,
    input          en,
    input  [W-1:0] d,
    output [W-1:0] q
);

reg [W-1:0] rff_q;
always @(posedge clk or negedge rstn)begin
    if(!rstn) begin
        rff_q <= {W{1'b0}};
    end else if(flush) begin
        rff_q <= (rff_q & ~mask) | (flush_val & mask);
    end else if(en) begin
        rff_q <= d;
    end
end

assign q = rff_q;

endmodule

// adder
module adder #(
    parameter W = 32
) (
    input [W-1:0] in0,
    input [W-1:0] in1,

    output [W-1:0] out
);
// 暴力加法器，其效果依赖于综合器
assign out = in0 + in1;
// reg [W-1:0] reg_out;
// always @(*) begin
//     reg_out = in0 + in1;
// end

// assign out = reg_out;

endmodule

module shifter_r #(
    parameter W = 32
) (
    input [W-1:0] in0,
    input [4:0] in1,

    output [W-1:0] out
);

assign out = $signed(in0) >>> in1;

endmodule

module reverse #(
    parameter W = 32
) (
    input [W-1:0] in,
    output [W-1:0] out
);
    genvar i; 
    generate   
        for (i=0; i < W; i=i+1) 
        begin : inverse
            assign out[i] = in[W-1-i];
        end
    endgenerate

endmodule


//to do
module alu #(
    parameter W = 32
) (
    input [3:0] op,

    input [W-1:0] in0,
    input [W-1:0] in1,

    output comp,
    output [W-1:0] out
);

wire [W-1:0] wire_op1;
wire [W-1:0] wire_op2;

wire [W-1:0] op1;
wire [W-1:0] op2;
wire [W-1:0] adder_out;

assign op1 = in0; 
assign op2 = in1;
// control signals
wire  is_sub, is_cmp, is_cmp_u, is_cmp_inv, is_cmp_eq;
assign is_sub = op[3];
assign is_cmp = (op >= `ALU_OP_SLT);
assign is_cmp_u = op[1];
assign is_cmp_inv = op[0];
assign is_cmp_eq = ~op[3];

// ADD, SUB  
wire [W-1:0] op2_inv, op1_xor_op2;
assign op2_inv = is_sub ? ~op2 : op2;
assign op1_xor_op2 = op1 ^ op2;
assign wire_op1 = op1;
assign wire_op2 = op2_inv + {{(31){1'b0}}, is_sub};
adder alu_adder_u (.in0(wire_op1), .in1(wire_op2), .out(adder_out));

// SLT, SLTU
wire  slt;
//classic
assign slt = (op1[31] == op2[31]) ? adder_out[31] : ((is_cmp_u ? op2[31] : op1[31]));
assign comp = is_cmp_inv ^ (is_cmp_eq ? (op1_xor_op2 == `WORD_DEASSERT) : slt);

// SLL, SRL, SRA
wire [W-1:0] shout;

wire [31:0] shin_hi;
wire [W-1:0] shin_r, shin_l, shin, shout_r, shout_l;

wire [4:0] shamt;
assign shamt = op2[4:0];

assign shin_r = op1;

reverse reverse_u1(.in(shin_r), .out(shin_l));
assign shin = ((op == `ALU_OP_SRL) | (op == `ALU_OP_SRA)) ? shin_r : shin_l;
assign shout_r = $signed({{is_sub & shin[31]}, shin[31:0]}) >>> shamt;
reverse reverse_u2(.in(shout_r), .out(shout_l));
assign shout = (((op == `ALU_OP_SRL) | (op == `ALU_OP_SRA)) ? shout_r : `WORD_DEASSERT)
              | ((op == `ALU_OP_SLL) ? shout_l : `WORD_DEASSERT);


// AND, OR, XOR
wire [W-1:0]  logic_out;

assign logic_out = (((op == `ALU_OP_XOR) | (op == `ALU_OP_OR)) ? op1_xor_op2 : `WORD_DEASSERT)
                 | (((op == `ALU_OP_OR) | (op == `ALU_OP_AND)) ? op1 & op2 : `WORD_DEASSERT);

// alu out
wire [W-1:0] alu_out;
assign alu_out = (op == `ALU_OP_ADD) | (op == `ALU_OP_SUB) ? adder_out : ({{(31){1'b0}}, is_cmp & slt} | logic_out | shout);

assign out = alu_out;


// reg [W-1:0] reg_out; 
// reg reg_comp;
// reg reg_bge;
// reg reg_blt;

// reg reg_sub_mode;
// reg reg_add_mode;
// reg reg_logic_mode;
// reg reg_compare_mode;

// reg [W-1:0] reg_in0;
// reg [W-1:0] reg_in1;
// reg [W-1:0] reg_addout;
// reg [W-1:0] reg_logicout;
// always @(*) begin
//     (* parallel_case, full_case *)
//     reg_logic_mode = `ASSERT;
//     case(op)
//         alu_slli, alu_sll: reg_logicout = in0 << in1[4:0];
//         alu_srli, alu_srl: reg_logicout = in0 >> in1[4:0];
//         alu_srai, alu_sra: reg_logicout = $signed(in0) >>> in1[4:0];
//         alu_andi, alu_and: reg_logicout = in0 & in1;
//         alu_ori , alu_or : reg_logicout = in0 | in1;    
//         alu_xori, alu_xor: reg_logicout = in0 ^ in1;     
//         default : begin
//             reg_logicout = 32'h0;
//             reg_logic_mode = `DEASSERT;
//         end
//     endcase
// end

endmodule