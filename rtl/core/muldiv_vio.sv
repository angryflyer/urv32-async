//branch pre unit
`include "macro.v"

module muldiv_vio (
    input  clk,
    input  rstn,

    input  op_stall,
    input  op_valid,
    output op_ready,
    input  [2:0]  op,
    input  [31:0] op1,
    input  [31:0] op2,
    output [31:0] op_out
);

    logic  mul_op_valid;
    logic  mul_op_ready;
    logic  is_op1_signed, is_op2_signed;
    logic  is_mul, is_mulh, is_mulhsu, is_mulhu;
    logic  [32:0] mul_op1, mul_op2;
    logic  [63:0] mul_out;
    logic  [31:0] mul_op_out;
    logic  mul_op1_neg, mul_op2_neg;
    assign is_mul    = (op == `R_INST_FUNC3_MUL);
    assign is_mulh   = (op == `R_INST_FUNC3_MULH);
    assign is_mulhsu = (op == `R_INST_FUNC3_MULHSU);
    assign is_mulhu  = (op == `R_INST_FUNC3_MULHU);

    assign is_op1_signed = is_mul | is_mulh | is_mulhsu;
    assign is_op2_signed = is_mul | is_mulh;

    assign mul_op1_neg = is_op1_signed && op1[31];
    assign mul_op2_neg = is_op2_signed && op2[31];
    assign mul_op_valid= op_valid && ~op[2];

    assign mul_op1     = {33{mul_op_valid}} & $signed({mul_op1_neg, op1});
    assign mul_op2     = {33{mul_op_valid}} & $signed({mul_op2_neg, op2});

    assign mul_out     = $signed(mul_op1) * $signed(mul_op2);
    assign mul_op_ready= mul_op_valid;
    assign mul_op_out  = is_mul ? mul_out[31:0] : mul_out[63:32];

    logic  div_op_valid;
    logic  div_op_ready;
    logic  [31:0] div_op_out;
    assign div_op_valid= op_valid && op[2];
    div div_u (
        .clk(clk),
        .rstn(rstn),
        .op_stall(1'b0),
        .op_valid(div_op_valid),
        .op_ready(div_op_ready),
        .op(op),
        .op1(op1),
        .op2(op2),
        .op_out(div_op_out)
    );

    assign op_ready = mul_op_ready | div_op_ready;
    assign op_out   = mul_op_valid ? mul_op_out : div_op_out;
endmodule