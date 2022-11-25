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

    logic is_op1_signed, is_op2_signed;
    logic is_mul, is_mulh, is_mulhsu, is_mulhu;
    logic [32:0] mul_op1, mul_op2;
    logic [63:0] mul_out;
    logic mul_op1_neg, mul_op2_neg;
    assign is_mul    = (op == `R_INST_FUNC3_MUL);
    assign is_mulh   = (op == `R_INST_FUNC3_MULH);
    assign is_mulhsu = (op == `R_INST_FUNC3_MULHSU);
    assign is_mulhu  = (op == `R_INST_FUNC3_MULHU);

    assign is_op1_signed = is_mul | is_mulh | is_mulhsu;
    assign is_op2_signed = is_mul | is_mulh;

    assign mul_op1_neg = is_op1_signed && op1[31];
    assign mul_op2_neg = is_op2_signed && op2[31];

    assign mul_op1     = $signed({mul_op1_neg, op1});
    assign mul_op2     = $signed({mul_op2_neg, op2});

    assign mul_out     = $signed(mul_op1) * $signed(mul_op2);
    assign op_ready    = op_valid;
    assign op_out      = is_mul ? mul_out[31:0] : mul_out[63:32];

endmodule