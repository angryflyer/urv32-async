//branch pre unit
`include "macro.v"

module muldiv (
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

    localparam COUNT_FLUSH_VAL = 63;

    typedef enum logic [1:0] {
        IDLE  = 2'b00,
        BUSY  = 2'b01,
        DONE  = 2'b10
    } state_t;

    logic  mul_op_valid;
    logic  mul_op_ready;
    logic  is_op1_signed, is_op2_signed;
    logic  is_mul, is_mulh, is_mulhsu, is_mulhu;
    logic  [63:0] mul_op1, mul_op2;
    logic  [31:0] mul_op_out;

    assign is_mul    = (op == `R_INST_FUNC3_MUL);
    assign is_mulh   = (op == `R_INST_FUNC3_MULH);
    assign is_mulhsu = (op == `R_INST_FUNC3_MULHSU);
    assign is_mulhu  = (op == `R_INST_FUNC3_MULHU);

    assign is_op1_signed = is_mul | is_mulh | is_mulhsu;
    assign is_op2_signed = is_mul | is_mulh;

    assign mul_op_valid= op_valid && ~op[2];

    assign mul_op1 = {{32{is_op1_signed && op1[31]}},op1};
    assign mul_op2 = {{32{is_op2_signed && op2[31]}},op2};

    logic curt_state;
    logic next_state;
    logic is_idle, is_busy;

    logic [5:0] count_d, count_q;
    logic count_done;
    logic count_en;
    logic count_flush;

    stdffrv #(2) ff_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(IDLE),
        .d(next_state),
        .q(curt_state) 
    ); 

    assign is_idle  = (curt_state == IDLE);
    assign is_busy  = (curt_state == BUSY);
    assign count_d  = count_q - 1'b1;
    assign count_flush = mul_op_valid && is_idle && ~op_stall;
    assign count_en = is_busy && ~count_done && ~op_stall | mul_op_ready;
    stdffref #(6) ff_count_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(COUNT_FLUSH_VAL),
        .en(count_en),
        .d(count_d),
        .q(count_q) 
    ); 

    logic [63:0] op1_d, op1_q;
    assign op1_d = {op1_q[62:0],1'b0};
    stdffref #(64) ff_op1_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(mul_op1),
        .en(count_en),
        .d(op1_d),
        .q(op1_q) 
    ); 

    logic [63:0] op2_d, op2_q;
    logic op2_q_gate;
    assign op2_d = {1'b0,op2_q[63:1]};
    assign op2_q_gate = op2_q[0];
    stdffref #(64) ff_op2_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(mul_op2),
        .en(count_en),
        .d(op2_d),
        .q(op2_q) 
    );

    logic [63:0] op_result_d, op_result_q;
    assign op_result_d = op_result_q + ({64{op2_q_gate}} & op1_q);
    stdffref #(64) ff_op_result_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(64'h0),
        .en(count_en),
        .d(op_result_d),
        .q(op_result_q) 
    );

    assign count_done = (count_q == 6'h0);
    always_comb  begin
        next_state = curt_state;
        case(curt_state)
            IDLE : begin
                if(mul_op_valid & ~op_stall) begin
                    next_state = BUSY;
                end
            end
            BUSY : begin
                if(count_done & ~op_stall) begin
                    next_state = IDLE;
                end     
            end
        endcase
    end

    assign mul_op_ready = is_busy && count_done && ~op_stall;
    assign mul_op_out   = is_mul ? op_result_d[31:0] : op_result_d[63:32];


    logic  div_op_valid;
    logic  div_op_ready;
    logic  [31:0] div_op_out;
    assign div_op_valid= op_valid && op[2];
    div div_u (
        .clk(clk),
        .rstn(rstn),
        .op_stall(op_stall),
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