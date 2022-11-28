//branch pre unit
`include "macro.v"

module div (
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

    localparam COUNT_FLUSH_VAL = 32;

    typedef enum logic [1:0] {
        IDLE  = 2'b00,
        BUSY  = 2'b01,
        DONE  = 2'b10
    } state_t;

    logic is_op1_signed, is_op2_signed;
    logic is_div, is_divu, is_rem, is_remu;
    logic [31:0] div_op1_unsigned, div_op2_unsigned;
    logic [63:0] div_op1, div_op2;
    logic [31:0] div_rem;
    logic [31:0] div_quot;
    logic div_op_signed;
    logic rff_div_op_signed;
    assign is_div    = (op == `R_INST_FUNC3_DIV);
    assign is_divu   = (op == `R_INST_FUNC3_DIVU);
    assign is_rem    = (op == `R_INST_FUNC3_REM);
    assign is_remu   = (op == `R_INST_FUNC3_REMU);

    assign is_op1_signed = is_div  | is_rem;
    assign is_op2_signed = is_op1_signed;

    assign div_op1_neg = is_op1_signed && op1[31];
    assign div_op2_neg = is_op2_signed && op2[31];

    assign div_op_signed = div_op1_neg ^ div_op2_neg;

    assign div_op1_unsigned = div_op1_neg ? ~op1 + 1'b1 : op1;
    assign div_op2_unsigned = div_op2_neg ? ~op2 + 1'b1 : op2;

    assign div_op1 = {{32{1'b0}}, div_op1_unsigned};
    assign div_op2 = {div_op2_unsigned, {32{1'b0}}};

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
    assign count_flush = op_valid && is_idle && ~op_stall;
    assign count_en = is_busy && ~count_done | op_ready;
    stdffref #(6) ff_count_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(COUNT_FLUSH_VAL),
        .en(count_en),
        .d(count_d),
        .q(count_q) 
    ); 

    logic [63:0] op1_q_sl;
    logic [63:0] op1_d;
    logic [63:0] op1_q;
    assign op1_q_sl = {op1_q[62:0],1'b0};
    stdffref #(64) ff_op1_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(div_op1),
        .en(count_en),
        .d(op1_d),
        .q(op1_q) 
    ); 

    logic [63:0] op2_q;
    stdffre #(64) ff_op2_u (
        .clk(clk),
        .rstn(rstn),
        .en(count_flush),
        .d(div_op2),
        .q(op2_q) 
    );

    stdffre #(1) ff_div_op_signed_u (
        .clk(clk),
        .rstn(rstn),
        .en(count_flush),
        .d(div_op_signed),
        .q(ff_div_op_signed) 
    );

    assign op1_d = (op1_q_sl >= op2_q) ? (op1_q_sl - op2_q + 1'b1) : op1_q_sl;

    assign count_done = (count_q == 6'h0);
    always_comb  begin
        next_state = curt_state;
        case(curt_state)
            IDLE : begin
                if(op_valid) begin
                    next_state = BUSY;
                end
            end
            BUSY : begin
                if(count_done) begin
                    next_state = IDLE;
                end     
            end
        endcase
    end

assign div_quot = ff_div_op_signed ? ~op1_q[31:0] + 1'b1 : op1_q[31:0];
assign div_rem  = op1_q[63:32];
assign op_ready = is_busy && count_done && ~op_stall;
assign op_out   = (is_div | is_divu) ? div_quot : div_rem;

endmodule