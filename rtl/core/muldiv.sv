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
    output [31:0] op_do
);

    localparam COUNT_FLUSH_VAL = 31;

    typedef enum logic [1:0] {
        IDLE  = 2'b00,
        BUSY  = 2'b01,
        DONE  = 2'b10
    } state_t;

    logic curt_state;
    logic next_state;
    logic is_idle, is_busy;

    logic [4:0] count_d, count_q;
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
    assign count_flush = op_valid && (is_idle | (is_busy && count_done)) && ~op_stall;
    assign count_en = is_busy && ~count_done | op_ready;
    stdffref #(5) ff_count_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(COUNT_FLUSH_VAL),
        .en(count_en),
        .d(count_d),
        .q(count_q) 
    ); 

    logic [63:0] op1_d, op1_q;
    assign op1_d = op1_q << 1'b1;
    stdffref #(64) ff_op1_u (
        .clk(clk),
        .rstn(rstn),
        .flush(count_flush),
        .flush_val(op1),
        .en(count_en),
        .d(op1_d),
        .q(op1_q) 
    ); 

    logic [31:0] op2_d, op2_q;
    logic op2_q_gate;
    assign op2_d = op2;
    assign op2_q_gate = op2_q[31-count_q];
    stdffref #(32) ff_op2_u (
        .clk(clk),
        .rstn(rstn),
        .en(count_flush),
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

    assign count_done = (count_q == 5'h0);
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
                    next_state = op_valid ? BUSY : IDLE;
                end else begin
                    next_state = BUSY;
                end     
            end
        endcase
    end

assign op_ready = is_busy && count_done && ~op_stall;

endmodule