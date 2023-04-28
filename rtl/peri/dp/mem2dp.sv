//===============================================
//Name          : mem2dp
//Author        : angrybird
//Email         : 
//Date          : 2022-07-15
//Description   : mem to debug port
//                
//-----------------------------------------------
//===============================================

// `default_nettype none // check net declare
// DP_POSEDGE -> send and rcv at posedge
// Otherwise  -> send at negedge, rcv at posedge

// `define DP_POSEDGE

module mem2dp
    import urv_cfg::*;
    import urv_typedef::*;
#(
    parameter N_DW   = 32,
    parameter N_DM   = 4,
    parameter N_DIO  = 32 // testio data width
)
(  
    input  logic                       mem_req_valid,
    output logic                       mem_req_ready,
    input  mem_req_t                   mem_req,

    output logic                       mem_resp_valid,
    input  logic                       mem_resp_ready,
    output mem_resp_t                  mem_resp,
    //testio          
    input  logic                       dp_clk_i,    //Peripheral clock signal
    input  logic                       dp_rstn_i,   //Peripheral async reset signal
    input  logic     [1:0]             dp_mod_i,    //Peripheral mode select
    input  logic     [N_DIO-1:0]       dp_dat_i,    //Peripheral data in

    output logic                       dp_clk_o,    //testio clock signal 
    output logic                       dp_clk_oen,
    output logic     [N_DIO-1:0]       dp_dat_o, 
    output logic     [N_DIO-1:0]       dp_dat_oen,
    // option dp_int_o
    output logic                       dp_int_o
); 

// define state machine state value
localparam DP_IDLE         = 4'b0000;
localparam DP_WR_SEND      = 4'b0001;
localparam DP_WR_RCV_IDLE  = 4'b0010;
localparam DP_WR_RCV_ACK   = 4'b0011;
localparam DP_WR_RCV_PARITY= 4'b0100;
localparam DP_WR_RCV       = 4'b0101;
localparam DP_RD_SEND      = 4'b0110;
localparam DP_RD_RCV_IDLE  = 4'b0111;
localparam DP_RD_RCV_ACK   = 4'b1000;
localparam DP_RD_RCV       = 4'b1001;
localparam DP_RESP         = 4'b1010;
localparam DP_ACK_ERR      = 4'b1011;

// localparam MODE_INPUT_BIT   = 1'b1;
// localparam MODE_INPUT_BYTE  = 8'hff;
// localparam MODE_INPUT_HALF  = 16'hffff;
// localparam MODE_INPUT_WORD  = 32'hffff_ffff;

// localparam MODE_OUTPUT_BIT  = 1'b0;
// localparam MODE_OUTPUT_BYTE = 8'h0;
// localparam MODE_OUTPUT_HALF = 16'h0;
localparam MODE_OUTPUT_WORD = 32'h0;

localparam IDLE_DATA_VALUE  = 32'hffff_ffff;

localparam MODE_BIT         = 2'b00;
localparam MODE_BYTE        = 2'b01;
localparam MODE_HALF        = 2'b10;
localparam MODE_WORD        = 2'b11;

localparam DP_BIT           = 1;
localparam DP_BYTE          = 8;
localparam DP_HALF          = 16;
localparam DP_WORD          = 32;

mem_req_t  dff_mem_req;
mem_resp_t dff_mem_resp;

logic dp_valid;
logic dp_type;
logic dp_mem_resq_ready;
logic [N_DM-1:0] dp_strb;
logic [N_DW-1:0] dp_addr;
logic [N_DW-1:0] dp_data;
// logic mem_cap_rdy;              //ready to capture mem

// mux dp_cycle_count
// logic [7:0] dp_cycle_counter;

// state machine
logic [3:0] dp_current_state;
logic [3:0] dp_pre_state;
logic [3:0] dp_next_state;

logic is_idle;
logic is_wr_send;
// logic is_wr_rcv_idle, is_wr_rcv_ack;
// logic is_wr_rcv ;
logic is_rd_send;
// logic is_rd_rcv_idle, is_rd_rcv_ack;
logic is_rd_rcv ;
logic is_resp   ;
// logic is_ack_err;
// logic is_wr_rcv_parity;

// control signal
logic dp_send_done;
logic dp_rcv_start;
logic dp_rcv_ack;
logic dp_rcv_parity;
logic dp_rcv_stop;
logic dp_rcv_done;
logic is_send_mode;
logic is_counter_ena;

// state change : update cycle counter value
// logic is_state_change;
logic is_state_changed;
// logic is_counter_reload;

assign dp_int_o       = 1'b0;

assign mem_cap_rdy = mem_req_valid && is_idle;

always_ff @(posedge dp_clk_i) begin
    if(mem_cap_rdy)begin
        dff_mem_req<= mem_req;
    end
end

// reserve for mem response
// always_ff @(posedge dp_clk_i) begin
//     if(mem_cap_rdy)begin
//         dff_mem_resp.resp_type<= mem_req.req_type;
//         dff_mem_resp.resp_tid <= mem_req.req_tid;
//     end
// end

assign dp_mem_resq_ready = mem_resp_ready;
assign dp_valid = mem_req_valid;
assign dp_type  = (mem_req.req_type == MEM_WRITE);
assign dp_addr  = dff_mem_req.req_addr;
assign dp_data  = dff_mem_req.req_data;
assign dp_strb  = dff_mem_req.req_mask;

// generate control signal
assign is_idle        = dp_current_state == DP_IDLE       ;
assign is_wr_send     = dp_current_state == DP_WR_SEND    ;
// assign is_wr_rcv_idle = dp_current_state == DP_WR_RCV_IDLE;
// assign is_wr_rcv_ack  = dp_current_state == DP_WR_RCV_ACK ;
// assign is_wr_rcv_parity= dp_current_state == DP_WR_RCV_PARITY;
// assign is_wr_rcv      = dp_current_state == DP_WR_RCV     ;
assign is_rd_send     = dp_current_state == DP_RD_SEND    ;
// assign is_rd_rcv_idle = dp_current_state == DP_RD_RCV_IDLE;
// assign is_rd_rcv_ack  = dp_current_state == DP_RD_RCV_ACK ;
assign is_rd_rcv      = dp_current_state == DP_RD_RCV     ;
assign is_resp        = dp_current_state == DP_RESP       ;
// assign is_ack_err     = dp_current_state == DP_ACK_ERR    ;

assign is_send_mode    = is_wr_send | is_rd_send;
assign is_counter_ena  = is_send_mode | is_rd_rcv;


// fsm control from testio_trx
logic [5:0] dp_fsm_ctrl_mux;
logic [5:0] dp_fsm_ctrl_bit;
logic [5:0] dp_fsm_ctrl_byte;
logic [5:0] dp_fsm_ctrl_half;
logic [5:0] dp_fsm_ctrl_word;

assign dp_rcv_start   = dp_fsm_ctrl_mux[5];
assign dp_rcv_ack     = dp_fsm_ctrl_mux[4];
assign dp_rcv_parity  = dp_fsm_ctrl_mux[3];
assign dp_rcv_stop    = dp_fsm_ctrl_mux[2];
// rcv data done
assign dp_rcv_done    = dp_fsm_ctrl_mux[1];
// send data done
assign dp_send_done   = dp_fsm_ctrl_mux[0];

// assign is_state_change   = (dp_current_state != dp_next_state);
assign is_state_changed  = (dp_current_state != dp_pre_state);
// assign is_counter_reload = is_state_changed;

// declare testio trx control signal 
logic [5:0] dp_ctrl;
logic [5:0] dp_ctrl_bit;
logic [5:0] dp_ctrl_byte;
logic [5:0] dp_ctrl_half;
logic [5:0] dp_ctrl_word;

assign dp_ctrl = {is_state_changed, is_wr_send, is_rd_send, is_rd_rcv, is_send_mode, is_counter_ena};

//state transfer
always_ff @(posedge dp_clk_i or negedge dp_rstn_i) begin
    if(!dp_rstn_i) begin
        dp_pre_state <= DP_IDLE;
    end else begin
        dp_pre_state <= dp_current_state; 
    end
end

always_ff @(posedge dp_clk_i or negedge dp_rstn_i) begin
    if(!dp_rstn_i) begin
        dp_current_state <= DP_IDLE;
    end else begin
        dp_current_state <= dp_next_state; 
    end
end

//state switch
logic [3:0] dp_send_mode;
assign dp_send_mode = dp_type ? DP_WR_SEND : DP_RD_SEND;

always_comb begin
    case(dp_current_state)
        DP_IDLE        : begin
            dp_next_state = dp_valid ? dp_send_mode : dp_current_state;
        end
        DP_WR_SEND     : begin
            dp_next_state = dp_send_done ? DP_WR_RCV_IDLE : dp_current_state;
        end
        DP_WR_RCV_IDLE : begin
            dp_next_state = dp_rcv_start ? DP_WR_RCV_ACK  : dp_current_state;
        end
        DP_WR_RCV_ACK  : begin
            dp_next_state = dp_rcv_ack   ? DP_WR_RCV_PARITY: DP_ACK_ERR;
        end
        DP_WR_RCV_PARITY: begin
            dp_next_state = dp_rcv_parity? DP_WR_RCV      : DP_ACK_ERR;
        end
        DP_WR_RCV      : begin
            dp_next_state = dp_rcv_stop  ? DP_RESP        : dp_current_state;
        end
        DP_RD_SEND     : begin
            dp_next_state = dp_send_done ? DP_RD_RCV_IDLE : dp_current_state;
        end 
        DP_RD_RCV_IDLE : begin 
            dp_next_state = dp_rcv_start ? DP_RD_RCV_ACK  : dp_current_state;
        end
        DP_RD_RCV_ACK  : begin
            dp_next_state = dp_rcv_ack   ? DP_RD_RCV      : DP_ACK_ERR;
        end
        DP_RD_RCV      : begin
            dp_next_state = dp_rcv_done  ? DP_RESP        : dp_current_state;
        end
        DP_RESP        : begin
            dp_next_state = dp_mem_resq_ready  ? DP_IDLE  : dp_current_state;
        end
        DP_ACK_ERR     : begin
            dp_next_state = DP_IDLE;
        end
        default        : dp_next_state = DP_IDLE;
    endcase
end

// mux dp_cycle_count
// assign dp_cycle_count    = dp_cycle_count_0 & dp_cycle_count_1 & dp_cycle_count_2;

// calc resp data
logic [N_DW-1:0] resp_data_mux;
logic [N_DW-1:0] resp_data_bit;
logic [N_DW-1:0] resp_data_byte;
logic [N_DW-1:0] resp_data_half;
logic [N_DW-1:0] resp_data_word;

logic [N_DIO-1:0]   dp_send_data_mux;
logic [DP_BIT-1:0]  dp_send_data_bit;
logic [DP_BYTE-1:0] dp_send_data_byte;
logic [DP_HALF-1:0] dp_send_data_half;
logic [DP_WORD-1:0] dp_send_data_word;

logic [N_DIO-1:0]   dp_data_oen_mux;
logic [DP_BIT-1:0]  dp_data_oen_bit;
logic [DP_BYTE-1:0] dp_data_oen_byte;
logic [DP_HALF-1:0] dp_data_oen_half;
logic [DP_WORD-1:0] dp_data_oen_word;

logic [N_DIO-1:0] rff_dp_send_data;
logic [N_DIO-1:0] rff_dp_data_oen;

logic dp_en_bit;
logic dp_en_byte;
logic dp_en_half;
logic dp_en_word;

assign dp_en_bit  = (dp_mod_i == MODE_BIT);
assign dp_en_byte = (dp_mod_i == MODE_BYTE);
assign dp_en_half = (dp_mod_i == MODE_HALF);
assign dp_en_word = (dp_mod_i == MODE_WORD);

assign dp_ctrl_bit  = {6{dp_en_bit}}  & dp_ctrl;
assign dp_ctrl_byte = {6{dp_en_byte}} & dp_ctrl;
assign dp_ctrl_half = {6{dp_en_half}} & dp_ctrl;
assign dp_ctrl_word = {6{dp_en_word}} & dp_ctrl;

testio_trx #
(
  .DP_W(DP_BIT) 
) testio_trx_bit_u
(
    .req_type         (dp_type),      // read or write
    .req_addr         (dp_addr),
    .req_data         (dp_data),
    .req_strb         (dp_strb),
    .resp_data        (resp_data_bit),
    .resp_data_err    (),
   
    .dp_ctrl          (dp_ctrl_bit),   //control signal from state machine
    // testio          
    .dp_clk           (dp_clk_i),     //Peripheral clock signal
    .dp_rstn          (dp_rstn_i),    //Peripheral async reset signal
    .dp_din           (dp_dat_i[DP_BIT-1:0]),   //Peripheral data in

    .dp_dout          (dp_send_data_bit),
    .dp_doen          (dp_data_oen_bit),
    .dp_fsm_ctrl      (dp_fsm_ctrl_bit)
);

testio_trx #
(
  .DP_W(DP_BYTE) 
) testio_trx_byte_u
(
    .req_type         (dp_type),      // read or write
    .req_addr         (dp_addr),
    .req_data         (dp_data),
    .req_strb         (dp_strb),
    .resp_data        (resp_data_byte),
    .resp_data_err    (),

    .dp_ctrl          (dp_ctrl_byte), //control signal from state machine
    // testio          
    .dp_clk           (dp_clk_i),     //Peripheral clock signal
    .dp_rstn          (dp_rstn_i),    //Peripheral async reset signal
    .dp_din           (dp_dat_i[DP_BYTE-1:0]),   //Peripheral data in

    .dp_dout          (dp_send_data_byte),
    .dp_doen          (dp_data_oen_byte),
    .dp_fsm_ctrl      (dp_fsm_ctrl_byte)
);

testio_trx #
(
  .DP_W(DP_HALF) 
) testio_trx_half_u
(
    .req_type         (dp_type),      // read or write
    .req_addr         (dp_addr),
    .req_data         (dp_data),
    .req_strb         (dp_strb),
    .resp_data        (resp_data_half),
    .resp_data_err    (),   

    .dp_ctrl          (dp_ctrl_half), //control signal from state machine
    // testio          
    .dp_clk           (dp_clk_i),     //Peripheral clock signal
    .dp_rstn          (dp_rstn_i),    //Peripheral async reset signal
    .dp_din           (dp_dat_i[DP_HALF-1:0]),   //Peripheral data in

    .dp_dout          (dp_send_data_half),
    .dp_doen          (dp_data_oen_half),
    .dp_fsm_ctrl      (dp_fsm_ctrl_half)
);

testio_trx #
(
  .DP_W(DP_WORD) 
) testio_trx_word_u
(
    .req_type         (dp_type),      // read or write
    .req_addr         (dp_addr),
    .req_data         (dp_data),
    .req_strb         (dp_strb),
    .resp_data        (resp_data_word),
    .resp_data_err    (),   

    .dp_ctrl          (dp_ctrl_word), //control signal from state machine
    // testio          
    .dp_clk           (dp_clk_i),     //Peripheral clock signal
    .dp_rstn          (dp_rstn_i),    //Peripheral async reset signal
    .dp_din           (dp_dat_i[DP_WORD-1:0]),   //Peripheral data in

    .dp_dout          (dp_send_data_word),
    .dp_doen          (dp_data_oen_word),
    .dp_fsm_ctrl      (dp_fsm_ctrl_word)
);

always_ff @(negedge dp_clk_i or negedge dp_rstn_i) begin
    if(!dp_rstn_i) begin
        rff_dp_send_data <= 'h0;
    end else begin
        rff_dp_send_data <= dp_send_data_mux;
    end
end

always_ff @(negedge dp_clk_i or negedge dp_rstn_i) begin
    if(!dp_rstn_i) begin
        rff_dp_data_oen <= 'h0;
    end else begin
        rff_dp_data_oen <= dp_data_oen_mux;
    end
end

assign dp_fsm_ctrl_mux  = dp_mod_i[1] ? (dp_mod_i[0] ? dp_fsm_ctrl_word
                                                     : dp_fsm_ctrl_half)
                                      : (dp_mod_i[0] ? dp_fsm_ctrl_byte
                                                     : dp_fsm_ctrl_bit);

assign dp_send_data_mux = dp_mod_i[1] ? (dp_mod_i[0] ? dp_send_data_word[31:0]
                                                     : {2{dp_send_data_half[15:0]}})
                                      : (dp_mod_i[0] ? {4{dp_send_data_byte[7:0]}}
                                                     : {32{dp_send_data_bit[0]}});                                                 

assign dp_data_oen_mux  = dp_mod_i[1] ? (dp_mod_i[0] ? dp_data_oen_word[31:0]
                                                     : {2{dp_data_oen_half[15:0]}})
                                      : (dp_mod_i[0] ? {4{dp_data_oen_byte[7:0]}}
                                                     : {32{dp_data_oen_bit[0]}}); 

assign resp_data_mux    = dp_mod_i[1] ? (dp_mod_i[0] ? resp_data_word
                                                     : resp_data_half)
                                      : (dp_mod_i[0] ? resp_data_byte
                                                     : resp_data_bit);

// clk always output
// ti interface out
assign dp_clk_oen   = MODE_OUTPUT_WORD[0];
assign dp_clk_o     = dp_clk_i;
assign dp_dat_o     = rff_dp_send_data;
assign dp_dat_oen   = rff_dp_data_oen;

// ready when testio stay in idle, otherwise not ready
assign mem_req_ready = dp_valid && is_idle;

// mem resq
assign mem_resp_valid      = is_resp;
// assign mem_resp.resp_type  = dff_mem_resp.resp_type;
// assign mem_resp.resp_tid   = dff_mem_resp.resp_tid;
assign mem_resp.resp_data  = mem_resp_valid ? resp_data_mux : IDLE_DATA_VALUE;

endmodule

// `default_nettype wire