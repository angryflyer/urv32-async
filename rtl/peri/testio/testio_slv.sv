//===============================================
//Name          : testio
//Author        : zhaowei
//Email         : 
//Date          : 2022-07-15
//Description   : mem2testio
//                width can be configured
//-----------------------------------------------
//===============================================

// `default_nettype none // check net declare
// TI_POSEDGE -> send and rcv at posedge
// Otherwise  -> send at negedge, rcv at posedge

// `define TI_POSEDGE

module testio_slv
    import urv_cfg::*;
    import urv_typedef::*;
#(
    parameter DATA_W = 32,
    parameter MASK_W = 4,
    parameter TIO_W  = 32
)
(  
    //mem interface
    // input                              mem_clk_i,
    // input                              mem_rstn_i,
    
    input  logic                       mem_req_valid,
    output logic                       mem_req_ready,
    input  mem_req_t                   mem_req,

    output logic                       mem_resp_valid,
    input  logic                       mem_resp_ready,
    output mem_resp_t                  mem_resp,
    //testio          
    input  logic                       ti_clk_i,    //Peripheral clock signal
    input  logic                       ti_rstn_i,   //Peripheral async reset signal
    input  logic     [1:0]             ti_mod_i,    //Peripheral mode select
    input  logic     [TIO_W-1:0]       ti_dat_i,    //Peripheral data in

    output logic                       ti_clk_o,    //testio clock signal 
    output logic                       ti_clk_oen,
    output logic     [TIO_W-1:0]       ti_dat_o, 
    output logic     [TIO_W-1:0]       ti_dat_oen,
    // option ti_int_o
    output logic                       ti_int_o
); 

// define state machine state value
localparam TI_IDLE         = 4'b0000;
localparam TI_WR_SEND      = 4'b0001;
localparam TI_WR_RCV_IDLE  = 4'b0010;
localparam TI_WR_RCV_ACK   = 4'b0011;
localparam TI_WR_RCV_PARITY= 4'b0100;
localparam TI_WR_RCV       = 4'b0101;
localparam TI_RD_SEND      = 4'b0110;
localparam TI_RD_RCV_IDLE  = 4'b0111;
localparam TI_RD_RCV_ACK   = 4'b1000;
localparam TI_RD_RCV       = 4'b1001;
localparam TI_RESP         = 4'b1010;
localparam TI_ACK_ERR      = 4'b1011;

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

localparam TI_BIT           = 1;
localparam TI_BYTE          = 8;
localparam TI_HALF          = 16;
localparam TI_WORD          = 32;

mem_req_t  dff_mem_req;
mem_resp_t dff_mem_resp;

logic ti_valid;
logic ti_type;
logic ti_mem_resq_ready;
logic [MASK_W-1:0] ti_strb;
logic [DATA_W-1:0] ti_addr;
logic [DATA_W-1:0] ti_data;
// logic mem_cap_rdy;              //ready to capture mem

// mux ti_cycle_count
// logic [7:0] ti_cycle_counter;

// state machine
logic [3:0] ti_current_state;
logic [3:0] ti_pre_state;
logic [3:0] ti_next_state;

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
logic ti_send_done;
logic ti_rcv_start;
logic ti_rcv_ack;
logic ti_rcv_parity;
logic ti_rcv_stop;
logic ti_rcv_done;
logic is_send_mode;
logic is_counter_ena;

// state change : update cycle counter value
// logic is_state_change;
logic is_state_changed;
// logic is_counter_reload;

assign ti_int_o       = 1'b0;

assign mem_cap_rdy = mem_req_valid && is_idle;

always_ff @(posedge ti_clk_i) begin
    if(mem_cap_rdy)begin
        dff_mem_req<= mem_req;
    end
end

// reserve for mem response
// always_ff @(posedge ti_clk_i) begin
//     if(mem_cap_rdy)begin
//         dff_mem_resp.resp_type<= mem_req.req_type;
//         dff_mem_resp.resp_tid <= mem_req.req_tid;
//     end
// end

assign ti_mem_resq_ready = mem_resp_ready;
assign ti_valid = mem_req_valid;
assign ti_type  = (mem_req.req_type == MEM_WRITE);
assign ti_addr  = dff_mem_req.req_addr;
assign ti_data  = dff_mem_req.req_data;
assign ti_strb  = dff_mem_req.req_mask;

// generate control signal
assign is_idle        = ti_current_state == TI_IDLE       ;
assign is_wr_send     = ti_current_state == TI_WR_SEND    ;
// assign is_wr_rcv_idle = ti_current_state == TI_WR_RCV_IDLE;
// assign is_wr_rcv_ack  = ti_current_state == TI_WR_RCV_ACK ;
// assign is_wr_rcv_parity= ti_current_state == TI_WR_RCV_PARITY;
// assign is_wr_rcv      = ti_current_state == TI_WR_RCV     ;
assign is_rd_send     = ti_current_state == TI_RD_SEND    ;
// assign is_rd_rcv_idle = ti_current_state == TI_RD_RCV_IDLE;
// assign is_rd_rcv_ack  = ti_current_state == TI_RD_RCV_ACK ;
assign is_rd_rcv      = ti_current_state == TI_RD_RCV     ;
assign is_resp        = ti_current_state == TI_RESP       ;
// assign is_ack_err     = ti_current_state == TI_ACK_ERR    ;

assign is_send_mode    = is_wr_send | is_rd_send;
assign is_counter_ena  = is_send_mode | is_rd_rcv;


// fsm control from testio_trx
logic [5:0] ti_fsm_ctrl_mux;
logic [5:0] ti_fsm_ctrl_bit;
logic [5:0] ti_fsm_ctrl_byte;
logic [5:0] ti_fsm_ctrl_half;
logic [5:0] ti_fsm_ctrl_word;

assign ti_rcv_start   = ti_fsm_ctrl_mux[5];
assign ti_rcv_ack     = ti_fsm_ctrl_mux[4];
assign ti_rcv_parity  = ti_fsm_ctrl_mux[3];
assign ti_rcv_stop    = ti_fsm_ctrl_mux[2];
// rcv data done
assign ti_rcv_done    = ti_fsm_ctrl_mux[1];
// send data done
assign ti_send_done   = ti_fsm_ctrl_mux[0];

// assign is_state_change   = (ti_current_state != ti_next_state);
assign is_state_changed  = (ti_current_state != ti_pre_state);
// assign is_counter_reload = is_state_changed;

// declare testio trx control signal 
logic [5:0] ti_ctrl;
logic [5:0] ti_ctrl_bit;
logic [5:0] ti_ctrl_byte;
logic [5:0] ti_ctrl_half;
logic [5:0] ti_ctrl_word;

assign ti_ctrl = {is_state_changed, is_wr_send, is_rd_send, is_rd_rcv, is_send_mode, is_counter_ena};

//state transfer
always_ff @(posedge ti_clk_i or negedge ti_rstn_i) begin
    if(!ti_rstn_i) begin
        ti_pre_state <= TI_IDLE;
    end else begin
        ti_pre_state <= ti_current_state; 
    end
end

always_ff @(posedge ti_clk_i or negedge ti_rstn_i) begin
    if(!ti_rstn_i) begin
        ti_current_state <= TI_IDLE;
    end else begin
        ti_current_state <= ti_next_state; 
    end
end

//state switch
logic [3:0] ti_send_mode;
assign ti_send_mode = ti_type ? TI_WR_SEND : TI_RD_SEND;

always_comb begin
    case(ti_current_state)
        TI_IDLE        : begin
            ti_next_state = ti_valid ? ti_send_mode : ti_current_state;
        end
        TI_WR_SEND     : begin
            ti_next_state = ti_send_done ? TI_WR_RCV_IDLE : ti_current_state;
        end
        TI_WR_RCV_IDLE : begin
            ti_next_state = ti_rcv_start ? TI_WR_RCV_ACK  : ti_current_state;
        end
        TI_WR_RCV_ACK  : begin
            ti_next_state = ti_rcv_ack   ? TI_WR_RCV_PARITY: TI_ACK_ERR;
        end
        TI_WR_RCV_PARITY: begin
            ti_next_state = ti_rcv_parity? TI_WR_RCV      : TI_ACK_ERR;
        end
        TI_WR_RCV      : begin
            ti_next_state = ti_rcv_stop  ? TI_RESP        : ti_current_state;
        end
        TI_RD_SEND     : begin
            ti_next_state = ti_send_done ? TI_RD_RCV_IDLE : ti_current_state;
        end 
        TI_RD_RCV_IDLE : begin 
            ti_next_state = ti_rcv_start ? TI_RD_RCV_ACK  : ti_current_state;
        end
        TI_RD_RCV_ACK  : begin
            ti_next_state = ti_rcv_ack   ? TI_RD_RCV      : TI_ACK_ERR;
        end
        TI_RD_RCV      : begin
            ti_next_state = ti_rcv_done  ? TI_RESP        : ti_current_state;
        end
        TI_RESP        : begin
            ti_next_state = ti_mem_resq_ready  ? TI_IDLE  : ti_current_state;
        end
        TI_ACK_ERR     : begin
            ti_next_state = TI_IDLE;
        end
        default        : ti_next_state = TI_IDLE;
    endcase
end

// mux ti_cycle_count
// assign ti_cycle_count    = ti_cycle_count_0 & ti_cycle_count_1 & ti_cycle_count_2;

// calc resp data
logic [DATA_W-1:0] resp_data_mux;
logic [DATA_W-1:0] resp_data_bit;
logic [DATA_W-1:0] resp_data_byte;
logic [DATA_W-1:0] resp_data_half;
logic [DATA_W-1:0] resp_data_word;

logic [TIO_W-1:0]   ti_send_data_mux;
logic [TI_BIT-1:0]  ti_send_data_bit;
logic [TI_BYTE-1:0] ti_send_data_byte;
logic [TI_HALF-1:0] ti_send_data_half;
logic [TI_WORD-1:0] ti_send_data_word;

logic [TIO_W-1:0]   ti_data_oen_mux;
logic [TI_BIT-1:0]  ti_data_oen_bit;
logic [TI_BYTE-1:0] ti_data_oen_byte;
logic [TI_HALF-1:0] ti_data_oen_half;
logic [TI_WORD-1:0] ti_data_oen_word;

logic [TIO_W-1:0] rff_ti_send_data;
logic [TIO_W-1:0] rff_ti_data_oen;

logic ti_en_bit;
logic ti_en_byte;
logic ti_en_half;
logic ti_en_word;

assign ti_en_bit  = (ti_mod_i == MODE_BIT);
assign ti_en_byte = (ti_mod_i == MODE_BYTE);
assign ti_en_half = (ti_mod_i == MODE_HALF);
assign ti_en_word = (ti_mod_i == MODE_WORD);

assign ti_ctrl_bit  = {6{ti_en_bit}}  & ti_ctrl;
assign ti_ctrl_byte = {6{ti_en_byte}} & ti_ctrl;
assign ti_ctrl_half = {6{ti_en_half}} & ti_ctrl;
assign ti_ctrl_word = {6{ti_en_word}} & ti_ctrl;

testio_trx #
(
  .TI_W(TI_BIT) 
) testio_trx_bit_u
(
    .req_type         (ti_type),      // read or write
    .req_addr         (ti_addr),
    .req_data         (ti_data),
    .req_strb         (ti_strb),
    .resp_data        (resp_data_bit),
    .resp_data_err    (),
   
    .ti_ctrl          (ti_ctrl_bit),   //control signal from state machine
    // testio          
    .ti_clk           (ti_clk_i),     //Peripheral clock signal
    .ti_rstn          (ti_rstn_i),    //Peripheral async reset signal
    .ti_din           (ti_dat_i[TI_BIT-1:0]),   //Peripheral data in

    .ti_dout          (ti_send_data_bit),
    .ti_doen          (ti_data_oen_bit),
    .ti_fsm_ctrl      (ti_fsm_ctrl_bit)
);

testio_trx #
(
  .TI_W(TI_BYTE) 
) testio_trx_byte_u
(
    .req_type         (ti_type),      // read or write
    .req_addr         (ti_addr),
    .req_data         (ti_data),
    .req_strb         (ti_strb),
    .resp_data        (resp_data_byte),
    .resp_data_err    (),

    .ti_ctrl          (ti_ctrl_byte), //control signal from state machine
    // testio          
    .ti_clk           (ti_clk_i),     //Peripheral clock signal
    .ti_rstn          (ti_rstn_i),    //Peripheral async reset signal
    .ti_din           (ti_dat_i[TI_BYTE-1:0]),   //Peripheral data in

    .ti_dout          (ti_send_data_byte),
    .ti_doen          (ti_data_oen_byte),
    .ti_fsm_ctrl      (ti_fsm_ctrl_byte)
);

testio_trx #
(
  .TI_W(TI_HALF) 
) testio_trx_half_u
(
    .req_type         (ti_type),      // read or write
    .req_addr         (ti_addr),
    .req_data         (ti_data),
    .req_strb         (ti_strb),
    .resp_data        (resp_data_half),
    .resp_data_err    (),   

    .ti_ctrl          (ti_ctrl_half), //control signal from state machine
    // testio          
    .ti_clk           (ti_clk_i),     //Peripheral clock signal
    .ti_rstn          (ti_rstn_i),    //Peripheral async reset signal
    .ti_din           (ti_dat_i[TI_HALF-1:0]),   //Peripheral data in

    .ti_dout          (ti_send_data_half),
    .ti_doen          (ti_data_oen_half),
    .ti_fsm_ctrl      (ti_fsm_ctrl_half)
);

testio_trx #
(
  .TI_W(TI_WORD) 
) testio_trx_word_u
(
    .req_type         (ti_type),      // read or write
    .req_addr         (ti_addr),
    .req_data         (ti_data),
    .req_strb         (ti_strb),
    .resp_data        (resp_data_word),
    .resp_data_err    (),   

    .ti_ctrl          (ti_ctrl_word), //control signal from state machine
    // testio          
    .ti_clk           (ti_clk_i),     //Peripheral clock signal
    .ti_rstn          (ti_rstn_i),    //Peripheral async reset signal
    .ti_din           (ti_dat_i[TI_WORD-1:0]),   //Peripheral data in

    .ti_dout          (ti_send_data_word),
    .ti_doen          (ti_data_oen_word),
    .ti_fsm_ctrl      (ti_fsm_ctrl_word)
);

always_ff @(negedge ti_clk_i or negedge ti_rstn_i) begin
    if(!ti_rstn_i) begin
        rff_ti_send_data <= 'h0;
    end else begin
        rff_ti_send_data <= ti_send_data_mux;
    end
end

always_ff @(negedge ti_clk_i or negedge ti_rstn_i) begin
    if(!ti_rstn_i) begin
        rff_ti_data_oen <= 'h0;
    end else begin
        rff_ti_data_oen <= ti_data_oen_mux;
    end
end

assign ti_fsm_ctrl_mux  = ti_mod_i[1] ? (ti_mod_i[0] ? ti_fsm_ctrl_word
                                                     : ti_fsm_ctrl_half)
                                      : (ti_mod_i[0] ? ti_fsm_ctrl_byte
                                                     : ti_fsm_ctrl_bit);

assign ti_send_data_mux = ti_mod_i[1] ? (ti_mod_i[0] ? ti_send_data_word[31:0]
                                                     : {2{ti_send_data_half[15:0]}})
                                      : (ti_mod_i[0] ? {4{ti_send_data_byte[7:0]}}
                                                     : {32{ti_send_data_bit[0]}});                                                 

assign ti_data_oen_mux  = ti_mod_i[1] ? (ti_mod_i[0] ? ti_data_oen_word[31:0]
                                                     : {2{ti_data_oen_half[15:0]}})
                                      : (ti_mod_i[0] ? {4{ti_data_oen_byte[7:0]}}
                                                     : {32{ti_data_oen_bit[0]}}); 

assign resp_data_mux    = ti_mod_i[1] ? (ti_mod_i[0] ? resp_data_word
                                                     : resp_data_half)
                                      : (ti_mod_i[0] ? resp_data_byte
                                                     : resp_data_bit);

// clk always output
// ti interface out
assign ti_clk_oen   = MODE_OUTPUT_WORD[0];
assign ti_clk_o     = ti_clk_i;
assign ti_dat_o     = rff_ti_send_data;
assign ti_dat_oen   = rff_ti_data_oen;

// ready when testio stay in idle, otherwise not ready
assign mem_req_ready = ti_valid && is_idle;

// mem resq
assign mem_resp_valid      = is_resp;
// assign mem_resp.resp_type  = dff_mem_resp.resp_type;
// assign mem_resp.resp_tid   = dff_mem_resp.resp_tid;
assign mem_resp.resp_data  = mem_resp_valid ? resp_data_mux : IDLE_DATA_VALUE;

endmodule

// `default_nettype wire