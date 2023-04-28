//===============================================
//Name          : dp_trx
//Author        : angrybird
//Email         : 
//Date          : 2022-08-26
//Description   : dp_trx
//                debug port send and receive
//                
//-----------------------------------------------
//===============================================

// `default_nettype none // check net declare
// DP_POSEDGE -> send and rcv at posedge
// Otherwise  -> send at negedge, rcv at posedge

// `define DP_POSEDGE

module dp_trx
#(
    parameter N_DW    = 32,
    parameter N_DM    = 4,
    parameter N_DIO   = 1
)
(  
    input   logic                         req_type,  // read or write
    input   logic    [N_DW-1:0]           req_addr,
    input   logic    [N_DW-1:0]           req_data,
    input   logic    [N_DM-1:0]           req_strb,

    output  logic    [N_DW-1:0]           resp_data,
    output  logic                         resp_data_err,

    // input                              dp_en,     //enable testio trx from testio mode  
    input   logic    [5:0]                dp_ctrl,   //control signal from state machine 
    // testio          
    input   logic                         dp_clk,    //Peripheral clock signal
    input   logic                         dp_rstn,   //Peripheral async reset signal
    input   logic    [N_DIO-1:0]          dp_din,    //Peripheral data in

    output  logic    [N_DIO-1:0]          dp_dout,
    output  logic    [N_DIO-1:0]          dp_doen,
    output  logic    [5:0]                dp_fsm_ctrl
); 
localparam N_CNT_BITS       = 8;
localparam DP_START_CYCLE   = 1;
localparam DP_TYPE_CYCLE    = 1;  // read or write
localparam DP_PARITY_CYCLE  = 1; 
localparam DP_STOP_CYCLE    = 1; 
localparam DP_ACK_CYCLE     = 1;

localparam DP_ADDR_CYCLE    = N_DW / N_DIO; 
localparam DP_STRB_CYCLE    = (N_DIO == 1) ? N_DM / N_DIO : 1; 
localparam DP_DATA_CYCLE    = N_DW / N_DIO; 

localparam DP_HOST_CYC_COM  = DP_START_CYCLE  + DP_TYPE_CYCLE      + DP_PARITY_CYCLE    + DP_STOP_CYCLE - 1'b1;
localparam DP_WR_ACK_CYC    = DP_START_CYCLE  + DP_ACK_CYCLE       + DP_STOP_CYCLE - 1'b1;
localparam DP_RD_ACK_CYC    = DP_PARITY_CYCLE + DP_STOP_CYCLE - 1'b1;

localparam DP_WR_HOST_CYC   = DP_HOST_CYC_COM + DP_ADDR_CYCLE  + DP_STRB_CYCLE  + DP_DATA_CYCLE;

localparam DP_RD_HOST_CYC   = DP_HOST_CYC_COM + DP_ADDR_CYCLE;

localparam DP_RD_TARG_CYC   = DP_RD_ACK_CYC   + DP_DATA_CYCLE;

// max bit width such as 32bit mode
localparam DP_SEND_BUF_W    = DP_WR_HOST_CYC + 1;
localparam DP_RCV_BUF_W     = DP_RD_TARG_CYC + 1;
localparam DP_MASK_W = (N_DIO == 1) ? N_DM : N_DIO;

// declare cmd bit
localparam bit START        = 1'b0; 
localparam bit STOP         = 1'b1; 
localparam bit WRITE        = 1'b1; 
localparam bit READ         = 1'b0; 
localparam bit ACK_OK       = 1'b0;
localparam bit ACK_ERR      = 1'b1;
localparam bit PARITY_OK    = 1'b0; // only for wr ack check
localparam bit PARITY_ERR   = 1'b1; // only for wr ack check

localparam MODE_INPUT_WORD  = 'hffff_ffff;

localparam MODE_OUTPUT_WORD = 'h0;

localparam IDLE_DATA_VALUE  = 'hffff_ffff;

// send/receive cycle numbers
logic [N_CNT_BITS-1:0] dp_wr_host_cycle;
logic [N_CNT_BITS-1:0] dp_rd_host_cycle;
logic [N_CNT_BITS-1:0] dp_rd_targ_cycle;
logic [DP_SEND_BUF_W-1:0][N_DIO-1:0] dp_wr_send_data_mux;

// mux dp_cycle_count
logic [N_CNT_BITS-1:0] dp_cycle_count;
logic [N_CNT_BITS-1:0] dp_cycle_counter;

//send/receive data buf
logic [DP_SEND_BUF_W-1:0][N_DIO-1:0] dp_send_buf;
logic [N_DIO-1:0] dp_data_parity;

logic [DP_RCV_BUF_W-1:0][N_DIO-1:0] dp_rcv_buf;
logic [N_DIO-1:0] dp_rcv_data_parity;

logic [N_DM-1:0] dp_strb;
logic [N_DW-1:0] dp_addr;
logic [N_DW-1:0] dp_data;

logic is_wr_send;
logic is_rd_send;
logic is_rd_rcv ;

// control signal
logic is_send_mode;
logic is_counter_ena;

// state change : update cycle counter value
logic is_state_changed;
logic is_counter_reload;
logic is_cycle_count_eq_zero;
logic is_cycle_count_eq_one;

logic dp_send_done;
logic dp_rcv_start;
logic dp_rcv_ack;
logic dp_rcv_parity;
logic dp_rcv_stop;
logic dp_rcv_done;

// check dp_cycle_counter
assign is_cycle_count_eq_zero = (dp_cycle_counter == 8'h0);
assign is_cycle_count_eq_one  = (dp_cycle_counter == 8'h1);

// always check dp_din for dp_rcv_start and dp_rcv_ack
assign dp_rcv_start   = (dp_din[0]    == START);
assign dp_rcv_ack     = (dp_din[0]    == ACK_OK);
assign dp_rcv_parity  = (dp_din       == {N_DIO{PARITY_OK}});
assign dp_rcv_stop    = (dp_din[0]    == STOP);
// rcv data done
assign dp_rcv_done    = is_cycle_count_eq_zero;
// send data done
assign dp_send_done   = is_cycle_count_eq_zero;

// prepere req data
assign dp_addr  = req_addr;
assign dp_data  = req_data;
assign dp_strb  = req_strb;

assign dp_fsm_ctrl[5] = dp_rcv_start;
assign dp_fsm_ctrl[4] = dp_rcv_ack;
assign dp_fsm_ctrl[3] = dp_rcv_parity;
assign dp_fsm_ctrl[2] = dp_rcv_stop;
assign dp_fsm_ctrl[1] = dp_rcv_done;
assign dp_fsm_ctrl[0] = dp_send_done;

// data width handle
logic [DP_SEND_BUF_W-1:0][N_DIO-1:0] dp_wr_send_data;
logic [DP_SEND_BUF_W-1:0][N_DIO-1:0] dp_rd_send_data;

logic [N_DIO-1:0]      dp_wr_send_parity_w;
logic [N_DIO-1:0]      dp_start_w, dp_write_w, dp_read_w, dp_stop_w;
logic [N_DW-1:0]       dp_addr_w;
logic [N_DW-1:0]       dp_data_w;
logic [DP_MASK_W-1:0]  dp_strb_w;

assign dp_start_w = {N_DIO{START}};
assign dp_write_w = {N_DIO{WRITE}};
assign dp_read_w  = {N_DIO{READ}};
assign dp_stop_w  = {N_DIO{STOP}};
assign dp_addr_w  = dp_addr;
assign dp_data_w  = dp_data;
assign dp_strb_w  = {{(DP_MASK_W-N_DM){1'b0}},dp_strb};
assign dp_wr_send_parity_w = {N_DIO{PARITY_OK}};
assign dp_wr_send_data    = {dp_start_w, dp_write_w, dp_addr_w, dp_strb_w, dp_data_w, dp_wr_send_parity_w, dp_stop_w};
assign dp_rd_send_data    = {dp_start_w, dp_read_w, dp_addr_w, dp_wr_send_parity_w, dp_stop_w, {((DP_STRB_CYCLE+DP_DATA_CYCLE)*N_DIO){1'b1}}};

assign dp_wr_host_cycle = DP_WR_HOST_CYC[N_CNT_BITS-1:0];
assign dp_rd_host_cycle = DP_RD_HOST_CYC[N_CNT_BITS-1:0];
assign dp_rd_targ_cycle = DP_RD_TARG_CYC[N_CNT_BITS-1:0];

assign dp_wr_send_data_mux  = is_wr_send ? dp_wr_send_data : dp_rd_send_data;

// control signal
assign is_state_changed = dp_ctrl[5];
assign is_wr_send       = dp_ctrl[4];
assign is_rd_send       = dp_ctrl[3];
assign is_rd_rcv        = dp_ctrl[2];
assign is_send_mode     = dp_ctrl[1];
assign is_counter_ena   = dp_ctrl[0];

assign is_counter_reload = is_state_changed;

// mux dp_cycle_count
assign dp_cycle_count    = is_wr_send ? dp_wr_host_cycle
                         : is_rd_send ? dp_rd_host_cycle
                         : is_rd_rcv  ? dp_rd_targ_cycle
                         : 8'hff;

// cycle counter
always_ff @(posedge dp_clk or negedge dp_rstn) begin
    if(!dp_rstn) begin
        dp_cycle_counter <= 8'hff;
    end else if(is_counter_reload) begin
        dp_cycle_counter <= dp_cycle_count;
    end else if(is_counter_ena) begin
        dp_cycle_counter <= dp_cycle_counter - 1'b1;
    end
end

logic [N_DIO-1:0] rff_dp_rcv_data;
logic [N_DIO-1:0] rff_dp_data_parity;
stdffr #(N_DIO) dp_rcv_data_u (
    .clk(dp_clk), 
    .rstn(dp_rstn), 
    .d(dp_din), 
    .q(rff_dp_rcv_data)
);

//shift send_buf
always_ff @(negedge dp_clk or negedge dp_rstn) begin
    if(!dp_rstn) begin
        dp_send_buf  <= 'h0;
    end else if(is_state_changed) begin
        for (int i = 0; i < DP_SEND_BUF_W; i++) begin
            if(is_send_mode) begin
                dp_send_buf[i]  <= dp_wr_send_data_mux[DP_SEND_BUF_W-1-i];
            end
        end
    end else if(is_send_mode) begin
        dp_send_buf  <= {{1{1'b0}}, dp_send_buf[DP_SEND_BUF_W-1:1]}; //dp_send_buf <= dp_send_buf >> N_DIO
    end
end

//dp_send_data_parity calc
always_ff @(negedge dp_clk or negedge dp_rstn) begin
    if(!dp_rstn) begin
        dp_data_parity   <= 'h0;
    end else if(is_state_changed) begin
        dp_data_parity   <= 'h0;
    end else if(is_send_mode) begin
        dp_data_parity   <= dp_data_parity ^ dp_send_buf[0];
    end
end

//shift rcv_buf
always_ff @(posedge dp_clk or negedge dp_rstn) begin
    if(!dp_rstn) begin
        dp_rcv_buf           <= 'h0;
    end else if(is_state_changed) begin
        dp_rcv_buf           <= dp_rcv_buf;
    end else if(is_rd_rcv) begin
        dp_rcv_buf           <= {dp_rcv_buf[DP_RCV_BUF_W-1-1:0], rff_dp_rcv_data};
    end
end

//dp_rcv_data_parity calc
always_ff @(posedge dp_clk or negedge dp_rstn) begin
    if(!dp_rstn) begin
        dp_rcv_data_parity   <= 'h0;
    end else if(is_state_changed) begin
        dp_rcv_data_parity   <= 'h0;
    end else if(is_rd_rcv) begin
        dp_rcv_data_parity   <= dp_rcv_data_parity ^ rff_dp_rcv_data;
    end
end

// for store rcv data parity 
logic [N_DIO-1:0] rff_dp_rcv_data_parity;
stdffre #(N_DIO) dp_rcv_data_parity_u (
    .clk(dp_clk), 
    .rstn(dp_rstn), 
    .en(is_cycle_count_eq_one),
    .d(dp_rcv_data_parity), 
    .q(rff_dp_rcv_data_parity)
);

//------------------------
// configure dp_data_oen
// send mdoe : output
// otherwise : input
//------------------------
logic [N_DIO-1:0] dp_data_oen;
always_ff @(negedge dp_clk or negedge dp_rstn) begin
    if(!dp_rstn) begin
        dp_data_oen <= MODE_INPUT_WORD[N_DIO-1:0];
    end else begin
        dp_data_oen <= (is_send_mode && ~dp_send_done) ? MODE_OUTPUT_WORD[N_DIO-1:0] : MODE_INPUT_WORD[N_DIO-1:0];
    end
end

logic [N_DIO-1:0] dp_send_data;

// send data
assign dp_send_data = (is_send_mode && ~is_state_changed) ? (is_cycle_count_eq_one ? dp_data_parity : dp_send_buf[0]) : IDLE_DATA_VALUE[N_DIO-1:0];

assign dp_dout   = dp_send_data;
assign dp_doen   = dp_data_oen;

// calc resp data
assign resp_data = dp_rcv_buf[N_DW/N_DIO + 2 - 1 : 2];
assign resp_data_err = ~(rff_dp_rcv_data_parity == dp_rcv_buf[1]) && ~is_rd_rcv;
endmodule

// `default_nettype wire