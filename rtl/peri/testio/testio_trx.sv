//===============================================
//Name          : testio_trx
//Author        : zhaowei
//Email         : 
//Date          : 2022-08-26
//Description   : testio_trx
//                testio send and receive
//                width can be configured
//-----------------------------------------------
//===============================================

// `default_nettype none // check net declare
// TI_POSEDGE -> send and rcv at posedge
// Otherwise  -> send at negedge, rcv at posedge

// `define TI_POSEDGE

module testio_trx
#(
    parameter DATA_W = 32,
    parameter MASK_W = 4,
    parameter TI_W   = 1
)
(  
    input   logic                           req_type,  // read or write
    input   logic    [DATA_W-1:0]           req_addr,
    input   logic    [DATA_W-1:0]           req_data,
    input   logic    [MASK_W-1:0]           req_strb,

    output  logic    [DATA_W-1:0]           resp_data,
    output  logic                           resp_data_err,

    // input                              ti_en,     //enable testio trx from testio mode  
    input   logic    [5:0]                  ti_ctrl,   //control signal from state machine 
    // testio          
    input   logic                           ti_clk,    //Peripheral clock signal
    input   logic                           ti_rstn,   //Peripheral async reset signal
    input   logic    [TI_W-1:0]             ti_din,    //Peripheral data in

    output  logic    [TI_W-1:0]             ti_dout,
    output  logic    [TI_W-1:0]             ti_doen,
    output  logic    [5:0]                  ti_fsm_ctrl
); 
localparam N_CNT_BITS       = 8;
localparam TI_START_CYCLE   = 1;
localparam TI_TYPE_CYCLE    = 1;  // read or write
localparam TI_PARITY_CYCLE  = 1; 
localparam TI_STOP_CYCLE    = 1; 
localparam TI_ACK_CYCLE     = 1;

localparam TI_ADDR_CYCLE    = DATA_W / TI_W; 
localparam TI_STRB_CYCLE    = (TI_W == 1) ? MASK_W / TI_W : 1; 
localparam TI_DATA_CYCLE    = DATA_W / TI_W; 

localparam TI_HOST_CYC_COM  = TI_START_CYCLE  + TI_TYPE_CYCLE      + TI_PARITY_CYCLE    + TI_STOP_CYCLE - 1'b1;
localparam TI_WR_ACK_CYC    = TI_START_CYCLE  + TI_ACK_CYCLE       + TI_STOP_CYCLE - 1'b1;
localparam TI_RD_ACK_CYC    = TI_PARITY_CYCLE + TI_STOP_CYCLE - 1'b1;

localparam TI_WR_HOST_CYC   = TI_HOST_CYC_COM + TI_ADDR_CYCLE  + TI_STRB_CYCLE  + TI_DATA_CYCLE;

localparam TI_RD_HOST_CYC   = TI_HOST_CYC_COM + TI_ADDR_CYCLE;

localparam TI_RD_TARG_CYC   = TI_RD_ACK_CYC   + TI_DATA_CYCLE;

// max bit width such as 32bit mode
localparam TI_SEND_BUF_W    = TI_WR_HOST_CYC + 1;
localparam TI_RCV_BUF_W     = TI_RD_TARG_CYC + 1;
localparam TI_MASK_W = (TI_W == 1) ? MASK_W : TI_W;

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
logic [N_CNT_BITS-1:0] ti_wr_host_cycle;
logic [N_CNT_BITS-1:0] ti_rd_host_cycle;
logic [N_CNT_BITS-1:0] ti_rd_targ_cycle;
logic [TI_SEND_BUF_W-1:0][TI_W-1:0] ti_wr_send_data_mux;

// mux ti_cycle_count
logic [N_CNT_BITS-1:0] ti_cycle_count;
logic [N_CNT_BITS-1:0] ti_cycle_counter;

//send/receive data buf
logic [TI_SEND_BUF_W-1:0][TI_W-1:0] ti_send_buf;
logic [TI_W-1:0] ti_data_parity;

logic [TI_RCV_BUF_W-1:0][TI_W-1:0] ti_rcv_buf;
logic [TI_W-1:0] ti_rcv_data_parity;

logic [MASK_W-1:0] ti_strb;
logic [DATA_W-1:0] ti_addr;
logic [DATA_W-1:0] ti_data;

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

logic ti_send_done;
logic ti_rcv_start;
logic ti_rcv_ack;
logic ti_rcv_parity;
logic ti_rcv_stop;
logic ti_rcv_done;

// check ti_cycle_counter
assign is_cycle_count_eq_zero = (ti_cycle_counter == 8'h0);
assign is_cycle_count_eq_one  = (ti_cycle_counter == 8'h1);

// always check ti_din for ti_rcv_start and ti_rcv_ack
assign ti_rcv_start   = (ti_din[0]    == START);
assign ti_rcv_ack     = (ti_din[0]    == ACK_OK);
assign ti_rcv_parity  = (ti_din       == {TI_W{PARITY_OK}});
assign ti_rcv_stop    = (ti_din[0]    == STOP);
// rcv data done
assign ti_rcv_done    = is_cycle_count_eq_zero;
// send data done
assign ti_send_done   = is_cycle_count_eq_zero;

// prepere req data
assign ti_addr  = req_addr;
assign ti_data  = req_data;
assign ti_strb  = req_strb;

assign ti_fsm_ctrl[5] = ti_rcv_start;
assign ti_fsm_ctrl[4] = ti_rcv_ack;
assign ti_fsm_ctrl[3] = ti_rcv_parity;
assign ti_fsm_ctrl[2] = ti_rcv_stop;
assign ti_fsm_ctrl[1] = ti_rcv_done;
assign ti_fsm_ctrl[0] = ti_send_done;

// data width handle
logic [TI_SEND_BUF_W-1:0][TI_W-1:0] ti_wr_send_data;
logic [TI_SEND_BUF_W-1:0][TI_W-1:0] ti_rd_send_data;

logic [TI_W-1:0]      ti_wr_send_parity_w;
logic [TI_W-1:0]      ti_start_w, ti_write_w, ti_read_w, ti_stop_w;
logic [DATA_W-1:0]    ti_addr_w;
logic [DATA_W-1:0]    ti_data_w;
logic [TI_MASK_W-1:0] ti_strb_w;

assign ti_start_w = {TI_W{START}};
assign ti_write_w = {TI_W{WRITE}};
assign ti_read_w  = {TI_W{READ}};
assign ti_stop_w  = {TI_W{STOP}};
assign ti_addr_w  = ti_addr;
assign ti_data_w  = ti_data;
assign ti_strb_w  = {{(TI_MASK_W-MASK_W){1'b0}},ti_strb};
assign ti_wr_send_parity_w = {TI_W{PARITY_OK}};
assign ti_wr_send_data    = {ti_start_w, ti_write_w, ti_addr_w, ti_strb_w, ti_data_w, ti_wr_send_parity_w, ti_stop_w};
assign ti_rd_send_data    = {ti_start_w, ti_read_w, ti_addr_w, ti_wr_send_parity_w, ti_stop_w, {((TI_STRB_CYCLE+TI_DATA_CYCLE)*TI_W){1'b1}}};

assign ti_wr_host_cycle = TI_WR_HOST_CYC[N_CNT_BITS-1:0];
assign ti_rd_host_cycle = TI_RD_HOST_CYC[N_CNT_BITS-1:0];
assign ti_rd_targ_cycle = TI_RD_TARG_CYC[N_CNT_BITS-1:0];

assign ti_wr_send_data_mux  = is_wr_send ? ti_wr_send_data : ti_rd_send_data;

// control signal
assign is_state_changed = ti_ctrl[5];
assign is_wr_send       = ti_ctrl[4];
assign is_rd_send       = ti_ctrl[3];
assign is_rd_rcv        = ti_ctrl[2];
assign is_send_mode     = ti_ctrl[1];
assign is_counter_ena   = ti_ctrl[0];

assign is_counter_reload = is_state_changed;

// mux ti_cycle_count
assign ti_cycle_count    = is_wr_send ? ti_wr_host_cycle
                         : is_rd_send ? ti_rd_host_cycle
                         : is_rd_rcv  ? ti_rd_targ_cycle
                         : 8'hff;

// cycle counter
always_ff @(posedge ti_clk or negedge ti_rstn) begin
    if(!ti_rstn) begin
        ti_cycle_counter <= 8'hff;
    end else if(is_counter_reload) begin
        ti_cycle_counter <= ti_cycle_count;
    end else if(is_counter_ena) begin
        ti_cycle_counter <= ti_cycle_counter - 1'b1;
    end
end

logic [TI_W-1:0] rff_ti_rcv_data;
logic [TI_W-1:0] rff_ti_data_parity;
stdffr #(TI_W) ti_rcv_data_u (
    .clk(ti_clk), 
    .rstn(ti_rstn), 
    .d(ti_din), 
    .q(rff_ti_rcv_data)
);

//shift send_buf
always_ff @(negedge ti_clk or negedge ti_rstn) begin
    if(!ti_rstn) begin
        ti_send_buf  <= 'h0;
    end else if(is_state_changed) begin
        for (int i = 0; i < TI_SEND_BUF_W; i++) begin
            if(is_send_mode) begin
                ti_send_buf[i]  <= ti_wr_send_data_mux[TI_SEND_BUF_W-1-i];
            end
        end
    end else if(is_send_mode) begin
        ti_send_buf  <= {{1{1'b0}}, ti_send_buf[TI_SEND_BUF_W-1:1]}; //ti_send_buf <= ti_send_buf >> TI_W
    end
end

//ti_send_data_parity calc
always_ff @(negedge ti_clk or negedge ti_rstn) begin
    if(!ti_rstn) begin
        ti_data_parity   <= 'h0;
    end else if(is_state_changed) begin
        ti_data_parity   <= 'h0;
    end else if(is_send_mode) begin
        ti_data_parity   <= ti_data_parity ^ ti_send_buf[0];
    end
end

//shift rcv_buf
always_ff @(posedge ti_clk or negedge ti_rstn) begin
    if(!ti_rstn) begin
        ti_rcv_buf           <= 'h0;
    end else if(is_state_changed) begin
        ti_rcv_buf           <= ti_rcv_buf;
    end else if(is_rd_rcv) begin
        ti_rcv_buf           <= {ti_rcv_buf[TI_RCV_BUF_W-1-1:0], rff_ti_rcv_data};
    end
end

//ti_rcv_data_parity calc
always_ff @(posedge ti_clk or negedge ti_rstn) begin
    if(!ti_rstn) begin
        ti_rcv_data_parity   <= 'h0;
    end else if(is_state_changed) begin
        ti_rcv_data_parity   <= 'h0;
    end else if(is_rd_rcv) begin
        ti_rcv_data_parity   <= ti_rcv_data_parity ^ rff_ti_rcv_data;
    end
end

// for store rcv data parity 
logic [TI_W-1:0] rff_ti_rcv_data_parity;
stdffre #(TI_W) ti_rcv_data_parity_u (
    .clk(ti_clk), 
    .rstn(ti_rstn), 
    .en(is_cycle_count_eq_one),
    .d(ti_rcv_data_parity), 
    .q(rff_ti_rcv_data_parity)
);

//------------------------
// configure ti_data_oen
// send mdoe : output
// otherwise : input
//------------------------
logic [TI_W-1:0] ti_data_oen;
always_ff @(negedge ti_clk or negedge ti_rstn) begin
    if(!ti_rstn) begin
        ti_data_oen <= MODE_INPUT_WORD[TI_W-1:0];
    end else begin
        ti_data_oen <= (is_send_mode && ~ti_send_done) ? MODE_OUTPUT_WORD[TI_W-1:0] : MODE_INPUT_WORD[TI_W-1:0];
    end
end

logic [TI_W-1:0] ti_send_data;

// send data
assign ti_send_data = (is_send_mode && ~is_state_changed) ? (is_cycle_count_eq_one ? ti_data_parity : ti_send_buf[0]) : IDLE_DATA_VALUE[TI_W-1:0];

assign ti_dout   = ti_send_data;
assign ti_doen   = ti_data_oen;

// calc resp data
assign resp_data = ti_rcv_buf[DATA_W/TI_W + 2 - 1 : 2];
assign resp_data_err = ~(rff_ti_rcv_data_parity == ti_rcv_buf[1]) && ~is_rd_rcv;
endmodule

// `default_nettype wire