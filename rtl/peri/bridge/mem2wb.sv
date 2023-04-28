
module mem2wb
    import urv_cfg::*;
    import urv_typedef::*; 
#(
    parameter N_AW = 32,
    parameter N_DW = 32,
    parameter N_DM = N_DW / 8   
) (
    input  logic                    clk,
    input  logic                    rstn,
    //from mem master
    input  logic                    mem_req_valid,
    output logic                    mem_req_ready,
    input  mem_req_t                mem_req,

    output logic                    mem_resp_valid,
    input  logic                    mem_resp_ready,
    output mem_resp_t               mem_resp,
    //wishbone
    output logic                    wb_stb_o,	
    output logic [N_AW-1:0]         wb_addr_o,	 
    output logic                    wb_we_o,         
    output logic [N_DW-1:0]         wb_data_o,	
    output logic [N_DM-1:0]         wb_sel_o,
    output logic                    wb_cyc_o, 
    input  logic                    wb_ack_i,
    input  logic                    wb_err_i, //not used here	
    input  logic [N_DW-1:0]         wb_data_i
   
);
// define state machine state value
localparam WB_IDLE         = 2'b00;
localparam WB_START        = 2'b01;
localparam WB_END          = 2'b10;

logic memif_req_ready;
logic memif_resp_valid;
logic [N_DW-1:0]   memif_resp_data;
cpu_req_type_t memif_resp_type;

logic wb_valid;
logic wb_stb;
logic wb_cyc;
logic wb_we;
logic [N_DW-1:0]   wb_addr;
logic [N_DW-1:0]   wb_data;
logic [N_DM-1:0]   wb_mask;

logic [1:0] wb_current_state, wb_next_state;

logic is_idle, is_wb_start, is_wb_end;

logic is_wb_ack;
logic is_resp_ready;

assign memif_we       = (mem_req.req_type == REQ_WRITE);

// state machine transition
assign is_idle        = (wb_current_state == WB_IDLE);
assign is_wb_start    = (wb_current_state == WB_START);
assign is_wb_end      = (wb_current_state == WB_END);

std_dffr #(2) wb_state_u (
    .clk(clk), 
    .rstn(rstn), 
    .d(wb_next_state), 
    .q(wb_current_state)
);

// state transition condition
assign is_wb_ack     = wb_valid && wb_ack_i;
assign is_resp_ready = mem_resp_ready;

// state transition
always_comb begin
    case(1'b1)
        is_idle        : begin
            wb_next_state = mem_req_valid ? WB_START : wb_current_state;
        end
        is_wb_start    : begin
            wb_next_state = is_wb_ack     ? WB_END   : wb_current_state;
        end
        is_wb_end      : begin
            wb_next_state = is_resp_ready ? WB_IDLE  : wb_current_state;
        end
        default        : wb_next_state = WB_IDLE;
    endcase
end

// state machine event
always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        wb_valid<= 0;
        wb_stb  <= 0;
        wb_cyc  <= 0;
        wb_we   <= 0;
        wb_tid  <= 0;
        wb_addr <= 0;
        wb_data <= 0;
        wb_mask <= 0;
        memif_resp_valid <= 0;
    end else begin 
        case(1'b1)
            is_idle: begin
                if(mem_req_valid) begin
                    wb_valid<= 1'b1;
                    wb_stb  <= 1'b1;
                    wb_cyc  <= 1'b1;
                    wb_we   <= memif_we;
                    wb_addr <= mem_req.req_paddr;
                    wb_data <= mem_req.req_data;
                    wb_mask <= mem_req.req_mask;
                    memif_req_ready <= 1'b1;
                end else begin
                    wb_valid<= 1'b0;
                    wb_stb  <= 1'b0;
                    wb_cyc  <= 1'b0; 
                    wb_we   <= 1'b0; 
                    memif_req_ready <= 1'b0;                  
                end
            end
            is_wb_start: begin
                memif_req_ready <= 1'b0;
                ////////////////////////////
                if(wb_ack_i) begin
                    wb_stb  <= 1'b0;
                    wb_cyc  <= 1'b0;
                    wb_we   <= 1'b0;
                    memif_resp_valid<= 1'b1;
                    memif_resp_type <= wb_we ? REQ_WRITE : REQ_READ;
                    memif_resp_data <= wb_data_i;
                end
            end
            is_wb_end: begin
                if(mem_resp_ready) begin
                    memif_resp_valid<= 1'b0;
                end  
            end
        endcase
    end
end

assign mem_req_ready        = memif_req_ready;
assign mem_resp_valid       = memif_resp_valid;
assign mem_resp.resp_type   = memif_resp_type;
assign mem_resp.resp_data   = memif_resp_data;

assign wb_cyc_o             = wb_cyc;
assign wb_stb_o             = wb_stb;
assign wb_we_o              = wb_we;
assign wb_addr_o            = wb_addr;
assign wb_data_o            = wb_data;
assign wb_sel_o             = wb_mask;

endmodule