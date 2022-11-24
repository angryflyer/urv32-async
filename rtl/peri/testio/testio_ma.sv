//===============================================
//Name          : testio ma
//Author        : zhaowei
//Email         : 
//Date          : 2022-08-11
//Description   : testio_ma to mem nic
//                only 2 lines
//-----------------------------------------------
//===============================================

module testio_ma
    import urv_cfg::*;
    import urv_typedef::*;
(  
    //testio          
    // input       logic                  clk_i,    //Peripheral clock signal
    input       logic                  rstn_i,   //Peripheral async reset signal

    output      logic                  test_intr,

    // IO Ports
    input       logic                  test_clk,
    input       logic                  test_din,
    output      logic                  test_dout,
    output      logic                  test_doen,

    //mem interface
    output      logic                  mem_req_valid,
    input       logic                  mem_req_ready,
    output      mem_req_t              mem_req,

    input       logic                  mem_resp_valid,
    output      logic                  mem_resp_ready,
    input       mem_resp_t             mem_resp 
);

assign test_intr         = 1'b0;

localparam int                  N_CNT_BITS   = 7;
localparam bit [N_CNT_BITS-1:0] N_CMD_BITS   = 1;
// localparam bit [N_CNT_BITS-1:0] N_ADDR_BITS  = 40;
// localparam bit [N_CNT_BITS-1:0] N_STRB_BITS  = 8;
// localparam bit [N_CNT_BITS-1:0] N_DATA_BITS  = 64;
localparam bit [N_CNT_BITS-1:0] N_ADDR_BITS  = 32;
localparam bit [N_CNT_BITS-1:0] N_STRB_BITS  = 4;
localparam bit [N_CNT_BITS-1:0] N_DATA_BITS  = 32;
localparam bit [N_CNT_BITS-1:0] N_PRTY_BITS  = 1;
localparam bit [N_CNT_BITS-1:0] N_ACK_BITS   = 1;
localparam bit [N_CNT_BITS-1:0] N_CMDBUF_BITS = N_CMD_BITS + N_ADDR_BITS + N_DATA_BITS + N_STRB_BITS + N_PRTY_BITS;

localparam bit [N_CMD_BITS-1:0] CMD_RD       = 0;
localparam bit [N_CMD_BITS-1:0] CMD_WR       = 1;
localparam bit                  START        = 0;
localparam bit                  STOP         = 1;
localparam bit [N_ACK_BITS-1:0] ACK_OK       = {N_ACK_BITS{1'b0}};
localparam bit [N_ACK_BITS-1:0] ACK_ERR      = {N_ACK_BITS{1'b1}};

logic [N_DATA_BITS-1:0] resp_data;

enum logic [2:0] {
    CDC_IDLE,
    CDC_CMD,
    CDC_RREQ,
    CDC_RRSP,
    CDC_WREQ,
    CDC_WRSP
} rff_cdcst, next_cdcst;

logic test_rstn;
assign test_rstn = rstn_i;

logic dff_test_din;
always_ff @(posedge test_clk) begin
    dff_test_din <= test_din;
end

logic test_din_en;
logic dff_test_din_en;
logic test_din_neg;

logic [N_CNT_BITS-1:0] rff_cmd_idx_max;
logic [N_CNT_BITS-1:0] rff_cmd_idx;

assign test_din_en  = |rff_cmd_idx;
assign test_din_neg = (dff_test_din == 1'b1) && (test_din == 1'b0);

always_ff @(posedge test_clk or negedge test_rstn) begin
if (test_rstn == 1'b0) 
    dff_test_din_en <= 1'd0;
else 
    dff_test_din_en <= test_din_en;
end

logic [N_CMD_BITS-1:0]    cmd;
always_comb begin
if (cmd==CMD_WR)
    rff_cmd_idx_max = 1 + N_ADDR_BITS + N_STRB_BITS + N_DATA_BITS + N_PRTY_BITS;  
else if (cmd==CMD_RD)
    rff_cmd_idx_max = 1 + N_ADDR_BITS + N_PRTY_BITS;  
else 
    rff_cmd_idx_max = {N_CNT_BITS{1'b1}};
end

always_ff @(posedge test_clk or negedge test_rstn) begin
if (test_rstn == 1'b0)
    rff_cmd_idx <= {N_CNT_BITS{1'b0}};
else if (rff_cmd_idx>=rff_cmd_idx_max)
    rff_cmd_idx <= {N_CNT_BITS{1'b0}};
else if (test_din_en)
    rff_cmd_idx <= rff_cmd_idx + 1'b1;
else if (test_doen && test_din_neg)
    rff_cmd_idx <= rff_cmd_idx + 1'b1;
else 
    rff_cmd_idx <= {N_CNT_BITS{1'b0}};
end

logic [N_CMDBUF_BITS:1] dff_cmdbuf;
always_ff @(posedge test_clk) begin
if (test_din_en) begin
    dff_cmdbuf[rff_cmd_idx] <= test_din;
end
end

logic [N_ADDR_BITS-1:0]   addr;
logic [N_STRB_BITS-1:0]   wstrb;
logic [N_DATA_BITS-1:0]   wdata;
logic [N_PRTY_BITS-1:0]   prty;

generate
for (genvar i = 0; i < N_CMD_BITS; i++) begin : CMD_REORDER_GEN
    assign cmd[i]   = dff_cmdbuf[N_CMD_BITS - i];
end
for (genvar i = 0; i < N_ADDR_BITS; i++) begin : ADDR_REORDER_GEN
    assign addr[i]  = dff_cmdbuf[N_CMD_BITS + N_ADDR_BITS - i];
end
for (genvar i = 0; i < N_STRB_BITS; i++) begin : STRB_REORDER_GEN
    assign wstrb[i] = dff_cmdbuf[N_CMD_BITS + N_ADDR_BITS + N_STRB_BITS - i];
end
for (genvar i = 0; i < N_DATA_BITS; i++) begin : WDATA_REORDER_GEN
    assign wdata[i] = dff_cmdbuf[N_CMD_BITS + N_ADDR_BITS + N_STRB_BITS + N_DATA_BITS - i];
end
for (genvar i = 0; i < N_PRTY_BITS; i++) begin : PRTY_REORDER_GEN
    assign prty[i]  = (cmd==CMD_WR) ?  dff_cmdbuf[N_CMD_BITS + N_ADDR_BITS + N_STRB_BITS + N_DATA_BITS + N_PRTY_BITS - i] :
                                                dff_cmdbuf[N_CMD_BITS + N_ADDR_BITS + N_PRTY_BITS - i];
end
endgenerate


logic prty_wreq_exp;
logic prty_rreq_exp;
logic [N_ACK_BITS-1:0] ack;

assign prty_wreq_exp = (^{CMD_WR,addr,wstrb,wdata});
assign prty_rreq_exp = (^{CMD_RD,addr});

always_ff @(posedge test_clk) begin
if (cmd==CMD_WR) 
    ack <= (prty==prty_wreq_exp) ? ACK_OK : ACK_ERR;
else if (cmd==CMD_RD)
    ack <= (prty==prty_rreq_exp) ? ACK_OK : ACK_ERR;
else 
    ack <= ack;
end


logic [N_PRTY_BITS-1:0] rprty;
logic [N_PRTY_BITS-1:0] wprty;
assign rprty = (^{ack,resp_data});
assign wprty = (^{ack                   });


logic [1+N_ACK_BITS+N_DATA_BITS+N_PRTY_BITS+1-1:0] dff_payload;
logic [N_CNT_BITS-1:0]                             rff_payload_idx;
logic                                              rff_payload_in_progress;

always_ff @(posedge test_clk) begin
if (cmd==CMD_RD) begin
    if (mem_resp_valid & mem_resp_ready) 
        dff_payload <= {START, ack, resp_data, rprty, STOP};
    else /*if (rff_payload_idx == (1+N_ACK_BITS+N_DATA_BITS+N_PRTY_BITS+1-1))*/
        dff_payload <= dff_payload;
end
else if (cmd==CMD_WR) begin
    if ((rff_cdcst == CDC_WRSP) && (mem_resp_valid == 1'b1)) 
        dff_payload <= {START, ack, wprty, STOP, 32'hFFFF_FFFF};
    else /*if (rff_payload_idx == (1+N_ACK_BITS+1-1))*/
        dff_payload <= dff_payload;
end
else begin
    dff_payload <= dff_payload;
end
end

always_ff @(posedge test_clk or negedge test_rstn) begin
if (test_rstn == 1'b0) 
    rff_payload_idx <= {N_CNT_BITS{1'b0}};
else case (cmd)
    CMD_RD: begin
    if (mem_resp_valid & mem_resp_ready) 
        rff_payload_idx <= {N_CNT_BITS{1'b0}};
    else if (rff_payload_in_progress)
        rff_payload_idx <= rff_payload_idx + 1'b1;
    end
    CMD_WR: begin
    if ((rff_cdcst == CDC_WRSP) && (mem_resp_valid == 1'b1)) 
        rff_payload_idx <= {N_CNT_BITS{1'b0}};
    else if (rff_payload_in_progress) 
        rff_payload_idx <= rff_payload_idx + 1'b1;
    end
endcase 
end

always_ff @(posedge test_clk or negedge test_rstn) begin
if (test_rstn == 1'b0) 
    rff_payload_in_progress <= 1'b0;
else case (cmd)
    CMD_RD: begin
    if (mem_resp_valid & mem_resp_ready) 
        rff_payload_in_progress <= 1'b1;
    else if (rff_payload_idx == (1+N_ACK_BITS+N_DATA_BITS+N_PRTY_BITS+1-1)) 
        rff_payload_in_progress <= 1'b0;
    end
    CMD_WR: begin
    if ((rff_cdcst == CDC_WRSP) && (mem_resp_valid == 1'b1)) 
        rff_payload_in_progress <= 1'b1;
    else if (rff_payload_idx == (1+N_ACK_BITS+N_PRTY_BITS+1-1)) 
        rff_payload_in_progress <= 1'b0;
    end
endcase 
end



always_ff @(posedge test_clk or negedge test_rstn) begin
if (test_rstn == 1'b0) begin
    rff_cdcst <= CDC_IDLE;
end else begin
    rff_cdcst <= next_cdcst;
end
end

always_comb begin
next_cdcst = rff_cdcst;
case (rff_cdcst)
    CDC_IDLE: begin
        if (dff_test_din_en && !test_din_en) begin
            next_cdcst = CDC_CMD;
        end
    end
    CDC_CMD: begin
        case (cmd)
            CMD_RD: begin
            next_cdcst = CDC_RREQ;
            end
            CMD_WR: begin
            next_cdcst = CDC_WREQ;
            end
            default: begin
            next_cdcst = CDC_IDLE;
            end
        endcase
    end
    CDC_RREQ: begin
        if (mem_req_ready) begin
            next_cdcst = CDC_RRSP;
        end
    end
    CDC_RRSP: begin
        if (mem_resp_valid) begin
            next_cdcst = CDC_IDLE;
        end
    end
    CDC_WREQ: begin
        if (mem_req_ready) begin
            next_cdcst = CDC_WRSP;
        end
    end
    CDC_WRSP: begin
        if (mem_resp_valid) begin
            next_cdcst = CDC_IDLE;
        end
    end
endcase
end

always_comb begin
test_doen = 1'b1;
// default output high
test_dout = 1'b1;
if (rff_payload_in_progress == 1'b1) begin
    test_doen = 1'b0;
    test_dout = dff_payload[1 + N_ACK_BITS + N_DATA_BITS + N_PRTY_BITS + 1 - 1 - rff_payload_idx];
end
end

assign mem_req_valid     = (rff_cdcst == CDC_WREQ) | (rff_cdcst == CDC_RREQ);
assign mem_req.req_type  = cmd ? MEM_WRITE : MEM_READ;
// assign mem_req.req_tid   = 'h0;
assign mem_req.req_addr  = addr;
assign mem_req.req_mask  = wstrb;
assign mem_req.req_data  = wdata;

assign mem_resp_ready    = (rff_cdcst == CDC_WRSP) | (rff_cdcst == CDC_RRSP);
assign resp_data         = mem_resp.resp_data;

endmodule