//===============================================
//Name          : sdp2mem
//Author        : angrybird
//Email         : 
//Date          : 2022-08-11
//Description   : serial debug port to mem
//                serial debug interface
//-----------------------------------------------
//===============================================

module sdp2mem
    import urv_cfg::*;
    import urv_typedef::*;
(  
    output      logic                  sdp_irq,

    // dp Ports
    input       logic                  sdp_ck,
    input       logic                  sdp_di,
    output      logic                  sdp_do,
    output      logic                  sdp_doen,
    input       logic                  sdp_rstn,

    // mem interface
    output      logic                  mem_req_valid,
    input       logic                  mem_req_ready,
    output      mem_req_t              mem_req,

    input       logic                  mem_resp_valid,
    output      logic                  mem_resp_ready,
    input       mem_resp_t             mem_resp 
);

assign sdp_irq         = 1'b0;

localparam int               N_CNT_W   = 7;
localparam bit [N_CNT_W-1:0] N_CMD_W   = 1;
// localparam bit [N_CNT_W-1:0] N_ADDR_W  = 40;
// localparam bit [N_CNT_W-1:0] N_STRB_W  = 8;
// localparam bit [N_CNT_W-1:0] N_DATA_W  = 64;
localparam bit [N_CNT_W-1:0] N_ADDR_W  = 32;
localparam bit [N_CNT_W-1:0] N_STRB_W  = 4;
localparam bit [N_CNT_W-1:0] N_DATA_W  = 32;
localparam bit [N_CNT_W-1:0] N_PRTY_W  = 1;
localparam bit [N_CNT_W-1:0] N_ACK_W   = 1;
localparam bit [N_CNT_W-1:0] N_CMDBUF_W = N_CMD_W + N_ADDR_W + N_DATA_W + N_STRB_W + N_PRTY_W;

localparam bit [N_CMD_W-1:0] CMD_RD       = 0;
localparam bit [N_CMD_W-1:0] CMD_WR       = 1;
localparam bit               START        = 0;
localparam bit               STOP         = 1;
localparam bit [N_ACK_W-1:0] ACK_OK       = {N_ACK_W{1'b0}};
localparam bit [N_ACK_W-1:0] ACK_ERR      = {N_ACK_W{1'b1}};

logic [N_DATA_W-1:0] resp_data;

enum logic [2:0] {
    MEM_IDLE,
    MEM_CMD,
    MEM_RREQ,
    MEM_RRSP,
    MEM_WREQ,
    MEM_WRSP
} rff_cdcst, next_cdcst;

// logic sdp_rstn;
// assign sdp_rstn = rstn_i;

logic dff_sdp_din;
always_ff @(posedge sdp_ck) begin
    dff_sdp_din <= sdp_di;
end

logic sdp_din_en;
logic dff_sdp_din_en;
logic sdp_din_neg;

logic [N_CNT_W-1:0] rff_cmd_idx_max;
logic [N_CNT_W-1:0] rff_cmd_idx;

assign sdp_din_en  = |rff_cmd_idx;
assign sdp_din_neg = (dff_sdp_din == 1'b1) && (sdp_di == 1'b0);

always_ff @(posedge sdp_ck or negedge sdp_rstn) begin
if (sdp_rstn == 1'b0) 
    dff_sdp_din_en <= 1'd0;
else 
    dff_sdp_din_en <= sdp_din_en;
end

logic [N_CMD_W-1:0]    cmd;
always_comb begin
if (cmd==CMD_WR)
    rff_cmd_idx_max = 1 + N_ADDR_W + N_STRB_W + N_DATA_W + N_PRTY_W;  
else if (cmd==CMD_RD)
    rff_cmd_idx_max = 1 + N_ADDR_W + N_PRTY_W;  
else 
    rff_cmd_idx_max = {N_CNT_W{1'b1}};
end

always_ff @(posedge sdp_ck or negedge sdp_rstn) begin
if (sdp_rstn == 1'b0)
    rff_cmd_idx <= {N_CNT_W{1'b0}};
else if (rff_cmd_idx>=rff_cmd_idx_max)
    rff_cmd_idx <= {N_CNT_W{1'b0}};
else if (sdp_din_en)
    rff_cmd_idx <= rff_cmd_idx + 1'b1;
else if (sdp_doen && sdp_din_neg)
    rff_cmd_idx <= rff_cmd_idx + 1'b1;
else 
    rff_cmd_idx <= {N_CNT_W{1'b0}};
end

logic [N_CMDBUF_W:1] dff_cmdbuf;
always_ff @(posedge sdp_ck) begin
if (sdp_din_en) begin
    dff_cmdbuf[rff_cmd_idx] <= sdp_di;
end
end

logic [N_ADDR_W-1:0]   addr;
logic [N_STRB_W-1:0]   wstrb;
logic [N_DATA_W-1:0]   wdata;
logic [N_PRTY_W-1:0]   prty;

generate
for (genvar i = 0; i < N_CMD_W; i++) begin : CMD_REORDER_GEN
    assign cmd[i]   = dff_cmdbuf[N_CMD_W - i];
end
for (genvar i = 0; i < N_ADDR_W; i++) begin : ADDR_REORDER_GEN
    assign addr[i]  = dff_cmdbuf[N_CMD_W + N_ADDR_W - i];
end
for (genvar i = 0; i < N_STRB_W; i++) begin : STRB_REORDER_GEN
    assign wstrb[i] = dff_cmdbuf[N_CMD_W + N_ADDR_W + N_STRB_W - i];
end
for (genvar i = 0; i < N_DATA_W; i++) begin : WDATA_REORDER_GEN
    assign wdata[i] = dff_cmdbuf[N_CMD_W + N_ADDR_W + N_STRB_W + N_DATA_W - i];
end
for (genvar i = 0; i < N_PRTY_W; i++) begin : PRTY_REORDER_GEN
    assign prty[i]  = (cmd==CMD_WR) ?  dff_cmdbuf[N_CMD_W + N_ADDR_W + N_STRB_W + N_DATA_W + N_PRTY_W - i] :
                                                dff_cmdbuf[N_CMD_W + N_ADDR_W + N_PRTY_W - i];
end
endgenerate


logic prty_wreq_exp;
logic prty_rreq_exp;
logic [N_ACK_W-1:0] ack;

assign prty_wreq_exp = (^{CMD_WR,addr,wstrb,wdata});
assign prty_rreq_exp = (^{CMD_RD,addr});

always_ff @(posedge sdp_ck) begin
if (cmd==CMD_WR) 
    ack <= (prty==prty_wreq_exp) ? ACK_OK : ACK_ERR;
else if (cmd==CMD_RD)
    ack <= (prty==prty_rreq_exp) ? ACK_OK : ACK_ERR;
else 
    ack <= ack;
end


logic [N_PRTY_W-1:0] rprty;
logic [N_PRTY_W-1:0] wprty;
assign rprty = (^{ack,resp_data});
assign wprty = (^{ack                   });


logic [1+N_ACK_W+N_DATA_W+N_PRTY_W+1-1:0] dff_payload;
logic [N_CNT_W-1:0]                             rff_payload_idx;
logic                                              rff_payload_in_progress;

always_ff @(posedge sdp_ck) begin
if (cmd==CMD_RD) begin
    if (mem_resp_valid & mem_resp_ready) 
        dff_payload <= {START, ack, resp_data, rprty, STOP};
    else /*if (rff_payload_idx == (1+N_ACK_W+N_DATA_W+N_PRTY_W+1-1))*/
        dff_payload <= dff_payload;
end
else if (cmd==CMD_WR) begin
    if ((rff_cdcst == MEM_WRSP) && (mem_resp_valid == 1'b1)) 
        dff_payload <= {START, ack, wprty, STOP, 32'hFFFF_FFFF};
    else /*if (rff_payload_idx == (1+N_ACK_W+1-1))*/
        dff_payload <= dff_payload;
end
else begin
    dff_payload <= dff_payload;
end
end

always_ff @(posedge sdp_ck or negedge sdp_rstn) begin
if (sdp_rstn == 1'b0) 
    rff_payload_idx <= {N_CNT_W{1'b0}};
else case (cmd)
    CMD_RD: begin
    if (mem_resp_valid & mem_resp_ready) 
        rff_payload_idx <= {N_CNT_W{1'b0}};
    else if (rff_payload_in_progress)
        rff_payload_idx <= rff_payload_idx + 1'b1;
    end
    CMD_WR: begin
    if ((rff_cdcst == MEM_WRSP) && (mem_resp_valid == 1'b1)) 
        rff_payload_idx <= {N_CNT_W{1'b0}};
    else if (rff_payload_in_progress) 
        rff_payload_idx <= rff_payload_idx + 1'b1;
    end
endcase 
end

always_ff @(posedge sdp_ck or negedge sdp_rstn) begin
if (sdp_rstn == 1'b0) 
    rff_payload_in_progress <= 1'b0;
else case (cmd)
    CMD_RD: begin
    if (mem_resp_valid & mem_resp_ready) 
        rff_payload_in_progress <= 1'b1;
    else if (rff_payload_idx == (1+N_ACK_W+N_DATA_W+N_PRTY_W+1-1)) 
        rff_payload_in_progress <= 1'b0;
    end
    CMD_WR: begin
    if ((rff_cdcst == MEM_WRSP) && (mem_resp_valid == 1'b1)) 
        rff_payload_in_progress <= 1'b1;
    else if (rff_payload_idx == (1+N_ACK_W+N_PRTY_W+1-1)) 
        rff_payload_in_progress <= 1'b0;
    end
endcase 
end



always_ff @(posedge sdp_ck or negedge sdp_rstn) begin
if (sdp_rstn == 1'b0) begin
    rff_cdcst <= MEM_IDLE;
end else begin
    rff_cdcst <= next_cdcst;
end
end

always_comb begin
next_cdcst = rff_cdcst;
case (rff_cdcst)
    MEM_IDLE: begin
        if (dff_sdp_din_en && !sdp_din_en) begin
            next_cdcst = MEM_CMD;
        end
    end
    MEM_CMD: begin
        case (cmd)
            CMD_RD: begin
            next_cdcst = MEM_RREQ;
            end
            CMD_WR: begin
            next_cdcst = MEM_WREQ;
            end
            default: begin
            next_cdcst = MEM_IDLE;
            end
        endcase
    end
    MEM_RREQ: begin
        if (mem_req_ready) begin
            next_cdcst = MEM_RRSP;
        end
    end
    MEM_RRSP: begin
        if (mem_resp_valid) begin
            next_cdcst = MEM_IDLE;
        end
    end
    MEM_WREQ: begin
        if (mem_req_ready) begin
            next_cdcst = MEM_WRSP;
        end
    end
    MEM_WRSP: begin
        if (mem_resp_valid) begin
            next_cdcst = MEM_IDLE;
        end
    end
endcase
end

always_comb begin
sdp_doen = 1'b1;
// default output high
sdp_do = 1'b1;
if (rff_payload_in_progress == 1'b1) begin
    sdp_doen = 1'b0;
    sdp_do = dff_payload[1 + N_ACK_W + N_DATA_W + N_PRTY_W + 1 - 1 - rff_payload_idx];
end
end

assign mem_req_valid     = (rff_cdcst == MEM_WREQ) | (rff_cdcst == MEM_RREQ);
assign mem_req.req_type  = cmd ? MEM_WRITE : MEM_READ;
// assign mem_req.req_tid   = 'h0;
assign mem_req.req_addr  = addr;
assign mem_req.req_mask  = wstrb;
assign mem_req.req_data  = wdata;
assign mem_req.req_burst = 1'b1;

assign mem_resp_ready    = (rff_cdcst == MEM_WRSP) | (rff_cdcst == MEM_RRSP);
assign resp_data         = mem_resp.resp_data;

endmodule