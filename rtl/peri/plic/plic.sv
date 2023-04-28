///////////////////////
// description:
///////////////////////
`include "inst_index.v"
`include "macro.v"
// `include "cfg.sv"
// `include "typedef.sv"

module plic 
  import urv_cfg::*;
  import urv_typedef::*; 
#(
    parameter PLIC_PRI_W = 3,
    parameter PLIC_IE_W  = 1,
    parameter PLIC_IRQ_N = 32,
    parameter PLIC_CLAIM_W = $clog2(PLIC_IRQ_N + 1)
)
(
    input                   clk,
    input                   rstn,

    // mem_if interface
    input                   mem_req_valid,
    output                  mem_req_ready,
    input  mem_req_t        mem_req, 

    output                  mem_resp_valid,
    input                   mem_resp_ready,
    output mem_resp_t       mem_resp, 

    input  [PLIC_IRQ_N-1:0] ext_irq_src,
    output                  ext_irq    
);
//fixme

logic [PLIC_IRQ_N-1:0][PLIC_PRI_W-1:0] ff_plic_pri; // 1~(PLIC_IRQ_N-1)
logic [PLIC_IRQ_N-1:0]   ff_plic_ip; // PLIC_IRQ_N
logic [PLIC_IRQ_N-1:0]   ff_plic_ie; // PLIC_IRQ_N + 1
logic [PLIC_PRI_W-1:0]   ff_plic_pri_thres; // PLIC_IRQ_N + 2
logic [PLIC_CLAIM_W-1:0] ff_plic_claim_complete;     // PLIC_IRQ_N + 3

// sync async irq with clk
logic [PLIC_IRQ_N-1:0] ff_ext_irq_src_t, ff_ext_irq_src;
stdffr #(PLIC_IRQ_N) ff_irq_sync_t_u (
    .clk(clk),
    .rstn(rstn),
    .d(ext_irq_src),
    .q(ff_ext_irq_src_t) 
);
stdffr #(PLIC_IRQ_N) ff_irq_sync_u (
    .clk(clk),
    .rstn(rstn),
    .d(ff_ext_irq_src_t),
    .q(ff_ext_irq_src) 
);

logic [PLIC_IRQ_N + 3:0] ff_plic_wr_en;
logic [PLIC_IRQ_N + 3:0] ff_plic_rd_en;
logic ff_busy;
logic ff_ready;
logic ff_taken;
logic ff_wr_en;
logic ff_rd_en;
logic mem_req_handshaked;
logic mem_resp_handshaked;

logic [PLIC_CLAIM_W-1:0] ff_addr;
logic [MEM_DATA_W-1:0] ff_wdata;
logic [MEM_MASK_W-1:0] ff_wmask;
logic [MEM_DATA_W-1:0] ff_bit_mask;

logic [MEM_DATA_W-1:0] ff_out;
logic [MEM_DATA_W-1:0] resp_data;

assign ff_wdata         = mem_req.req_data;
assign ff_wmask         = mem_req.req_mask;
assign ff_addr          = mem_req.req_addr[PLIC_CLAIM_W-1+2:2]; // 64 registers
assign ff_ready         = ~ff_busy;
assign ff_taken         = mem_req_handshaked;
assign ff_wr_en         = mem_req_handshaked && (mem_req.req_type == MEM_WRITE);
assign ff_rd_en         = mem_req_handshaked && (mem_req.req_type == MEM_READ);

assign mem_req_handshaked = mem_req_valid  && mem_req_ready;
assign mem_resp_handshaked= mem_resp_valid && mem_resp_ready;

generate
	for(genvar i = 1; i < PLIC_IRQ_N + 4; i = i + 1) begin : FF_WR_EN_GEN
		assign ff_plic_wr_en[i] = ff_wr_en && (ff_addr == i);
	end
endgenerate

generate
	for(genvar i = 1; i < PLIC_IRQ_N + 4; i = i + 1) begin : FF_RD_EN_GEN
		assign ff_plic_rd_en[i] = ff_rd_en && (ff_addr == i);
	end
endgenerate

stdffref #(1) ff_busy_u (
    .clk(clk),
    .rstn(rstn),
    .flush(mem_req_handshaked),
    .flush_val(1'b1),
    .en(mem_resp_handshaked),
    .d(1'b0),
    .q(ff_busy) 
);

stdffre #(MEM_DATA_W) ff_resp_data_u (
    .clk(clk),
    .rstn(rstn),
    .en(ff_taken),
    .d(ff_out),
    .q(resp_data) 
);

genvar i;
generate
for(i=0;i<(MEM_DATA_W/8);i++)begin : GEN_FF_BIT_MASK
    assign  ff_bit_mask[8*i+:8]  = {8{ff_wmask[i]}};
end
endgenerate

generate
	for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_GEN
        stdffrem #(PLIC_PRI_W) ff_plic_pri_u (
            .clk(clk),
            .rstn(rstn),
            .mask(ff_bit_mask[PLIC_PRI_W-1:0]),
            .en(ff_plic_wr_en[i]),
            .d(ff_wdata[PLIC_PRI_W-1:0]),
            .q(ff_plic_pri[i])    
        );
	end
endgenerate

logic [PLIC_IRQ_N-1:0] ff_plic_ip_bit_wr_en;
logic [PLIC_IRQ_N-1:0] ff_plic_ip_en;
logic [PLIC_IRQ_N-1:0] ff_plic_ip_d;

logic plic_id_rd_en, plic_id_wr_en;
assign plic_id_rd_en = ff_plic_rd_en[PLIC_IRQ_N+3];
assign plic_id_wr_en = ff_plic_wr_en[PLIC_IRQ_N+3];

// plic_gateway
logic [PLIC_IRQ_N-1:0] ff_plic_gateway;
logic [PLIC_IRQ_N-1:0] ff_plic_gateway_d;
logic [PLIC_IRQ_N-1:0] ff_plic_gateway_en;


assign ff_plic_gateway_en = ff_plic_gateway | {PLIC_IRQ_N{plic_id_wr_en}};

generate
	for(genvar i = 0; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_GATEWAY_D_GEN
		assign ff_plic_gateway_d[i] = plic_id_wr_en && (ff_wdata == i) ? 1'b1
                                    : ff_plic_gateway[i] ? ~ff_plic_ip_bit_wr_en[i]
                                    : ff_plic_gateway[i];
	end
endgenerate

generate
	for(genvar i = 0; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_GATEWAY_GEN
        stdffrve #(1) ff_plic_gateway_u (
            .clk(clk),
            .rstn(rstn),
            .rst_val(1'b1),
            .en(ff_plic_gateway_en[i]),
            .d(ff_plic_gateway_d[i]),
            .q(ff_plic_gateway[i])    
        );
	end
endgenerate

logic [PLIC_IRQ_N-1:0] ff_plic_ip_up;
generate
	for(genvar i = 0; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_IP_BIT_WR_EN_GEN
		assign ff_plic_ip_bit_wr_en[i] = ff_plic_gateway[i] && ff_ext_irq_src[i] && ~ff_plic_ip[i];
	end
endgenerate
generate
	for(genvar i = 0; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_IP_UP_GEN
		assign ff_plic_ip_up[i] = (plic_id_rd_en && (ff_plic_claim_complete == i)) ? 1'b0 
                                : ff_plic_ip_bit_wr_en[i] ? 1'b1 
                                : ff_plic_ip[i];
	end
endgenerate
// assign ff_plic_ip_bit_wr_en[0] = 1'b0;
assign ff_plic_ip_en           = ff_plic_ip_bit_wr_en | {PLIC_IRQ_N{plic_id_rd_en}};
assign ff_plic_ip_d            = ff_plic_ip_up;

generate
	for(genvar i = 0; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_IP_GEN
        stdffrefm #(1) ff_plic_ip_u (
            .clk(clk),
            .rstn(rstn),
            .flush(ff_plic_wr_en[PLIC_IRQ_N]),
            .flush_val(ff_wdata[i]),
            .mask(ff_bit_mask[i]),
            .en(ff_plic_ip_en[i]),
            .d(ff_plic_ip_d[i]),
            .q(ff_plic_ip[i])    
        );
	end
endgenerate

stdffrem #(PLIC_IRQ_N) ff_plic_ie_u (
    .clk(clk),
    .rstn(rstn),
    .mask(ff_bit_mask),
    .en(ff_plic_wr_en[PLIC_IRQ_N+1]),
    .d(ff_wdata[PLIC_IRQ_N-1:0]),
    .q(ff_plic_ie)    
);
logic [PLIC_IRQ_N-1:0][PLIC_PRI_W-1:0] plic_pri_com_op0;
logic [PLIC_IRQ_N-1:0][PLIC_PRI_W-1:0] plic_pri_com_op1;
logic [PLIC_IRQ_N-1:0][PLIC_PRI_W-1:0] plic_pri_mux_op0;
logic [PLIC_IRQ_N-1:0][PLIC_PRI_W-1:0] plic_pri_mux_op1; 
logic [PLIC_IRQ_N-1:0] plic_pri_com_out;
logic [PLIC_IRQ_N-1:0][PLIC_PRI_W-1:0] plic_pri_mux_out;

logic [PLIC_IRQ_N-1:0][PLIC_CLAIM_W-1:0] plic_id_mux_op0;
logic [PLIC_IRQ_N-1:0][PLIC_CLAIM_W-1:0] plic_id_mux_op1; 
logic [PLIC_IRQ_N-1:0][PLIC_CLAIM_W-1:0] plic_id_mux_out; 

// mux priority
generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_MUX_OP0_GEN
        if(i == 1) begin
            assign plic_pri_mux_op0[i] = {(PLIC_PRI_W){1'b0}};
        end else begin
            assign plic_pri_mux_op0[i] = plic_pri_mux_out[i-1];
        end
    end
endgenerate

generate
	for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_MUX_OP1_GEN
		assign plic_pri_mux_op1[i] = {(PLIC_PRI_W){ff_plic_ip[i]}} & ff_plic_pri[i];
	end
endgenerate

generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_COM_OP0_GEN
        assign plic_pri_com_op0[i] = {(PLIC_PRI_W){ff_plic_ie[i]}} & plic_pri_mux_op1[i];
    end
endgenerate

generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_COM_OP1_GEN
        if(i == 1) begin
            assign plic_pri_com_op1[i] = {(PLIC_PRI_W){1'b0}};
        end else begin
            assign plic_pri_com_op1[i] = plic_pri_mux_out[i-1];
        end
    end
endgenerate

generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_COM_OUT_GEN
        assign plic_pri_com_out[i] = unsigned'(plic_pri_com_op0[i]) > unsigned'(plic_pri_com_op1[i]);
    end
endgenerate

generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_PRI_MUX_OUT_GEN
        assign plic_pri_mux_out[i] = plic_pri_com_out[i] ? plic_pri_mux_op1[i] : plic_pri_mux_op0[i];
    end
endgenerate

// mux id
generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_ID_MUX_OP0_GEN
        if(i == 1) begin
            assign plic_id_mux_op0[i] = {(PLIC_CLAIM_W){1'b0}};
        end else begin
            assign plic_id_mux_op0[i] = plic_id_mux_out[i-1];
        end
    end
endgenerate

generate
	for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_ID_MUX_OP1_GEN
		assign plic_id_mux_op1[i] = i;
	end
endgenerate

generate
    for(genvar i = 1; i < PLIC_IRQ_N; i = i + 1) begin : FF_PLIC_ID_MUX_OUT_GEN
        assign plic_id_mux_out[i] = plic_pri_com_out[i] ? plic_id_mux_op1[i] : plic_id_mux_op0[i];
    end
endgenerate

stdffrem #(PLIC_PRI_W) ff_plic_pri_thres_u (
    .clk(clk),
    .rstn(rstn),
    .mask(ff_bit_mask[PLIC_PRI_W-1:0]),
    .en(ff_plic_wr_en[PLIC_IRQ_N+2]),
    .d(ff_wdata[PLIC_PRI_W-1:0]),
    .q(ff_plic_pri_thres)    
);
assign ext_irq = unsigned'(plic_pri_mux_out[PLIC_IRQ_N-1]) > unsigned'(ff_plic_pri_thres);

stdffre #(PLIC_CLAIM_W) ff_plic_claim_complete_u (
    .clk(clk),
    .rstn(rstn),
    .en(ext_irq),
    .d(plic_id_mux_out[PLIC_IRQ_N-1]),
    .q(ff_plic_claim_complete)    
);

logic [PLIC_PRI_W-1:0] ff_out_pri;

assign ff_out_pri = ff_plic_pri[ff_addr];

assign ff_out = ff_plic_rd_en[PLIC_IRQ_N]   ? ff_plic_ip
              : ff_plic_rd_en[PLIC_IRQ_N+1] ? ff_plic_ie
              : ff_plic_rd_en[PLIC_IRQ_N+2] ? ff_plic_pri_thres
              : ff_plic_rd_en[PLIC_IRQ_N+3] ? ff_plic_claim_complete
              : ff_out_pri;

// assign mem_req_ready       = mem_req_valid;
assign mem_req_ready       = ff_ready;
assign mem_resp_valid      = ff_busy;
assign mem_resp.resp_data  = resp_data;
assign mem_resp.resp_last  = mem_resp_valid;

endmodule