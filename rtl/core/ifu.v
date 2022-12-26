//流水线译码模块
`include "inst_index.v"
//流水线取指模块
`include "macro.v"

module ifu 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input clk,
    input rstn,

    input [31:0] rst_pc,

    input ac2if_flush,
    input [31:0] ac2if_flush_pc,
    input ac2if_stall,

    // instruction fetch bus
    output           imem_req_valid,
    input            imem_req_ready,
    output mem_req_t imem_req,

    //ifu <-> bpu
    output bp_req_valid,
    output [`BP_ADDR_BITS-1:0] bp_req_pc,
    input  bp_req_ready, // not use now
    input  bp_resp_valid,
    input  bp_resp_match,
    input  [`BP_ADDR_W-1:0]  bp_resp_addr,
    input  [31:0] bp_resp_pc,

    // ifu to idu to iex
    output if2id_bp_taken,
    output if2id_bp_match,
    output [`BP_ADDR_W-1:0] if2id_bp_addr,
    output [31:0] if2id_bp_pc,

    // ifu to idu 
    output if_valid,
    output [31:0] if_pc,
    output [31:0] if_pc_plus,
    output is_inst_ilen32,
    output [31:0] if_inst,

    output if2ac_hazard
);
    wire pc_valid;
    wire if_ready;
    wire imem_req_handshaked;

    wire [31:0] inst_addr;
    wire [31:0] pc_plus;
    wire [31:0] pc_next;
    wire [31:0] bp_pc;

    wire if_stall;
    assign if2ac_hazard = 1'b0;
    assign if_stall = ac2if_stall;
    pc pc_u (
        .clk(clk),
        .rstn(rstn),
        .rst_pc(rst_pc),
        .flush(ac2if_flush),
        .flush_pc(ac2if_flush_pc),
        // .stall(if_stall),
        .bp_valid(bp_resp_valid),
        .bp_pc(bp_pc),
        .if_ready(if_ready),
        .pc_valid(pc_valid),
        .pc_plus(pc_plus),
        .pc_value(inst_addr),
        .pc_next(pc_next)
    );

    //flush信号清空流水线
    assign if_valid    = if_ready;
    assign if_pc       = inst_addr;
    assign if_pc_plus  = pc_plus;

    assign if2id_bp_taken = bp_resp_valid;
    assign if2id_bp_match = bp_resp_match;
    assign if2id_bp_addr  = bp_resp_addr;
    assign if2id_bp_pc    = bp_resp_pc;

`ifdef BPU_FUNC0
    assign bp_pc        = bp_resp_pc;
    assign bp_req_pc    = if_pc[`BP_ADDR_BITS-1:0];
`endif
`ifdef BPU_FUNC1
    // assign bp_pc        = if_pc + inst_b_imm;
    // assign bp_req_pc    = bp_pc[`BP_ADDR_BITS-1:0];
`endif
    assign bp_req_valid = if_ready;

    assign imem_req_valid      = pc_valid && ~ac2if_flush && ~ac2if_stall;
    assign imem_req.req_type   = MEM_READ;   
    assign imem_req.req_addr   = inst_addr;
    assign imem_req_handshaked = imem_req_valid && imem_req_ready;
    assign if_ready            = imem_req_handshaked;
endmodule