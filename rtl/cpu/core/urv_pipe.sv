//流水线译码测试
`include "inst_index.v"
`include "macro.v"

module urv_pipe
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  logic                    clk,
    input  logic                    rstn,
    input  logic [MEM_ADDR_W-1:0]   rst_pc,

    input  logic                    ext_irq,
    input  logic                    soft_irq,
    input  logic                    time_irq,
    input  logic [CSR_TIME_W-1:0]   time_val,
    // instruction fetch bus
    output logic                    imem_req_valid,
    input  logic                    imem_req_ready,
    output mem_req_t                imem_req, 

    input  logic                    imem_resp_valid,
    output logic                    imem_resp_ready,
    input  mem_resp_t               imem_resp,
    // memery bus
    output logic                    dmem_req_valid,
    input  logic                    dmem_req_ready,
    output mem_req_t                dmem_req, 

    input  logic                    dmem_resp_valid,
    output logic                    dmem_resp_ready,
    input  mem_resp_t               dmem_resp
);

logic        if2id_valid;
logic [31:0] if2id_pc;
logic [31:0] if2id_pc_plus;
logic [31:0] if2id_inst;

logic        id2iex_valid;
logic [31:0] id2iex_pc;
logic [31:0] id2iex_pc_plus;
logic [31:0] id2iex_inst;
logic [4:0]  id2iex_rs1;
logic [4:0]  id2iex_rs2;
logic [4:0]  id2iex_rd;
logic [31:0] id2iex_rd1;
logic [31:0] id2iex_rd2;

logic        iex2ac_jump_valid;
logic [31:0] iex2ac_jump_pc;
logic [31:0] iex2lsu_pc;

logic        iex2lsu_valid;
logic [31:0] iex2lsu_pc_plus;
logic [31:0] iex2lsu_inst;
logic [4:0]  iex2lsu_rs1;
logic [4:0]  iex2lsu_rs2;
logic [4:0]  iex2lsu_rd;

logic        lsu2wb_valid;
logic [31:0] lsu2wb_pc;
logic [31:0] lsu2wb_pc_plus;
logic [31:0] lsu2wb_inst;
logic [4:0]  lsu2wb_rs1;
logic [4:0]  lsu2wb_rs2;
logic [4:0]  lsu2wb_rd;

`ifndef SYNTHESIS
    logic [63:0] id2iex_inst_ascii;
`endif

logic        id2iex_rf_we;
logic [1:0]  id2iex_rf_rd_sel0;
logic        id2iex_rf_rd_sel1;
logic        id2iex_alu_op2_sel;
logic [3:0]  id2iex_alu_op;
logic        id2iex_mul_valid;
logic        id2iex_mem_we;
logic        id2iex_mem_re;
logic [2:0]  id2iex_mem_byte_sel;
logic        id2iex_branch;
logic        id2iex_pc_src0;
logic        id2iex_jal_jalr;
logic [31:0] id2iex_imm;
logic [11:0] id2iex_csr;
logic        id_hazard_stall;

`ifndef SYNTHESIS
    logic [63:0] iex2lsu_inst_ascii;
`endif

logic        iex2lsu_rf_we;
// logic [1:0]  iex2lsu_rf_rd_sel0;
logic        iex2lsu_rf_rd_sel1;
logic        iex2lsu_mem_we;
logic        iex2lsu_mem_re;
logic [2:0]  iex2lsu_mem_byte_sel;
logic [31:0] iex2lsu_dout;
logic [31:0] iex2lsu_rf_rd2;

`ifndef SYNTHESIS
	logic [63:0] lsu2wb_inst_ascii;
`endif
logic        lsu2wb_rf_we;
logic        lsu2wb_rf_rd_sel1;
logic [31:0] lsu2wb_dout;
logic        lsu2wb_mem_valid;
logic [2:0]  lsu2wb_mem_byte_sel;
logic [1:0]  lsu2wb_mem_addr_offset;
logic [31:0] lsu2wb_mem_rdata;
// read and write registers / reg bus	
logic [31:0] rf2id_rd1;
logic [31:0] rf2id_rd2;

logic        wb2rf_wren;
logic [4:0]  wb2rf_waddr;
logic [31:0] wb2rf_wdata;
logic [4:0]  id2rf_raddr1;
logic [4:0]  id2rf_raddr2;

logic        if2ac_hazard;
logic        id2ac_hazard;
logic        iex2ac_hazard;
logic        ac2if_stall;
logic        ac2id_stall;
logic        ac2iex_stall;
logic        lsu2ac_hazard;
logic        wb2ac_hazard;
logic        ac2lsu_stall;
logic        ac2wb_stall;
logic        ac2if_flush;
logic [31:0] ac2if_flush_pc;

logic        wb2csr_wren;
logic [11:0] wb2csr_waddr;
logic [31:0] wb2csr_wdata;
logic [11:0] id2csr_ra;
logic        ac2csr_instret_stall;
logic [31:0] csr2id_rd;
//id to iex
logic        id2iex_csr_we;
logic        id2iex_csr_re;
logic [2:0]  id2iex_csr_sel;
logic [31:0] id2iex_csr_rd;
//iex to csr
logic        iex2csr_wren;
logic [11:0] iex2csr_waddr;
logic [31:0] iex2csr_wdata;
//lsu to wb
logic        lsu2wb_csr_we;
logic        lsu2wb_csr_re;
logic [11:0] lsu2wb_csr_wa;
logic [31:0] lsu2wb_csr_wd;	
//iex to lsu
logic        iex2lsu_csr_we;
logic        iex2lsu_csr_re;
logic [31:0] iex2lsu_csr_wd;

logic        ac2id_flush;
logic        ac2iex_flush;
logic        ac2lsu_flush;

// ifu <-> bpu
logic        bp_req_valid;
logic        bp_req_ready;
logic [`BP_ADDR_BITS-1:0] bp_req_pc;
logic        bp_resp_valid;
logic        bp_resp_match;
logic [`BP_ADDR_W-1:0] bp_resp_addr;
logic [31:0] bp_resp_pc;
// exu <-> bpu
logic        flush_valid;
logic        flush_new_pc;
logic [3:0]  flush_type;       //0->miss, 1->hit
logic [`BP_ADDR_W-1:0]  flush_addr; //record 6 kinds of branch instruction
logic [`BP_ADDR_BITS-1:0] flush_bp_pc;
logic [31:0] flush_pc;
logic        flush_ras_valid; // pop or push valid: rs1 or rd = x1 or x5;
logic [1:0]  flush_ras_type;  // action: 2'b00 > push, 2'b01 > pop, 2'b10 -> pop then push;
logic [31:0] flush_ras_pc;    // push ras pc

logic        if2id_bp_taken;
logic        if2id_bp_match;
logic [`BP_ADDR_W-1:0] if2id_bp_addr;
logic [31:0] if2id_bp_pc;
logic [31:0] id2iex_bp_pc;

logic        id2iex_bp_taken;
logic        id2iex_bp_match;
logic [`BP_ADDR_W-1:0] id2iex_bp_addr;

logic        iex_except_valid;
logic [3:0]  iex_except_code;
logic        iex_valid;
logic [31:0] iex_pc;
logic [31:0] iex_next_pc;
logic        iex_mret;

logic        trap_jump_valid;
logic [31:0] trap_jump_pc;
logic        ac2csr_stall;

acu acu_u(
    .clk(clk),
    .rstn(rstn),

    .if2ac_hazard(if2ac_hazard),
    .id2ac_hazard(id2ac_hazard),
    .iex2ac_hazard(iex2ac_hazard),
    .lsu2ac_hazard(lsu2ac_hazard),
    .wb2ac_hazard(wb2ac_hazard),
    .dbg2ac_stall(1'b0),

    .iex2lsu_valid(iex2lsu_valid),

    .iex2ac_jump_valid(iex2ac_jump_valid),
    .iex2ac_jump_pc(iex2ac_jump_pc),
    // from csr
    .trap_jump_valid(trap_jump_valid),
    .trap_jump_pc(trap_jump_pc), 

    .ac2if_flush(ac2if_flush),
    .ac2if_flush_pc(ac2if_flush_pc),

    .ac2id_flush(ac2id_flush),
    .ac2iex_flush(ac2iex_flush),
    .ac2lsu_flush(ac2lsu_flush),

    .ac2if_stall(ac2if_stall),
    .ac2id_stall(ac2id_stall),
    .ac2iex_stall(ac2iex_stall),
    .ac2lsu_stall(ac2lsu_stall),	
    .ac2wb_stall(ac2wb_stall),
    .ac2csr_stall(ac2csr_stall),
    .ac2csr_instret_stall(ac2csr_instret_stall)
);

bpu bpu_u(
    .clk(clk),
    .rstn(rstn),

    // ifu <-> bpu
    .req_valid(bp_req_valid),
    .req_ready(bp_req_ready),
    .req_pc(bp_req_pc),
    .resp_valid(bp_resp_valid),
    .resp_match(bp_resp_match),
    .resp_addr(bp_resp_addr),
    .resp_pc(bp_resp_pc),

    // exu <-> bpu
    .flush_valid(flush_valid),
    .flush_new_pc(flush_new_pc),
    .flush_type(flush_type),        //0->miss, 1->hit
    .flush_addr(flush_addr), //record 6 kinds of branch instruction
    .flush_bp_pc(flush_bp_pc),
    .flush_pc(flush_pc),
    .flush_ras_valid(flush_ras_valid), // pop or push valid: rs1 or rd = x1 or x5;
    .flush_ras_type(flush_ras_type),  // action: 2'b00 > push, 2'b01 > pop, 2'b10 -> pop then push;
    .flush_ras_pc(flush_ras_pc)    // push ras pc	
);

csr csr_u(
    .clk(clk),
    .rstn(rstn),
    // from clint
    .soft_irq(soft_irq),
    .time_irq(time_irq),
    .time_val(time_val), 
    // from plic
    .ext_irq(ext_irq),
    // from exu
    .iex_except_valid(iex_except_valid),
    .iex_except_code(iex_except_code),
    .iex_valid(iex_valid),
    .iex_pc(iex_pc),
    .iex_next_pc(iex_next_pc),
    .iex_mret(iex_mret),	
    // idu <-> csr wbu <-> csr 
    // .wren(wb2csr_wren),
    // .waddr(wb2csr_waddr),
    // .wdata(wb2csr_wdata),
    // from iex
    .wren(iex2csr_wren),
    .waddr(iex2csr_waddr),
    .wdata(iex2csr_wdata),
    .raddr(id2csr_ra),
    .instret_stall(ac2csr_instret_stall),
    .ac2csr_stall(ac2csr_stall),
    .rdata(csr2id_rd),
    // csr to acu
    .trap_jump_valid(trap_jump_valid),
    .trap_jump_pc(trap_jump_pc)    
);

gpr gpr_u(
    .clk(clk),
    .rstn(rstn),
    // input
    .wren(wb2rf_wren),
    .waddr(wb2rf_waddr),
    .wdata(wb2rf_wdata),
    .raddr1(id2rf_raddr1),
    .raddr2(id2rf_raddr2),
    // output
    .rdata1(rf2id_rd1),
    .rdata2(rf2id_rd2)
);

(* keep_hierarchy="yes" *)
wbu wbu_u(
    .clk(clk),
    .rstn(rstn),
    //lsu to wb
    .lsu2wb_valid(lsu2wb_valid),
    .lsu2wb_pc(lsu2wb_pc),
    .lsu2wb_pc_plus(lsu2wb_pc_plus),
    .lsu2wb_inst(lsu2wb_inst),
    .lsu2wb_rs1(lsu2wb_rs1),
    .lsu2wb_rs2(lsu2wb_rs2),
    .lsu2wb_rd(lsu2wb_rd),

    `ifndef SYNTHESIS
    .lsu2wb_inst_ascii(lsu2wb_inst_ascii), 
    `endif
    //iex to lsu control signal
    .lsu2wb_rf_we(lsu2wb_rf_we),
    .lsu2wb_rf_rd_sel1(lsu2wb_rf_rd_sel1),
    .lsu2wb_dout(lsu2wb_dout),
    .lsu2wb_mem_valid(lsu2wb_mem_valid),
    .lsu2wb_mem_byte_sel(lsu2wb_mem_byte_sel),
    .lsu2wb_mem_addr_offset(lsu2wb_mem_addr_offset),
    //from lsu
    .dmem_resp_valid(dmem_resp_valid),
    .dmem_resp_ready(dmem_resp_ready),
    .dmem_resp(dmem_resp), 
    // //from lsu
    // .lsu2wb_csr_we(lsu2wb_csr_we),
    // .lsu2wb_csr_re(lsu2wb_csr_re),
    // .lsu2wb_csr_wd(lsu2wb_csr_wd),		
    //from aux ctrl
    .ac2wb_stall(ac2wb_stall),
    //to acu
    .wb2ac_hazard(wb2ac_hazard),
    //wb to rf
    .wb2rf_wren(wb2rf_wren),
    .wb2rf_waddr(wb2rf_waddr),
    .wb2rf_wdata(wb2rf_wdata)
    // //to csr
    // .wb2csr_wren(wb2csr_wren),
    // .wb2csr_waddr(wb2csr_waddr),
    // .wb2csr_wdata(wb2csr_wdata)		
);

(* keep_hierarchy="yes" *)
lsu lsu_u(
    .clk(clk),
    .rstn(rstn),

    .iex2lsu_valid(iex2lsu_valid),
    .iex2lsu_pc(iex2lsu_pc),
    .iex2lsu_pc_plus(iex2lsu_pc_plus),
    .iex2lsu_inst(iex2lsu_inst),
    .iex2lsu_rs1(iex2lsu_rs1),
    .iex2lsu_rs2(iex2lsu_rs2),
    .iex2lsu_rd(iex2lsu_rd),

    `ifndef SYNTHESIS
    .iex2lsu_inst_ascii(iex2lsu_inst_ascii), 
    `endif
    //iex to lsu control signal
    .iex2lsu_rf_we(iex2lsu_rf_we),
    .iex2lsu_rf_rd_sel1(iex2lsu_rf_rd_sel1),
    .iex2lsu_mem_we(iex2lsu_mem_we),
    .iex2lsu_mem_re(iex2lsu_mem_re),
    .iex2lsu_mem_byte_sel(iex2lsu_mem_byte_sel),
    .iex2lsu_dout(iex2lsu_dout),
    .iex2lsu_rf_rd2(iex2lsu_rf_rd2),
    // // from iex
    // .iex2lsu_csr_we(iex2lsu_csr_we),
    // .iex2lsu_csr_re(iex2lsu_csr_re),
    // .iex2lsu_csr_wd(iex2lsu_csr_wd),
    //forward from wb to lsu
    //lord type -> store type
    .wb2rf_wren(wb2rf_wren),
    .wb2rf_waddr(wb2rf_waddr),
    .wb2rf_wdata(wb2rf_wdata),

    //from aux ctrl
    .ac2lsu_stall(ac2lsu_stall),
    .ac2lsu_flush(ac2lsu_flush),
    // LSU memery bus
    .dmem_req_valid(dmem_req_valid),
    .dmem_req_ready(dmem_req_ready),
    .dmem_req(dmem_req), 

    //lsu to wb
    .lsu2wb_valid(lsu2wb_valid),
    .lsu2wb_pc(lsu2wb_pc),
    .lsu2wb_pc_plus(lsu2wb_pc_plus),
    .lsu2wb_inst(lsu2wb_inst),	
    .lsu2wb_rs1(lsu2wb_rs1),
    .lsu2wb_rs2(lsu2wb_rs2),
    .lsu2wb_rd(lsu2wb_rd),
    `ifndef SYNTHESIS
    .lsu2wb_inst_ascii(lsu2wb_inst_ascii),
    `endif
    .lsu2wb_rf_we(lsu2wb_rf_we),
    .lsu2wb_rf_rd_sel1(lsu2wb_rf_rd_sel1),
    .lsu2wb_dout(lsu2wb_dout),
    .lsu2wb_mem_valid(lsu2wb_mem_valid),
    .lsu2wb_mem_byte_sel(lsu2wb_mem_byte_sel),
    .lsu2wb_mem_addr_offset(lsu2wb_mem_addr_offset),
    // // to lsu
    // .lsu2wb_csr_we(lsu2wb_csr_we),
    // .lsu2wb_csr_re(lsu2wb_csr_re),
    // .lsu2wb_csr_wa(lsu2wb_csr_wa),
    // .lsu2wb_csr_wd(lsu2wb_csr_wd),
    .lsu2ac_hazard(lsu2ac_hazard)
);

exu exu_u(
    .clk(clk),
    .rstn(rstn),

    .id2iex_valid(id2iex_valid),
    .id2iex_pc(id2iex_pc),
    .id2iex_pc_plus(id2iex_pc_plus),
    .id2iex_inst(id2iex_inst),
    .id2iex_rs1(id2iex_rs1),
    .id2iex_rs2(id2iex_rs2),
    .id2iex_rd(id2iex_rd),
    // ifu to idu to iex
    .id2iex_bp_taken(id2iex_bp_taken),
    .id2iex_bp_match(id2iex_bp_match),
    .id2iex_bp_addr(id2iex_bp_addr),
    .id2iex_bp_pc(id2iex_bp_pc),

    `ifndef SYNTHESIS
    .id2iex_inst_ascii(id2iex_inst_ascii), 
    `endif
    //input id to iex
    .id2iex_rf_we(id2iex_rf_we),
    .id2iex_rf_rd_sel0(id2iex_rf_rd_sel0),
    .id2iex_rf_rd_sel1(id2iex_rf_rd_sel1),
    .id2iex_alu_op2_sel(id2iex_alu_op2_sel),
    .id2iex_alu_op(id2iex_alu_op),
    .id2iex_mul_valid(id2iex_mul_valid),
    .id2iex_mem_we(id2iex_mem_we),
    .id2iex_mem_re(id2iex_mem_re),
    .id2iex_mem_byte_sel(id2iex_mem_byte_sel),
    .id2iex_branch(id2iex_branch),
    .id2iex_pc_src0(id2iex_pc_src0),
    .id2iex_jal_jalr(id2iex_jal_jalr),
    .id2iex_imm(id2iex_imm),
    .id2iex_csr(id2iex_csr),
    // from id
    .id2iex_csr_we(id2iex_csr_we),
    .id2iex_csr_re(id2iex_csr_re),
    .id2iex_csr_sel(id2iex_csr_sel),
    .id2iex_csr_rd(id2iex_csr_rd),
    // read and write registers / reg bus
    // input	
    .id2iex_rd1(id2iex_rd1),
    .id2iex_rd2(id2iex_rd2),
    // input
    //forward from lsu to iex
    .lsu2wb_rf_we(lsu2wb_rf_we),
    .lsu2wb_rd(lsu2wb_rd),
    .lsu2wb_dout(lsu2wb_dout),
    //forward from wb to iex
    .wb2rf_wren(wb2rf_wren),
    .wb2rf_waddr(wb2rf_waddr),
    .wb2rf_wdata(wb2rf_wdata),
    // //forward csr from lsu to iex
    // .lsu2wb_csr_we(lsu2wb_csr_we),
    // .lsu2wb_csr_wa(lsu2wb_csr_wa),
    // .lsu2wb_csr_wd(lsu2wb_csr_wd),
    // //forward csr from wb to iex
    // .wb2csr_wren(wb2csr_wren),
    // .wb2csr_waddr(wb2csr_waddr),
    // .wb2csr_wdata(wb2csr_wdata),
    //from aux ctrl
    .ac2iex_flush(ac2iex_flush),
    .ac2iex_stall(ac2iex_stall),
    //output
    //iex to lsu control signal
    .iex2lsu_valid(iex2lsu_valid),
    .iex2lsu_pc(iex2lsu_pc),
    .iex2lsu_pc_plus(iex2lsu_pc_plus),
    .iex2lsu_inst(iex2lsu_inst),
    .iex2lsu_rs1(iex2lsu_rs1),
    .iex2lsu_rs2(iex2lsu_rs2),
    .iex2lsu_rd(iex2lsu_rd),
    `ifndef SYNTHESIS
    .iex2lsu_inst_ascii(iex2lsu_inst_ascii),	
    `endif
    .iex2lsu_rf_we(iex2lsu_rf_we),
    .iex2lsu_rf_rd_sel1(iex2lsu_rf_rd_sel1),
    .iex2lsu_mem_we(iex2lsu_mem_we),
    .iex2lsu_mem_re(iex2lsu_mem_re),
    .iex2lsu_mem_byte_sel(iex2lsu_mem_byte_sel),
    .iex2lsu_dout(iex2lsu_dout),
    .iex2lsu_rf_rd2(iex2lsu_rf_rd2),
    // // to lsu
    // .iex2lsu_csr_we(iex2lsu_csr_we),
    // .iex2lsu_csr_re(iex2lsu_csr_re),
    // .iex2lsu_csr_wd(iex2lsu_csr_wd),
    // to csr
    .iex2csr_wren(iex2csr_wren),
    .iex2csr_waddr(iex2csr_waddr),
    .iex2csr_wdata(iex2csr_wdata),
    .iex_jump_valid(iex2ac_jump_valid),
    .iex_jump_pc(iex2ac_jump_pc),
    // exu to csr
    .iex_except_valid(iex_except_valid),
    .iex_except_code(iex_except_code),
    .iex_valid(iex_valid),
    .iex_pc(iex_pc),
    .iex_next_pc(iex_next_pc),
    .iex_mret(iex_mret),
    // exu <-> bpu
    .flush_valid(flush_valid),
    .flush_new_pc(flush_new_pc),
    .flush_type(flush_type),        //0->miss, 1->hit
    .flush_addr(flush_addr), //record 6 kinds of branch instruction
    .flush_bp_pc(flush_bp_pc),
    .flush_pc(flush_pc),
    .flush_ras_valid(flush_ras_valid), // pop or push valid: rs1 or rd = x1 or x5;
    .flush_ras_type(flush_ras_type),  // action: 2'b00 > push, 2'b01 > pop, 2'b10 -> pop then push;
    .flush_ras_pc(flush_ras_pc),    // push ras pc

    .iex2ac_hazard(iex2ac_hazard)
);

idu idu_u(
    .clk(clk),
    .rstn(rstn),
    //input if to id
    .if2id_valid(if2id_valid),
    .if2id_pc(if2id_pc),
    .if2id_pc_plus(if2id_pc_plus),
    //instuction fetch bus
    .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready),
    .imem_resp(imem_resp), 

    // ifu to idu to iex
    .if2id_bp_taken(if2id_bp_taken),
    .if2id_bp_match(if2id_bp_match),
    .if2id_bp_addr(if2id_bp_addr),
    .if2id_bp_pc(if2id_bp_pc),

    .rf2id_rd1(rf2id_rd1),
    .rf2id_rd2(rf2id_rd2),
    .csr2id_rd(csr2id_rd),	

    .iex2lsu_mem_re(iex2lsu_mem_re), 
    .iex2lsu_rd(iex2lsu_rd),
    //from aux ctrl
    .ac2id_flush(ac2id_flush),
    .ac2id_stall(ac2id_stall),
    // //forward from wb to iex
    // .wb2rf_wren(wb2rf_wren),
    // .wb2rf_waddr(wb2rf_waddr),
    // .wb2rf_wdata(wb2rf_wdata),
    // //forward csr from wb to id
    // .wb2csr_wren(wb2csr_wren),
    // .wb2csr_waddr(wb2csr_waddr),
    // .wb2csr_wdata(wb2csr_wdata),
    //output id to iex
    .id2iex_valid(id2iex_valid),
    .id2iex_pc(id2iex_pc),
    .id2iex_pc_plus(id2iex_pc_plus),
    .id2iex_inst(id2iex_inst),
    .id2iex_rs1(id2iex_rs1),
    .id2iex_rs2(id2iex_rs2),
    .id2iex_rd(id2iex_rd),
    // ifu to idu to iex
    .id2iex_bp_taken(id2iex_bp_taken),
    .id2iex_bp_match(id2iex_bp_match),
    .id2iex_bp_addr(id2iex_bp_addr),
    .id2iex_bp_pc(id2iex_bp_pc),

    `ifndef SYNTHESIS
    .id2iex_inst_ascii(id2iex_inst_ascii),
    `endif
    //id to ex control signal
    .id2iex_rf_we(id2iex_rf_we),
    .id2iex_rf_rd_sel0(id2iex_rf_rd_sel0),
    .id2iex_rf_rd_sel1(id2iex_rf_rd_sel1),
    .id2iex_rd1(id2iex_rd1),
    .id2iex_rd2(id2iex_rd2),
    .id2iex_alu_op2_sel(id2iex_alu_op2_sel),
    .id2iex_alu_op(id2iex_alu_op),
    .id2iex_mul_valid(id2iex_mul_valid),
    .id2iex_mem_we(id2iex_mem_we),
    .id2iex_mem_re(id2iex_mem_re),
    .id2iex_mem_byte_sel(id2iex_mem_byte_sel),
    .id2iex_branch(id2iex_branch),
    .id2iex_pc_src0(id2iex_pc_src0),
    .id2iex_jal_jalr(id2iex_jal_jalr),

    .id2iex_imm(id2iex_imm),
    .id2iex_csr(id2iex_csr),

    .id2rf_rs1(id2rf_raddr1),
    .id2rf_rs2(id2rf_raddr2),
    //id to csr
    .id2csr_ra(id2csr_ra),
    //id to iex
    .id2iex_csr_we(id2iex_csr_we),
    .id2iex_csr_re(id2iex_csr_re),
    .id2iex_csr_sel(id2iex_csr_sel),
    .id2iex_csr_rd(id2iex_csr_rd),

    .id2ac_hazard(id2ac_hazard)
);

ifu ifu_u(
    //input
    .clk(clk),
    .rstn(rstn),
    .rst_pc(rst_pc),
    //from aux ctrl
    .ac2if_flush(ac2if_flush),
    .ac2if_flush_pc(ac2if_flush_pc),
    .ac2if_stall(ac2if_stall),
    //instuction fetch bus
    .imem_req_valid(imem_req_valid),
    .imem_req_ready(imem_req_ready),
    .imem_req(imem_req), 

    // bpu to idu
    .bp_req_valid(bp_req_valid),
    .bp_req_pc(bp_req_pc),
    .bp_req_ready(bp_req_ready), // not use now
    .bp_resp_valid(bp_resp_valid),
    .bp_resp_match(bp_resp_match),
    // .bp_resp_valid(1'b0),
    // .bp_resp_match(1'b0),
    .bp_resp_addr(bp_resp_addr),
    .bp_resp_pc(bp_resp_pc),
    // ifu to idu to iex
    .if2id_bp_taken(if2id_bp_taken),
    .if2id_bp_match(if2id_bp_match),
    .if2id_bp_addr(if2id_bp_addr),
    .if2id_bp_pc(if2id_bp_pc),

    //output
    .if_valid(if2id_valid),
    .if_pc(if2id_pc),
    .if_pc_plus(if2id_pc_plus),
    .if_inst(if2id_inst),
    //to aux ctrl
    .if2ac_hazard(if2ac_hazard)
);

endmodule