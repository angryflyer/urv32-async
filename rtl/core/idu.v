//流水线译码模块
`include "inst_index.v"
//`include "component.v"
`include "macro.v"

module idu 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input         clk,
    input         rstn,

    //from if to id
    input         if2id_valid,
    input [31:0]  if2id_pc,
    input [31:0]  if2id_pc_plus,
    //from memery
    input             imem_resp_valid,
    output            imem_resp_ready,
    input  mem_resp_t imem_resp, 

    // from ifu
    input         if2id_bp_taken,
    input         if2id_bp_match,
    input [`BP_ADDR_W-1:0]   if2id_bp_addr,

    //from rf to id
    input [31:0]  rf2id_rd1,
    input [31:0]  rf2id_rd2,

    //from csr to id
    input [31:0]  csr2id_rd,	

    //from iex to id, flush pipeline
    input         ac2id_flush, 
    input         ac2id_stall,	
    //hazard rd from iex to id
    //hazard detect uint, input signal
    input         iex2lsu_mem_re,
    input [4:0]   iex2lsu_rd,

    // //forward rd from wb to id
    // input wb2rf_wren,
    // input [4:0]  wb2rf_waddr,
    // input [31:0] wb2rf_wdata,

    // //forward csr from wb to id
    // input wb2csr_wren,
    // input [11:0] wb2csr_waddr,
    // input [31:0] wb2csr_wdata,

    //id to iex
    output        id2iex_valid,
    output [31:0] id2iex_pc, 
    output [31:0] id2iex_pc_plus,
    output [31:0] id2iex_inst,
    //use by forward
    output [4:0]  id2iex_rs1,
    output [4:0]  id2iex_rs2,	
    output [4:0]  id2iex_rd,

    // to iex
    output        id2iex_bp_taken,
    output        id2iex_bp_match,
    output [`BP_ADDR_W-1:0]  id2iex_bp_addr,

    `ifndef SYNTHESIS
        output [63:0] id2iex_inst_ascii, 
    `endif
    //id to ex control signal
    output        id2iex_rf_we,
    output [1:0]  id2iex_rf_rd_sel0,
    output        id2iex_rf_rd_sel1,
    output [31:0] id2iex_rd1,
    output [31:0] id2iex_rd2,	
    output        id2iex_alu_op2_sel,
    output [3:0]  id2iex_alu_op,
    output        id2iex_mem_we,
    output        id2iex_mem_re,
    output [2:0]  id2iex_mem_byte_sel,
    output        id2iex_branch,
    output        id2iex_pc_src0,
    output        id2iex_jal_jalr,
    output [31:0] id2iex_imm,
    // output [31:0] id2iex_uimm,
    output [11:0] id2iex_csr,
    //id to regfile
    output [4:0]  id2rf_rs1,
    output [4:0]  id2rf_rs2,
    //id to csr
    output [11:0] id2csr_ra,
    //id to iex
    output id2iex_csr_we,
    output id2iex_csr_re,
    //id2iex_csr_sel[0]->alu_op_sel
    //id2iex_csr_sel[1]->select source of rf_wd and csr_wd 
    output [2:0]  id2iex_csr_sel,
    output [31:0] id2iex_csr_rd,

    //id stall to if pc
    output id2ac_hazard
);

	wire [4:0]   inst_op_type;
	wire [2:0]   inst_func3;
    wire [4:0]   inst_func5;
	wire [6:0]   inst_func7;
	wire [11:0]  inst_func12;

	`ifndef SYNTHESIS
		reg [63:0] inst_index;
	`endif

	wire is_u_type;
	wire is_jal_type;
	wire is_jalr_type;
	wire is_b_branch_type;
	wire is_i_load_type;
	wire is_s_store_type;
	wire is_i_imm_type;
	wire is_i_shift_type;
	wire is_r_type;
	wire is_r_m_type;
	wire is_i_fence_type;
	wire is_i_ecall_ebreak_type;
	wire is_i_csr_type;
    wire is_i_tri_imi_type;
    wire is_inst_func7_null;
    wire is_inst_func3_null;

	//u type
	wire is_lui, is_auipc;
	//j type
	wire is_jal;
	//i type
	wire is_jalr;
	//b type branch
	wire is_beq, is_bne, is_blt, is_bge, is_bltu, is_bgeu;
	//i type load
	wire is_lb, is_lh, is_lw, is_lbu, is_lhu;
	//s type store
	wire is_sb, is_sh, is_sw;
	//i type
	wire is_addi, is_slti, is_sltiu, is_xori, is_ori, is_andi;
    wire is_slli, is_srli, is_srai;
	//r type
    wire is_add, is_sub, is_sll, is_slt, is_sltu, is_xor, is_srl, is_sra, is_or, is_and;
	//fence
	wire is_fence, is_fencei;
	wire is_ecall, is_ebreak;
	//csr
	wire is_csrrw, is_csrrs, is_csrrc, is_csrrwi, is_csrrsi, is_csrrci;
	//r type mul/div
	wire is_mul, is_mulh, is_mulhsu, is_mulhu, is_div, is_divu, is_rem, is_remu;
    //TRI
    wire is_uret, is_sret, is_mret;
    //IMI
    wire is_wfi;

	// wire reg_id_valid          = if2id_valid;
	// wire [31:0] reg_id_pc      = if2id_pc;
	// wire [31:0] reg_id_pc_plus = if2id_pc_plus;
	// wire [31:0] reg_id_inst    = if2id_inst;

	reg reg_id_valid;
	reg [31:0] reg_id_pc;
	wire [31:0] reg_id_inst;
	reg [31:0] reg_id_pc_plus;
	reg reg_id_bp_taken;
	reg reg_id_bp_match;
	reg [`BP_ADDR_W-1:0] reg_id_bp_addr;
	always @(posedge clk or negedge rstn) begin
		if(!rstn) begin
			reg_id_valid   <= `DEASSERT;
			reg_id_pc      <= `WORD_DEASSERT;
			reg_id_pc_plus <= `WORD_DEASSERT;
			// reg_id_inst    <= `WORD_DEASSERT;	
			reg_id_bp_taken<= `DEASSERT;
			reg_id_bp_match<= `DEASSERT;
			reg_id_bp_addr <= `WORD_DEASSERT;		
		end else if(~ac2id_stall) begin
			reg_id_valid   <= if2id_valid;
			reg_id_pc      <= if2id_pc;
			// reg_id_inst    <= if2id_inst;
			reg_id_pc_plus <= if2id_pc_plus;
			reg_id_bp_taken<= if2id_bp_taken;
			reg_id_bp_match<= if2id_bp_match;
			reg_id_bp_addr <= if2id_bp_addr;
		end
	end

    //停止信号ac2id_stall打1拍后给到指令buffer/指令寄存器	
    reg ibuf_stall;
    wire ibuf_stall_up;
    assign ibuf_stall_up = reg_id_valid && imem_resp_valid && ac2id_stall && ~ibuf_stall;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            ibuf_stall <= `DEASSERT;
        end else if(ibuf_stall_up) begin
            ibuf_stall <= ac2id_stall;
        end else if(~ac2id_stall && ibuf_stall) begin
            ibuf_stall <= ac2id_stall;
        end
    end
    //指令buffer，深度1级	
    //ac2id_stall信号打1拍后生效
    reg  [31:0] ibuf;
    reg  ff_imem_resp_valid;
    wire imem_resp_valid_mux;       
    //指令buffer在ac2id_stall有效时，ibuf_stall才有效
    //撤销ac2id_stall后即开始恢复运行	
    wire ibuf_stall_en = ~(ibuf_stall && ac2id_stall);
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            ibuf     <= `WORD_DEASSERT; 
            ff_imem_resp_valid <= `DEASSERT;
        end 
        else if(ibuf_stall_en)  begin
            ibuf     <= imem_resp.resp_data; 
            ff_imem_resp_valid <= imem_resp_valid;
        end
    end 
    //指令根据ibuf_stall信号进行选择ibuf还是mem2if_rdata
    assign reg_id_inst     = (ibuf_stall) ? ibuf : imem_resp.resp_data; 
    assign imem_resp_valid_mux = (ibuf_stall) ? ff_imem_resp_valid : imem_resp_valid;
    assign imem_resp_ready = 1'b1;
    
	//将解码出的指令对应数值输出到inst_index索引，以便执行模块直接调用执行
	`ifndef SYNTHESIS
	assign id2iex_inst_ascii   = ac2id_flush ? `INST_NULL : inst_index;
	`endif
	assign id2iex_valid        = (!ac2id_flush) && reg_id_valid && imem_resp_valid_mux;
	assign id2iex_pc           = reg_id_pc;
	assign id2iex_pc_plus      = reg_id_pc_plus;
	//regard fence as nop instruction
	assign id2iex_inst         = (is_fence | ac2id_flush) ? `I_INST_NOP : reg_id_inst;

	assign id2iex_bp_taken     = reg_id_bp_taken;
	assign id2iex_bp_match     = reg_id_bp_match;
	assign id2iex_bp_addr      = reg_id_bp_addr;

	assign id2csr_ra           = reg_id_inst[31:20];
	assign id2rf_rs2           = reg_id_inst[24:20];
	assign id2rf_rs1           = reg_id_inst[19:15];
	assign id2iex_rd           = reg_id_inst[11:7];
	//use by forward
	assign id2iex_rs1          = id2rf_rs1;
	assign id2iex_rs2          = id2rf_rs2;	


	wire  [31:0] inst_imm;
	assign id2iex_imm          = inst_imm;

	//id to ex pc_src control signal
	wire pc_src0, is_jal_jalr, is_branch;	
	assign pc_src0           = is_jalr;
	assign is_jal_jalr       = is_jal | is_jalr;
	assign is_branch         = is_b_branch_type;

	//id to ex rf_rd[1:0] control signal
	wire [1:0] rf_rd_sel0;
	assign rf_rd_sel0[1] = is_lui | is_jal_jalr;
	assign rf_rd_sel0[0] = is_u_type;

	//id to ex rf_rd_sel1 control signal
	wire rf_rd_sel1;
	assign rf_rd_sel1 = is_i_load_type;

	//id to ex mem control signal
	wire mem_we, mem_re;
	assign mem_we = is_s_store_type;
	assign mem_re = is_i_load_type;

	//id to ex rf_we control signal
	wire rf_we;
	assign rf_we = is_u_type | is_jal_jalr | is_i_load_type | is_i_imm_type | is_r_m_type | is_i_csr_type;

	//id to ex csr control signal
	wire id2iex_rs1_eq_zero = (id2iex_rs1 == 5'h0);
	wire id2iex_rs1_neq_zero = ~id2iex_rs1_eq_zero;
	wire id2iex_rd_eq_zero = (id2iex_rd == 5'h0);
	wire id2iex_rd_neq_zero = ~id2iex_rd_eq_zero;
	wire csr_csrrs_csrrc    = is_csrrs  | is_csrrc;
	wire csr_csrrsi_csrrci  = is_csrrsi | is_csrrci;
	// wire csr_csrrs_csrrsi   = is_csrrs  | is_csrrsi;
	wire csr_csrrc_csrrci   = is_csrrc  | is_csrrci;
	wire csr_reg_we  = is_csrrw  | (csr_csrrs_csrrc && id2iex_rs1_neq_zero); 
	wire csr_imm_we  = is_csrrwi | (csr_csrrsi_csrrci && id2iex_rs1_neq_zero);
	wire csr_reg_re  = (is_csrrw  && id2iex_rd_neq_zero) | csr_csrrs_csrrc; 
	wire csr_imm_re  = (is_csrrwi && id2iex_rd_neq_zero) | csr_csrrsi_csrrci;
	wire csr_we      = csr_reg_we | csr_imm_we;
	wire csr_re      = csr_reg_re | csr_imm_re;
	wire [2:0] id_csr_sel;

	assign id_csr_sel[0]  = csr_csrrs_csrrc;
	assign id_csr_sel[1]  = csr_csrrs_csrrc | csr_csrrsi_csrrci;
	assign id_csr_sel[2]  = is_i_csr_type;
	assign id2iex_csr_sel = id_csr_sel;

	assign id2iex_csr_we = csr_we && (!ac2id_flush);
	assign id2iex_csr_re = csr_re && (!ac2id_flush);

	//id to ex rf rd1 and rd2
	assign id2iex_rd1 = rf2id_rd1;
	assign id2iex_rd2 = rf2id_rd2;

	//mux forward data to id2iex_csr_rd
	assign id2iex_csr_rd = csr2id_rd;
	//id to ex csr address
	assign id2iex_csr = id2csr_ra;
 
	//id to ex alu_op2 control signal
	wire alu_op2_sel;
	assign alu_op2_sel = is_jalr | is_s_store_type | is_i_load_type | is_i_imm_type | is_i_csr_type;

	//id to ex alu_op control signal
	wire [3:0] alu_op;
	reg  [3:0] reg_alu_op;
	assign alu_op = reg_alu_op;

	always @(*) begin
		(*parallel_case*)
		case(1'b1) 
			is_slli, is_sll : reg_alu_op = `ALU_OP_SLL;
			is_beq          : reg_alu_op = `ALU_OP_SEQ;
			is_bne          : reg_alu_op = `ALU_OP_SNE;
			is_xori, is_xor : reg_alu_op = `ALU_OP_XOR;
			is_srli, is_srl : reg_alu_op = `ALU_OP_SRL;
			is_ori, is_or, is_csrrs, is_csrrsi   : reg_alu_op = `ALU_OP_OR;	
			is_andi, is_and, is_csrrc, is_csrrci : reg_alu_op = `ALU_OP_AND;
			is_sub          : reg_alu_op = `ALU_OP_SUB;	
			is_srai, is_sra : reg_alu_op = `ALU_OP_SRA;	
			is_blt, is_slti, is_slt : reg_alu_op = `ALU_OP_SLT;	
			is_bge          : reg_alu_op = `ALU_OP_SGE;	
			is_bltu, is_sltiu, is_sltu : reg_alu_op = `ALU_OP_SLTU;	
			is_bgeu         : reg_alu_op = `ALU_OP_SGEU;	
			default   : reg_alu_op = `ALU_OP_ADD;
		endcase
	end

	//id to ex mem byte control signal
	reg [2:0] mem_byte_sel;
	always @(*) begin
		(*parallel_case*)
		case(1'b1)
			is_lb, is_sb : mem_byte_sel = `MEM_BYTE_S;
			is_lbu       : mem_byte_sel = `MEM_BYTE_U;
			is_lh, is_sh : mem_byte_sel = `MEM_HALF_S;
			is_lhu       : mem_byte_sel = `MEM_HALF_U;
			is_lw, is_sw : mem_byte_sel = `MEM_WORD_S;
			default      : mem_byte_sel = `MEM_BYTE_S;
		endcase
	end	

	//id to ex obtain imm
	wire [31:0] inst_u_imm, inst_j_imm, inst_b_imm, inst_i_imm, inst_s_imm, inst_z_uimm;
	reg [2:0] imm_src;
	//imm_src
	always @(*) begin
		(*parallel_case*)
		case(1'b1)
//			is_u_type							   : imm_src = `U_IMM;
			is_jal                                 : imm_src = `J_IMM;
			is_jalr, is_i_load_type, is_i_imm_type : imm_src = `I_IMM;
			is_branch                              : imm_src = `B_IMM;
			is_s_store_type                        : imm_src = `S_IMM;
			is_i_csr_type                          : imm_src = `Z_IMM;
			default: imm_src = `U_IMM;
		endcase
	end	

    // mul/div op encode
    wire mul_op_valid;
    wire is_mul_div;
    wire [2:0] mul_op;
    assign mul_op_valid = is_mul_div;
    assign mul_op       = {3{mul_op_valid}} & inst_func3; 

muldiv muldiv_u(
	.clk(clk),
	.rstn(rstn),
    .op_stall(1'b0),
	.op_valid(mul_op_valid),
	.op_ready(),
    .op(),
	.op1(32'hffffffff),
	.op2(32'hffffffff),
    .op_do()
);

	//expand signed imm
	assign inst_u_imm = {reg_id_inst[31:12], 12'b0};
	assign inst_j_imm = {{11{reg_id_inst[31]}}, reg_id_inst[31], reg_id_inst[19:12], reg_id_inst[20], reg_id_inst[30:21], 1'b0};
	assign inst_i_imm = {{20{reg_id_inst[31]}}, reg_id_inst[31:20]};
	assign inst_b_imm = {{19{reg_id_inst[31]}}, reg_id_inst[31], reg_id_inst[7], reg_id_inst[30:25], reg_id_inst[11:8], 1'b0};
	assign inst_s_imm = {{20{reg_id_inst[31]}}, reg_id_inst[31:25], reg_id_inst[11:7]};
	assign inst_z_uimm = {27'b0, reg_id_inst[19:15]};

	wire [31:0] inst_z_imm; 
	assign inst_z_imm = csr_csrrc_csrrci ? ~inst_z_uimm : inst_z_uimm;
	//mux imm to ex
	wire [31:0] inst_imm_out0, inst_imm_out1;
	wire [1:0]  inst_imm_out0_sel;
	wire        inst_imm_out1_sel;
	wire        inst_imm_out_sel;

	assign inst_imm_out_sel  = imm_src[2];
	assign inst_imm_out0_sel = imm_src[1:0];
	assign inst_imm_out1_sel = imm_src[0];

	mux4to1 mux_imm_u0(.sel(inst_imm_out0_sel), .in0(inst_u_imm), .in1(inst_j_imm), .in2(inst_i_imm), .in3(inst_b_imm), .out(inst_imm_out0));
	mux2to1 mux_imm_u1(.sel(inst_imm_out1_sel), .in0(inst_s_imm), .in1(inst_z_imm), .out(inst_imm_out1));
	assign  inst_imm = inst_imm_out_sel ? inst_imm_out1 : inst_imm_out0;

	//hazard detection unit
	wire id_hazard_valid;
	wire iex2id_rd_eq_rs1, iex2id_rd_eq_rs2;
	wire is_rs1_rs2_type;
	assign iex2id_rd_eq_rs1 = (iex2lsu_rd == id2iex_rs1);	
	assign iex2id_rd_eq_rs2 = (iex2lsu_rd == id2iex_rs2);
	assign is_rs1_rs2_type = is_b_branch_type || is_r_type;
	//如果指令没有用到rs2，则仅需对比rs1即可，避免不必要的nop
	assign id_hazard_valid = iex2lsu_mem_re && (iex2id_rd_eq_rs1 || (iex2id_rd_eq_rs2 && is_rs1_rs2_type));

	//如果load后跟随的指令是store，则流水线不停止
	// 114:	fe842783          	lw	a5,-24(s0)
	// 118:	00e78023          	sb	a4,0(a5)
	// 11c:	fe842783          	lw	a5,-24(s0)
	// issues happened with upon instruction
	// assign id2ac_hazard        = id_hazard_valid && (!is_s_store_type);
	assign id2ac_hazard        = id_hazard_valid | (reg_id_valid && ~imem_resp_valid_mux);

    assign id2iex_rf_we        = rf_we && (!ac2id_flush);
	assign id2iex_rf_rd_sel0   = rf_rd_sel0;
	assign id2iex_rf_rd_sel1   = rf_rd_sel1;
	assign id2iex_alu_op2_sel  = alu_op2_sel;
	assign id2iex_alu_op       = alu_op;
	assign id2iex_mem_we       = mem_we && (!ac2id_flush);
	assign id2iex_mem_re       = mem_re && (!ac2id_flush);
	assign id2iex_mem_byte_sel = mem_byte_sel;
	assign id2iex_branch       = is_branch && (!ac2id_flush);
	assign id2iex_pc_src0      = pc_src0;
	assign id2iex_jal_jalr     = is_jal_jalr && (!ac2id_flush); 

	//obtain opcode and func
	assign inst_op_type = reg_id_inst[6:2];
	assign inst_func7   = reg_id_inst[31:25];
	assign inst_func12  = id2iex_csr;
	assign inst_func3   = reg_id_inst[14:12];
    assign inst_func5   = reg_id_inst[24:20];

	//judge inst type
	assign is_u_type       = reg_id_valid && (inst_op_type[2:0] == `U_INST_OPCODE_1  );
	assign is_jal_type     = reg_id_valid && (inst_op_type == `J_INST_OPCODE_JAL); 
	assign is_jalr_type    = reg_id_valid && (inst_op_type == `I_INST_OPCODE_5  ); 
	assign is_b_branch_type = reg_id_valid && (inst_op_type == `B_INST_OPCODE_1  );
	assign is_i_load_type  = reg_id_valid && (inst_op_type == `I_INST_OPCODE_1  );
	assign is_s_store_type = reg_id_valid && (inst_op_type == `S_INST_OPCODE_1  );
	assign is_i_imm_type   = reg_id_valid && (inst_op_type == `I_INST_OPCODE_2  );
	assign is_i_shift_type = is_i_imm_type;
	assign is_r_type       = reg_id_valid && (inst_op_type == `R_INST_OPCODE_1  );
	assign is_r_m_type     = is_r_type;
	assign is_i_fence_type = reg_id_valid && (inst_op_type == `I_INST_OPCODE_3  );
	assign is_i_csr_type   = reg_id_valid && (inst_op_type == `I_INST_OPCODE_4  );
	assign is_i_ecall_ebreak_type = is_i_csr_type;
    assign is_i_tri_imi_type      = is_i_csr_type;
 
    //指令译码
	//u
	assign is_lui   = is_u_type && (inst_op_type[4:3] == `U_INST_OPCODE_LUI  );
	assign is_auipc = is_u_type && (inst_op_type[4:3] == `U_INST_OPCODE_AUIPC);
	//J + I
	assign is_jal  = is_jal_type;
	assign is_jalr = is_jalr_type;	
	//B
	assign is_beq  = is_b_branch_type && (inst_func3 == `B_INST_FUNC3_BEQ );
	assign is_bne  = is_b_branch_type && (inst_func3 == `B_INST_FUNC3_BNE );
	assign is_blt  = is_b_branch_type && (inst_func3 == `B_INST_FUNC3_BLT );
	assign is_bge  = is_b_branch_type && (inst_func3 == `B_INST_FUNC3_BGE );
	assign is_bltu = is_b_branch_type && (inst_func3 == `B_INST_FUNC3_BLTU);
	assign is_bgeu = is_b_branch_type && (inst_func3 == `B_INST_FUNC3_BGEU);
	//I
	assign is_lb   = is_i_load_type && (inst_func3 == `I_INST_FUNC3_LB );
	assign is_lh   = is_i_load_type && (inst_func3 == `I_INST_FUNC3_LH );
	assign is_lw   = is_i_load_type && (inst_func3 == `I_INST_FUNC3_LW );
	assign is_lbu  = is_i_load_type && (inst_func3 == `I_INST_FUNC3_LBU);
	assign is_lhu  = is_i_load_type && (inst_func3 == `I_INST_FUNC3_LHU);
	//S
	assign is_sb   = is_s_store_type && (inst_func3 == `S_INST_FUNC3_SB);	
	assign is_sh   = is_s_store_type && (inst_func3 == `S_INST_FUNC3_SH);	
	assign is_sw   = is_s_store_type && (inst_func3 == `S_INST_FUNC3_SW);
	//I
	assign is_addi   = is_i_imm_type && (inst_func3 == `I_INST_FUNC3_ADDI );//ADDI rd, rs1, imm
	assign is_slti   = is_i_imm_type && (inst_func3 == `I_INST_FUNC3_SLTI );
	assign is_sltiu  = is_i_imm_type && (inst_func3 == `I_INST_FUNC3_SLTIU);
	assign is_xori   = is_i_imm_type && (inst_func3 == `I_INST_FUNC3_XORI );
	assign is_ori    = is_i_imm_type && (inst_func3 == `I_INST_FUNC3_ORI  );	
	assign is_andi   = is_i_imm_type && (inst_func3 == `I_INST_FUNC3_ANDI );
	
	assign is_inst_func7_slli_srli = (inst_func7 == `I_INST_FUNC7_SLLI_SRLI);
	assign is_inst_func7_srai      = (inst_func7 == `I_INST_FUNC7_SRAI     );	
	//I
	assign is_slli   = is_i_shift_type && (inst_func3 == `I_INST_FUNC3_SLLI) && is_inst_func7_slli_srli;	
	assign is_srli   = is_i_shift_type && (inst_func3 == `I_INST_FUNC3_SRLI) && is_inst_func7_slli_srli;	
	assign is_srai   = is_i_shift_type && (inst_func3 == `I_INST_FUNC3_SRAI) && is_inst_func7_srai     ;	

	assign is_inst_func7_null    = (inst_func7 == `R_INST_FUNC7_NULL   );
	assign is_inst_func7_sub_sra = (inst_func7 == `R_INST_FUNC7_SUB_SRA);	
	assign is_inst_func7_m       = (inst_func7 == `R_INST_FUNC7_M      );	
	//R
	assign is_add    = is_r_type && (inst_func3 == `R_INST_FUNC3_ADD ) && is_inst_func7_null   ;//ADDI rd, rs1, rs2
	assign is_sub    = is_r_type && (inst_func3 == `R_INST_FUNC3_SUB ) && is_inst_func7_sub_sra;
	assign is_sll    = is_r_type && (inst_func3 == `R_INST_FUNC3_SLL ) && is_inst_func7_null   ;
	assign is_slt    = is_r_type && (inst_func3 == `R_INST_FUNC3_SLT ) && is_inst_func7_null   ;
	assign is_sltu   = is_r_type && (inst_func3 == `R_INST_FUNC3_SLTU) && is_inst_func7_null   ;	
	assign is_xor    = is_r_type && (inst_func3 == `R_INST_FUNC3_XOR ) && is_inst_func7_null   ;	
	assign is_srl    = is_r_type && (inst_func3 == `R_INST_FUNC3_SRL ) && is_inst_func7_null   ;
	assign is_sra    = is_r_type && (inst_func3 == `R_INST_FUNC3_SRA ) && is_inst_func7_sub_sra;
	assign is_or     = is_r_type && (inst_func3 == `R_INST_FUNC3_OR  ) && is_inst_func7_null   ;	
	assign is_and    = is_r_type && (inst_func3 == `R_INST_FUNC3_AND ) && is_inst_func7_null   ;	
	//M
    assign is_mul_div = is_r_m_type && is_inst_func7_m;
	assign is_mul    = is_mul_div && (inst_func3 == `R_INST_FUNC3_MUL   );
	assign is_mulh   = is_mul_div && (inst_func3 == `R_INST_FUNC3_MULH  );
	assign is_mulhsu = is_mul_div && (inst_func3 == `R_INST_FUNC3_MULHSU);
	assign is_mulhu  = is_mul_div && (inst_func3 == `R_INST_FUNC3_MULHU );
	assign is_div    = is_mul_div && (inst_func3 == `R_INST_FUNC3_DIV   );	
	assign is_divu   = is_mul_div && (inst_func3 == `R_INST_FUNC3_DIVU  );	
	assign is_rem    = is_mul_div && (inst_func3 == `R_INST_FUNC3_REM   );
	assign is_remu   = is_mul_div && (inst_func3 == `R_INST_FUNC3_REMU  );

	//fence
	assign is_fence  = is_i_fence_type && (inst_func3 == `I_INST_FUNC3_FENCE) ;
	assign is_fencei = is_i_fence_type && (inst_func3 == `I_INST_FUNC3_FENCEI);

	//ecall ebreak
    assign is_inst_func3_null = inst_func3 == `I_INST_FUNC3;
	assign is_ecall  = is_i_ecall_ebreak_type && is_inst_func3_null && (inst_func12 == `I_INST_FUNC12_ECALL) ;
	assign is_ebreak = is_i_ecall_ebreak_type && is_inst_func3_null && (inst_func12 == `I_INST_FUNC12_EBREAK);

	//csr
	assign is_csrrw    = is_i_csr_type && (inst_func3 == `I_INST_FUNC3_CSRRW) ;
	assign is_csrrs    = is_i_csr_type && (inst_func3 == `I_INST_FUNC3_CSRRS) ;
	assign is_csrrc    = is_i_csr_type && (inst_func3 == `I_INST_FUNC3_CSRRC) ;
	assign is_csrrwi   = is_i_csr_type && (inst_func3 == `I_INST_FUNC3_CSRRWI);
	assign is_csrrsi   = is_i_csr_type && (inst_func3 == `I_INST_FUNC3_CSRRSI);
	assign is_csrrci   = is_i_csr_type && (inst_func3 == `I_INST_FUNC3_CSRRCI);

    //TRI
    wire is_inst_func5_tri;
    wire is_inst_func7_sret_wfi;
    assign is_inst_func5_tri      = inst_func5 == `I_INST_FUNC5_TRI;
    assign is_inst_func7_sret_wfi = inst_func7 == `I_INST_FUNC7_SRET_WFI;
    assign is_uret     = is_i_tri_imi_type && is_inst_func3_null && is_inst_func5_tri && (inst_func7 == `I_INST_FUNC7_URET);
    assign is_sret     = is_i_tri_imi_type && is_inst_func3_null && is_inst_func5_tri && is_inst_func7_sret_wfi;
    assign is_mret     = is_i_tri_imi_type && is_inst_func3_null && is_inst_func5_tri && (inst_func7 == `I_INST_FUNC7_MRET);

    //IMI
    wire is_inst_func5_imi;
    assign is_inst_func5_imi = inst_func5 == `I_INST_FUNC5_IMI;
    assign is_wfi      = is_i_tri_imi_type && is_inst_func3_null && is_inst_func5_imi && is_inst_func7_sret_wfi;

    //todo
    wire inst_legal;
    wire is_i_load_valid;
    wire is_s_store_valid;
    wire is_i_tri_imi_valid;
    assign is_i_load_valid    = is_lb   | is_lh   | is_lw   | is_lbu |is_lhu;
    assign is_s_store_valid   = is_sb   | is_sh   | is_sw;
    assign is_i_tri_imi_valid = is_uret | is_sret | is_mret | is_wfi;

    assign inst_legal    = is_u_type        | is_jal_jalr   | is_b_branch_type | is_i_load_valid 
                         | is_s_store_valid | is_i_imm_type | is_r_type        | is_i_fence_type
                         | is_i_csr_type    | is_i_tri_imi_valid;

	//only for simulation
	`ifndef SYNTHESIS
	always @(*) begin
		(*parallel_case*)		
		case(1'b1)
			(is_lui)    : inst_index = `U_INST_LUI;
			(is_auipc)  : inst_index = `U_INST_AUIPC;

			(is_jal)    : inst_index = `J_INST_JAL;
			(is_jalr)   : inst_index = `I_INST_JALR;

			(is_beq)    : inst_index = `B_INST_BEQ;
			(is_bne)    : inst_index = `B_INST_BNE;
			(is_blt)    : inst_index = `B_INST_BLT;
			(is_bge)    : inst_index = `B_INST_BGE;
			(is_bltu)   : inst_index = `B_INST_BLTU;
			(is_bgeu)   : inst_index = `B_INST_BGEU;

			(is_lb)     : inst_index = `I_INST_LB;
			(is_lh)     : inst_index = `I_INST_LH;
			(is_lw)     : inst_index = `I_INST_LW;
			(is_lbu)    : inst_index = `I_INST_LBU;
			(is_lhu)    : inst_index = `I_INST_LHU;		

			(is_sb)     : inst_index = `S_INST_SB;	
			(is_sh)     : inst_index = `S_INST_SH;	
			(is_sw)     : inst_index = `S_INST_SW;	

			(is_addi)   : inst_index = `I_INST_ADDI;	
			(is_slti)   : inst_index = `I_INST_SLTI;	
			(is_sltiu)  : inst_index = `I_INST_SLTIU;	
			(is_xori)   : inst_index = `I_INST_XORI;	
			(is_ori)    : inst_index = `I_INST_ORI;	
			(is_andi)   : inst_index = `I_INST_ANDI;	

			(is_slli)   : inst_index = `I_INST_SLLI;
			(is_srli)   : inst_index = `I_INST_SRLI;
			(is_srai)   : inst_index = `I_INST_SRAI;

			(is_add)    : inst_index = `R_INST_ADD;
			(is_sub)    : inst_index = `R_INST_SUB;
			(is_sll)    : inst_index = `R_INST_SLL;
			(is_slt)    : inst_index = `R_INST_SLT;
			(is_sltu)   : inst_index = `R_INST_SLTU;
			(is_xor)    : inst_index = `R_INST_XOR;
			(is_srl)    : inst_index = `R_INST_SRL;
			(is_sra)    : inst_index = `R_INST_SRA;
			(is_or)     : inst_index = `R_INST_OR;
			(is_and)    : inst_index = `R_INST_AND;

			(is_fence)  : inst_index = `I_INST_FENCE;
			(is_fencei) : inst_index = `I_INST_FENCEI;

			(is_ecall)  : inst_index = `I_INST_ECALL;
			(is_ebreak) : inst_index = `I_INST_EBREAK;

			(is_csrrw)  : inst_index = `I_INST_CSRRW;
			(is_csrrs)  : inst_index = `I_INST_CSRRS;
			(is_csrrc)  : inst_index = `I_INST_CSRRC;
			(is_csrrwi) : inst_index = `I_INST_CSRRWI;
			(is_csrrsi) : inst_index = `I_INST_CSRRSI;
			(is_csrrci) : inst_index = `I_INST_CSRRCI;

			(is_mul)    : inst_index = `R_INST_MUL;
			(is_mulh)   : inst_index = `R_INST_MULH;
			(is_mulhsu) : inst_index = `R_INST_MULHSU;
			(is_mulhu)  : inst_index = `R_INST_MULHU;
			(is_div)    : inst_index = `R_INST_DIV;
			(is_divu)   : inst_index = `R_INST_DIVU;
			(is_rem)    : inst_index = `R_INST_REM;
			(is_remu)   : inst_index = `R_INST_REMU;

			(is_uret)   : inst_index = `I_INST_URET;
			(is_sret)   : inst_index = `I_INST_SRET;
			(is_mret)   : inst_index = `I_INST_MRET;
			(is_wfi)    : inst_index = `I_INST_WFI;
			default     : inst_index = `INST_NULL;
		endcase
		end
		`endif
endmodule