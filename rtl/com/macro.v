//宏定义
//`define INTEL_FPGA

`define ROM_BASE_ADDR        32'h0
`define ROM_WIDTH_IN_BYTE    32'h20000

`define RAM_BASE_ADDR        32'h20000
`define RAM_WIDTH_IN_BYTE    32'h20000

// `define RAM

`ifdef  RAM
`define RST_PC              `RAM_BASE_ADDR
`endif

`ifdef  ROM
`define RST_PC              `ROM_BASE_ADDR
`endif


`define ILEN32_BASE_TYPE     2'b11
`define ILEN32_EXT_TYPE      3'b111  
`define INST_ILEGAL_TYPE_ONE 32'h00000000
`define INST_ILEGAL_TYPE_TWO 32'hffffffff
`define INST_LEGAL           1'b1
`define INST_ILEGAL          1'b0
`define IS_ILEN32            1'b1
`define IS_ILEN16            1'b0

`define PC_ILEN32            32'h4
`define PC_ILEN16            32'h2

`define ASSERT            1'b1
`define BYTE_ASSERT       8'hff
`define HALF_ASSERT       16'hffff
`define WORD_ASSERT       32'hffff_ffff
`define DWORD_ASSERT      32'hffff_ffff_ffff_ffff
`define DEASSERT          1'b0
`define BYTE_DEASSERT     8'h0
`define HALF_DEASSERT     16'h0
`define WORD_DEASSERT     32'h0
`define DWORD_DEASSERT    64'h0
`define BYTE_WSTB         4'b0001
`define HALF_WSTB         4'b0011
`define WORD_WSTB         4'b1111

// define bpu_parameter
`define BPU_FUNC0

`ifndef BPU_FUNC0
    `define BPU_FUNC1
`endif

`ifdef  BPU_FUNC0
    `define BP_ADDR_DEPTH     64
    `define BP_ADDR_BITS      32
    `define BP_ADDR_W         $clog2(`BP_ADDR_DEPTH)
`endif

`ifdef  BPU_FUNC1
    `define BP_ADDR_DEPTH     8192
    `define BP_ADDR_BITS      $clog2(`BP_ADDR_DEPTH)
    `define BP_ADDR_W         1
`endif

//I type指令
`define I_INST_OPCODE_1    5'b00000 //opcode[6:2]
`define I_INST_FUNC3_LB    3'b000 //func3[14:12]
`define I_INST_FUNC3_LH    3'b001
`define I_INST_FUNC3_LW    3'b010
`define I_INST_FUNC3_LBU   3'b100
`define I_INST_FUNC3_LHU   3'b101

//I type指令
`define I_INST_OPCODE_2    5'b00100
`define I_INST_FUNC3_ADDI  3'b000
`define I_INST_FUNC3_SLTI  3'b010
`define I_INST_FUNC3_SLTIU 3'b011
`define I_INST_FUNC3_XORI  3'b100
`define I_INST_FUNC3_ORI   3'b110
`define I_INST_FUNC3_ANDI  3'b111
`define I_INST_FUNC3_SLLI  3'b001
`define I_INST_FUNC3_SRLI  3'b101
`define I_INST_FUNC3_SRAI  3'b101
`define I_INST_FUNC7_SLLI_SRLI 7'h0
`define I_INST_FUNC7_SRAI  7'b0100000 //func7[31:25]
`define I_INST_NOP         32'h00000013

//I type指令
`define I_INST_OPCODE_3      5'b00011 //opcode[6:2]
`define I_INST_FUNC3_FENCE   3'b000
`define I_INST_FUNC3_FENCEI  3'b001

//I type指令
`define I_INST_OPCODE_4           5'b11100 //opcode[6:2]
`define I_INST_FUNC3              3'b000
`define I_INST_FUNC5_TRI          5'b00010 
`define I_INST_FUNC5_IMI          5'b00101 
`define I_INST_FUNC7_URET         7'h0
`define I_INST_FUNC7_SRET_WFI     7'b0001000
`define I_INST_FUNC7_MRET         7'b0011000    
`define I_INST_FUNC12_ECALL       12'h0 //func12[31:20]
`define I_INST_FUNC12_EBREAK      12'h1
`define I_INST_FUNC12_URET        12'h2
`define I_INST_FUNC12_SRET        12'h102
`define I_INST_FUNC12_MRET        12'h302
`define I_INST_FUNC12_WFI         12'h105

//I type csr指令
`define I_INST_FUNC3_CSRRW  3'b001
`define I_INST_FUNC3_CSRRS  3'b010
`define I_INST_FUNC3_CSRRC  3'b011
`define I_INST_FUNC3_CSRRWI 3'b101
`define I_INST_FUNC3_CSRRSI 3'b110
`define I_INST_FUNC3_CSRRCI 3'b111

//I type指令 JALR
`define I_INST_OPCODE_5     5'b11001 //opcode[6:2]
`define I_INST_OPCODE_JALR  3'b000

//U type指令
`define U_INST_OPCODE_1     3'b101 //opcode[4:2]
`define U_INST_OPCODE_LUI   2'b01
`define U_INST_OPCODE_AUIPC 2'b00 //opcode[6:5]

//J type指令
`define J_INST_OPCODE_JAL   5'b11011

//B type指令
`define B_INST_OPCODE_1     5'b11000
`define B_INST_FUNC3_BEQ    3'b000
`define B_INST_FUNC3_BNE    3'b001
`define B_INST_FUNC3_BLT    3'b100
`define B_INST_FUNC3_BGE    3'b101
`define B_INST_FUNC3_BLTU   3'b110
`define B_INST_FUNC3_BGEU   3'b111

//S type指令
`define S_INST_OPCODE_1     5'b01000
`define S_INST_FUNC3_SB     3'b000
`define S_INST_FUNC3_SH     3'b001
`define S_INST_FUNC3_SW     3'b010 

//R type指令
`define R_INST_OPCODE_1     5'b01100
`define R_INST_FUNC3_ADD    3'b000
`define R_INST_FUNC3_SUB    3'b000
`define R_INST_FUNC3_SLL    3'b001
`define R_INST_FUNC3_SLT    3'b010
`define R_INST_FUNC3_SLTU   3'b011
`define R_INST_FUNC3_XOR    3'b100
`define R_INST_FUNC3_SRL    3'b101
`define R_INST_FUNC3_SRA    3'b101
`define R_INST_FUNC3_OR     3'b110
`define R_INST_FUNC3_AND    3'b111

`define R_INST_FUNC7_SUB_SRA  7'b0100000
`define R_INST_FUNC7_NULL     7'h0

`define R_INST_OPCODE_2       5'b01100
`define R_INST_FUNC3_MUL      3'b000
`define R_INST_FUNC3_MULH     3'b001
`define R_INST_FUNC3_MULHSU   3'b010
`define R_INST_FUNC3_MULHU    3'b011
`define R_INST_FUNC3_DIV      3'b100
`define R_INST_FUNC3_DIVU     3'b101
`define R_INST_FUNC3_REM      3'b110
`define R_INST_FUNC3_REMU     3'b111

`define R_INST_FUNC7_M        7'h1


//imm_src
`define U_IMM                 3'b000
`define J_IMM                 3'b001
`define I_IMM                 3'b010
`define B_IMM                 3'b011
`define S_IMM                 3'b100
`define Z_IMM                 3'b101

//alu_op_src
`define ALU_OP_ADD            4'b0000 //0
`define ALU_OP_SLL            4'b0001 //1
`define ALU_OP_SEQ            4'b0010 //2
`define ALU_OP_SNE            4'b0011 //3
`define ALU_OP_XOR            4'b0100 //4
`define ALU_OP_SRL            4'b0101 //5
`define ALU_OP_OR             4'b0110 //6
`define ALU_OP_AND            4'b0111 //7
`define ALU_OP_SUB            4'b1010 //10
`define ALU_OP_SRA            4'b1011 //11
`define ALU_OP_SLT            4'b1100 //12
`define ALU_OP_SGE            4'b1101 //13
`define ALU_OP_SLTU           4'b1110 //14
`define ALU_OP_SGEU           4'b1111 //15


//mem_byte_sel
`define MEM_BYTE_S            3'b000 //0
`define MEM_BYTE_U            3'b001 //1
`define MEM_HALF_S            3'b010 //2
`define MEM_HALF_U            3'b011 //3
`define MEM_WORD_S            3'b100 //4
// `define MEM_WORD_U            3'b000 //5
// `define MEM_DWORD_S           3'b000 //6

// //mem_wstb
// `define MEM_WSTB_0         4'b0000 //0
// `define MEM_WSTB_1            4'b0001 //1
// `define MEM_WSTB_2            4'b0010 //2
// `define MEM_WSTB_3            4'b0011 //3
// `define MEM_WSTB_4            4'b0100 //4


//inst_index[7:0]支持256条指令识别索引，顺序folow risc-v spec，基本指令集 + fencei + CSR + M扩展，55条指令，-IM 