//inst_index[7:0]支持256条指令识别索引，顺序folow risc-v spec，基本指令集 + fencei + CSR + M扩展，55条指令，-IM 

//`define SYNTHESIS

`ifndef SYNTHESIS
    //需要更多的LE资源，但便于simulation观察
	`define INST_NULL     "NOP"

    `define U_INST_LUI    "LUI"
    `define U_INST_AUIPC  "AUIPC"

    `define J_INST_JAL    "JAL"

    `define I_INST_JALR   "JALR"

    `define B_INST_BEQ    "BEQ"
    `define B_INST_BNE    "BNE"

    `define B_INST_BLT    "BLT"
    `define B_INST_BGE    "BGE"
    `define B_INST_BLTU   "BLTU"
    `define B_INST_BGEU   "BGEU"

    `define I_INST_LB     "LB"
    `define I_INST_LH     "LH"
    `define I_INST_LW     "LW"
    `define I_INST_LBU    "LBU"
    `define I_INST_LHU    "LHU"

    `define S_INST_SB     "SB"
    `define S_INST_SH     "SH"
    `define S_INST_SW     "SW"

    `define I_INST_ADDI   "ADDI"
    `define I_INST_SLTI   "SLTI"
    `define I_INST_SLTIU  "SLTIU"
    `define I_INST_XORI   "XORI"
    `define I_INST_ORI    "ORI"
    `define I_INST_ANDI   "ANDI"
    `define I_INST_SLLI   "SLLI"
    `define I_INST_SRLI   "SRLI"
    `define I_INST_SRAI   "SRAI"

    `define R_INST_ADD    "ADD"
    `define R_INST_SUB    "SUB"
    `define R_INST_SLL    "SLL"
    `define R_INST_SLT    "SLT"
    `define R_INST_SLTU   "SLTU"
    `define R_INST_XOR    "XOR"
    `define R_INST_SRL    "SRL"
    `define R_INST_SRA    "SRA"
    `define R_INST_OR     "OR"
    `define R_INST_AND    "AND"

    `define I_INST_FENCE  "FENCE"

    //RV32/RV64 Zifencei Standard Extension
    `define I_INST_FENCEI "FENCEI"

    `define I_INST_ECALL  "ECALL"
    `define I_INST_EBREAK "EBREAK"

    //RV32/RV64 Zicsr Standard Extension
    `define I_INST_CSRRW  "CSRRW"
    `define I_INST_CSRRS  "CSRRS"
    `define I_INST_CSRRC  "CSRRC"
    `define I_INST_CSRRWI "CSRRWI"
    `define I_INST_CSRRSI "CSRRSI"
    `define I_INST_CSRRCI "CSRRCI"

    //RV32M Standard Extension
    `define R_INST_MUL    "MUL"
    `define R_INST_MULH   "MULH"
    `define R_INST_MULHSU "MULHSU"
    `define R_INST_MULHU  "MULHU"
    `define R_INST_DIV    "DIV"
    `define R_INST_DIVU   "DIVU"
    `define R_INST_REM    "REM"
    `define R_INST_REMU   "REMU"

    // TRI
    `define I_INST_URET   "URET"
    `define I_INST_SRET   "SRET"
    `define I_INST_MRET   "MRET"

    // IMI
    `define I_INST_WFI    "WFI"
`else

	`define INST_NULL     8'hff

    `define U_INST_LUI    8'd0 
    `define U_INST_AUIPC  8'd1

    `define J_INST_JAL    8'd2

    `define I_INST_JALR   8'd3

    `define B_INST_BEQ    8'd4
    `define B_INST_BNE    8'd5

    `define B_INST_BLT    8'd6
    `define B_INST_BGE    8'd7
    `define B_INST_BLTU   8'd8
    `define B_INST_BGEU   8'd9

    `define I_INST_LB     8'd10
    `define I_INST_LH     8'd11
    `define I_INST_LW     8'd12
    `define I_INST_LBU    8'd13
    `define I_INST_LHU    8'd14

    `define S_INST_SB     8'd15
    `define S_INST_SH     8'd16
    `define S_INST_SW     8'd17

    `define I_INST_ADDI   8'd18
    `define I_INST_SLTI   8'd19
    `define I_INST_SLTIU  8'd20
    `define I_INST_XORI   8'd21
    `define I_INST_ORI    8'd22
    `define I_INST_ANDI   8'd23
    `define I_INST_SLLI   8'd24
    `define I_INST_SRLI   8'd25
    `define I_INST_SRAI   8'd26

    `define R_INST_ADD    8'd27
    `define R_INST_SUB    8'd28
    `define R_INST_SLL    8'd29
    `define R_INST_SLT    8'd30
    `define R_INST_SLTU   8'd31
    `define R_INST_XOR    8'd32
    `define R_INST_SRL    8'd33
    `define R_INST_SRA    8'd34
    `define R_INST_OR     8'd35
    `define R_INST_AND    8'd36

    `define I_INST_FENCE  8'd37

    //RV32/RV64 Zifencei Standard Extension
    `define I_INST_FENCEI 8'd38

    `define I_INST_ECALL  8'd39
    `define I_INST_EBREAK 8'd40

    //RV32/RV64 Zicsr Standard Extension
    `define I_INST_CSRRW  8'd41
    `define I_INST_CSRRS  8'd42
    `define I_INST_CSRRC  8'd43
    `define I_INST_CSRRWI 8'd44
    `define I_INST_CSRRSI 8'd45
    `define I_INST_CSRRCI 8'd46

    //RV32M Standard Extension
    `define R_INST_MUL    8'd47
    `define R_INST_MULH   8'd48
    `define R_INST_MULHSU 8'd49
    `define R_INST_MULHU  8'd50
    `define R_INST_DIV    8'd51
    `define R_INST_DIVU   8'd52
    `define R_INST_REM    8'd53
    `define R_INST_REMU   8'd54

    // TRI
    `define I_INST_URET   8'd55
    `define I_INST_SRET   8'd56
    `define I_INST_MRET   8'd57

    // IMI
    `define I_INST_WFI    8'd58
`endif
