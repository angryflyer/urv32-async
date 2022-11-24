//流水线辅助控制器
//专用于结构冒险，数据冒险和控制冒险发生时的对流水线的控制
//注意该控制器不包括信息前递
`include "macro.v"

module acu #(
    parameter ADDR_W  = 32
) (
    input clk,
    input rstn,

    //from if
    input if2ac_hazard,
    //from id
    input id2ac_hazard,
    //from lsu
    input lsu2ac_hazard,
    //from wbu
    input wb2ac_hazard,
    //from debug
    input dbg2ac_stall,
    //from iex
    input iex2lsu_valid,
    //from iex
    input iex2ac_jump_valid,
    input [ADDR_W-1:0] iex2ac_jump_pc,
    // from csr
    input trap_jump_valid,
    input [ADDR_W-1:0] trap_jump_pc,    

    //flush_pc
    output [ADDR_W-1:0] ac2if_flush_pc,
    //flush 
    output ac2if_flush,
    output ac2id_flush,
    output ac2iex_flush,
    output ac2lsu_flush,

    //stall
    output ac2if_stall,
    output ac2id_stall,
    output ac2iex_stall,
    output ac2lsu_stall,
    //debug stall all, including ac2wb_stall
    output ac2wb_stall,
    output ac2csr_stall,

    output ac2csr_instret_stall
);

    wire   pipe_stall;
    assign pipe_stall       = lsu2ac_hazard | wb2ac_hazard | dbg2ac_stall;

    assign ac2if_stall      = pipe_stall | id2ac_hazard;
    assign ac2id_stall      = pipe_stall | (id2ac_hazard && ~ac2if_flush);
    assign ac2iex_stall     = pipe_stall;
    assign ac2lsu_stall     = pipe_stall;
    assign ac2wb_stall      = pipe_stall;
    assign ac2csr_stall     = pipe_stall | id2ac_hazard;

    assign ac2if_flush      = iex2ac_jump_valid | trap_jump_valid;
    assign ac2if_flush_pc   = trap_jump_valid ? trap_jump_pc : iex2ac_jump_pc;
    assign ac2id_flush      = id2ac_hazard | ac2if_flush;
    assign ac2iex_flush     = ac2iex_stall;
    assign ac2lsu_flush     = wb2ac_hazard;
    assign ac2csr_instret_stall = ~iex2lsu_valid;
endmodule