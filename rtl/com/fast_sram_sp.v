//===============================================
//Name          : fast_sram_sp
//Author        : angrybird
//Email         : 
//Date          : 2023-02-25
//Description   : single port sync sram
//                width can be configured
//===============================================
module fast_sram_sp 
#(
    //default size = NUM_DEPTH * (NUM_WIDTH / 8) = 2KB
    parameter N_DW = 32, 
    parameter N_DP = 512,
    parameter N_DM = N_DW / 8,
    parameter N_AW = N_DP == 1 ? 1 : $clog2(N_DP)
) (
    input  wire            clk,
    input  wire            csn, // chip select/enable, active low
    input  wire            wen, // write enable, active low
    input  wire [N_DM-1:0] web, // byte select, active low
    input  wire [N_AW-1:0] addr,
    input  wire [N_DW-1:0] din,
    output wire [N_DW-1:0] dout
);

// declare mem
reg  [N_DW-1:0] mem[N_DP-1:0];

// write mem
wire wr_en;
wire [N_DM-1:0] wr_mask;
assign wr_en = ~csn & ~wen;
assign wr_mask = {(N_DM){wr_en}} & ~web;

always @(posedge clk) begin
    for(int i = 0; i < N_DM; i++) begin: WRITE_MEM_GEN
        if(wr_mask[i]) mem[addr][(i+1)*8 - 1 -: 8] <= din[(i+1)*8 - 1 -: 8];
    end
end

// read mem
wire rd_en;
reg  [N_DW-1:0] ff_dout;
assign rd_en = ~csn & wen;
always @(posedge clk) begin
    // ff_dout <= rd_en ? mem[addr] : {N_DW{1'b0}};
    if(rd_en) ff_dout <= mem[addr];
end

assign dout = ff_dout;

endmodule