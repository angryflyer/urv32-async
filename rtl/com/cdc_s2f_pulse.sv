// async_dmux
// slow to fast
// better Ffast >= 2 * Fslow
// do not support continue transfer  
`include "macro.v"
`include "component.v"

module cdc_s2f_pulse #(
    parameter W = 32
)
(
    input  logic src_clk,
    input  logic src_rstn,
    input  logic src_en,
    input  logic [W-1:0] src_in,

    input  logic dest_clk,
    input  logic dest_rstn,
    output logic dest_en,
    output logic [W-1:0] dest_out
);

logic rff_src_en;
stdffr #(1) ff_src_en_u (
    .clk(src_clk),
    .rstn(src_rstn),
    .d(src_en),
    .q(rff_src_en)
);

// 2cycles dest clock domain sync
logic rff_dest_en0;
stdffr #(1) ff_dest_en0_u (
    .clk(dest_clk),
    .rstn(dest_rstn),
    .d(rff_src_en),
    .q(rff_dest_en0)
);

logic rff_dest_en1;
stdffr #(1) ff_dest_en1_u (
    .clk(dest_clk),
    .rstn(dest_rstn),
    .d(rff_dest_en0),
    .q(rff_dest_en1)
);

// continue 1cycle
logic rff_dest_en2;
stdffr #(1) ff_dest_en2_u (
    .clk(dest_clk),
    .rstn(dest_rstn),
    .d(rff_dest_en1),
    .q(rff_dest_en2)
);

// 1cycle sync valid to data
logic rff_dest_en;
stdffr #(1) ff_dest_en_u (
    .clk(dest_clk),
    .rstn(dest_rstn),
    .d(dmux_sel),
    .q(rff_dest_en)
);

// data register with src clk
logic [W-1:0] rff_src_in;
stdffr #(W) ff_src_in_u (
    .clk(src_clk),
    .rstn(src_rstn),
    .d(src_in),
    .q(rff_src_in)
);

// data register with dest clk
logic [W-1:0] rff_dest_out;
logic [W-1:0] dmux;
logic  dmux_sel;
stdffr #(W) ff_dest_out_u (
    .clk(dest_clk),
    .rstn(dest_rstn),
    .d(dmux),
    .q(rff_dest_out)
);
assign dmux_sel = (rff_dest_en1 && ~rff_dest_en2);
assign dmux     = dmux_sel ? rff_src_in : rff_dest_out;

assign dest_en  = rff_dest_en;
assign dest_out = rff_dest_out;

endmodule