// async_dmux
// slow to fast
// better Ffast >= 2 * Fslow
// do not support continue transfer  
`include "macro.v"
`include "component.v"

module async_dmux #(
    parameter W = 32
)
(
    input  logic clk_d,
    input  logic rstn_d,
    input  logic val_d,
    input  logic [W-1:0] d,

    input  logic clk_q,
    input  logic rstn_q,
    output logic val_q,
    output logic [W-1:0] q
);

logic rff_val_d;
stdffr #(1) ff_val_d_u (
    .clk(clk_d),
    .rstn(rstn_d),
    .d(val_d),
    .q(rff_val_d)
);

// 2cycles dest clock domain sync
logic rff_val_q0;
stdffr #(1) ff_val_q0_u (
    .clk(clk_q),
    .rstn(rstn_q),
    .d(rff_val_d),
    .q(rff_val_q0)
);

logic rff_val_q1;
stdffr #(1) ff_val_q1_u (
    .clk(clk_q),
    .rstn(rstn_q),
    .d(rff_val_q0),
    .q(rff_val_q1)
);

// continue 1cycle
logic rff_val_q2;
stdffr #(1) ff_val_q2_u (
    .clk(clk_q),
    .rstn(rstn_q),
    .d(rff_val_q1),
    .q(rff_val_q2)
);

// 1cycle sync valid to data
logic rff_val_q;
stdffr #(1) ff_val_q_u (
    .clk(clk_q),
    .rstn(rstn_q),
    .d(dmux_sel),
    .q(rff_val_q)
);

// data register with src clk
logic [W-1:0] rff_d;
stdffr #(W) ff_d_u (
    .clk(clk_d),
    .rstn(rstn_d),
    .d(d),
    .q(rff_d)
);

// data register with dest clk
logic [W-1:0] rff_q;
logic [W-1:0] dmux;
logic  dmux_sel;
stdffr #(W) ff_q_u (
    .clk(clk_q),
    .rstn(rstn_q),
    .d(dmux),
    .q(rff_q)
);
assign dmux_sel = (rff_val_q1 && ~rff_val_q2);
assign dmux     = dmux_sel ? rff_d : rff_q;

assign val_q    = rff_val_q;
assign q        = rff_q;

endmodule