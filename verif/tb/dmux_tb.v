`timescale 1 ns / 1 ps


module dmux_tb;

reg            clock;
reg            rstn_d;
reg            clock_slow;
reg            rstn_q;

reg            mem_valid;  // strobe/request
reg            mem_ready;
reg  [31:0]    mem_data;  // strobe/request



integer i,j;

// External clock is used by default.  Make this artificially fast for the
// simulation.  Normally this would be a slow clock and the digital PLL
// would be the fast clock.

always #12.5 clock <= (clock === 1'b0);
always #5    clock_slow <= (clock_slow === 1'b0);

initial begin
    clock = 0;
    clock_slow = 0;
    rstn_d = 0;
    rstn_q = 0;
end


    initial begin
    $fsdbDumpfile("dmux_tb.fsdb");
    $fsdbDumpvars(0, dmux_tb, "+all");
    end

initial begin
    clock = 0;
    mem_valid ='h0;  // strobe/request
    mem_ready ='h0;
    mem_data  ='h0;  // strobe/request
end
initial

begin
   $display("Monitor: Test Started");
   #20
   rstn_d = 1;
   rstn_q = 1;
   #10
   mem_write('h1);   
   mem_write('h2);
   mem_write('h3);
   mem_write('h4);
   mem_write('h5);
   mem_write('h6);
   mem_write('h7);
   mem_write('h8);   
   mem_write('h9);
   mem_write('ha);
   mem_write('hb);
   mem_write('hc);
   mem_write('hd);
   mem_write('he);
   mem_write('hf);
   $display("###################################################");
   #100
   $finish;
end

wire mem_valid_q;  // strobe/request
wire [31:0] mem_data_q;  // strobe/request

async_dmux async_dmux_u(
    .clk_d(clock),
    .rstn_d(rstn_d),
    .val_d(mem_valid),
    .d(mem_data),

    .clk_q(clock_slow),
    .rstn_q(rstn_q),
    .val_q(mem_valid_q),
    .q(mem_data_q)
);

task mem_write;
input [31:0] data;
begin
   repeat (1) @(posedge clock);
   #1;
   mem_valid  ='h1;  // write
   mem_data   =data;  // data output
//    wait(mem_ready == 'h1)
   repeat (1) @(posedge clock);
   #1;
   mem_valid  ='h0;  // write
   mem_data   ='h0;  // data output
end
endtask

endmodule
