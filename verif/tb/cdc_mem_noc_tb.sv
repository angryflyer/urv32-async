`timescale 1 ns / 1 ps


module cdc_mem_noc_tb
    import urv_cfg::*;
    import urv_typedef::*; 
;

logic            clock;
logic            rstn_d;
logic            clock_slow;
logic            rstn_q;

logic            mem_req_valid;  // strobe/request
logic            mem_req_ready;
mem_req_t        mem_req;  // strobe/request



integer i,j;

// External clock is used by default.  Make this artificially fast for the
// simulation.  Normally this would be a slow clock and the digital PLL
// would be the fast clock.

always #5 clock      <= (clock      === 1'b0);
always #5 clock_slow <= (clock_slow === 1'b0);

initial begin
    clock = 0;
    clock_slow = 0;
    rstn_d = 0;
    rstn_q = 0;
end


    initial begin
    $fsdbDumpfile("cdc_mem_noc_tb.fsdb");
    $fsdbDumpvars(0, cdc_mem_noc_tb, "+all");
    end

initial begin
    clock = 0;
    mem_req_valid ='h0;  // strobe/request
    // mem_req_ready ='h0;
    mem_req.req_addr  ='h0;  // strobe/request
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
   repeat (1) @(posedge clock);
   #1;
   mem_req_valid ='h0;
   $display("###################################################");
   #100
   $finish;
end

logic mem_req_valid_q;  // strobe/request
logic mem_req_ready_q;
mem_req_t mem_req_q;  // strobe/request

cdc_mem_noc cdc_mem_noc_u(
    .src_clk(clock),
    .src_rstn(rstn_d),
    .src_req_valid(mem_req_valid),
    .src_req_ready(mem_req_ready),
    .src_req(mem_req),

    .dest_clk(clock_slow),
    .dest_rstn(rstn_q),
    .dest_req_valid(mem_req_valid_q),
    .dest_req_ready(1'b1),
    .dest_req(mem_req_q)
);

task mem_write;
input [31:0] data;
begin
   repeat (1) @(posedge clock);
   #1;
   mem_req_valid  ='h1;  // write
   mem_req.req_addr   =data;  // data output
   wait(mem_req_ready == 'h1);
//    repeat (1) @(posedge clock);
//    #1;
//    mem_req_valid  ='h0;  // write
//    mem_req.req_addr   ='h0;  // data output
end
endtask

endmodule
