`timescale 1 ns / 1 ps


module mem2ahb_tb;
    import urv_cfg::*;
    import urv_typedef::*; 

logic            clock;
logic            rstn;
logic            clock_slow;

logic            mem_req_valid;  // strobe/request
logic            mem_req_ready;
mem_req_t        mem_req;  // strobe/request

logic            mem_resp_valid;  // strobe/request
logic            mem_resp_ready;
mem_resp_t      mem_resp;  // strobe/request

    //ahb master to ahb slave
logic [1:0]              htrans;
logic [2:0]              hburst;
logic [32-1:0]           haddr;
logic [2:0]              hsize;
logic [32-1:0]           hwdata;
logic                    hwrite;
logic                    hmastlock;
logic [6:0]              hprot;
logic [32-1:0]           hrdata;
logic                    hresp;
logic                    hready;

integer i,j;

// External clock is used by default.  Make this artificially fast for the
// simulation.  Normally this would be a slow clock and the digital PLL
// would be the fast clock.

always #5 clock      <= (clock      === 1'b0);
always #5 clock_slow <= (clock_slow === 1'b0);

initial begin
    clock = 0;
    clock_slow = 0;
    rstn = 0;
end


    initial begin
    $fsdbDumpfile("mem2ahb_tb.fsdb");
    $fsdbDumpvars(0, mem2ahb_tb, "+all");
    end

initial begin
    clock = 0;
    mem_req_valid ='h0;  // strobe/request
    mem_req.req_addr  ='h0;  // strobe/request
    mem_resp_ready ='h0;
end
initial

begin
    $display("Monitor: Test Started");
    #20
    rstn = 1;
    #10
    repeat (1) @(posedge clock);
    #1;
    mem_write('h0,'h0);   
    mem_write('h4,'h4);
    mem_write('h8,'h8);
    mem_write('hc,'hc);
    mem_write('h10,'h10);
    mem_write('h14,'h14);
    mem_write('h18,'h18);
    mem_write('h1c,'h1c);   
    mem_write('h20,'h20);
    mem_write('h24,'h24);
    mem_write('h28,'h28);
    mem_write('h2c,'h2c);
    mem_write('h30,'h30);
    mem_write('h34,'h34);
    mem_write('h38,'h38);
    mem_req_valid ='h0;
    repeat (1) @(posedge clock);
    #1;
    mem_read('h0, 'h0);
    mem_read('h4, 'h4);
    mem_read('h8, 'h8);
    mem_read('hC, 'hc);
    mem_read('h10,'h10);
    mem_req_valid ='h0;
    repeat (1) @(posedge clock);
    #1;
    $display("###################################################");
    #100
    $finish;
end

mem2ahb mem2ahb_u (
    .clk(clock),
    .rstn(rstn),
    .mem_req_valid(mem_req_valid),
    .mem_req_ready(mem_req_ready),
    .mem_req(mem_req),
    .mem_resp_valid(mem_resp_valid),
    .mem_resp_ready(mem_resp_ready),
    .mem_resp(mem_resp),
    //ahb master to ahb slave
    .htrans(htrans),
    .hburst(hburst),
    .haddr(haddr),
    .hsize(hsize),
    .hwdata(hwdata),
    .hwrite(hwrite),
    .hmastlock(hmastlock),
    .hprot(hprot),
    .hrdata(hrdata),
    .hresp(hresp),
    .hready(hready)
);

ahb_sram #(
    .MEMSIZE(256)
) ahb_sram_u (
    .clk(clock),
    .rstn(rstn),
    .hsel(1'b1),
    .clk_strobe(1'b1),
    .base_addr(32'h0),
    .htrans(htrans),
    .hburst(hburst),
    .haddr(haddr),
    .hsize(hsize),
    .hwdata(hwdata),
    .hwrite(hwrite),
    .hready(hready),
    .hrdata(hrdata),
    .hresp(hresp),
    .hreadyout(hready)
);

task mem_write;
input [31:0] addr;
input [31:0] data;
    begin
        // repeat (1) @(posedge clock);
        // #1;
        mem_req_valid      = 1'b1;  // write
        mem_req.req_addr   = addr;       // data output
        mem_req.req_type   = MEM_WRITE;  // data output
        mem_req.req_mask   = 4'h1;
        mem_req.req_data   = data;
        mem_req.req_burst  = 'h1;
        wait(mem_req_ready == 1'b1);
        repeat (1) @(posedge clock);
        #1;
        // mem_req_valid  = 1'b0;  // write
        // mem_req.req_addr   ='h0;  // data output
        mem_resp_ready     = 1'b1;
        // mem_resp_ready = 0;
        if(~mem_resp_valid) mem_req_valid = 0;
        wait(mem_resp_valid == 1'b1);        
        // repeat (1) @(posedge clock);
        // #1;
        // mem_resp_ready     = 1'b1;
        
    end
endtask

task mem_read;
input [31:0] addr;
input [31:0] exp_data;
reg [31:0] data;
    begin
        // repeat (1) @(posedge clock);
        // #1;
        mem_req_valid      = 1'b1;  // write
        mem_req.req_addr   = addr;       // data output
        mem_req.req_type   = MEM_READ;  // data output
        mem_req.req_burst  = 'h1;
        wait(mem_req_ready == 1'b1);
        repeat (1) @(posedge clock);
        #1;
        // mem_req_valid      = 1'b0;  // write
        // mem_req.req_addr   ='h0;  // data output
        mem_resp_ready     = 1'b1;
        // mem_resp_ready = 0;
        wait(mem_resp_valid == 1'b1);        
        // repeat (1) @(posedge clock);
        // #1;
        // mem_resp_ready     = 1'b1;
        data = mem_resp.resp_data;
    end
    if(data == exp_data)
    begin
       $display("STATUS: ADDRESS: %x RD: %x", addr, data);
    end
    else begin
       $display("ERROR:  ADDRESS: %x EXP: %x RD: %x", addr, exp_data, data);
    end
endtask

endmodule
