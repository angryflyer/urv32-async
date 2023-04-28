module sdp_agent
(
    input         clk,
    input  [15:0] div_coe,
    output        sdp_ck,
    inout         sdp_dio,
    input         sdp_di,
    output reg    sdp_do,
    output reg    sdp_doen
);

parameter N_AW = 32;
parameter N_DW = 32;
parameter N_DM = 4;

reg [15:0] clk_count;
reg write_done;
reg read_done;
reg [99:0] cnt;
reg clk_div;

// tri/inout sim
assign sdp_di   = sdp_doen ? sdp_dio : 1'bz;
assign sdp_dio = sdp_doen ? 1'bz : sdp_do;

initial 
begin
    sdp_doen = 1'b1;
	sdp_do = 1'b1;
 	clk_div = 0;
	clk_count = 0;
end

always @(posedge clk)
begin
   if (clk_count == 'h0) begin
        clk_div  = ~clk_div;
        clk_count = div_coe;	
   end else begin
        clk_count = clk_count - 1;	
   end
end

// assign sdp_ck = (div_coe == 'h0) ? clk : clk_div;
assign sdp_ck = clk_div;

task sdp_mem_write;
    input [N_AW-1:0] addr;
    input [N_DW-1:0] data;
int i;

begin
    $display("run sdp write operation, addr is %h, write data is %h", addr, data);
        cnt = 100'b1;
    @(posedge sdp_ck);
        write_done = 1'b0;
        sdp_doen  = 1'b0;
  //default is high level
    @(posedge sdp_ck);
        sdp_do = 1'b1;
  //sent start flag
    @(posedge sdp_ck);
        sdp_do = 1'b0;
  //sent write flag
    @(posedge sdp_ck);
        sdp_do = 1'b1;
  //sent addr 
  for(i=N_AW-1; i>= 0; i--) begin : GEN_WRITE_ADDR
    @(posedge sdp_ck);
        sdp_do = addr[i];
        cnt = cnt + addr[i];
  end 
  //sent N_DM 
  for(i=N_DM-1; i>= 0; i--) begin : GEN_WRITE_STRB_W
    @(posedge sdp_ck);
        sdp_do = 1'b1;
  end 
  //sent data 
  for(i=N_DW-1; i>= 0; i--) begin : GEN_WRITE_DATA
    @(posedge sdp_ck);
        sdp_do = data[i];
        cnt = cnt + data[i];
  end 
  //sent parity flag
    @(posedge sdp_ck);
        sdp_do = cnt[0] ;
  //sent stop flag
    @(posedge sdp_ck);
        sdp_do = 1'b1;
    @(posedge sdp_ck);
        sdp_doen  = 1'b1;
    @(negedge sdp_di);
    repeat (2) @(posedge sdp_ck);
        if(sdp_di != 1'b0)
    $display("ERROR: write ack signal error!");
    @(negedge sdp_ck);
        //if(sdp_di != cnt[0])
        if(sdp_di != 1'b0)
    $display("ERROR: write parity check error!");
        write_done = 1'b1;
end

endtask

task sdp_mem_read;
    input [N_AW-1:0] addr;
    int i;
    output [N_DW-1:0] data;

begin
    @(posedge sdp_ck);
        read_done = 1'b0;
        sdp_doen  = 1'b0;
        cnt = 100'b0;
        data = 64'h0;
        //default is high level
    @(posedge sdp_ck);
    @(posedge sdp_ck);
        sdp_do = 1'bz;
        //sent start flag
    @(posedge sdp_ck);
        sdp_do = 1'b0;
        //sent read flag
    @(posedge sdp_ck);
        sdp_do = 1'b0;
        //sent addr 
        for(i=N_AW-1; i>= 0; i--) begin : GEN_READ_ADDR
            @(posedge sdp_ck);
                sdp_do = addr[i];
                cnt = cnt + addr[i];
        end 
    //sent parity flag
    @(posedge sdp_ck);
        sdp_do = cnt[0] ;
    //sent stop flag
    @(posedge sdp_ck);
        sdp_do = 1'b1;
    @(posedge sdp_ck);
        sdp_doen  = 1'b1;
        //clear parity count
        cnt = 100'b0;
    @(negedge sdp_di);
        repeat (2) @(negedge sdp_ck);
        if(sdp_di != 1'b0)
        $display("ERROR: read ack signal error!");
        for(i=N_DW-1; i>= 0; i--) begin : GEN_READ_DATA
            @(negedge sdp_ck);
                data[i] = sdp_di;
                cnt = cnt + data[i];
        end 
    @(negedge sdp_ck);
        if(sdp_di != cnt[0]) begin
            $display("ERROR: read parity check error!");
        end
        $display("run sdp read operation, addr is %h, read data is %h", addr, data);
        read_done = 1'b1;
end

endtask


endmodule