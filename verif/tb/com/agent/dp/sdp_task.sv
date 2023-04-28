parameter N_AW = 32;
parameter N_DW = 32;
parameter N_DM = 4;

task sdp_write;
input [N_AW-1:0] addr;
input [N_DW-1:0] data;

begin
  tb_sdp_master.sdp_mem_write(addr, data);
end
endtask

task sdp_read;
input  [N_AW-1:0] addr;
output [N_DW-1:0] data;
reg    [N_DW-1:0] data;
begin
  tb_sdp_master.sdp_mem_read(addr, data);
end

endtask

task sdp_read_check;
input [N_AW-1:0] addr;
input [N_DW-1:0] exp_data;
reg   [N_DW-1:0] data;

begin
  tb_sdp_master.sdp_mem_read(addr, data);
end
  if(data == exp_data) begin
    // $display("STATUS: ADDRESS: %x RD: %x", addr, data);
  end else begin
    $display("ERROR:  ADDRESS: %x EXP: %x RD: %x", addr, exp_data, data);
    // check_fail = 1;
    $finish(0);
  end

endtask
