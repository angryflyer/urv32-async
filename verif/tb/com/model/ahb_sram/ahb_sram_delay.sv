module ahb_sram #(
  parameter ADDR_WIDTH=32,
  parameter DATA_WIDTH=32,
  parameter MEMSIZE=4096
)(
   input                      clk,
   input                      rstn,
   input                      clk_strobe,
   input                      hsel,
   input [ADDR_WIDTH-1:0]     base_addr,
   input [ADDR_WIDTH-1:0]     haddr,
   input                      hwrite,
   input [2:0]                hsize,
   input [2:0]                hburst,
   input [1:0]                htrans,
   input                      hready,
   input [DATA_WIDTH-1:0]     hwdata,
   output reg                 hreadyout,
   output reg                 hresp,
   output reg [DATA_WIDTH-1:0]    hrdata,
   input [6:0]                hprot,
   input                      hnonsec,
   input [3:0]                hmaster,
   input                      hexcl,
   output                     hexokay   
);


reg [31:0] mem[MEMSIZE/4];

assign hexokay = 1'b1;

wire signed [ADDR_WIDTH-1:0] addr_offset;
assign addr_offset = haddr - base_addr;

reg [$clog2(MEMSIZE)-1:0] offset_dff;
reg data_phase_dff;
reg write_dff;
// reg [DATA_WIDTH-1:0] hrdata;
reg [2:0] hsize_dff;
reg [DATA_WIDTH-1:0] temp_data;
// reg hreadyout;
// reg hresp;

int hready_dly_en;
int hready_dly_cnt;
int bus_error_rand;
bit to_send_error;
bit [31:0] error_addr;

bit en_very_long_delay = 0;
bit en_bus_error = 0;

initial begin
   if($test$plusargs("ahb_sram_long_delay"))begin
       en_very_long_delay = 1;
   end
   if($test$plusargs("ahb_sram_bus_error"))begin
       en_bus_error = 1;
   end
end


always @(posedge clk or rstn) begin
   if(rstn==1'b0) begin
       hrdata = 0;
       offset_dff = 0;
       write_dff = 0;
       data_phase_dff = 0;
       hsize_dff = 0;
       hreadyout = 1;
       hresp = 0;
       hready_dly_cnt = 0;
       to_send_error = 0;
   end
   else if(clk_strobe == 1'b1) begin
           if(hready==1'b1) begin
               if((htrans==2'b10)||(htrans==2'b11)) begin
                   if((haddr>=base_addr)&&(haddr<base_addr+MEMSIZE)&&(hsel==1'b1)) begin
                       offset_dff     <= addr_offset[$clog2(MEMSIZE)-1:2];
                       write_dff      <= hwrite;
                       hsize_dff      <= hsize;
                       data_phase_dff <= 1;
                       if(hwrite==0) begin
                           hrdata         <= mem[addr_offset[$clog2(MEMSIZE)-1:2]];
                       end
                       if(en_bus_error) begin
                           bus_error_rand = $urandom_range(4,0);
                           if(bus_error_rand==0) begin
                               to_send_error = 1;
                               error_addr = haddr;
                           end
                           else begin
                               to_send_error = 0;
                               hresp <= 0;
                           end
                       end
                       else begin
                           to_send_error = 0;
                           hresp <= 0;
                       end
                       hready_dly_en = $urandom_range(1,0);
                       if(to_send_error==1) hready_dly_en = 1;
                       if(hready_dly_en==1) begin
                           if(en_very_long_delay) begin
                               hready_dly_cnt = $urandom_range(10000,1000);
                           end
                           else begin
                               hready_dly_cnt = $urandom_range(5,2);
                           end
                           hreadyout <= 0;
                           if(to_send_error==1) begin
                               if(hready_dly_cnt==1) begin
                                  hresp <= 1;
                               end
                               else begin
                                 hresp <= 0;
                               end
                           end
                       end
                   end
                   else begin
                       data_phase_dff <= 0;
                       if(hwrite==0) begin
                           hrdata       <= 32'h0;
                       end
                   end
               end
               else begin
                  data_phase_dff  <= 0;
                  if(to_send_error==1) begin
                      hresp <= 0;
                      to_send_error = 0;
                  end
               end

               if(data_phase_dff==1) begin
                    if(write_dff==1) begin
                        temp_data = mem[offset_dff];
                        if(hsize_dff==0) begin
                            temp_data[offset_dff[1:0]*8+:8] = hwdata[offset_dff[1:0]*8+:8];
                        end
                        else if(hsize_dff==1) begin
                            temp_data[offset_dff[0]*16+:16] = hwdata[offset_dff[0]*16+:16];
                        end
                        else if(hsize_dff==2) begin
                            temp_data = hwdata;
                        end
                        mem[offset_dff] = temp_data;
                    end
               end
           end
           else begin
               if(hready_dly_cnt > 1) begin
                    if(to_send_error==1) begin
                        if(hready_dly_cnt==2) begin
                           hresp <= 1;
                        end
                        else begin
                           hresp <= 0;
                        end
                    end
                    else begin
                        hresp <= 0;
                    end
                    hready_dly_cnt = hready_dly_cnt - 1;
               end
               else begin
                    if(to_send_error==1) begin
                        hresp <= 1;
                        $display("ahb_sram send bus_error at haddr=%x time=%t", error_addr, $time);
                    end
                    hreadyout <= 1;
               end
           end
   end
end


endmodule
