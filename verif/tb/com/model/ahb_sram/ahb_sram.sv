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
   output                     hreadyout,
   output                     hresp,
   output [DATA_WIDTH-1:0]    hrdata

);


reg [31:0] mem[MEMSIZE/4];

assign hreadyout = 1'b1;
assign hresp = 1'b0;

wire [ADDR_WIDTH-1:0] addr_offset;
assign addr_offset = haddr - base_addr;

reg [$clog2(MEMSIZE)-3:0] offset_dff;
reg data_phase_dff;
reg write_dff;
reg [DATA_WIDTH-1:0] hrdata;

wire htrans_valid;
assign htrans_valid = ((htrans==2'b10)||(htrans==2'b11))&&(haddr>=base_addr);

always @(posedge clk or negedge rstn) begin
   if(~rstn) begin
       hrdata <= 0;
       offset_dff <= 0;
       write_dff <= 0;
       data_phase_dff <= 0;
   end
   else begin
           if(hready==1'b1) begin
               if(htrans_valid) begin
                   offset_dff     <= addr_offset[$clog2(MEMSIZE)-3:2];
                   write_dff      <= hwrite;
                   data_phase_dff <= 1;
                   if(hwrite==0) begin
                       hrdata         <= mem[addr_offset[$clog2(MEMSIZE)-3:2]];
                   end
               end
               else begin
                  data_phase_dff  <= 0;
               end

               if(data_phase_dff==1) begin
                    if(write_dff==1) begin
                        mem[offset_dff] = hwdata;
                    end
               end
           end
   end
end

endmodule