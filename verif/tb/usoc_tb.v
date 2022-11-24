
`include "inst_index.v"
`include "macro.v"
`timescale 1ns / 1ns

//SIM_FREQ unit:MHz
`ifndef SIM_FREQ
`define SIM_FREQ 32'd32
`endif

//SIM_PERIOD unit:ns
`ifndef SIM_PERIOD
`define SIM_PERIOD 15.625
`endif

`ifndef SIM_TIME
`define SIM_TIME 32'd10
`endif


module usoc_tb;
    reg clk;
    reg rstn;

    wire iex2ac_jump_valid;
    wire [31:0] iex2ac_jump_pc;
    wire [31:0] iex2lsu_pc;

    reg [31:0] log;
    initial begin      
        log = $fopen("./vcs_log_async.txt","w");
    end

    localparam SIMULATION_PERIOD = `SIM_PERIOD;
    initial begin
        clk = 0;
        $display("\nSIMULATION_PERIOD %.3fns with frequency %2dMHz", SIMULATION_PERIOD, `SIM_FREQ);
        // repeat(1000000) #15.625 clk = ~clk;
        forever #(SIMULATION_PERIOD) clk = ~clk;
        // forever #20 clk = ~clk;
    end

    initial begin
        rstn = 0;
        #50 rstn = 1;
    end

    //Use Backdoor Load image
    initial begin
        string     image_path;
        int        image_fd;
        bit [31:0] data;
        logic[31:0]bdata;
        int        num_byte;
        bit [31:0] wdata;
        bit [31:0] addr;
        bit [31:0] image_addr;
        bit [31:0] adr;
        //Loading Image
        if ($value$plusargs("backdoor_load_image=%s", image_path)) begin
            // $display("Start Loading image from backdoor");
            // $display("backdoor load image = %s", image_path);
            image_fd = $fopen(image_path, "rb");
        if (image_fd == 0)
            $display("TOP", $sformatf("Unable to open %s.bin for read", image_path));
            image_addr = `RST_PC;
            addr = image_addr;
            num_byte = $fread(bdata, image_fd);
        while (num_byte > 0) begin                            
            // align byte endianess
            if (num_byte == 4) begin
                data[31:24] = bdata[7:0];
                data[23:16] = bdata[15:8];
                data[15:8]  = bdata[23:16];
                data[7:0]   = bdata[31:24];
            end
            else if (num_byte == 3) begin
                data[31:24] = 0;
                data[23:16] = bdata[15:8];
                data[15:8]  = bdata[23:16];
                data[7:0]   = bdata[31:24];
            end
            else if (num_byte == 2) begin
                data[31:24] = 0;
                data[23:16] = 0;
                data[15:8]  = bdata[23:16];
                data[7:0]   = bdata[31:24];
            end
            else if (num_byte == 1) begin
                data[31:24] = 0;
                data[23:16] = 0;
                data[15:8]  = 0;
                data[7:0]   = bdata[31:24];
            end
            wdata = data[31:0];

            adr = image_addr;
            // $display("backdoor adr=%h wdata=%h bdata=%h data=%h num_byte=%h", adr, wdata, bdata, data, num_byte);		  
            //backdoor_load_to_mem
            `ifdef ROM
                usoc_tb.usoc_u.rom_u.mem[adr[17:2]] = wdata;	  
            `endif
            `ifdef RAM
                usoc_tb.usoc_u.ram_u.mem[(adr[17:0] ^ `RAM_BASE_ADDR) >> 2] = wdata;	  
            `endif
            image_addr += 4;
            num_byte = $fread(bdata, image_fd);               //该函数地址自增
            end
            $fclose(image_fd);
            $display("Done Loading Image from backdoor. addr range: %h %h\n",
                    addr, image_addr);
        end
    end

    localparam SIMULATION_FREQ      = `SIM_FREQ * 32'd1000000;
    localparam CYCLE_1MS            = SIMULATION_FREQ / 32'd1000;
    localparam SIMULATION_CYCLE_END = `SIM_TIME * CYCLE_1MS;
    localparam CYCLE_100MS          = 32'd100 * CYCLE_1MS;
    localparam SIMULATION_TIME      = `SIM_TIME;

    reg [31:0] cycle_num;
    reg [31:0] cycle_100ms_count;
    reg [31:0] cycle_100ms_cmp;
    reg cycle_time_100ms_flag, cycle_time_out;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) 
        begin
            cycle_num             <= 32'h0;
            cycle_100ms_count     <= 32'h0;
            cycle_100ms_cmp       <= CYCLE_100MS;
            cycle_time_out        <= 1'b0;
            cycle_time_100ms_flag <= 1'b0;
        end 
        else if(cycle_num == cycle_100ms_cmp) 
        begin
            cycle_100ms_count     <= cycle_100ms_count + 1'b1;
            cycle_100ms_cmp       <= cycle_100ms_cmp + CYCLE_100MS;
            cycle_time_100ms_flag <= 1'b1;
        end 
        else if(cycle_num == SIMULATION_CYCLE_END)
        begin
            cycle_time_out    <= 1'b1;
        end 
        else 
        begin
            cycle_num <= cycle_num + 1'b1;
            cycle_time_100ms_flag <= 1'b0;
        end
    end

    wire print_valid;
    wire [31:0] print_data; 
    assign print_valid = usoc_tb.usoc_u.mem_noc_router_1to4_u.sn3_req_valid 
                      && usoc_tb.usoc_u.mem_noc_router_1to4_u.sn3_req_ready 
                      && (usoc_tb.usoc_u.mem_noc_router_1to4_u.sn3_req.req_addr == 32'h10010000);
    assign print_data  = print_valid ? usoc_tb.usoc_u.mem_noc_router_1to4_u.sn3_req.req_data : 32'h0;
    always @(posedge clk) begin
        if(print_valid) begin
                $write("%c",print_data);
                //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
                $fwrite(log,"%c", print_data); 
                // $display("0x%x",print_data);
        end
    end

    // wire print_valid;
    // wire [31:0] print_pc, print_inst;
    // wire [63:0] print_inst_ascii;
    // reg  [31:0] print_count; 
    // wire [31:0] print_alu_out;
    // assign print_valid = usoc_tb.usoc_u.urv32_u.iex_u.iex2lsu_valid;
    // assign print_pc    = usoc_tb.usoc_u.urv32_u.iex_u.iex2lsu_pc; 
    // assign print_inst  = usoc_tb.usoc_u.urv32_u.iex_u.iex2lsu_inst;
    // assign print_inst_ascii = usoc_tb.usoc_u.urv32_u.iex_u.iex2lsu_inst_ascii;
    // assign print_alu_out    = usoc_tb.usoc_u.urv32_u.iex_u.alu_u.out;
    // always @(posedge clk or negedge rstn) begin
    //     if(!rstn) begin
    //         print_count <= 64'h1;
    //     end else
    //     if(print_valid) begin
    //             $write("cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x\n",print_count, print_pc, print_inst, print_inst_ascii, print_alu_out);
    //             //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
    //             $fwrite(log,"cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x\n",print_count, print_pc, print_inst, print_inst_ascii, print_alu_out); 
    //             // $display("0x%x",print_data);
    //             print_count <= print_count + 1'b1;
    //     end
    // end

    always @(posedge clk) begin
        if(cycle_time_out) 
        begin
            $display("time over max limitation, auto finish at %dms", SIMULATION_TIME);
            $fclose(log); 
            $finish(1);
        end 
        else if(cycle_time_100ms_flag) 
        begin
            $display("time-consuming %dms", cycle_100ms_count * 32'd100);
        end
    end

    initial
    begin
        `ifdef FSDB_USOC  
        $fsdbDumpfile("usoc_tb.fsdb");  //生成fsdb文件，记录仿真信息
        $fsdbDumpvars(0, usoc_tb);  //指定层次数，记录信号 
        $fsdbDumpvars("+struct");
        $fsdbDumpvars("+mda");
        $fsdbDumpvars("+all");
        $fsdbDumpon;
        `endif   
        `ifdef FSDB_INST  
        $fsdbDumpfile("inst.fsdb");  //生成fsdb文件，记录仿真信息
        $fsdbDumpvars(0, usoc_tb);  //指定层次数，记录信号 
        $fsdbDumpvars("+struct");
        $fsdbDumpvars("+mda");
        $fsdbDumpvars("+all");
        $fsdbDumpon;
        `endif 
        // $dumpfile("usoc_tb.vcd");  //生成vcd文件，记录仿真信息
        // $dumpvars(0, usoc_tb);  //指定层次数，记录信号
    end


usoc usoc_u(
    .clk(clk),
    .rstn(rstn),
    // output
	.iex2lsu_pc(iex2lsu_pc),
    .iex2ac_jump_valid(iex2ac_jump_valid),
    .iex2ac_jump_pc(iex2ac_jump_pc)    
);

endmodule