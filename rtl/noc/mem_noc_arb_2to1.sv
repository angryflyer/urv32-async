//存储器总线mem noc
//用于取指和数据存取，且可以访问同一个设备
`include "inst_index.v"
`include "macro.v"

module mem_noc_arb_2to1 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input             clk,
    input             rstn,

    // noc master 0
    input             mn0_req_valid,
    output            mn0_req_ready,
    input  mem_req_t  mn0_req, 

    output            mn0_resp_valid,
    input             mn0_resp_ready,
    output mem_resp_t mn0_resp,
    input             mn0_tid,   
    // noc master 1
    input             mn1_req_valid,
    output            mn1_req_ready,
    input  mem_req_t  mn1_req, 

    output            mn1_resp_valid, 
    input             mn1_resp_ready,
    output mem_resp_t mn1_resp, 
    input             mn1_tid, 

    //noc slave 
    output            sn_req_valid,
    input             sn_req_ready,
    output mem_req_t  sn_req, 

    input             sn_resp_valid,
    output            sn_resp_ready,
    input  mem_resp_t sn_resp,
    input             sn_tid
);

    localparam MEM_REQ_T_W = $bits(mem_req_t);
    localparam MEM_RESP_T_W = $bits(mem_resp_t);

    typedef enum logic {
        SNOC0 = 1'b0,
        SNOC1 = 1'b1
    } noc_tid_t;

    typedef enum logic {
        NOC_REQ  = 1'b0,
        NOC_RESP = 1'b1
    } noc_state_t;

    typedef enum logic {
        MNOC0  = 1'b0,
        MNOC1  = 1'b1
    } arb_state_t;

    logic last_arb_state;
    logic arb_state_en;
    logic curt_noc_state;
    logic next_noc_state;
	logic arb_state, next_arb_state;

    logic is_sn_req, is_sn_resp;

    logic is_noc_req, is_noc_resp;

    logic            mn0_sn_req_valid;
    logic            mn0_sn_req_ready;
    mem_req_t        mn0_sn_req;

    logic            mn0_sn_resp_valid;
    logic            mn0_sn_resp_ready;
    mem_resp_t       mn0_sn_resp;

    logic            mn1_sn_req_valid;
    logic            mn1_sn_req_ready;
    mem_req_t        mn1_sn_req; 

    logic            mn1_sn_resp_valid;
    logic            mn1_sn_resp_ready;
    mem_resp_t       mn1_sn_resp;

	logic is_mn0_eq_sn_addr;
	logic is_mn1_eq_sn_addr;

	logic is_mn0_ready;
	logic is_mn1_ready;
    logic mn0_ready;
    logic mn1_ready;

	logic is_mn0_sn, is_mn1_sn;

    logic [MEM_ADDR_W-1:0] mn0_addr, mn1_addr;
    logic mn0_valid, mn1_valid;

    // logic mn0_req_handshaked;
    // logic mn1_req_handshaked;
    logic mn0_resp_handshaked;
    logic mn1_resp_handshaked;
    logic sn_req_handshaked;
    logic sn_resp_handshaked;

    // assign mn0_req_handshaked = mn0_req_valid && mn0_req_ready;
    // assign mn1_req_handshaked = mn1_req_valid && mn1_req_ready;
    assign mn0_resp_handshaked = mn0_resp_valid && mn0_resp_ready && mn0_resp.resp_last;
    assign mn1_resp_handshaked = mn1_resp_valid && mn1_resp_ready && mn1_resp.resp_last;
    assign sn_req_handshaked  = sn_req_valid  && sn_req_ready;
    assign sn_resp_handshaked = sn_resp_valid && sn_resp_ready && sn_resp.resp_last;

    assign arb_state_en   = (is_noc_req && sn_req_handshaked) | (is_noc_resp && sn_resp_handshaked);

    stdffrv #(1) ff_noc_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(NOC_REQ),
        .d(next_noc_state),
        .q(curt_noc_state) 
    );    

    stdffrve #(1) last_arb_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(MNOC0),
        .en(arb_state_en),
        .d(arb_state),
        .q(last_arb_state) 
    );

    always_comb  begin
        next_noc_state = curt_noc_state;
        case(curt_noc_state)
            NOC_REQ  : begin
                next_noc_state = sn_req_handshaked  ? NOC_RESP : NOC_REQ;
            end
            NOC_RESP : begin
                next_noc_state = ~sn_resp_handshaked ? NOC_RESP 
                               : sn_req_handshaked   ? NOC_RESP
                               : NOC_REQ;            
            end
        endcase
    end

    assign is_noc_req = (curt_noc_state == NOC_REQ );
    assign is_noc_resp= (curt_noc_state == NOC_RESP);

    assign mn0_valid = mn0_req_valid;
    assign mn1_valid = mn1_req_valid;
    assign mn0_addr  = mn0_req.req_addr;
    assign mn1_addr  = mn1_req.req_addr;
    assign mn0_req_ready = mn0_ready;
    assign mn1_req_ready = mn1_ready;

	assign is_mn0_eq_sn_addr = mn0_valid && (mn0_tid == sn_tid);
	assign is_mn1_eq_sn_addr = mn1_valid && (mn1_tid == sn_tid);

	assign is_mn0_sn = is_mn0_eq_sn_addr && !mn0_ready;
	assign is_mn1_sn = is_mn1_eq_sn_addr && !mn1_ready;

    stdffrv #(1) arb_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(MNOC0),
        .d(next_arb_state),
        .q(arb_state) 
    ); 

	always_comb begin
		next_arb_state = arb_state;
		case(arb_state)
			MNOC0 : begin 
				next_arb_state = (!is_mn0_sn & (is_noc_req | (is_noc_resp & mn0_resp_handshaked))) ? (is_mn1_sn ? MNOC1 : MNOC0) : MNOC0;
			end
			MNOC1 : begin
				next_arb_state = (!is_mn1_sn & (is_noc_req | (is_noc_resp & mn1_resp_handshaked))) ? (is_mn0_sn ? MNOC0 : MNOC1) : MNOC1;
			end
		endcase
	end

    assign sn_req_valid   = arb_state ? mn1_req_valid : mn0_req_valid;
    assign sn_req         = arb_state ? mn1_req       : mn0_req;

    assign mn0_ready      = !(arb_state) ? sn_req_ready : 1'b0;
    assign mn1_ready      = (arb_state)  ? sn_req_ready : 1'b0;

    assign mn0_resp_valid = !(last_arb_state) ? sn_resp_valid: 1'b0;
    assign mn0_resp       = !(last_arb_state) ? sn_resp : {MEM_RESP_T_W{1'b0}};

    assign mn1_resp_valid = (last_arb_state) ? sn_resp_valid : 1'b0;
    assign mn1_resp       = (last_arb_state) ? sn_resp : {MEM_RESP_T_W{1'b0}};

    assign sn_resp_ready  = last_arb_state ? mn1_resp_ready : mn0_resp_ready;
endmodule
