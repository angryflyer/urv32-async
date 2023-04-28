
`include "inst_index.v"
`include "macro.v"

module mem_noc_router_1to4
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  logic            clk,
    input  logic            rstn,

    //noc master 0 
    input  logic            mn_req_valid,
    output logic            mn_req_ready,
    input  mem_req_t        mn_req,

    output logic            mn_resp_valid,
    input  logic            mn_resp_ready,
    output mem_resp_t       mn_resp,
    //noc slave 0 
    output logic            sn0_req_valid,
    input  logic            sn0_req_ready,
    output mem_req_t        sn0_req,

    input  logic            sn0_resp_valid,
    output logic            sn0_resp_ready,
    input  mem_resp_t       sn0_resp,
    // input  [MEM_ADDR_W-1:0] sn0_base_addr,
    //noc slave 1 
    output logic            sn1_req_valid,
    input  logic            sn1_req_ready,
    output mem_req_t        sn1_req,

    input  logic            sn1_resp_valid,
    output logic            sn1_resp_ready,
    input  mem_resp_t       sn1_resp,
    // input  [MEM_ADDR_W-1:0] sn1_base_addr,
    //noc slave 2 
    output logic            sn2_req_valid,
    input  logic            sn2_req_ready,
    output mem_req_t        sn2_req,

    input  logic            sn2_resp_valid,
    output logic            sn2_resp_ready,
    input  mem_resp_t       sn2_resp,
    // input  [MEM_ADDR_W-1:0] sn2_base_addr,
    //noc slave 3 
    output logic            sn3_req_valid,
    input  logic            sn3_req_ready,
    output mem_req_t        sn3_req,

    input  logic            sn3_resp_valid,
    output logic            sn3_resp_ready,
    input  mem_resp_t       sn3_resp
    // input  [MEM_ADDR_W-1:0] sn3_base_addr
);
    localparam MEM_REQ_T_W  = $bits(mem_req_t);
    localparam MEM_RESP_T_W = $bits(mem_resp_t);

    typedef enum logic [1:0] {
        SNOC0 = 2'b00,
        SNOC1 = 2'b01,
        SNOC2 = 2'b10,
        SNOC3 = 2'b11
    } noc_tid_t;

    typedef enum logic {
        NOC_REQ  = 1'b0,
        NOC_RESP = 1'b1
    } noc_state_t;

    logic [1:0] mn_tid;
    logic [1:0] ff_mn_tid;
    logic       curt_noc_state;
    logic       next_noc_state;
    
    // assign mn_tid = ((mn_req.req_addr ^ MEM_BASE_ADDR_CLINT) < MEM_REG_ADDR_W_IN_BYTE) ? SNOC0
    //               : ((mn_req.req_addr ^ MEM_BASE_ADDR_PLIC)  < MEM_REG_ADDR_W_IN_BYTE) ? SNOC1
    //               : ((mn_req.req_addr ^ MEM_BASE_ADDR_DM)    < MEM_REG_ADDR_W_IN_BYTE) ? SNOC2
    //               : SNOC3;
    assign mn_tid = (mn_req.req_addr[31:12] == MEM_BASE_ADDR_CLINT[31:12])  ? SNOC0
                    : (mn_req.req_addr[31:12] == MEM_BASE_ADDR_PLIC[31:12]) ? SNOC1
                    : (mn_req.req_addr[31:12] == MEM_BASE_ADDR_DM[31:12])   ? SNOC2
                    : SNOC3;

    logic mn_sn0_tid;
    logic mn_sn1_tid;
    logic mn_sn2_tid;
    logic mn_sn3_tid; 
    logic ff_mn_tid_en;
    logic mn_req_handshaked;
    logic mn_resp_handshaked;

    logic is_noc_req, is_noc_resp;

    logic mn_req_ready_en;

    assign mn_sn0_tid = mn_req_valid && (mn_tid == SNOC0);
    assign mn_sn1_tid = mn_req_valid && (mn_tid == SNOC1);
    assign mn_sn2_tid = mn_req_valid && (mn_tid == SNOC2);
    assign mn_sn3_tid = mn_req_valid && (mn_tid == SNOC3);

    assign mn_req_handshaked = mn_req_valid  && mn_req_ready;
    assign mn_resp_handshaked= mn_resp_valid && mn_resp_ready;

    assign ff_mn_tid_en   = (is_noc_req && mn_req_handshaked) | (is_noc_resp && mn_resp_handshaked);

    stdffrv #(1) ff_noc_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(NOC_REQ),
        .d(next_noc_state),
        .q(curt_noc_state) 
    );    

    stdffrve #(2) ff_mn_tid_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(SNOC0),
        .en(ff_mn_tid_en),
        .d(mn_tid),
        .q(ff_mn_tid) 
    );

    always_comb  begin
        next_noc_state = curt_noc_state;
        case(curt_noc_state)
            NOC_REQ  : begin
                next_noc_state = mn_req_handshaked  ? NOC_RESP : NOC_REQ;
            end
            NOC_RESP : begin
                next_noc_state = ~mn_resp_handshaked ? NOC_RESP 
                                : mn_req_handshaked   ? NOC_RESP
                                : NOC_REQ;            
            end
        endcase
    end

    assign is_noc_req = (curt_noc_state == NOC_REQ );
    assign is_noc_resp= (curt_noc_state == NOC_RESP);

    assign sn0_req_valid = is_noc_req  ? mn_sn0_tid 
                            : is_noc_resp ? mn_resp_handshaked && mn_sn0_tid
                            : 1'b0;
    assign sn0_req       = mn_req;

    assign sn1_req_valid = is_noc_req  ? mn_sn1_tid 
                            : is_noc_resp ? mn_resp_handshaked && mn_sn1_tid
                            : 1'b0;
    assign sn1_req       = mn_req;

    assign sn2_req_valid = is_noc_req  ? mn_sn2_tid 
                            : is_noc_resp ? mn_resp_handshaked && mn_sn2_tid
                            : 1'b0;
    assign sn2_req       = mn_req;

    assign sn3_req_valid = is_noc_req  ? mn_sn3_tid 
                            : is_noc_resp ? mn_resp_handshaked && mn_sn3_tid
                            : 1'b0;
    assign sn3_req       = mn_req;

    assign mn_req_ready_en  = is_noc_req | (is_noc_resp && mn_resp_handshaked);

    always_comb  begin
        case(mn_tid)
            SNOC0 : begin
                mn_req_ready = mn_req_ready_en && sn0_req_ready;
            end
            SNOC1 : begin
                mn_req_ready = mn_req_ready_en && sn1_req_ready;          
            end
            SNOC2 : begin
                mn_req_ready = mn_req_ready_en && sn2_req_ready;          
            end
            SNOC3 : begin
                mn_req_ready = mn_req_ready_en && sn3_req_ready;          
            end
            default : begin
                mn_req_ready = 1'b0;
            end
        endcase
    end

    assign sn0_resp_ready = (ff_mn_tid == SNOC0) ? mn_resp_ready : 1'b0;
    assign sn1_resp_ready = (ff_mn_tid == SNOC1) ? mn_resp_ready : 1'b0;
    assign sn2_resp_ready = (ff_mn_tid == SNOC2) ? mn_resp_ready : 1'b0;
    assign sn3_resp_ready = (ff_mn_tid == SNOC3) ? mn_resp_ready : 1'b0;

    always_comb  begin
        case(ff_mn_tid)
            SNOC0 : begin
                mn_resp_valid = sn0_resp_valid;
                mn_resp       = sn0_resp;
            end
            SNOC1 : begin
                mn_resp_valid = sn1_resp_valid; 
                mn_resp       = sn1_resp;         
            end
            SNOC2 : begin
                mn_resp_valid = sn2_resp_valid;    
                mn_resp       = sn2_resp;     
            end
            SNOC3 : begin
                mn_resp_valid = sn3_resp_valid; 
                mn_resp       = sn3_resp;         
            end
            default : begin
                mn_resp_valid = 1'b0;
                mn_resp       = {MEM_RESP_T_W{1'b0}};
            end
        endcase
    end

endmodule
