
module imem_noc_router_1to2
    import urv_cfg::*;
    import urv_typedef::*; 
#(
    parameter DEC_NUM       = 2,
    parameter DEC_TAG_H     = 31,
    parameter DEC_TAG_L     = 28,
    parameter DEC_TAG_VAL   = {(DEC_TAG_H-DEC_TAG_L+1){1'b0}}
) (
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
    output logic            sn0_tid,
    //noc slave 1 
    output logic            sn1_req_valid,
    input  logic            sn1_req_ready,
    output mem_req_t        sn1_req,

    input  logic            sn1_resp_valid,
    output logic            sn1_resp_ready,
    input  mem_resp_t       sn1_resp,
    output logic            sn1_tid
);
    localparam MEM_REQ_T_W  = $bits(mem_req_t);
    localparam MEM_RESP_T_W = $bits(mem_resp_t);
    localparam DEC_TAG_W    = DEC_TAG_H - DEC_TAG_L + 1;

    typedef enum logic {
        SNOC0 = 1'b0,
        SNOC1 = 1'b1
    } noc_tid_t;

    typedef enum logic {
        NOC_REQ  = 1'b0,
        NOC_RESP = 1'b1
    } noc_state_t;

    logic [0:0] mn_tid;
    logic [0:0] ff_mn_tid;
    logic       curt_noc_state;
    logic       next_noc_state;

    logic       [DEC_TAG_W-1:0]                 dec_tag;
    logic                                       is_tag_match;
    logic                                       is_idx_match;
    // obtain dec_tag and dec_idx
    assign dec_tag      = mn_req.req_addr[DEC_TAG_H:DEC_TAG_L];
    assign is_tag_match = (dec_tag == DEC_TAG_VAL);

    // assign mn_tid = ((mn_req.req_addr ^ MEM_BASE_ADDR_ROM) < ROM_WIDTH_IN_BYTE) ? SNOC0
    //               : SNOC1;    

    assign mn_tid = is_tag_match ? SNOC0
                  : SNOC1;

    logic mn_sn0_tid;
    logic mn_sn1_tid;
    logic ff_mn_tid_en;
    logic mn_req_handshaked;
    logic mn_resp_handshaked;

    logic is_noc_req, is_noc_resp;

    logic mn_req_ready_en;

    assign mn_sn0_tid = mn_req_valid && (mn_tid == SNOC0);
    assign mn_sn1_tid = mn_req_valid && (mn_tid == SNOC1);

    assign mn_req_handshaked = mn_req_valid  && mn_req_ready;
    assign mn_resp_handshaked= mn_resp_valid && mn_resp_ready && mn_resp.resp_last;

    assign ff_mn_tid_en   = (is_noc_req && mn_req_handshaked) | (is_noc_resp && mn_resp_handshaked);

    stdffrv #(1) ff_noc_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(NOC_REQ),
        .d(next_noc_state),
        .q(curt_noc_state) 
    );    

    stdffrve #(1) ff_mn_tid_u (
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
                                : mn_req_handshaked  ? NOC_RESP
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

    assign mn_req_ready_en  = is_noc_req | (is_noc_resp && mn_resp_handshaked);

    always_comb  begin
        case(mn_tid)
            SNOC0 : begin
                mn_req_ready = mn_req_ready_en && sn0_req_ready;
            end
            SNOC1 : begin
                mn_req_ready = mn_req_ready_en && sn1_req_ready;          
            end
            default : begin
                mn_req_ready = 1'b0;
            end
        endcase
    end

    // assign sn0_resp_ready = (ff_mn_tid == SNOC0) ? mn_resp_ready : 1'b0;
    // assign sn1_resp_ready = (ff_mn_tid == SNOC1) ? mn_resp_ready : 1'b0;

    assign sn0_resp_ready = mn_resp_ready;
    assign sn1_resp_ready = mn_resp_ready;

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
            default : begin
                mn_resp_valid = 1'b0;
                mn_resp       = {MEM_RESP_T_W{1'b0}};
            end
        endcase
    end

    assign sn0_tid = mn_tid;
    assign sn1_tid = mn_tid;

endmodule
