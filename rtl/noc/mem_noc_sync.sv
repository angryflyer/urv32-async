//===============================================
//Name          : mem_noc_sync
//Author        : angrybird
//Email         : 
//Date          : 2023-03-05
//Description   : mem_noc_sync/delay one cycle/registered
//              : MOD = 0 -> direct
//              : MOD = 1 -> reg output
//===============================================

module mem_noc_sync #(
    parameter MOD = 1,
    parameter W   = 1
)
(
    input  logic            clk,
    input  logic            rstn,

    // src
    input  logic            src_valid,
    output logic            src_ready,
    input  logic [W-1:0]    src, 

    output logic            dst_valid, 
    input  logic            dst_ready,
    output logic [W-1:0]    dst
);

if(MOD == 0) begin : DIRECT

    assign dst_valid = src_valid;
    assign dst       = src;
    assign src_ready = dst_ready;

end else if(MOD == 1) begin : REG_OUT

    logic src_handshaked;

    assign src_handshaked = src_valid  & src_ready;
    assign src_ready      = ~dst_valid | dst_ready;

    always_ff @(posedge clk or negedge rstn) begin
        if(~rstn) begin
            dst_valid <= 1'b0;
        end else if(src_ready) begin
            dst_valid <= src_valid;
        end
    end

    always_ff @(posedge clk or negedge rstn) begin
        if(~rstn) begin
            dst       <= {W{1'b0}};
        end else if(src_handshaked) begin
            dst       <= src;
        end
    end
end

endmodule
