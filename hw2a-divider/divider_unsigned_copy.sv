/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    // TODO: your code here
    logic [31:0] temp_dividend [0:32];
    logic [31:0] temp_remainder [0:32];
    logic [31:0] temp_quotient [0:32];

        assign temp_dividend[0] = i_dividend;
    assign temp_remainder[0] = 32'b0;
    assign temp_quotient[0] = 32'b0;

        genvar i;
        generate
        for ( i = 0; i < 32; i++) begin : divu_iter_block
            divu_1iter iter (
                .i_dividend(temp_dividend[i]),
                .i_divisor(i_divisor),
                .i_remainder(temp_remainder[i]),
                .i_quotient(temp_quotient[i]),
                .o_dividend(temp_dividend[i+1]),
                .o_remainder(temp_remainder[i+1]),
                .o_quotient(temp_quotient[i+1])
            );
        end
    endgenerate

        assign
         o_quotient = temp_quotient[32];
        assign o_remainder = temp_remainder[32];

endmodule


module divu_1iter (
    input  logic [31:0] i_dividend,
    input  logic [31:0] i_divisor,
    input  logic [31:0] i_remainder,
    input  logic [31:0] i_quotient,
    output logic [31:0] o_dividend,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);
  /*
    for (int i = 0; i < 32; i++) {
        remainder = (remainder << 1) | ((dividend >> 31) & 0x1);
        if (remainder < divisor) {
            quotient = (quotient << 1);
        } else {
            quotient = (quotient << 1) | 0x1;
            remainder = remainder - divisor;
        }
        dividend = dividend << 1;
    }
    */

    // TODO: your code here
    logic [31:0] temp_remainder;
    logic [31:0] temp_quotient;
    logic [31:0] temp_dividend;

    always_comb begin
        temp_remainder = (i_remainder << 1) | (i_dividend[31] ? 32'b1 : 32'b0);
        temp_dividend = i_dividend << 1;
        if (temp_remainder >= i_divisor) begin
            temp_remainder = temp_remainder - i_divisor;
            temp_quotient = (i_quotient << 1) | 1;
        end else begin
            temp_quotient = i_quotient << 1;
        end

        o_dividend = temp_dividend;
        o_remainder = temp_remainder;
        o_quotient = temp_quotient;
    end

endmodule
