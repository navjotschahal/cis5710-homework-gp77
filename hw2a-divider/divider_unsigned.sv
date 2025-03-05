/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] remainder_stage [0: 8];
    wire [31:0] quotient_stage[0:8];
    wire [31:0] dividend_stage[0:8];

    assign remainder_stage[0] = 32'b0;
    assign quotient_stage[0] = 32'b0;
    assign dividend_stage[0] = i_dividend;

    genvar i;
    generate
        for (i = 0; i < 8; i++) begin : iter
            divu_4iter iter (
                .i_dividend(dividend_stage[i]),
                .i_divisor(i_divisor),
                .i_remainder(remainder_stage[i]),
                .i_quotient(quotient_stage[i]),
                .o_dividend(dividend_stage[i+1]),
                .o_remainder(remainder_stage[i+1]),
                .o_quotient(quotient_stage[i+1])
            );
        end
    endgenerate

    assign o_quotient = quotient_stage[8];
    assign o_remainder = remainder_stage[8];



endmodule


module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] new_remainder;
    wire comparison;

    assign new_remainder = (i_remainder << 1) | (i_dividend >> 31);
    assign o_dividend = i_dividend << 1;
    assign comparison = new_remainder < i_divisor;

    assign o_quotient = comparison ? (i_quotient << 1) : ((i_quotient << 1) | 1);
    assign o_remainder = comparison ? new_remainder : (new_remainder - i_divisor);

endmodule


module divu_4iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    wire [31:0] dividend_stage [0:4];
    wire [31:0] remainder_stage [0:4];
    wire [31:0] quotient_stage [0:4];

    assign dividend_stage[0] = i_dividend;
    assign remainder_stage[0] = i_remainder;
    assign quotient_stage[0] = i_quotient;

    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : iter
            divu_1iter iter (
                .i_dividend(dividend_stage[i]),
                .i_divisor(i_divisor),
                .i_remainder(remainder_stage[i]),
                .i_quotient(quotient_stage[i]),
                .o_dividend(dividend_stage[i+1]),
                .o_remainder(remainder_stage[i+1]),
                .o_quotient(quotient_stage[i+1])
            );
        end
    endgenerate

    assign o_dividend = dividend_stage[4];
    assign o_remainder = remainder_stage[4];
    assign o_quotient = quotient_stage[4];

endmodule

