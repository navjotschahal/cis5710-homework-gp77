/* INSERT NAME AND PENNKEY HERE */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module DividerUnsignedPipelined (
    input wire clk, rst, stall,
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

    // Pipeline registers
    logic [31:0] dividend_pipeline [0:8];
    logic [31:0] remainder_pipeline [0:8];
    logic [31:0] quotient_pipeline [0:8];

    // Initial values
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            dividend_pipeline[0] <= 32'b0;
            remainder_pipeline[0] <= 32'b0;
            quotient_pipeline[0] <= 32'b0;
        end else if (!stall) begin
            dividend_pipeline[0] <= i_dividend;
            remainder_pipeline[0] <= 32'b0;
            quotient_pipeline[0] <= 32'b0;
        end
    end

    // Pipeline stages
    genvar i;
    generate
        for (i = 0; i < 8; i++) begin : pipeline_stages
            always_ff @(posedge clk or posedge rst) begin
                if (rst) begin
                    dividend_pipeline[i+1] <= 32'b0;
                    remainder_pipeline[i+1] <= 32'b0;
                    quotient_pipeline[i+1] <= 32'b0;
                end else if (!stall) begin
                    dividend_pipeline[i+1] <= dividend_pipeline[i];
                    remainder_pipeline[i+1] <= remainder_pipeline[i];
                    quotient_pipeline[i+1] <= quotient_pipeline[i];
                end
            end

            divu_1iter iter (
                .i_dividend(dividend_pipeline[i]),
                .i_divisor(i_divisor),
                .i_remainder(remainder_pipeline[i]),
                .i_quotient(quotient_pipeline[i]),
                .o_dividend(dividend_pipeline[i+1]),
                .o_remainder(remainder_pipeline[i+1]),
                .o_quotient(quotient_pipeline[i+1])
            );
        end
    endgenerate

    // Output assignments
    assign o_quotient = quotient_pipeline[8];
    assign o_remainder = remainder_pipeline[8];

endmodule


module divu_1iter (
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    input  wire  [31:0] i_remainder,
    input  wire  [31:0] i_quotient,
    output logic [31:0] o_dividend,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

    // Reuse the code from HW2A
    logic [31:0] temp_remainder;
    logic [31:0] temp_quotient;
    logic [31:0] temp_dividend;

    always_comb begin
        temp_remainder = (i_remainder << 1) | (i_dividend >> 31);
        temp_dividend = i_dividend << 1;
        if (temp_remainder < i_divisor) begin
            temp_quotient = i_quotient << 1;
        end else begin
            temp_remainder = temp_remainder - i_divisor;
            temp_quotient = (i_quotient << 1) | 1;
        end
    end

    assign o_dividend = temp_dividend;
    assign o_remainder = temp_remainder;
    assign o_quotient = temp_quotient;

endmodule