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

    // TODO: your code here


// Pipeline registers
    logic [31:0] dividend_pipeline [0:7];
    logic [31:0] remainder_pipeline [0:7];
    logic [31:0] quotient_pipeline [0:7];
    logic [31:0] divisor_pipeline [0:7]; 
    
    // Intermediate wires for divu_1iter outputs
    wire [31:0] next_dividend [0:7];
    wire [31:0] next_remainder [0:7];
    wire [31:0] next_quotient [0:7];
    wire [31:0] next_divisor [0:7];


        divu_4iter divu_4iter_ins (
        .i_dividend(i_dividend),
        // .i_divisor(divisor_pipeline[i]),
        .i_divisor(i_divisor),
        .i_remainder(32'b0),
        .i_quotient(32'b0),
        .o_dividend(next_dividend[0]),
        .o_remainder(next_remainder[0]),
        .o_quotient(next_quotient[0]),
        .o_divisor(next_divisor[0])
    );


    

    // Pipeline stages
    genvar i;
    generate
        for (i = 1; i < 8; i++) begin : pipeline_stages

            divu_4iter divu_4iter_ins (
                .i_dividend(dividend_pipeline[i]),
                // .i_divisor(divisor_pipeline[i]),
                .i_divisor(divisor_pipeline[i]),
                .i_remainder(remainder_pipeline[i]),
                .i_quotient(quotient_pipeline[i]),
                .o_dividend(next_dividend[i]),
                .o_remainder(next_remainder[i]),
                .o_quotient(next_quotient[i]),
                .o_divisor(next_divisor[i])
            );
        end

    endgenerate

    always_ff @(posedge clk) begin
        integer j ;
        for (j = 0; j < 7; j++) begin
            if (rst) begin
                dividend_pipeline[j+1] <= 32'b0;
                remainder_pipeline[j+1] <= 32'b0;
                quotient_pipeline[j+1] <= 32'b0;
            end else if (!stall) begin
                dividend_pipeline[j+1] <= next_dividend[j];
                remainder_pipeline[j+1] <= next_remainder[j];
                quotient_pipeline[j+1] <= next_quotient[j];
                divisor_pipeline[j+1] <= next_divisor[j];
            end
        end
    end
    // Output assignments
    assign o_quotient = next_quotient[7];
    assign o_remainder = next_remainder[7];
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
    output wire [31:0] o_quotient,
    output wire [31:0] o_divisor
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
    assign o_divisor = i_divisor;

endmodule
