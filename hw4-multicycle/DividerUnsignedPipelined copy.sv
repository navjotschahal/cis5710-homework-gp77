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

    logic [31:0] divisor_reg;

// Pipeline registers
    logic [31:0] dividend_pipeline [0:8];
    logic [31:0] remainder_pipeline [0:8];
    logic [31:0] quotient_pipeline [0:8];
    // logic [31:0] divisor_pipeline [0:8]; 
    
    // Intermediate wires for divu_1iter outputs
    logic [31:0] next_dividend [0:7];
    logic [31:0] next_remainder [0:7];
    logic [31:0] next_quotient [0:7];

    // Initial values
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            dividend_pipeline[0] <= 32'b0;
            remainder_pipeline[0] <= 32'b0;
            quotient_pipeline[0] <= 32'b0;
            // divisor_pipeline[0] <= 32'b0; 
            divisor_reg <= 32'b0;
        end else if (!stall) begin
            dividend_pipeline[0] <= i_dividend;
            remainder_pipeline[0] <= 32'b0;
            quotient_pipeline[0] <= 32'b0;
            // divisor_pipeline[0] <= i_divisor; 
            divisor_reg <= i_divisor;
        end
    end

    // Pipeline stages
    genvar i;
generate
    for (i = 0; i < 8; i++) begin : pipeline_stages
    
        divu_1iter divu_1iter_ins (
            .i_dividend(dividend_pipeline[i]),
            // .i_divisor(divisor_pipeline[i]),
            .i_divisor(divisor_reg),
            .i_remainder(remainder_pipeline[i]),
            .i_quotient(quotient_pipeline[i]),
            .o_dividend(next_dividend[i]),
            .o_remainder(next_remainder[i]),
            .o_quotient(next_quotient[i])
        );

        // Update pipeline registers (sequential)
        always_ff @(posedge clk or posedge rst) begin
            if (rst) begin
                dividend_pipeline[i+1] <= 32'b0;
                remainder_pipeline[i+1] <= 32'b0;
                quotient_pipeline[i+1] <= 32'b0;
                // divisor_pipeline[i+1] <= 32'b0; // Reset divisor pipeline
            end else if (!stall) begin
                dividend_pipeline[i+1] <= next_dividend[i];
                remainder_pipeline[i+1] <= next_remainder[i];
                quotient_pipeline[i+1] <= next_quotient[i];
                // divisor_pipeline[i+1] <= divisor_pipeline[i]; // Pass divisor to next stage
            end
        end

        // $display("dividend_pipeline[%0d]: %d", i, dividend_pipeline[i]);
        // $display("remainder_pipeline[%0d]: %d", i, remainder_pipeline[i]);
        // $display("quotient_pipeline[%0d]: %d", i, quotient_pipeline[i]);
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

  // TODO: copy your code from HW2A here

logic [31:0] temp_remainder;
    logic [31:0] temp_quotient;
    logic [31:0] temp_dividend;

    always_comb begin
        temp_remainder = (i_remainder << 1) | (i_dividend >> 31);
        temp_dividend = i_dividend << 1;
        // log in console to check the values
        // $display("temp_remainder: %d", temp_remainder);
        // $display("i_divisor: %d", i_divisor);

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
