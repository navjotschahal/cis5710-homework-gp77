`timescale 1ns / 1ns

module DividerUnsignedPipelined (
    input wire clk,
    input wire rst,
    input wire [31:0] i_dividend,
    input wire [31:0] i_divisor,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    // Declare registers for pipeline stages
    logic [31:0] stage1_dividend[4];
    logic [31:0] stage1_remainder[4];
    logic [31:0] stage1_quotient[4];

    logic [31:0] stage1_dividend_output[4];
    logic [31:0] stage1_remainder_output[4];
    logic [31:0] stage1_quotient_output[4];

    // Add initial values for stage one
    assign stage1_dividend[0] = i_dividend;
    assign stage1_remainder[0] = 32'b0;
    assign stage1_quotient[0] = 32'b0;

    logic [31:0] stage2_dividend[4];
    logic [31:0] stage2_remainder[4];
    logic [31:0] stage2_quotient[4];

    logic [31:0] stage2_dividend_output[4];
    logic [31:0] stage2_remainder_output[4];
    logic [31:0] stage2_quotient_output[4];

    logic [31:0] stage3_dividend[4];
    logic [31:0] stage3_remainder[4];
    logic [31:0] stage3_quotient[4];

    logic [31:0] stage3_dividend_output[4];
    logic [31:0] stage3_remainder_output[4];
    logic [31:0] stage3_quotient_output[4];

    logic [31:0] stage4_dividend[4];
    logic [31:0] stage4_remainder[4];
    logic [31:0] stage4_quotient[4];

    logic [31:0] stage4_dividend_output[4];
    logic [31:0] stage4_remainder_output[4];
    logic [31:0] stage4_quotient_output[4];

    logic [31:0] stage5_dividend[4];
    logic [31:0] stage5_remainder[4];
    logic [31:0] stage5_quotient[4];

    logic [31:0] stage5_dividend_output[4];
    logic [31:0] stage5_remainder_output[4];
    logic [31:0] stage5_quotient_output[4];

    logic [31:0] stage6_dividend[4];
    logic [31:0] stage6_remainder[4];
    logic [31:0] stage6_quotient[4];

    logic [31:0] stage6_dividend_output[4];
    logic [31:0] stage6_remainder_output[4];
    logic [31:0] stage6_quotient_output[4];

    logic [31:0] stage7_dividend[4];
    logic [31:0] stage7_remainder[4];
    logic [31:0] stage7_quotient[4];

    logic [31:0] stage7_dividend_output[4];
    logic [31:0] stage7_remainder_output[4];
    logic [31:0] stage7_quotient_output[4];

    logic [31:0] stage8_dividend[4];
    logic [31:0] stage8_remainder[4];
    logic [31:0] stage8_quotient[4];

    logic [31:0] stage8_dividend_output[4];
    logic [31:0] stage8_remainder_output[4];
    logic [31:0] stage8_quotient_output[4];



    logic [31:0] divisor_temp;

    // Pipeline stage 1
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : stage1
            divu_1iter div_stage1_instance(
                .i_dividend(stage1_dividend[i]),
                .i_divisor(i_divisor),
                .i_remainder(stage1_remainder[i]),
                .i_quotient(stage1_quotient[i]),
                .o_dividend(stage1_dividend_output[i]),
                .o_remainder(stage1_remainder_output[i]),
                .o_quotient(stage1_quotient_output[i])
            );
            assign stage1_dividend[i + 1] = stage1_dividend_output[i];
            assign stage1_remainder[i + 1] = stage1_remainder_output[i];
            assign stage1_quotient[i + 1] = stage1_quotient_output[i];
        end
    endgenerate

    // Pipeline stage 2
    genvar j;
    generate
        for (j = 0; j < 4; j = j + 1) begin : stage2
            divu_1iter div_stage2_instance(
                .i_dividend(stage2_dividend[j]),
                .i_divisor(divisor_temp),
                .i_remainder(stage2_remainder[j]),
                .i_quotient(stage2_quotient[j]),
                .o_dividend(stage2_dividend_output[j]),
                .o_remainder(stage2_remainder_output[j]),
                .o_quotient(stage2_quotient_output[j])
            );

            assign stage2_dividend[j + 1] = stage2_dividend_output[j];
            assign stage2_remainder[j + 1] = stage2_remainder_output[j];
            assign stage2_quotient[j + 1] = stage2_quotient_output[j];
        end
    endgenerate

    // Pipeline stage 3
    genvar k;
    generate
        for (k = 0; k < 4; k = k + 1) begin : stage3
            divu_1iter div_stage3_instance(
                .i_dividend(stage3_dividend[k]),
                .i_divisor(divisor_temp),
                .i_remainder(stage3_remainder[k]),
                .i_quotient(stage3_quotient[k]),
                .o_dividend(stage3_dividend_output[k]),
                .o_remainder(stage3_remainder_output[k]),
                .o_quotient(stage3_quotient_output[k])
            );

            assign stage3_dividend[k + 1] = stage3_dividend_output[k];
            assign stage3_remainder[k + 1] = stage3_remainder_output[k];
            assign stage3_quotient[k + 1] = stage3_quotient_output[k];
        end
    endgenerate

    // Pipeline stage 4
    genvar l;
    generate
        for (l = 0; l < 4; l = l + 1) begin : stage4
            divu_1iter div_stage4_instance(
                .i_dividend(stage4_dividend[l]),
                .i_divisor(divisor_temp),
                .i_remainder(stage4_remainder[l]),
                .i_quotient(stage4_quotient[l]),
                .o_dividend(stage4_dividend_output[l]),
                .o_remainder(stage4_remainder_output[l]),
                .o_quotient(stage4_quotient_output[l])
            );

            assign stage4_dividend[l + 1] = stage4_dividend_output[l];
            assign stage4_remainder[l + 1] = stage4_remainder_output[l];
            assign stage4_quotient[l + 1] = stage4_quotient_output[l];
        end
    endgenerate

    // Pipeline stage 5
    genvar m;
    generate
        for (m = 0; m < 4; m = m + 1) begin : stage5
            divu_1iter div_stage5_instance(
                .i_dividend(stage5_dividend[m]),
                .i_divisor(divisor_temp),
                .i_remainder(stage5_remainder[m]),
                .i_quotient(stage5_quotient[m]),
                .o_dividend(stage5_dividend_output[m]),
                .o_remainder(stage5_remainder_output[m]),
                .o_quotient(stage5_quotient_output[m])
            );

            assign stage5_dividend[m + 1] = stage5_dividend_output[m];
            assign stage5_remainder[m + 1] = stage5_remainder_output[m];
            assign stage5_quotient[m + 1] = stage5_quotient_output[m];
        end
    endgenerate

    // Pipeline stage 6
    genvar n;
    generate
        for (n = 0; n < 4; n = n + 1) begin : stage6
            divu_1iter div_stage6_instance(
                .i_dividend(stage6_dividend[n]),
                .i_divisor(divisor_temp),
                .i_remainder(stage6_remainder[n]),
                .i_quotient(stage6_quotient[n]),
                .o_dividend(stage6_dividend_output[n]),
                .o_remainder(stage6_remainder_output[n]),
                .o_quotient(stage6_quotient_output[n])
            );

            assign stage6_dividend[n + 1] = stage6_dividend_output[n];
            assign stage6_remainder[n + 1] = stage6_remainder_output[n];
            assign stage6_quotient[n + 1] = stage6_quotient_output[n];
        end
    endgenerate

    // Pipeline stage 7
    genvar o;
    generate
        for (o = 0; o < 4; o = o + 1) begin : stage7
            divu_1iter div_stage7_instance(
                .i_dividend(stage7_dividend[o]),
                .i_divisor(divisor_temp),
                .i_remainder(stage7_remainder[o]),
                .i_quotient(stage7_quotient[o]),
                .o_dividend(stage7_dividend_output[o]),
                .o_remainder(stage7_remainder_output[o]),
                .o_quotient(stage7_quotient_output[o])
            );

            assign stage7_dividend[o + 1] = stage7_dividend_output[o];
            assign stage7_remainder[o + 1] = stage7_remainder_output[o];
            assign stage7_quotient[o + 1] = stage7_quotient_output[o];
        end
    endgenerate

    // Pipeline stage 8
    genvar p;
    generate
        for (p = 0; p < 4; p = p + 1) begin : stage8
            divu_1iter div_stage8_instance(
                .i_dividend(stage8_dividend[p]),
                .i_divisor(divisor_temp),
                .i_remainder(stage8_remainder[p]),
                .i_quotient(stage8_quotient[p]),
                .o_dividend(stage8_dividend_output[p]),
                .o_remainder(stage8_remainder_output[p]),
                .o_quotient(stage8_quotient_output[p])
            );

            assign stage8_dividend[p + 1] = stage8_dividend_output[p];
            assign stage8_remainder[p + 1] = stage8_remainder_output[p];
            assign stage8_quotient[p + 1] = stage8_quotient_output[p];
        end
    endgenerate

    logic [31:0] stage2_dividend_intermediate;
    logic [31:0] stage2_remainder_intermediate;
    logic [31:0] stage2_quotient_intermediate;

    // Update registers at clock cycles
    always_ff @(posedge clk) begin
        if (rst) begin
            // Add reset logic
            /*for (k = 0; k <= 16; k = k + 1) begin
                stage1_dividend[k] <= 0;
                stage1_remainder[k] <= 0;
                stage1_quotient[k] <= 0;
            end */

            stage2_dividend_intermediate <= 0;
            stage2_remainder_intermediate <= 0;
            stage2_quotient_intermediate <= 0;
            divisor_temp <= 0;

        end else begin
            // First stage to second stage
            stage2_dividend_intermediate <= stage1_dividend_output[4];
            stage2_remainder_intermediate <= stage1_remainder_output[4];
            stage2_quotient_intermediate <= stage1_quotient_output[4];
            divisor_temp <= i_divisor;
        end
    end
endmodule

module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:32] o_quotient 
);
    wire [31:0] intermediate_remainder; 
    wire [31:0] new_quotient;
    wire [31:0] new_remainder;

    assign intermediate_remainder = (i_remainder << 1) | ((i_dividend >> 31) & 32'h00000001);

    assign new_quotient = intermediate_remainder < i_divisor ? i_quotient << 1 : (i_quotient << 1) | 32'h00000001;
    assign new_remainder = intermediate_remainder >= i_divisor ? intermediate_remainder - i_divisor : intermediate_remainder;

    assign o_dividend = i_dividend << 1;
    assign o_quotient = new_quotient;
    assign o_remainder = new_remainder;

endmodule
