`timescale 1ns/1ps
`include "cla.sv"

module SystemDemo(input wire [6:0] btn,
                 output wire [7:0] led);
   wire [31:0] sum;
//    reg [6:0] btn_reg;

    integer i;
    initial begin
        btn = 6'b0000000;
        for (i = 0; i < 128; i = i + 1) begin
        #10000 btn = i[6:0];
        end
    end

   cla cla_inst(.a(32'd26), .b({26'b0, btn[1], btn[2], btn[3], btn[5], btn[4], btn[6]}), .cin(1'b0), .sum(sum));
   assign led = sum[7:0];

  

endmodule