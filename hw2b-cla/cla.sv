`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1 (
    input  wire a,
    b,
    output wire g,
    p
);
  assign g = a & b;
  assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4 (
    input wire [3:0] gin,
    pin,
    input wire cin,
    output wire gout,
    pout,
    output wire [2:0] cout
);

  assign cout[0] = gin[0] | (pin[0] & cin);
  assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
  assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);

  assign gout = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]);
  assign pout = &pin;  // Equivalent to pin[3] & pin[2] & pin[1] & pin[0]

endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8 (
    input wire [7:0] gin,
    pin,
    input wire cin,
    output wire gout,
    pout,
    output wire [6:0] cout
);

  wire g4_0, p4_0, g4_1, p4_1;  // These must be defined before use!


  wire [2:0] cout_low, cout_high;  // Use temporary carry wires

  gp4 gp_low (
      .gin (gin[3:0]),
      .pin (pin[3:0]),
      .cin (cin),
      .gout(g4_0),
      .pout(p4_0),
      .cout(cout_low)
  );

  gp4 gp_high (
      .gin (gin[7:4]),
      .pin (pin[7:4]),
      .cin (g4_0 | (p4_0 & cin)),
      .gout(g4_1),
      .pout(p4_1),
      .cout(cout_high)
  );

  assign gout = g4_1 | (p4_1 & g4_0);
  assign pout = p4_1 & p4_0;

  assign cout[2:0] = cout_low;
  assign cout[5:3] = cout_high;

endmodule

module cla (
    input  wire [31:0] a,
    b,
    input  wire        cin,
    output wire [31:0] sum
);

  wire [31:0] g, p;
  wire [30:0] c;
  wire g8_1, p8_1, g8_2, p8_2, g8_3, p8_3, g8_4, p8_4;

  wire [6:0] c_internal1, c_internal2, c_internal3;

  gp8 gp_block1 (
      .gin (g[7:0]),
      .pin (p[7:0]),
      .cin (cin),
      .gout(g8_1),
      .pout(p8_1),
      .cout(c_internal1)
  );

  gp8 gp_block2 (
      .gin (g[15:8]),
      .pin (p[15:8]),
      .cin (c_internal1[6]),
      .gout(g8_2),
      .pout(p8_2),
      .cout(c_internal2)
  );

  gp8 gp_block3 (
      .gin (g[23:16]),
      .pin (p[23:16]),
      .cin (c_internal2[6]),
      .gout(g8_3),
      .pout(p8_3),
      .cout(c_internal3)
  );

  gp8 gp_block4 (
      .gin (g[31:24]),
      .pin (p[31:24]),
      .cin (c_internal3[6]),
      .gout(g8_4),
      .pout(p8_4),
      .cout(c[29:23])
  );

  // Compute sum bits
  assign sum[0] = a[0] ^ b[0] ^ cin;

  genvar i;
  generate
    for (i = 1; i < 32; i = i + 1) begin : sum_generation
      assign sum[i] = a[i] ^ b[i] ^ c[i-1];
    end
  endgenerate


endmodule
