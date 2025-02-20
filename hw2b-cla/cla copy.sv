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
  assign pout = &pin;

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

  // Intermediate signals for 4-bit groups
  wire gout_low, pout_low, gout_high, pout_high;
  wire c4;
  wire [2:0] cout_low, cout_high;

  // Lower 4-bit gp4 block (bits 0-3)
  gp4 gp4_low (
      .gin (gin[3:0]),
      .pin (pin[3:0]),
      .cin (cin),
      .gout(gout_low),
      .pout(pout_low),
      .cout(cout_low)
  );

  // Compute carry into upper block
  assign c4 = gout_low | (pout_low & cin);

  gp4 gp4_high (
      .gin (gin[7:4]),
      .pin (pin[7:4]),
      .cin (c4),
      .gout(gout_high),
      .pout(pout_high),
      .cout(cout_high)
  );

  assign cout[2:0] = cout_low;
  assign cout[6:3] = {cout_high, c4};

  // Compute final generate and propagate
  assign gout = gout_high | (pout_high & gout_low);
  assign pout = pout_low & pout_high;

endmodule


module cla (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire        cin,
    output wire [31:0] sum
);

  wire [31:0] g, p;
  assign g = a & b;
  assign p = a | b;

  wire [6:0] carry_internal_0, carry_internal_1, carry_internal_2, carry_internal_3;
  wire [31:0] carry;
  wire gout_0, pout_0, gout_1, pout_1, gout_2, pout_2, gout_3, pout_3;

  gp8 gp8_0 (
      .gin (g[7:0]),
      .pin (p[7:0]),
      .cin (cin),
      .gout(gout_0),
      .pout(pout_0),
      .cout(carry_internal_0)
  );

  wire carry_8, carry_16, carry_24, carry_32;

  assign carry_8  = gout_0 | (pout_0 & cin);
  assign carry[8] = carry_8;

  gp8 gp8_1 (
      .gin (g[15:8]),
      .pin (p[15:8]),
      .cin (carry[8]),
      .gout(gout_1),
      .pout(pout_1),
      .cout(carry_internal_1)
  );

  assign carry_16  = gout_1 | (pout_1 & carry_8);
  assign carry[16] = carry_16;

  gp8 gp8_2 (
      .gin (g[23:16]),
      .pin (p[23:16]),
      .cin (carry[16]),
      .gout(gout_2),
      .pout(pout_2),
      .cout(carry_internal_2)
  );

  assign carry_24  = gout_2 | (pout_2 & carry_16);
  assign carry[24] = carry_24;

  gp8 gp8_3 (
      .gin (g[31:24]),
      .pin (p[31:24]),
      .cin (carry[24]),
      .gout(gout_3),
      .pout(pout_3),
      .cout(carry_internal_3)
  );

  assign carry[0] = cin;
  assign carry[7:1] = carry_internal_0[6:0];
  assign carry[15:9] = carry_internal_1[6:0];
  assign carry[23:17] = carry_internal_2[6:0];
  assign carry_32 = gout_3 | (pout_3 & carry_24);
  assign carry[31:25] = carry_internal_3[6:0];

  assign sum = a ^ b ^ carry;

endmodule
