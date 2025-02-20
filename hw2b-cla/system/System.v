module gp4 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [3:0] gin;
	input wire [3:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [2:0] cout;
	assign cout[0] = gin[0] | (pin[0] & cin);
	assign cout[1] = (gin[1] | (pin[1] & gin[0])) | ((pin[1] & pin[0]) & cin);
	assign cout[2] = ((gin[2] | (pin[2] & gin[1])) | ((pin[2] & pin[1]) & gin[0])) | (((pin[2] & pin[1]) & pin[0]) & cin);
	assign gout = ((gin[3] | (pin[3] & gin[2])) | ((pin[3] & pin[2]) & gin[1])) | (((pin[3] & pin[2]) & pin[1]) & gin[0]);
	assign pout = &pin;
endmodule
module gp8 (
	gin,
	pin,
	cin,
	gout,
	pout,
	cout
);
	input wire [7:0] gin;
	input wire [7:0] pin;
	input wire cin;
	output wire gout;
	output wire pout;
	output wire [6:0] cout;
	wire gout_low;
	wire pout_low;
	wire gout_high;
	wire pout_high;
	wire c4;
	wire [2:0] cout_low;
	wire [2:0] cout_high;
	gp4 gp4_low(
		.gin(gin[3:0]),
		.pin(pin[3:0]),
		.cin(cin),
		.gout(gout_low),
		.pout(pout_low),
		.cout(cout_low)
	);
	assign c4 = gout_low | (pout_low & cin);
	gp4 gp4_high(
		.gin(gin[7:4]),
		.pin(pin[7:4]),
		.cin(c4),
		.gout(gout_high),
		.pout(pout_high),
		.cout(cout_high)
	);
	assign cout[2:0] = cout_low;
	assign cout[6:3] = {cout_high, c4};
	assign gout = gout_high | (pout_high & gout_low);
	assign pout = pout_low & pout_high;
endmodule
module cla (
	a,
	b,
	cin,
	sum
);
	input wire [31:0] a;
	input wire [31:0] b;
	input wire cin;
	output wire [31:0] sum;
	wire [31:0] g;
	wire [31:0] p;
	assign g = a & b;
	assign p = a | b;
	wire [6:0] carry_internal_0;
	wire [6:0] carry_internal_1;
	wire [6:0] carry_internal_2;
	wire [6:0] carry_internal_3;
	wire [31:0] carry;
	wire gout_0;
	wire pout_0;
	wire gout_1;
	wire pout_1;
	wire gout_2;
	wire pout_2;
	wire gout_3;
	wire pout_3;
	gp8 gp8_0(
		.gin(g[7:0]),
		.pin(p[7:0]),
		.cin(cin),
		.gout(gout_0),
		.pout(pout_0),
		.cout(carry_internal_0)
	);
	wire carry_8;
	wire carry_16;
	wire carry_24;
	wire carry_32;
	assign carry_8 = gout_0 | (pout_0 & cin);
	assign carry[8] = carry_8;
	gp8 gp8_1(
		.gin(g[15:8]),
		.pin(p[15:8]),
		.cin(carry[8]),
		.gout(gout_1),
		.pout(pout_1),
		.cout(carry_internal_1)
	);
	assign carry_16 = gout_1 | (pout_1 & carry_8);
	assign carry[16] = carry_16;
	gp8 gp8_2(
		.gin(g[23:16]),
		.pin(p[23:16]),
		.cin(carry[16]),
		.gout(gout_2),
		.pout(pout_2),
		.cout(carry_internal_2)
	);
	assign carry_24 = gout_2 | (pout_2 & carry_16);
	assign carry[24] = carry_24;
	gp8 gp8_3(
		.gin(g[31:24]),
		.pin(p[31:24]),
		.cin(carry[24]),
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
	assign sum = (a ^ b) ^ carry;
endmodule
module SystemDemo (
	btn,
	led
);
	input wire [6:0] btn;
	output wire [7:0] led;
	wire [31:0] sum;
	integer i;
	initial begin
		btn = 6'b000000;
		for (i = 0; i < 128; i = i + 1)
			#(10000) btn = i[6:0];
	end
	cla cla_inst(
		.a(32'd26),
		.b({26'b00000000000000000000000000, btn[1], btn[2], btn[3], btn[5], btn[4], btn[6]}),
		.cin(1'b0),
		.sum(sum)
	);
	assign led = sum[7:0];
endmodule