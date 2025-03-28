module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	clk_mem,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire clk_mem;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "4.16667" *) (* FREQUENCY_PIN_CLKOS = "4.01003" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
		.PLLRST_ENA("DISABLED"),
		.INTFB_WAKE("DISABLED"),
		.STDBY_ENABLE("DISABLED"),
		.DPHASE_SOURCE("DISABLED"),
		.OUTDIVIDER_MUXA("DIVA"),
		.OUTDIVIDER_MUXB("DIVB"),
		.OUTDIVIDER_MUXC("DIVC"),
		.OUTDIVIDER_MUXD("DIVD"),
		.CLKI_DIV(6),
		.CLKOP_ENABLE("ENABLED"),
		.CLKOP_DIV(128),
		.CLKOP_CPHASE(64),
		.CLKOP_FPHASE(0),
		.CLKOS_ENABLE("ENABLED"),
		.CLKOS_DIV(133),
		.CLKOS_CPHASE(97),
		.CLKOS_FPHASE(2),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(1)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
		.CLKOS(clk_mem),
		.CLKFB(clkfb),
		.CLKINTFB(clkfb),
		.PHASESEL0(1'b0),
		.PHASESEL1(1'b0),
		.PHASEDIR(1'b1),
		.PHASESTEP(1'b1),
		.PHASELOADREG(1'b1),
		.PLLWAKESYNC(1'b0),
		.ENCLKOP(1'b0),
		.LOCK(locked)
	);
endmodule
module divider_unsigned (
	i_dividend,
	i_divisor,
	o_remainder,
	o_quotient
);
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	wire [31:0] remainder_stage [0:8];
	wire [31:0] quotient_stage [0:8];
	wire [31:0] dividend_stage [0:8];
	assign remainder_stage[0] = 32'b00000000000000000000000000000000;
	assign quotient_stage[0] = 32'b00000000000000000000000000000000;
	assign dividend_stage[0] = i_dividend;
	genvar _gv_i_1;
	generate
		for (_gv_i_1 = 0; _gv_i_1 < 8; _gv_i_1 = _gv_i_1 + 1) begin : iter
			localparam i = _gv_i_1;
			divu_4iter iter(
				.i_dividend(dividend_stage[i]),
				.i_divisor(i_divisor),
				.i_remainder(remainder_stage[i]),
				.i_quotient(quotient_stage[i]),
				.o_dividend(dividend_stage[i + 1]),
				.o_remainder(remainder_stage[i + 1]),
				.o_quotient(quotient_stage[i + 1])
			);
		end
	endgenerate
	assign o_quotient = quotient_stage[8];
	assign o_remainder = remainder_stage[8];
endmodule
module divu_1iter (
	i_dividend,
	i_divisor,
	i_remainder,
	i_quotient,
	o_dividend,
	o_remainder,
	o_quotient
);
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	input wire [31:0] i_remainder;
	input wire [31:0] i_quotient;
	output wire [31:0] o_dividend;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	wire [31:0] new_remainder;
	wire comparison;
	assign new_remainder = (i_remainder << 1) | (i_dividend >> 31);
	assign o_dividend = i_dividend << 1;
	assign comparison = new_remainder < i_divisor;
	assign o_quotient = (comparison ? i_quotient << 1 : (i_quotient << 1) | 1);
	assign o_remainder = (comparison ? new_remainder : new_remainder - i_divisor);
endmodule
module divu_4iter (
	i_dividend,
	i_divisor,
	i_remainder,
	i_quotient,
	o_dividend,
	o_remainder,
	o_quotient
);
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	input wire [31:0] i_remainder;
	input wire [31:0] i_quotient;
	output wire [31:0] o_dividend;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
	wire [31:0] dividend_stage [0:4];
	wire [31:0] remainder_stage [0:4];
	wire [31:0] quotient_stage [0:4];
	assign dividend_stage[0] = i_dividend;
	assign remainder_stage[0] = i_remainder;
	assign quotient_stage[0] = i_quotient;
	genvar _gv_i_2;
	generate
		for (_gv_i_2 = 0; _gv_i_2 < 4; _gv_i_2 = _gv_i_2 + 1) begin : iter
			localparam i = _gv_i_2;
			divu_1iter iter(
				.i_dividend(dividend_stage[i]),
				.i_divisor(i_divisor),
				.i_remainder(remainder_stage[i]),
				.i_quotient(quotient_stage[i]),
				.o_dividend(dividend_stage[i + 1]),
				.o_remainder(remainder_stage[i + 1]),
				.o_quotient(quotient_stage[i + 1])
			);
		end
	endgenerate
	assign o_dividend = dividend_stage[4];
	assign o_remainder = remainder_stage[4];
	assign o_quotient = quotient_stage[4];
endmodule
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
module RegFile (
	rd,
	rd_data,
	rs1,
	rs1_data,
	rs2,
	rs2_data,
	clk,
	we,
	rst
);
	reg _sv2v_0;
	input wire [4:0] rd;
	input wire [31:0] rd_data;
	input wire [4:0] rs1;
	output reg [31:0] rs1_data;
	input wire [4:0] rs2;
	output reg [31:0] rs2_data;
	input wire clk;
	input wire we;
	input wire rst;
	localparam signed [31:0] NumRegs = 32;
	reg [31:0] regs [0:31];
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < NumRegs; i = i + 1)
				regs[i] <= 32'd0;
		end
		else if (we && (rd != 5'd0))
			regs[rd] <= rd_data;
	always @(*) begin
		if (_sv2v_0)
			;
		rs1_data = (rs1 != 5'd0 ? regs[rs1] : 32'd0);
		rs2_data = (rs2 != 5'd0 ? regs[rs2] : 32'd0);
	end
	initial _sv2v_0 = 0;
endmodule
module DatapathSingleCycle (
	clk,
	rst,
	halt,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output reg halt;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output reg [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output reg [31:0] store_data_to_dmem;
	output reg [3:0] store_we_to_dmem;
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;
	wire [11:0] imm_i;
	assign imm_i = insn_from_imem[31:20];
	wire [4:0] imm_shamt = insn_from_imem[24:20];
	wire [11:0] imm_s;
	assign imm_s[11:5] = insn_funct7;
	assign imm_s[4:0] = insn_rd;
	wire [12:0] imm_b;
	assign {imm_b[12], imm_b[10:5]} = insn_funct7;
	assign {imm_b[4:1], imm_b[11]} = insn_rd;
	assign imm_b[0] = 1'b0;
	wire [20:0] imm_j;
	assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};
	wire [31:0] imm_i_sext = {{20 {imm_i[11]}}, imm_i[11:0]};
	wire [31:0] imm_s_sext = {{20 {imm_s[11]}}, imm_s[11:0]};
	wire [31:0] imm_b_sext = {{19 {imm_b[12]}}, imm_b[12:0]};
	wire [31:0] imm_j_sext = {{11 {imm_j[20]}}, imm_j[20:0]};
	localparam [6:0] OpLoad = 7'b0000011;
	localparam [6:0] OpStore = 7'b0100011;
	localparam [6:0] OpBranch = 7'b1100011;
	localparam [6:0] OpJalr = 7'b1100111;
	localparam [6:0] OpMiscMem = 7'b0001111;
	localparam [6:0] OpJal = 7'b1101111;
	localparam [6:0] OpRegImm = 7'b0010011;
	localparam [6:0] OpRegReg = 7'b0110011;
	localparam [6:0] OpEnviron = 7'b1110011;
	localparam [6:0] OpAuipc = 7'b0010111;
	localparam [6:0] OpLui = 7'b0110111;
	wire insn_lui = insn_opcode == OpLui;
	wire insn_auipc = insn_opcode == OpAuipc;
	wire insn_jal = insn_opcode == OpJal;
	wire insn_jalr = insn_opcode == OpJalr;
	wire insn_beq = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b000);
	wire insn_bne = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b001);
	wire insn_blt = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b100);
	wire insn_bge = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b101);
	wire insn_bltu = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b110);
	wire insn_bgeu = (insn_opcode == OpBranch) && (insn_from_imem[14:12] == 3'b111);
	wire insn_lb = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b000);
	wire insn_lh = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b001);
	wire insn_lw = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b010);
	wire insn_lbu = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b100);
	wire insn_lhu = (insn_opcode == OpLoad) && (insn_from_imem[14:12] == 3'b101);
	wire insn_sb = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b000);
	wire insn_sh = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b001);
	wire insn_sw = (insn_opcode == OpStore) && (insn_from_imem[14:12] == 3'b010);
	wire insn_addi = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b000);
	wire insn_slti = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b010);
	wire insn_sltiu = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b011);
	wire insn_xori = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b100);
	wire insn_ori = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b110);
	wire insn_andi = (insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b111);
	wire insn_slli = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b001)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srli = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srai = ((insn_opcode == OpRegImm) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_add = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b000)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sub = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b000)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_sll = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b001)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_slt = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b010)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sltu = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b011)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_xor = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b100)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_srl = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_sra = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b101)) && (insn_from_imem[31:25] == 7'b0100000);
	wire insn_or = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b110)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_and = ((insn_opcode == OpRegReg) && (insn_from_imem[14:12] == 3'b111)) && (insn_from_imem[31:25] == 7'd0);
	wire insn_mul = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b000);
	wire insn_mulh = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b001);
	wire insn_mulhsu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b010);
	wire insn_mulhu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b011);
	wire insn_div = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b100);
	wire insn_divu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b101);
	wire insn_rem = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b110);
	wire insn_remu = ((insn_opcode == OpRegReg) && (insn_from_imem[31:25] == 7'd1)) && (insn_from_imem[14:12] == 3'b111);
	wire insn_ecall = (insn_opcode == OpEnviron) && (insn_from_imem[31:7] == 25'd0);
	wire insn_fence = insn_opcode == OpMiscMem;
	reg [31:0] pcNext;
	reg [31:0] pcCurrent;
	always @(posedge clk)
		if (rst)
			pcCurrent <= 32'd0;
		else
			pcCurrent <= pcNext;
	assign pc_to_imem = pcCurrent;
	reg [31:0] cycles_current;
	reg [31:0] num_insns_current;
	always @(posedge clk)
		if (rst) begin
			cycles_current <= 0;
			num_insns_current <= 0;
		end
		else begin
			cycles_current <= cycles_current + 1;
			if (!rst)
				num_insns_current <= num_insns_current + 1;
		end
	wire [31:0] rs1_data;
	wire [31:0] rs2_data;
	reg rf_we;
	reg [4:0] rf_rd;
	reg [31:0] rf_rd_data;
	RegFile rf(
		.clk(clk),
		.rst(rst),
		.we(rf_we),
		.rd(rf_rd),
		.rd_data(rf_rd_data),
		.rs1(insn_rs1),
		.rs2(insn_rs2),
		.rs1_data(rs1_data),
		.rs2_data(rs2_data)
	);
	wire [31:0] cla_sum;
	reg [31:0] cla_b_input;
	always @(*) begin
		if (_sv2v_0)
			;
		case (1'b1)
			insn_addi: cla_b_input = imm_i_sext;
			insn_add: cla_b_input = rs2_data;
			insn_sub: cla_b_input = ~rs2_data + 1;
			default: cla_b_input = 32'd0;
		endcase
	end
	cla cla_adder(
		.a(rs1_data),
		.b(cla_b_input),
		.cin(1'b0),
		.sum(cla_sum)
	);
	wire [31:0] divu_quotient;
	wire [31:0] divu_remainder;
	divider_unsigned u_divider(
		.i_dividend(rs1_data),
		.i_divisor(rs2_data),
		.o_quotient(divu_quotient),
		.o_remainder(divu_remainder)
	);
	wire [31:0] signed_dividend = (rs1_data[31] ? ~rs1_data + 32'd1 : rs1_data);
	wire [31:0] signed_divisor = (rs2_data[31] ? ~rs2_data + 32'd1 : rs2_data);
	wire [31:0] s_div_quotient;
	wire [31:0] s_div_remainder;
	divider_unsigned u_signed_divider(
		.i_dividend(signed_dividend),
		.i_divisor(signed_divisor),
		.o_quotient(s_div_quotient),
		.o_remainder(s_div_remainder)
	);
	wire [31:0] calc_addr = rs1_data + imm_i_sext;
	wire [31:0] calc_store_addr = rs1_data + imm_s_sext;
	wire signed [63:0] mul_signed_prod = $signed({{32 {rs1_data[31]}}, rs1_data}) * $signed({{32 {rs2_data[31]}}, rs2_data});
	wire signed [63:0] mulhsu_prod = $signed({{32 {rs1_data[31]}}, rs1_data}) * $unsigned({32'b00000000000000000000000000000000, rs2_data});
	wire [63:0] mulhu_prod = {32'b00000000000000000000000000000000, rs1_data} * {32'b00000000000000000000000000000000, rs2_data};
	wire signed [63:0] prod;
	reg illegal_insn;
	always @(*) begin
		if (_sv2v_0)
			;
		illegal_insn = 1'b0;
		rf_we = 1'b0;
		rf_rd = 0;
		rf_rd_data = 0;
		halt = 1'b0;
		pcNext = pcCurrent + 4;
		addr_to_dmem = 32'd0;
		store_we_to_dmem = 4'b0000;
		store_data_to_dmem = 32'd0;
		case (insn_opcode)
			OpLui: begin
				rf_we = 1'b1;
				rf_rd = insn_rd;
				rf_rd_data = {insn_from_imem[31:12], 12'b000000000000};
			end
			OpAuipc: begin
				rf_we = 1'b1;
				rf_rd = insn_rd;
				rf_rd_data = pcCurrent + {insn_from_imem[31:12], 12'b000000000000};
			end
			OpRegImm:
				case (insn_funct3)
					3'b000: begin
						rf_we = 1'b1;
						rf_rd = insn_rd;
						rf_rd_data = cla_sum;
					end
					3'b010: begin
						rf_we = 1'b1;
						rf_rd = insn_rd;
						rf_rd_data = ($signed(rs1_data) < $signed(imm_i_sext) ? 32'd1 : 32'd0);
					end
					3'b011: begin
						rf_we = 1'b1;
						rf_rd = insn_rd;
						rf_rd_data = (rs1_data < imm_i_sext ? 32'd1 : 32'd0);
					end
					3'b100: begin
						rf_we = 1'b1;
						rf_rd = insn_rd;
						rf_rd_data = rs1_data ^ imm_i_sext;
					end
					3'b110: begin
						rf_we = 1'b1;
						rf_rd = insn_rd;
						rf_rd_data = rs1_data | imm_i_sext;
					end
					3'b111: begin
						rf_we = 1'b1;
						rf_rd = insn_rd;
						rf_rd_data = rs1_data & imm_i_sext;
					end
					3'b001:
						if (insn_from_imem[31:25] == 7'd0) begin
							rf_we = 1'b1;
							rf_rd = insn_rd;
							rf_rd_data = rs1_data << imm_shamt;
						end
						else
							illegal_insn = 1'b1;
					3'b101:
						if (insn_from_imem[31:25] == 7'd0) begin
							rf_we = 1'b1;
							rf_rd = insn_rd;
							rf_rd_data = rs1_data >> imm_shamt;
						end
						else if (insn_from_imem[31:25] == 7'b0100000) begin
							rf_we = 1'b1;
							rf_rd = insn_rd;
							rf_rd_data = $signed(rs1_data) >>> imm_shamt;
						end
						else
							illegal_insn = 1'b1;
					default: illegal_insn = 1'b1;
				endcase
			OpRegReg:
				if (insn_mul) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					rf_rd_data = mul_signed_prod[31:0];
				end
				else if (insn_mulh) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					rf_rd_data = mul_signed_prod[63:32];
				end
				else if (insn_mulhsu) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					rf_rd_data = mulhsu_prod[63:32];
				end
				else if (insn_mulhu) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					rf_rd_data = mulhu_prod[63:32];
				end
				else if (insn_div) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					if (rs2_data == 32'd0)
						rf_rd_data = 32'hffffffff;
					else if ((rs1_data == 32'h80000000) && (rs2_data == 32'hffffffff))
						rf_rd_data = 32'h80000000;
					else
						rf_rd_data = (rs1_data[31] ^ rs2_data[31] ? ~s_div_quotient + 32'd1 : s_div_quotient);
				end
				else if (insn_divu) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					if (rs2_data == 32'd0)
						rf_rd_data = 32'hffffffff;
					else
						rf_rd_data = divu_quotient;
				end
				else if (insn_rem) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					if (rs2_data == 32'd0)
						rf_rd_data = rs1_data;
					else if ((rs1_data == 32'h80000000) && (rs2_data == 32'hffffffff))
						rf_rd_data = 32'd0;
					else
						rf_rd_data = (rs1_data[31] ? ~s_div_remainder + 32'd1 : s_div_remainder);
				end
				else if (insn_remu) begin
					rf_we = 1'b1;
					rf_rd = insn_rd;
					if (rs2_data == 32'd0)
						rf_rd_data = rs1_data;
					else
						rf_rd_data = divu_remainder;
				end
				else
					case (insn_funct3)
						3'b000:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = cla_sum;
							end
							else if (insn_from_imem[31:25] == 7'b0100000) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = cla_sum;
							end
							else
								illegal_insn = 1'b1;
						3'b001:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = rs1_data << rs2_data[4:0];
							end
							else
								illegal_insn = 1'b1;
						3'b010:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = ($signed(rs1_data) < $signed(rs2_data) ? 32'd1 : 32'd0);
							end
							else
								illegal_insn = 1'b1;
						3'b011:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = (rs1_data < rs2_data ? 32'd1 : 32'd0);
							end
							else
								illegal_insn = 1'b1;
						3'b100:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = rs1_data ^ rs2_data;
							end
							else
								illegal_insn = 1'b1;
						3'b101:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = rs1_data >> rs2_data[4:0];
							end
							else if (insn_from_imem[31:25] == 7'b0100000) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = $signed(rs1_data) >>> rs2_data[4:0];
							end
							else
								illegal_insn = 1'b1;
						3'b110:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = rs1_data | rs2_data;
							end
							else
								illegal_insn = 1'b1;
						3'b111:
							if (insn_from_imem[31:25] == 7'd0) begin
								rf_we = 1'b1;
								rf_rd = insn_rd;
								rf_rd_data = rs1_data & rs2_data;
							end
							else
								illegal_insn = 1'b1;
						default: illegal_insn = 1'b1;
					endcase
			OpBranch:
				case (insn_funct3)
					3'b000:
						if (rs1_data == rs2_data)
							pcNext = pcCurrent + imm_b_sext;
					3'b001:
						if (rs1_data != rs2_data)
							pcNext = pcCurrent + imm_b_sext;
					3'b100:
						if ($signed(rs1_data) < $signed(rs2_data))
							pcNext = pcCurrent + imm_b_sext;
					3'b101:
						if ($signed(rs1_data) >= $signed(rs2_data))
							pcNext = pcCurrent + imm_b_sext;
					3'b110:
						if (rs1_data < rs2_data)
							pcNext = pcCurrent + imm_b_sext;
					3'b111:
						if (rs1_data >= rs2_data)
							pcNext = pcCurrent + imm_b_sext;
					default: illegal_insn = 1'b1;
				endcase
			OpEnviron:
				if (insn_from_imem[31:7] == 25'd0)
					halt = 1'b1;
				else
					illegal_insn = 1'b1;
			OpLoad: begin
				addr_to_dmem = {calc_addr[31:2], 2'b00};
				case (insn_funct3)
					3'b000: begin
						case (calc_addr[1:0])
							2'd0: rf_rd_data = {{24 {load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
							2'd1: rf_rd_data = {{24 {load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
							2'd2: rf_rd_data = {{24 {load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
							2'd3: rf_rd_data = {{24 {load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
							default: rf_rd_data = 32'd0;
						endcase
						rf_we = 1'b1;
						rf_rd = insn_rd;
					end
					3'b001: begin
						case (calc_addr[1])
							1'b0: rf_rd_data = {{16 {load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};
							1'b1: rf_rd_data = {{16 {load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};
							default: rf_rd_data = 32'd0;
						endcase
						rf_we = 1'b1;
						rf_rd = insn_rd;
					end
					3'b010: begin
						rf_rd_data = load_data_from_dmem;
						rf_we = 1'b1;
						rf_rd = insn_rd;
					end
					3'b100: begin
						case (calc_addr[1:0])
							2'd0: rf_rd_data = {24'd0, load_data_from_dmem[7:0]};
							2'd1: rf_rd_data = {24'd0, load_data_from_dmem[15:8]};
							2'd2: rf_rd_data = {24'd0, load_data_from_dmem[23:16]};
							2'd3: rf_rd_data = {24'd0, load_data_from_dmem[31:24]};
							default: rf_rd_data = 32'd0;
						endcase
						rf_we = 1'b1;
						rf_rd = insn_rd;
					end
					3'b101: begin
						case (calc_addr[1])
							1'b0: rf_rd_data = {16'd0, load_data_from_dmem[15:0]};
							1'b1: rf_rd_data = {16'd0, load_data_from_dmem[31:16]};
							default: rf_rd_data = 32'd0;
						endcase
						rf_we = 1'b1;
						rf_rd = insn_rd;
					end
					default: illegal_insn = 1'b1;
				endcase
			end
			OpStore: begin
				addr_to_dmem = {calc_store_addr[31:2], 2'b00};
				case (insn_funct3)
					3'b000: begin
						store_we_to_dmem = 4'b0001 << calc_store_addr[1:0];
						store_data_to_dmem = {4 {rs2_data[7:0]}};
					end
					3'b001: begin
						case (calc_store_addr[1:0])
							2'd0: store_we_to_dmem = 4'b0011;
							2'd2: store_we_to_dmem = 4'b1100;
							default: begin
								store_we_to_dmem = 4'b0000;
								illegal_insn = 1'b1;
							end
						endcase
						store_data_to_dmem = {2 {rs2_data[15:0]}};
					end
					3'b010: begin
						store_we_to_dmem = 4'b1111;
						store_data_to_dmem = rs2_data;
					end
					default: illegal_insn = 1'b1;
				endcase
			end
			OpJal: begin
				rf_we = 1'b1;
				rf_rd = insn_rd;
				rf_rd_data = pcCurrent + 4;
				pcNext = pcCurrent + imm_j_sext;
			end
			OpJalr: begin
				rf_we = 1'b1;
				rf_rd = insn_rd;
				rf_rd_data = pcCurrent + 4;
				pcNext = (rs1_data + imm_i_sext) & 32'hfffffffe;
			end
			OpMiscMem: pcNext = pcCurrent + 4;
			default: illegal_insn = 1'b1;
		endcase
	end
	always @(posedge clk)
		if (rst)
			pcCurrent <= 32'd0;
		else
			pcCurrent <= pcNext;
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clock_mem,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	parameter signed [31:0] NUM_WORDS = 512;
	input wire rst;
	input wire clock_mem;
	input wire [31:0] pc_to_imem;
	output reg [31:0] insn_from_imem;
	input wire [31:0] addr_to_dmem;
	output reg [31:0] load_data_from_dmem;
	input wire [31:0] store_data_to_dmem;
	input wire [3:0] store_we_to_dmem;
	reg [31:0] mem_array [0:NUM_WORDS - 1];
	initial $readmemh("mem_initial_contents.hex", mem_array);
	always @(*)
		if (_sv2v_0)
			;
	localparam signed [31:0] AddrMsb = $clog2(NUM_WORDS) + 1;
	localparam signed [31:0] AddrLsb = 2;
	always @(posedge clock_mem)
		if (rst)
			;
		else
			insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clock_mem)
		if (rst)
			;
		else begin
			if (store_we_to_dmem[0])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
			if (store_we_to_dmem[1])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
			if (store_we_to_dmem[2])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
			if (store_we_to_dmem[3])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
			load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
		end
	initial _sv2v_0 = 0;
endmodule
`default_nettype none
module SystemResourceCheck (
	external_clk_25MHz,
	btn,
	led
);
	input wire external_clk_25MHz;
	input wire [6:0] btn;
	output wire [7:0] led;
	wire clk_proc;
	wire clk_mem;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.clk_mem(clk_mem),
		.locked(clk_locked)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	MemorySingleCycle #(.NUM_WORDS(128)) memory(
		.rst(!clk_locked),
		.clock_mem(clk_mem),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we)
	);
	DatapathSingleCycle datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem(mem_data_loaded_value),
		.halt(led[0])
	);
endmodule