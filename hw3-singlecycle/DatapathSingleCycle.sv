`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`include "../hw2a-divider/divider_unsigned.sv"
`include "../hw2b-cla/cla.sv"

module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;
  logic [`REG_SIZE] regs[NumRegs];

  // TODO: your code here
  // Reset logic
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < NumRegs; i++) begin
        regs[i] <= 32'd0;
      end
    end else if (we && rd != 5'd0) begin
      regs[rd] <= rd_data;
    end
  end

  // Read logic
  always_comb begin
    rs1_data = (rs1 != 5'd0) ? regs[rs1] : 32'd0;
    rs2_data = (rs2 != 5'd0) ? regs[rs2] : 32'd0;
  end

endmodule

module DatapathSingleCycle (
    input wire clk,
    input wire rst,
    output logic halt,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`REG_SIZE] insn_from_imem,
    // addr_to_dmem is a read-write port
    output logic [`REG_SIZE] addr_to_dmem,
    input logic [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem
);

  // components of the instruction
  wire [6:0] insn_funct7;
  wire [4:0] insn_rs2;
  wire [4:0] insn_rs1;
  wire [2:0] insn_funct3;
  wire [4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;

  // setup for I, S, B & J type instructions
  // I - short immediates and loads
  wire [11:0] imm_i;
  assign imm_i = insn_from_imem[31:20];
  wire [ 4:0] imm_shamt = insn_from_imem[24:20];

  // S - stores
  wire [11:0] imm_s;
  assign imm_s[11:5] = insn_funct7, imm_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {
    insn_from_imem[31:12], 1'b0
  };

  wire [`REG_SIZE] imm_i_sext = {{20{imm_i[11]}}, imm_i[11:0]};
  wire [`REG_SIZE] imm_s_sext = {{20{imm_s[11]}}, imm_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  wire insn_lui = insn_opcode == OpLui;
  wire insn_auipc = insn_opcode == OpAuipc;
  wire insn_jal = insn_opcode == OpJal;
  wire insn_jalr = insn_opcode == OpJalr;

  wire insn_beq = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b000;
  wire insn_bne = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b001;
  wire insn_blt = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b100;
  wire insn_bge = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b101;
  wire insn_bltu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b110;
  wire insn_bgeu = insn_opcode == OpBranch && insn_from_imem[14:12] == 3'b111;

  wire insn_lb = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b000;
  wire insn_lh = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b001;
  wire insn_lw = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b010;
  wire insn_lbu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b100;
  wire insn_lhu = insn_opcode == OpLoad && insn_from_imem[14:12] == 3'b101;

  wire insn_sb = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b000;
  wire insn_sh = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b001;
  wire insn_sw = insn_opcode == OpStore && insn_from_imem[14:12] == 3'b010;

  wire insn_addi = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b000;
  wire insn_slti = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b010;
  wire insn_sltiu = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b011;
  wire insn_xori = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b100;
  wire insn_ori = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b110;
  wire insn_andi = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b111;

  wire insn_slli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_srli = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_srai = insn_opcode == OpRegImm && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;

  wire insn_add  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'd0;
  wire insn_sub  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b000 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_sll  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b001 && insn_from_imem[31:25] == 7'd0;
  wire insn_slt  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b010 && insn_from_imem[31:25] == 7'd0;
  wire insn_sltu = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b011 && insn_from_imem[31:25] == 7'd0;
  wire insn_xor  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b100 && insn_from_imem[31:25] == 7'd0;
  wire insn_srl  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'd0;
  wire insn_sra  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b101 && insn_from_imem[31:25] == 7'b0100000;
  wire insn_or   = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b110 && insn_from_imem[31:25] == 7'd0;
  wire insn_and  = insn_opcode == OpRegReg && insn_from_imem[14:12] == 3'b111 && insn_from_imem[31:25] == 7'd0;

  wire insn_mul    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b000;
  wire insn_mulh   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b001;
  wire insn_mulhsu = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b010;
  wire insn_mulhu  = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b011;
  wire insn_div    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b100;
  wire insn_divu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b101;
  wire insn_rem    = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b110;
  wire insn_remu   = insn_opcode == OpRegReg && insn_from_imem[31:25] == 7'd1 && insn_from_imem[14:12] == 3'b111;

  wire insn_ecall = insn_opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
  wire insn_fence = insn_opcode == OpMiscMem;

  // this code is only for simulation, not synthesis
`ifndef SYNTHESIS
  `include "RvDisassembler.sv"
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn_from_imem);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic...
  wire [(8*32)-1:0] disasm_wire;
  genvar i;
  for (i = 0; i < 32; i = i + 1) begin : gen_disasm
    assign disasm_wire[(((i+1))*8)-1:((i)*8)] = disasm_string[31-i];
  end
`endif

  // program counter
  logic [`REG_SIZE] pcNext, pcCurrent;
  always @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  assign pc_to_imem = pcCurrent;

  // cycle/insn_from_imem counters
  logic [`REG_SIZE] cycles_current, num_insns_current;
  always @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
      num_insns_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
      if (!rst) begin
        num_insns_current <= num_insns_current + 1;
      end
    end
  end

  // NOTE: don't rename your RegFile instance as the tests expect it to be `rf`
  // TODO: you will need to edit the port connections, however.
  wire [`REG_SIZE] rs1_data;
  wire [`REG_SIZE] rs2_data;

  // Define intermediate signals
  logic rf_we;
  logic [4:0] rf_rd;
  logic [`REG_SIZE] rf_rd_data;


  RegFile rf (
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

  logic [`REG_SIZE] cla_sum;

  logic [`REG_SIZE] cla_b_input;

  // always_comb begin
  //   if (insn_addi) begin
  //     cla_b_input = imm_i_sext;
  //   end else if (insn_add) begin
  //     cla_b_input = rs2_data;
  //   end else if (insn_sub) begin
  //     cla_b_input = ~rs2_data + 1;
  //   end else begin
  //     cla_b_input = 32'd0; // Default value
  //   end
  // end

  always_comb begin
    case (1'b1)
      insn_addi: cla_b_input = imm_i_sext;
      insn_add:  cla_b_input = rs2_data;
      insn_sub:  cla_b_input = ~rs2_data + 1;
      default:   cla_b_input = 32'd0;  // Default value
    endcase
  end

  cla cla_adder (
      .a  (rs1_data),
      .b  (cla_b_input),
      .cin(1'b0),
      .sum(cla_sum)
  );

  // cla cla_adder_add (
  //         .a(rs1_data),
  //         .b(rs2_data),
  //         .cin(1'b0),
  //         .sum(cla_sum)
  //       );

  // For the unsigned division instructions (divu and remu)
  logic [31:0] div_dividend, div_divisor;
  always_comb begin
    case (1'b1)
      insn_divu, insn_remu: begin
        div_dividend = rs1_data;
        div_divisor  = rs2_data;
      end

      default: begin
        div_dividend = 32'd0;
        div_divisor  = 32'd0;
      end
    endcase
  end


  // Wires for the unsigned divider outputs.
  wire [31:0] divu_quotient;
  wire [31:0] divu_remainder;

  // Instantiate divider
  divider_unsigned u_divider (
      .i_dividend (rs1_data),
      .i_divisor  (rs2_data),
      .o_quotient (divu_quotient),
      .o_remainder(divu_remainder)
  );

  wire [31:0] signed_dividend = (rs1_data[31]) ? -rs1_data : rs1_data;
  wire [31:0] signed_divisor = (rs2_data[31]) ? -rs2_data : rs2_data;
  wire [31:0] s_div_quotient;
  wire [31:0] s_div_remainder;

  divider_unsigned u_signed_divider (
      .i_dividend (signed_dividend),
      .i_divisor  (signed_divisor),
      .o_quotient (s_div_quotient),
      .o_remainder(s_div_remainder)
  );


  logic illegal_insn;



  logic [31:0] effective_addr;
  logic [15:0] halfword;


  always_comb begin
    illegal_insn = 1'b0;
    rf_we = 1'b0;  // Default to no write
    rf_rd = 0;  // 5'd0; // Default to register 0
    rf_rd_data = 0;  // 32'd0; // Default to 0
    halt = 1'b0;  // Default to no halt
    pcNext = pcCurrent + 4;  // Default to current PC
    addr_to_dmem = 32'd0;


    case (insn_opcode)
      OpLui: begin
        // Implementing lui
        rf_we = 1'b1;
        rf_rd = insn_rd;
        rf_rd_data = {insn_from_imem[31:12], 12'b0};
      end
      OpAuipc: begin
        // Implementing auipc
        rf_we = 1'b1;
        rf_rd = insn_rd;
        rf_rd_data = pcCurrent + {insn_from_imem[31:12], 12'b0};
      end
      OpRegImm: begin
        case (insn_funct3)
          3'b000: begin
            // Implementing addi
            rf_we = 1'b1;
            rf_rd = insn_rd;
            // rs2_data = imm_i_sext;
            rf_rd_data = cla_sum;
          end
          3'b010: begin
            // Implementing slti
            rf_we = 1'b1;
            rf_rd = insn_rd;
            rf_rd_data = ($signed(rs1_data) < $signed(imm_i_sext)) ? 32'd1 : 32'd0;
          end
          3'b011: begin
            // Implementing sltiu
            rf_we = 1'b1;
            rf_rd = insn_rd;
            rf_rd_data = (rs1_data < imm_i_sext) ? 32'd1 : 32'd0;
          end
          3'b100: begin
            // Implementing xori
            rf_we = 1'b1;
            rf_rd = insn_rd;
            rf_rd_data = rs1_data ^ imm_i_sext;
          end
          3'b110: begin
            // Implementing ori
            rf_we = 1'b1;
            rf_rd = insn_rd;
            rf_rd_data = rs1_data | imm_i_sext;
          end
          3'b111: begin
            // Implementing andi
            rf_we = 1'b1;
            rf_rd = insn_rd;
            rf_rd_data = rs1_data & imm_i_sext;
          end
          3'b001: begin
            // Implementing slli
            if (insn_from_imem[31:25] == 7'd0) begin
              rf_we = 1'b1;
              rf_rd = insn_rd;
              rf_rd_data = rs1_data << imm_shamt;
            end else begin
              illegal_insn = 1'b1;
            end
          end
          3'b101: begin
            if (insn_from_imem[31:25] == 7'd0) begin
              // Implementing srli
              rf_we = 1'b1;
              rf_rd = insn_rd;
              rf_rd_data = rs1_data >> imm_shamt;
            end else if (insn_from_imem[31:25] == 7'b0100000) begin
              // Implementing srai
              rf_we = 1'b1;
              rf_rd = insn_rd;
              rf_rd_data = $signed(rs1_data) >>> imm_shamt;
            end else begin
              illegal_insn = 1'b1;
            end
          end
          default: begin
            illegal_insn = 1'b1;
          end
        endcase
      end

      OpRegReg: begin
        if (insn_mul) begin
          // MUL: rd = (rs1 * rs2)[31:0]
          logic signed [63:0] prod;
          prod = $signed({{32{rs1_data[31]}}, rs1_data}) * $signed({{32{rs2_data[31]}}, rs2_data});
          rf_we = 1'b1;
          rf_rd = insn_rd;
          rf_rd_data = prod[31:0];
        end else if (insn_mulh) begin
          // MULH: rd = (signed(rs1) * signed(rs2))[63:32]
          logic signed [63:0] prod;
          prod = $signed({{32{rs1_data[31]}}, rs1_data}) * $signed({{32{rs2_data[31]}}, rs2_data});
          rf_we = 1'b1;
          rf_rd = insn_rd;
          rf_rd_data = prod[63:32];
        end else if (insn_mulhsu) begin
          // MULHSU: rd = (signed(rs1) * unsigned(rs2))[63:32]
          logic signed [63:0] prod;
          prod = $signed({{32{rs1_data[31]}}, rs1_data}) * $unsigned({32'b0, rs2_data});
          rf_we = 1'b1;
          rf_rd = insn_rd;
          rf_rd_data = prod[63:32];
        end else if (insn_mulhu) begin
          // MULHU: rd = (unsigned(rs1) * unsigned(rs2))[63:32]
          logic [63:0] prod;
          prod = {32'b0, rs1_data} * {32'b0, rs2_data};
          rf_we = 1'b1;
          rf_rd = insn_rd;
          rf_rd_data = prod[63:32];
        end else if (insn_div) begin
          // DIV: rd = rs1 / signed(rs2) using the divider module with sign adjustment.
          if (rs2_data == 32'd0) begin
            rf_rd_data = 32'hFFFFFFFF;  // Division by zero yields -1.
          end else if ((rs1_data == 32'h80000000) && (rs2_data == 32'hFFFFFFFF)) begin
            rf_rd_data = 32'h80000000;  // Special case: MIN_INT / -1.
          end else begin
            logic [31:0] abs_dividend;
            logic [31:0] abs_divisor;
            abs_dividend = (rs1_data[31]) ? ((~rs1_data) + 32'd1) : rs1_data;
            abs_divisor = (rs2_data[31]) ? ((~rs2_data) + 32'd1) : rs2_data;
            rf_rd_data = (rs1_data[31] ^ rs2_data[31])
                         ? ((~s_div_quotient) + 32'd1)
                         : s_div_quotient;
          end
          rf_we = 1'b1;
          rf_rd = insn_rd;
          // $display("DIV: rs1=%h, rs2=%h, result=%h, rd=%d", rs1_data, rs2_data, rf_rd_data,
          //          insn_rd);
        end else if (insn_divu) begin
          // DIVU: rd = rs1 / unsign(rs2) using the divider module.
          if (rs2_data == 32'd0) begin
            rf_rd_data = 32'hFFFFFFFF;
          end else begin
            rf_rd_data = divu_quotient;
          end
          rf_we = 1'b1;
          rf_rd = insn_rd;
        end else if (insn_rem) begin
          // REM: rd = rs1 % signed(rs2) using the divider module with sign adjustment
          if (rs2_data == 32'd0) begin
            rf_rd_data = rs1_data;
          end else if ((rs1_data == 32'h80000000) && (rs2_data == 32'hFFFFFFFF)) begin
            rf_rd_data = 32'd0;
          end else begin
            // The remainder takes the sign of the dividend.
            logic [31:0] abs_dividend;
            logic [31:0] abs_divisor;
            abs_dividend = (rs1_data[31]) ? ((~rs1_data) + 32'd1) : rs1_data;
            abs_divisor  = (rs2_data[31]) ? ((~rs2_data) + 32'd1) : rs2_data;
            rf_rd_data   = (rs1_data[31]) ? ((~s_div_remainder) + 32'd1) : s_div_remainder;
          end
          rf_we = 1'b1;
          rf_rd = insn_rd;
        end else if (insn_remu) begin
          // REMU: rd = rs1 % unsign(rs2)
          if (rs2_data == 32'd0) begin
            rf_rd_data = rs1_data;
          end else begin
            rf_rd_data = divu_remainder;
          end
          rf_we = 1'b1;
          rf_rd = insn_rd;
        end else begin
          // Remaining R-type operations.
          case (insn_funct3)
            3'b000: begin
              if (insn_from_imem[31:25] == 7'd0) begin
                // Implementing add using CLA adder.
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = cla_sum;
              end else if (insn_from_imem[31:25] == 7'b0100000) begin
                // Implementing sub.
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = cla_sum;
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b001: begin
              // Implementing sll.
              if (insn_from_imem[31:25] == 7'd0) begin
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = rs1_data << rs2_data[4:0];
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b010: begin
              // Implementing slt.
              if (insn_from_imem[31:25] == 7'd0) begin
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = ($signed(rs1_data) < $signed(rs2_data)) ? 32'd1 : 32'd0;
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b011: begin
              // Implementing sltu.
              if (insn_from_imem[31:25] == 7'd0) begin
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = (rs1_data < rs2_data) ? 32'd1 : 32'd0;
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b100: begin
              // Implementing xor.
              if (insn_from_imem[31:25] == 7'd0) begin
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = rs1_data ^ rs2_data;
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b101: begin
              if (insn_from_imem[31:25] == 7'd0) begin
                // Implementing srl.
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = rs1_data >> rs2_data[4:0];
              end else if (insn_from_imem[31:25] == 7'b0100000) begin
                // Implementing sra.
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = $signed(rs1_data) >>> rs2_data[4:0];
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b110: begin
              // Implementing or.
              if (insn_from_imem[31:25] == 7'd0) begin
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = rs1_data | rs2_data;
              end else begin
                illegal_insn = 1'b1;
              end
            end
            3'b111: begin
              // Implementing and.
              if (insn_from_imem[31:25] == 7'd0) begin
                rf_we = 1'b1;
                rf_rd = insn_rd;
                rf_rd_data = rs1_data & rs2_data;
              end else begin
                illegal_insn = 1'b1;
              end
            end
            default: begin
              illegal_insn = 1'b1;
            end
          endcase
        end
      end


      OpBranch: begin
        case (insn_funct3)
          3'b000: begin
            // Implementing beq
            if (rs1_data == rs2_data) begin
              pcNext = pcCurrent + imm_b_sext;
            end
          end
          3'b001: begin
            // Implementing bne
            if (rs1_data != rs2_data) begin
              pcNext = pcCurrent + imm_b_sext;
            end
          end
          3'b100: begin
            // Implementing blt
            if ($signed(rs1_data) < $signed(rs2_data)) begin
              pcNext = pcCurrent + imm_b_sext;
            end
          end
          3'b101: begin
            // Implementing bge
            if ($signed(rs1_data) >= $signed(rs2_data)) begin
              pcNext = pcCurrent + imm_b_sext;
            end
          end
          3'b110: begin
            // Implementing bltu
            if (rs1_data < rs2_data) begin
              pcNext = pcCurrent + imm_b_sext;
            end
          end
          3'b111: begin
            // Implementing bgeu
            if (rs1_data >= rs2_data) begin
              pcNext = pcCurrent + imm_b_sext;
            end
          end
          default: begin
            illegal_insn = 1'b1;
          end
        endcase
      end
      OpEnviron: begin
        if (insn_from_imem[31:7] == 25'd0) begin
          // Implementing ecall
          // Transfer control to OS (this is a placeholder, actual implementation may vary)
          // For now, we can set a halt signal or similar
          halt = 1'b1;
        end else begin
          illegal_insn = 1'b1;
        end
      end



      ////////////////////////////////////////////////
      OpLoad: begin
        // Compute effective address: rs1 + imm (already sign-extended)
        // logic [`REG_SIZE] effective_addr;
        // rs1_data + imm_i_sext;

        // Force memory access address to be word-aligned.
        effective_addr = rs1_data + imm_i_sext;  // Compute sum first
        addr_to_dmem   = {effective_addr[31:2], 2'b00};  // Now use slicing

        case (insn_funct3)
          3'b000: begin  // LB - Load Byte (Sign-Extend)
            case (effective_addr[1:0])
              2'b00:   rf_rd_data = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};
              2'b01:   rf_rd_data = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};
              2'b10:   rf_rd_data = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};
              2'b11:   rf_rd_data = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};
              default: rf_rd_data = 32'd0;
            endcase
            rf_we = 1'b1;
            rf_rd = insn_rd;
          end

          3'b001: begin  // lh: load halfword and sign-extend
            // For LH, we only need effective_addr[1] to select the halfword.
            case (effective_addr[1])
              1'b0: halfword = load_data_from_dmem[15:0];
              1'b1: halfword = load_data_from_dmem[31:16];
              default: halfword = 16'd0;  // Should never occur.
            endcase
            rf_rd_data = {{16{halfword[15]}}, halfword};
            rf_we = 1'b1;
            rf_rd = insn_rd;
            // $display(
            //     // "LH: effective_addr=%h, addr_to_dmem=%h, load_data=%h, halfword=%h, rf_rd_data=%h, rd=%d",
            //     // effective_addr, addr_to_dmem, load_data_from_dmem, halfword, rf_rd_data, insn_rd);
          end

          3'b010: begin  // lw: load word
            // addr_to_dmem = effective_addr;  // already word-aligned if properly computed.
            rf_rd_data = load_data_from_dmem;
            rf_we = 1'b1;
            rf_rd = insn_rd;
          end

          3'b100: begin  // lbu: load byte, zero-extended
            case (effective_addr[1:0])
              2'b00:   rf_rd_data = {24'd0, load_data_from_dmem[7:0]};
              2'b01:   rf_rd_data = {24'd0, load_data_from_dmem[15:8]};
              2'b10:   rf_rd_data = {24'd0, load_data_from_dmem[23:16]};
              2'b11:   rf_rd_data = {24'd0, load_data_from_dmem[31:24]};
              default: rf_rd_data = 32'd0;
            endcase
            rf_we = 1'b1;
            rf_rd = insn_rd;
          end

          3'b101: begin  // lhu: load halfword, zero-extended
            // logic [15:0] halfword;
            case (effective_addr[1])
              1'b0: halfword = load_data_from_dmem[15:0];
              1'b1: halfword = load_data_from_dmem[31:16];
              default: halfword = 16'd0;
            endcase
            rf_rd_data = {16'd0, halfword};
            rf_we = 1'b1;
            rf_rd = insn_rd;
          end

          default: begin
            illegal_insn = 1'b1;
          end
        endcase
      end

      OpStore: begin
        logic [`REG_SIZE] addr_temp;
        addr_temp = rs1_data + imm_s_sext;  // Store the computed address

        case (insn_funct3)
          3'b000: begin  // sb - store byte (8-bit)
            addr_to_dmem = {addr_temp[31:2], 2'b00};  // Force word alignment
            store_we_to_dmem = 4'b0001 << addr_temp[1:0];  // Enable only one byte
            store_data_to_dmem = {4{rs2_data[7:0]}};  // Replicate byte
          end

          3'b001: begin  // sh - store halfword (16-bit)
            // Compute effective address for store.
            // We force word alignment by taking the upper 30 bits and appending 2'b00.
            addr_to_dmem = {addr_temp[31:2], 2'b00};
            // Use the lower two bits of addr_temp to determine which halfword should be written.
            // For a properly aligned halfword, addr_temp[1:0] should be either 00 or 10.
            // If it is 00, write to lower half (store enable 4'b0011).
            // If it is 10, write to upper half (store enable 4'b1100).
            case (addr_temp[1:0])
              2'b00: store_we_to_dmem = 4'b0011;
              2'b10: store_we_to_dmem = 4'b1100;
              default: begin
                // This should not occur for a valid halfword-aligned address.
                store_we_to_dmem = 4'b0000;
                illegal_insn = 1'b1;
              end
            endcase
            // Replicate the 16-bit value across the two halfwords.
            store_data_to_dmem = {2{rs2_data[15:0]}};
          end


          3'b010: begin  // sw - store word (32-bit)
            addr_to_dmem = {addr_temp[31:2], 2'b00};  // Force word alignment
            store_we_to_dmem = 4'b1111;  // Enable all four bytes
            store_data_to_dmem = rs2_data;  // Store entire word
          end

          default: begin
            illegal_insn = 1'b1;
          end
        endcase

      end

      OpJal: begin
        // logic [`REG_SIZE] jal_offset;
        // logic [4:0] rd_jal;  // Temporary variable for correct JAL rd extraction

        // jal_offset = {
        //   {12{insn_from_imem[31]}},  // Sign-extend
        //   insn_from_imem[19:12],  // Bits 19:12
        //   insn_from_imem[20],  // Bit 20
        //   insn_from_imem[30:21],  // Bits 30:21
        //   1'b0  // Least significant bit 0
        // };

        // rd_jal = insn_from_imem[11:7];

        rf_we = 1'b1;
        rf_rd = insn_rd;
        rf_rd_data = pcCurrent + 4;

        pcNext = pcCurrent + imm_j_sext;

        // // Debugging output (for waveforms)
        // $display("JAL: pcCurrent=%h, pcNext=%h, jal_offset=%h, rd=%d, rf_rd_data=%h", pcCurrent,
        //          pcNext, jal_offset, rd_jal, rf_rd_data);
      end

      OpJalr: begin
        // Implementing JALR: rd = pcCurrent + 4; pcNext = (rs1_data + imm_i_sext) & ~1
        rf_we = 1'b1;
        rf_rd = insn_rd;
        rf_rd_data = pcCurrent + 4;
        // Compute the new PC: add rs1_data and the immediate then clear the LSB
        pcNext = (rs1_data + imm_i_sext) & ~32'd1;
        // $display("JALR: pcCurrent=%h, pcNext=%h, rs1_data=%h, imm_i_sext=%h, rd=%d, rf_rd_data=%h",
        //          pcCurrent, pcNext, rs1_data, imm_i_sext, insn_rd, rf_rd_data);
      end

      OpMiscMem: begin
        pcNext = pcCurrent + 4;
      end


      default: begin
        illegal_insn = 1'b1;
      end
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      pcCurrent <= 32'd0;
    end else begin
      pcCurrent <= pcNext;
    end
  end
  // assign pc_to_imem = pcCurrent;

endmodule

/* A memory module that supports 1-cycle reads and writes, with one read-only port
 * and one read+write port.
 */
module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. See RiscvProcessor for clock details.
    input wire clock_mem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] pc_to_imem,

    // the value at memory location pc_to_imem
    output logic [`REG_SIZE] insn_from_imem,

    // must always be aligned to a 4B boundary
    input wire [`REG_SIZE] addr_to_dmem,

    // the value at memory location addr_to_dmem
    output logic [`REG_SIZE] load_data_from_dmem,

    // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    input wire [`REG_SIZE] store_data_to_dmem,

    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input wire [3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  logic [`REG_SIZE] mem_array[NUM_WORDS];

`ifdef SYNTHESIS
  initial begin
    $readmemh("mem_initial_contents.hex", mem_array);
  end
`endif

  always_comb begin
    // memory addresses should always be 4B-aligned
    assert (pc_to_imem[1:0] == 2'b00);
    assert (addr_to_dmem[1:0] == 2'b00);
  end

  localparam int AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam int AddrLsb = 2;

  always @(posedge clock_mem) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clock_mem) begin
    if (rst) begin
    end else begin
      if (store_we_to_dmem[0]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
      end
      if (store_we_to_dmem[1]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
      end
      if (store_we_to_dmem[2]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
      end
      if (store_we_to_dmem[3]) begin
        mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
      end
      // dmem is "read-first": read returns value before the write
      load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
    end
  end
endmodule

/*
This shows the relationship between clock_proc and clock_mem. The clock_mem is
phase-shifted 90° from clock_proc. You could think of one proc cycle being
broken down into 3 parts. During part 1 (which starts @posedge clock_proc)
the current PC is sent to the imem. In part 2 (starting @posedge clock_mem) we
read from imem. In part 3 (starting @negedge clock_mem) we read/write memory and
prepare register/PC updates, which occur at @posedge clock_proc.

        ____
 proc: |    |______
           ____
 mem:  ___|    |___
*/
module Processor (
    input  wire  clock_proc,
    input  wire  clock_mem,
    input  wire  rst,
    output logic halt
);

  wire [`REG_SIZE] pc_to_imem, insn_from_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clock_mem          (clock_mem),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathSingleCycle datapath (
      .clk(clock_proc),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt)
  );

endmodule
