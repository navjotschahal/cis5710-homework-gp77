`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
`include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "cycle_status.sv"

module Disasm #(
    byte PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef SYNTHESIS
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
`endif
endmodule

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
  // genvar i;
  logic [`REG_SIZE] regs[NumRegs];


  // Register read with WD bypass
  always_comb begin
    // Handle x0 (hardwired to 0) and WD bypass for rs1
    if (rs1 == 5'b0) rs1_data = 32'b0;
    else if (we && rs1 == rd && rd != 5'b0) rs1_data = rd_data;  // WD bypass
    else rs1_data = regs[rs1];

    // Handle x0 (hardwired to 0) and WD bypass for rs2
    if (rs2 == 5'b0) rs2_data = 32'b0;
    else if (we && rs2 == rd && rd != 5'b0) rs2_data = rd_data;  // WD bypass
    else rs2_data = regs[rs2];
  end

  // Register write on positive clock edge
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int j = 0; j < NumRegs; j = j + 1) begin
        regs[j] <= 32'b0;
      end
    end else if (we && rd != 5'b0) begin
      regs[rd] <= rd_data;
    end
  end
endmodule

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

module DatapathPipelined (
    input wire clk,
    input wire rst,
    output logic [`REG_SIZE] pc_to_imem,
    input wire [`INSN_SIZE] insn_from_imem,
    // dmem is read/write
    output logic [`REG_SIZE] addr_to_dmem,
    input wire [`REG_SIZE] load_data_from_dmem,
    output logic [`REG_SIZE] store_data_to_dmem,
    output logic [3:0] store_we_to_dmem,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpcodeLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpcodeJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpcodeMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpcodeJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpcodeRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpcodeEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpcodeAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpcodeLui = 7'b01_101_11;

  // ALU operation type definitions
  typedef enum logic [3:0] {
    ALU_ADD,
    ALU_SUB,
    ALU_SLL,
    ALU_SLT,
    ALU_SLTU,
    ALU_XOR,
    ALU_SRL,
    ALU_SRA,
    ALU_OR,
    ALU_AND,
    ALU_LUI,
    ALU_AUIPC
  } alu_op_e;

  // Branch condition types
  typedef enum logic [2:0] {
    BR_EQ,
    BR_NE,
    BR_LT,
    BR_GE,
    BR_LTU,
    BR_GEU,
    BR_NONE
  } branch_type_e;

  // cycle counter, not really part of any stage but useful for orienting within GtkWave
  // do not rename this as the testbench uses this value
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  /***************/
  /* FETCH STAGE */
  /***************/

  logic          [ `REG_SIZE] f_pc_current;
  wire           [`INSN_SIZE] f_insn;
  cycle_status_e              f_cycle_status;

  // Signals for branch handling
  logic                       take_branch;  // Signal indicating branch is taken
  logic          [ `REG_SIZE] branch_target;  // Target address for the branch

  // Program counter logic with branch handling
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current   <= 32'd0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else if (take_branch) begin
      // On taken branch, update PC to branch target
      f_pc_current   <= branch_target;
      f_cycle_status <= CYCLE_TAKEN_BRANCH;
    end else begin
      // Normal sequential execution
      f_pc_current   <= f_pc_current + 4;
      f_cycle_status <= CYCLE_NO_STALL;
    end
  end

  // Send PC to imem
  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  // Fetch stage disassembly for debug
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  /****************/
  /* DECODE STAGE */
  /****************/

  // Decode stage state
  stage_decode_t decode_state;
  logic decode_bubble;  // Flag to insert bubble in decode stage

  // Decode stage pipeline register
  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state  <= '{pc: 0, insn: 0, cycle_status: CYCLE_RESET};
      decode_bubble <= 1'b0;
    end else if (take_branch) begin
      // Insert bubble when branch is taken
      decode_state  <= '{pc: 0, insn: 0, cycle_status: CYCLE_TAKEN_BRANCH};
      decode_bubble <= 1'b1;
    end else begin
      decode_state  <= '{pc: f_pc_current, insn: f_insn, cycle_status: f_cycle_status};
      decode_bubble <= 1'b0;
    end
  end

  // TODO: your code here, though you will also need to modify some of the code above
  // TODO: the testbench requires that your register file instance is named `rf`

  // Decode stage disassembly for debug
  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );

  // Instruction fields extraction
  wire [6:0] d_opcode = decode_state.insn[6:0];
  wire [2:0] d_funct3 = decode_state.insn[14:12];
  wire [6:0] d_funct7 = decode_state.insn[31:25];
  wire [4:0] d_rd = decode_state.insn[11:7];
  wire [4:0] d_rs1 = decode_state.insn[19:15];
  wire [4:0] d_rs2 = decode_state.insn[24:20];

  // Immediate format extraction (I, S, B, U, J types)
  wire [31:0] d_imm_i = {{20{decode_state.insn[31]}}, decode_state.insn[31:20]};
  wire [31:0] d_imm_s = {
    {20{decode_state.insn[31]}}, decode_state.insn[31:25], decode_state.insn[11:7]
  };
  wire [31:0] d_imm_b = {
    {19{decode_state.insn[31]}},
    decode_state.insn[31],
    decode_state.insn[7],
    decode_state.insn[30:25],
    decode_state.insn[11:8],
    1'b0
  };
  wire [31:0] d_imm_u = {decode_state.insn[31:12], 12'b0};
  wire [31:0] d_imm_j = {
    {11{decode_state.insn[31]}},
    decode_state.insn[31],
    decode_state.insn[19:12],
    decode_state.insn[20],
    decode_state.insn[30:21],
    1'b0
  };

  // Control signals
  logic d_reg_write;  // Register write enable
  logic d_mem_write;  // Memory write enable
  logic d_mem_read;  // Memory read enable
  logic [1:0] d_wb_sel;  // Writeback select (0: ALU, 1: MEM, 2: PC+4)
  logic d_alu_src;  // ALU source select (0: rs2, 1: imm)
  alu_op_e d_alu_op;  // ALU operation
  logic [31:0] d_imm;  // Selected immediate
  branch_type_e d_branch_type;  // Branch type
  logic d_is_div;  // Indicates division/remainder operation (for Milestone 2)

  // Control logic
  always_comb begin
    // Default values
    d_reg_write   = 1'b0;
    d_mem_write   = 1'b0;
    d_mem_read    = 1'b0;
    d_wb_sel      = 2'b00;  // ALU result
    d_alu_src     = 1'b0;  // rs2
    d_alu_op      = ALU_ADD;
    d_imm         = 32'b0;
    d_branch_type = BR_NONE;
    d_is_div      = 1'b0;

    if (!decode_bubble) begin
      case (d_opcode)
        OpcodeLui: begin
          d_reg_write = 1'b1;
          d_alu_op = ALU_LUI;
          d_alu_src = 1'b1;  // Use immediate
          d_imm = d_imm_u;
        end

        OpcodeAuipc: begin
          d_reg_write = 1'b1;
          d_alu_op = ALU_AUIPC;
          d_alu_src = 1'b1;  // Use immediate
          d_imm = d_imm_u;
        end

        OpcodeRegImm: begin  // I-type ALU operations (addi, slti, etc.)
          d_reg_write = 1'b1;
          d_alu_src = 1'b1;  // Use immediate
          d_imm = d_imm_i;

          case (d_funct3)
            3'b000:  d_alu_op = ALU_ADD;  // addi
            3'b010:  d_alu_op = ALU_SLT;  // slti
            3'b011:  d_alu_op = ALU_SLTU;  // sltiu
            3'b100:  d_alu_op = ALU_XOR;  // xori
            3'b110:  d_alu_op = ALU_OR;  // ori
            3'b111:  d_alu_op = ALU_AND;  // andi
            3'b001: begin
              if (d_funct7 == 7'b0) d_alu_op = ALU_SLL;  // slli
              else d_alu_op = ALU_ADD;  // Illegal instruction, default to ADD
            end
            3'b101: begin
              if (d_funct7 == 7'b0) d_alu_op = ALU_SRL;  // srli
              else if (d_funct7 == 7'b0100000) d_alu_op = ALU_SRA;  // srai
              else d_alu_op = ALU_ADD;  // Illegal instruction, default to ADD
            end
            default: d_alu_op = ALU_ADD;
          endcase
        end

        OpcodeRegReg: begin  // R-type operations (add, sub, etc.)
          d_reg_write = 1'b1;

          // Check for M-extension instructions (for milestone 2)
          if (d_funct7 == 7'b0000001) begin
            // For milestone 2, we'll need to identify the div/rem operations
            // and set d_is_div accordingly
            case (d_funct3)
              3'b100:  d_is_div = 1'b1;  // div
              3'b101:  d_is_div = 1'b1;  // divu
              3'b110:  d_is_div = 1'b1;  // rem
              3'b111:  d_is_div = 1'b1;  // remu
              default: d_is_div = 1'b0;
            endcase

            // For milestone 1, we don't need to implement M-extension
            d_alu_op = ALU_ADD;  // Default to ADD for now
          end else begin
            case (d_funct3)
              3'b000: begin
                if (d_funct7[5]) d_alu_op = ALU_SUB;  // sub
                else d_alu_op = ALU_ADD;  // add
              end
              3'b001:  d_alu_op = ALU_SLL;  // sll
              3'b010:  d_alu_op = ALU_SLT;  // slt
              3'b011:  d_alu_op = ALU_SLTU;  // sltu
              3'b100:  d_alu_op = ALU_XOR;  // xor
              3'b101: begin
                if (d_funct7[5]) d_alu_op = ALU_SRA;  // sra
                else d_alu_op = ALU_SRL;  // srl
              end
              3'b110:  d_alu_op = ALU_OR;  // or
              3'b111:  d_alu_op = ALU_AND;  // and
              default: d_alu_op = ALU_ADD;
            endcase
          end
        end

        OpcodeBranch: begin  // Branch instructions
          d_imm = d_imm_b;

          case (d_funct3)
            3'b000:  d_branch_type = BR_EQ;  // beq
            3'b001:  d_branch_type = BR_NE;  // bne
            3'b100:  d_branch_type = BR_LT;  // blt
            3'b101:  d_branch_type = BR_GE;  // bge
            3'b110:  d_branch_type = BR_LTU;  // bltu
            3'b111:  d_branch_type = BR_GEU;  // bgeu
            default: d_branch_type = BR_NONE;
          endcase
        end

        OpcodeJal: begin  // jal
          d_reg_write = 1'b1;
          d_wb_sel = 2'b10;  // PC+4
          d_imm = d_imm_j;
          d_branch_type = BR_EQ;  // Always take the branch
        end

        OpcodeJalr: begin  // jalr
          d_reg_write = 1'b1;
          d_wb_sel = 2'b10;  // PC+4
          d_alu_src = 1'b1;  // Use immediate
          d_imm = d_imm_i;
          d_branch_type = BR_EQ;  // Always take the branch
        end

        OpcodeEnviron: begin  // ecall
          // Handle ecall which should set the halt signal
          if (decode_state.insn[31:7] == 25'b0) begin
            // No specific control signals needed here
            // The halt logic is handled in writeback stage
          end
        end

        OpcodeMiscMem: begin  // fence instructions
          // No specific control signals needed for fence
        end

        // Add these cases to your existing control logic
        OpcodeLoad: begin
          // For Milestone 1, we don't implement loads, but define the opcode to avoid warnings
          d_reg_write = 1'b0;
          d_mem_read = 1'b0;  // Not enabling memory operations for Milestone 1
          d_alu_src = 1'b1;  // Use immediate
          d_imm = d_imm_i;
        end

        OpcodeStore: begin
          // For Milestone 1, we don't implement stores, but define the opcode to avoid warnings
          d_reg_write = 1'b0;
          d_mem_write = 1'b0;  // Not enabling memory operations for Milestone 1
          d_alu_src = 1'b1;  // Use immediate
          d_imm = d_imm_s;
        end

        default: begin
          // For all other instructions in milestone 1, defaults to no-op
        end
      endcase
    end
  end

  // Register file instance
  logic [`REG_SIZE] d_rs1_data, d_rs2_data;

  RegFile rf (
      .clk(clk),
      .rst(rst),
      .we(w_reg_write),
      .rd(w_rd),
      .rd_data(w_rd_data),
      .rs1(d_rs1),
      .rs1_data(d_rs1_data),
      .rs2(d_rs2),
      .rs2_data(d_rs2_data)
  );

  // Bypass signals
  logic e_reg_write, m_reg_write, w_reg_write;
  logic [4:0] e_rd, m_rd, w_rd;
  logic [`REG_SIZE] e_result, m_result, w_rd_data;

  // Forwarding logic for rs1
  logic [`REG_SIZE] d_rs1_value;
  always_comb begin
    // MX bypass (from Memory to Execute)
    if (e_reg_write && (e_rd != 0) && (e_rd == d_rs1)) d_rs1_value = e_result;
    // WX bypass (from Writeback to Execute)
    else if (m_reg_write && (m_rd != 0) && (m_rd == d_rs1)) d_rs1_value = m_result;
    // Normal register read (with WD bypass handled in RegFile)
    else
      d_rs1_value = d_rs1_data;
  end

  // Forwarding logic for rs2
  logic [`REG_SIZE] d_rs2_value;
  always_comb begin
    // MX bypass (from Memory to Execute)
    if (e_reg_write && (e_rd != 0) && (e_rd == d_rs2)) d_rs2_value = e_result;
    // WX bypass (from Writeback to Execute)
    else if (m_reg_write && (m_rd != 0) && (m_rd == d_rs2)) d_rs2_value = m_result;
    // Normal register read (with WD bypass handled in RegFile)
    else
      d_rs2_value = d_rs2_data;
  end

  /********************/
  /* EXECUTE STAGE    */
  /********************/

  // Execute stage state structure
  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [`REG_SIZE] rs1_value;
    logic [`REG_SIZE] rs2_value;
    logic [`REG_SIZE] imm;
    logic reg_write;
    logic mem_write;
    logic mem_read;
    logic [1:0] wb_sel;
    logic alu_src;
    alu_op_e alu_op;
    branch_type_e branch_type;
    logic [4:0] rd;
    logic is_div;
    cycle_status_e cycle_status;
  } stage_execute_t;

  stage_execute_t execute_state;

  // Execute stage pipeline register
  always_ff @(posedge clk) begin
    if (rst || take_branch) begin
      execute_state <= '{
          pc: 0,
          insn: 0,
          rs1_value: 0,
          rs2_value: 0,
          imm: 0,
          reg_write: 0,
          mem_write: 0,
          mem_read: 0,
          wb_sel: 0,
          alu_src: 0,
          alu_op: ALU_ADD,
          branch_type: BR_NONE,
          rd: 0,
          is_div: 0,
          cycle_status: rst ? CYCLE_RESET : CYCLE_TAKEN_BRANCH
      };
    end else begin
      execute_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          rs1_value: d_rs1_value,
          rs2_value: d_rs2_value,
          imm: d_imm,
          reg_write: d_reg_write,
          mem_write: d_mem_write,
          mem_read: d_mem_read,
          wb_sel: d_wb_sel,
          alu_src: d_alu_src,
          alu_op: d_alu_op,
          branch_type: d_branch_type,
          rd: d_rd,
          is_div: d_is_div,
          cycle_status: decode_state.cycle_status
      };
    end
  end

  // Execute stage disassembly for debug
  wire [255:0] e_disasm;
  Disasm #(
      .PREFIX("E")
  ) disasm_2execute (
      .insn  (execute_state.insn),
      .disasm(e_disasm)
  );

  // Extract control signals for easier reference
  assign e_reg_write = execute_state.reg_write;
  assign e_rd = execute_state.rd;

  // ALU implementation
  logic [`REG_SIZE] e_alu_in1, e_alu_in2, e_alu_out;
  logic [`REG_SIZE] e_alu_in1_final, e_alu_in2_final;

  // Carry lookahead adder for high-performance addition/subtraction
  logic [`REG_SIZE] cla_a, cla_b, cla_sum;
  logic cla_cin;

  // First compute the ALU inputs
  always_comb begin
    // Determine ALU inputs
    if (execute_state.alu_op == ALU_AUIPC) e_alu_in1 = execute_state.pc;
    else e_alu_in1 = execute_state.rs1_value;

    if (execute_state.alu_src) e_alu_in2 = execute_state.imm;
    else e_alu_in2 = execute_state.rs2_value;

    // Handle subtraction
    if (execute_state.alu_op == ALU_SUB) e_alu_in2_final = ~e_alu_in2 + 1;
    else e_alu_in2_final = e_alu_in2;

    e_alu_in1_final = e_alu_in1;
  end

  // Connect to CLA using assign statements
  assign cla_a   = e_alu_in1_final;
  assign cla_b   = e_alu_in2_final;
  assign cla_cin = 1'b0;

  cla adder (
      .a  (cla_a),
      .b  (cla_b),
      .cin(cla_cin),
      .sum(cla_sum)
  );

  // Compute ALU output in a separate block
  always_comb begin
    case (execute_state.alu_op)
      ALU_ADD, ALU_SUB: e_alu_out = cla_sum;
      ALU_SLL: e_alu_out = e_alu_in1 << e_alu_in2[4:0];
      ALU_SLT: e_alu_out = ($signed(e_alu_in1) < $signed(e_alu_in2)) ? 32'b1 : 32'b0;
      ALU_SLTU: e_alu_out = (e_alu_in1 < e_alu_in2) ? 32'b1 : 32'b0;
      ALU_XOR: e_alu_out = e_alu_in1 ^ e_alu_in2;
      ALU_SRL: e_alu_out = e_alu_in1 >> e_alu_in2[4:0];
      ALU_SRA: e_alu_out = $signed(e_alu_in1) >>> e_alu_in2[4:0];
      ALU_OR: e_alu_out = e_alu_in1 | e_alu_in2;
      ALU_AND: e_alu_out = e_alu_in1 & e_alu_in2;
      ALU_LUI: e_alu_out = e_alu_in2;  // Pass immediate directly for LUI
      ALU_AUIPC: e_alu_out = cla_sum;  // PC + imm
      default: e_alu_out = 32'b0;
    endcase
  end

  // Branch logic
  logic e_branch_taken;
  logic [`REG_SIZE] e_branch_target;

  always_comb begin
    // Determine if branch is taken based on branch type
    case (execute_state.branch_type)
      BR_EQ: e_branch_taken = (execute_state.rs1_value == execute_state.rs2_value);
      BR_NE: e_branch_taken = (execute_state.rs1_value != execute_state.rs2_value);
      BR_LT: e_branch_taken = ($signed(execute_state.rs1_value) < $signed(execute_state.rs2_value));
      BR_GE:
      e_branch_taken = ($signed(execute_state.rs1_value) >= $signed(execute_state.rs2_value));
      BR_LTU: e_branch_taken = (execute_state.rs1_value < execute_state.rs2_value);
      BR_GEU: e_branch_taken = (execute_state.rs1_value >= execute_state.rs2_value);
      default: e_branch_taken = 1'b0;
    endcase

    // Calculate branch target
    if (execute_state.insn[6:0] == OpcodeJalr) begin
      // JALR: rs1 + imm (with bit[0] set to 0)
      e_branch_target = (execute_state.rs1_value + execute_state.imm) & ~32'b1;
    end else if (execute_state.insn[6:0] == OpcodeJal || execute_state.insn[6:0] == OpcodeBranch) begin
      // JAL and branches: PC + imm
      e_branch_target = execute_state.pc + execute_state.imm;
    end else begin
      // Default (shouldn't be used)
      e_branch_target = execute_state.pc + 4;
    end

    // Determine if we should take the branch and which target to use
    if ((execute_state.insn[6:0] == OpcodeJal) || 
        (execute_state.insn[6:0] == OpcodeJalr) || 
        (execute_state.branch_type != BR_NONE && e_branch_taken)) begin
      take_branch = 1'b1;
    end else begin
      take_branch = 1'b0;
    end

    // Set branch target for fetch stage
    branch_target = e_branch_target;
  end

  // Execute result mux
  always_comb begin
    if (execute_state.wb_sel == 2'b10)  // PC+4 for JAL/JALR
      e_result = execute_state.pc + 4;
    else e_result = e_alu_out;  // ALU result
  end

  /********************/
  /* MEMORY STAGE     */
  /********************/

  // Memory stage state structure
  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [`REG_SIZE] alu_result;
    logic [`REG_SIZE] rs2_value;
    logic reg_write;
    logic mem_write;
    logic mem_read;
    logic [1:0] wb_sel;
    logic [4:0] rd;
    logic is_div;
    cycle_status_e cycle_status;
  } stage_memory_t;

  stage_memory_t memory_state;

  // Memory stage pipeline register
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
          pc: 0,
          insn: 0,
          alu_result: 0,
          rs2_value: 0,
          reg_write: 0,
          mem_write: 0,
          mem_read: 0,
          wb_sel: 0,
          rd: 0,
          is_div: 0,
          cycle_status: CYCLE_RESET
      };
    end else begin
      memory_state <= '{
          pc: execute_state.pc,
          insn: execute_state.insn,
          alu_result: e_result,
          rs2_value: execute_state.rs2_value,
          reg_write: execute_state.reg_write,
          mem_write: execute_state.mem_write,
          mem_read: execute_state.mem_read,
          wb_sel: execute_state.wb_sel,
          rd: execute_state.rd,
          is_div: execute_state.is_div,
          cycle_status: execute_state.cycle_status
      };
    end
  end

  // Memory stage disassembly for debug
  wire [255:0] m_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_3memory (
      .insn  (memory_state.insn),
      .disasm(m_disasm)
  );

  // Extract control signals for easier reference
  assign m_reg_write = memory_state.reg_write;
  assign m_rd = memory_state.rd;
  assign m_result = memory_state.alu_result;

  // Memory access signals (always ensure 4-byte alignment)
  assign addr_to_dmem = {memory_state.alu_result[31:2], 2'b00};  // Force alignment
  assign store_data_to_dmem = memory_state.rs2_value;
  assign store_we_to_dmem = memory_state.mem_write ? 4'b1111 : 4'b0000; // Full word writes for milestone 1

  // For milestone 1, we're not using memory reads, but this will be needed for milestone 2
  logic [`REG_SIZE] m_mem_data;
  assign m_mem_data = load_data_from_dmem;

  /**********************/
  /* WRITEBACK STAGE    */
  /**********************/

  // Writeback stage state structure
  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [`REG_SIZE] alu_result;
    logic [`REG_SIZE] mem_data;
    logic reg_write;
    logic [1:0] wb_sel;
    logic [4:0] rd;
    logic is_div;
    cycle_status_e cycle_status;
  } stage_writeback_t;

  stage_writeback_t writeback_state;

  // Writeback stage pipeline register
  always_ff @(posedge clk) begin
    if (rst) begin
      writeback_state <= '{
          pc: 0,
          insn: 0,
          alu_result: 0,
          mem_data: 0,
          reg_write: 0,
          wb_sel: 0,
          rd: 0,
          is_div: 0,
          cycle_status: CYCLE_RESET
      };
    end else begin
      writeback_state <= '{
          pc: memory_state.pc,
          insn: memory_state.insn,
          alu_result: memory_state.alu_result,
          mem_data: m_mem_data,
          reg_write: memory_state.reg_write,
          wb_sel: memory_state.wb_sel,
          rd: memory_state.rd,
          is_div: memory_state.is_div,
          cycle_status: memory_state.cycle_status
      };
    end
  end

  // Writeback stage disassembly for debug
  wire [255:0] w_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_4writeback (
      .insn  (writeback_state.insn),
      .disasm(w_disasm)
  );

  // Extract control signals for easier reference
  assign w_reg_write = writeback_state.reg_write;
  assign w_rd = writeback_state.rd;

  // Writeback data mux
  always_comb begin
    case (writeback_state.wb_sel)
      2'b00:   w_rd_data = writeback_state.alu_result;  // ALU result
      2'b01:   w_rd_data = writeback_state.mem_data;  // Memory load result (for milestone 2)
      2'b10:   w_rd_data = writeback_state.pc + 4;  // PC+4 for JAL/JALR
      default: w_rd_data = writeback_state.alu_result;
    endcase
  end

  // Set halt signal for ecall instruction (environ opcode)
  always_comb begin
    halt = (writeback_state.insn[6:0] == OpcodeEnviron && writeback_state.insn[31:7] == 25'b0);
  end

  // Set trace output signals
  assign trace_writeback_pc = writeback_state.pc;
  assign trace_writeback_insn = writeback_state.insn;
  assign trace_writeback_cycle_status = writeback_state.cycle_status;



endmodule

module MemorySingleCycle #(
    parameter int NUM_WORDS = 512
) (
    // rst for both imem and dmem
    input wire rst,

    // clock for both imem and dmem. The memory reads/writes on @(negedge clk)
    input wire clk,

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

  always @(negedge clk) begin
    if (rst) begin
    end else begin
      insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
    end
  end

  always @(negedge clk) begin
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

/* This design has just one clock for both processor and memory. */
module Processor (
    input wire clk,
    input wire rst,
    output logic halt,
    output wire [`REG_SIZE] trace_writeback_pc,
    output wire [`INSN_SIZE] trace_writeback_insn,
    output cycle_status_e trace_writeback_cycle_status
);

  wire [`INSN_SIZE] insn_from_imem;
  wire [`REG_SIZE] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(8192)
  ) memory (
      .rst                (rst),
      .clk                (clk),
      // imem is read-only
      .pc_to_imem         (pc_to_imem),
      .insn_from_imem     (insn_from_imem),
      // dmem is read-write
      .addr_to_dmem       (mem_data_addr),
      .load_data_from_dmem(mem_data_loaded_value),
      .store_data_to_dmem (mem_data_to_write),
      .store_we_to_dmem   (mem_data_we)
  );

  DatapathPipelined datapath (
      .clk(clk),
      .rst(rst),
      .pc_to_imem(pc_to_imem),
      .insn_from_imem(insn_from_imem),
      .addr_to_dmem(mem_data_addr),
      .store_data_to_dmem(mem_data_to_write),
      .store_we_to_dmem(mem_data_we),
      .load_data_from_dmem(mem_data_loaded_value),
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule
