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

  // TODO: your code here

  // Write logic - synchronous to clock
  always_ff @(posedge clk) begin
    if (rst) begin
      // Initialize all registers to 0 on reset
      for (int j = 0; j < NumRegs; j = j + 1) begin
        regs[j] <= 32'd0;
      end
    end else if (we && rd != 5'd0) begin
      // Only write if write enable is high and destination is not x0 (zero register)
      regs[rd] <= rd_data;
    end
  end

  // Read logic with WD bypass implementation
  always_comb begin
    // For rs1
    if (rs1 == 5'd0) begin
      // x0 register is hardwired to 0
      rs1_data = 32'd0;
    end else if (we && (rs1 == rd) && (rd != 5'd0)) begin
      // WD bypass: Forward the data being written to rd if it's the same register
      rs1_data = rd_data;
    end else begin
      // Normal register read
      rs1_data = regs[rs1];
    end

    // For rs2, same logic
    if (rs2 == 5'd0) begin
      rs2_data = 32'd0;
    end else if (we && (rs2 == rd) && (rd != 5'd0)) begin
      rs2_data = rd_data;  // WD bypass
    end else begin
      rs2_data = regs[rs2];
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

  // Pipeline control signals
  logic e_branch_taken;
  logic [`REG_SIZE] e_branch_target;

  // Structures for pipeline stages
  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [`REG_SIZE] rs1_data;
    logic [`REG_SIZE] rs2_data;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic write_rd;
    cycle_status_e cycle_status;
  } stage_execute_t;

  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [`REG_SIZE] alu_result;
      logic [`REG_SIZE] rs2_data;  // Add this field for store instructions
    logic [4:0] rd_addr;
    logic write_rd;
    cycle_status_e cycle_status;
  } stage_memory_t;

  typedef struct packed {
    logic [`REG_SIZE] pc;
    logic [`INSN_SIZE] insn;
    logic [`REG_SIZE] result;
    logic [4:0] rd_addr;
    logic write_rd;
    cycle_status_e cycle_status;
  } stage_writeback_t;

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

  // Fetch stage registers and wires
  logic [`REG_SIZE] f_pc_current;
  logic [`REG_SIZE] f_pc_next;
  wire [`INSN_SIZE] f_insn;
  cycle_status_e f_cycle_status;

  // Calculate next PC value
  always_comb begin
    if (load_use_hazard)
      // Stall: keep PC the same during load-use hazard
      f_pc_next = f_pc_current;
    else if (e_branch_taken)
      // When branch taken, use branch target from Execute stage
      f_pc_next = e_branch_target;
    else
      // Normal sequential execution
      f_pc_next = f_pc_current + 4;
  end

  // Program counter update logic
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current   <= 32'd0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      // Update PC
      f_pc_current <= f_pc_next;

      // Update cycle status
      if (e_branch_taken) f_cycle_status <= CYCLE_TAKEN_BRANCH;
      else f_cycle_status <= CYCLE_NO_STALL;
    end
  end

  // Connect to instruction memory
  assign pc_to_imem = f_pc_current;
  assign f_insn = insn_from_imem;

  // Disassembly for debugging
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  // // Here's how to disassemble an insn into a string you can view in GtkWave.
  // // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  // wire [255:0] f_disasm;
  // Disasm #(
  //     .PREFIX("F")
  // ) disasm_0fetch (
  //     .insn  (f_insn),
  //     .disasm(f_disasm)
  // );

  /****************/
  /* DECODE STAGE */
  /****************/

  // // this shows how to package up state in a `struct packed`, and how to pass it between stages
  // stage_decode_t decode_state;
  // always_ff @(posedge clk) begin
  //   if (rst) begin
  //     decode_state <= '{pc: 0, insn: 0, cycle_status: CYCLE_RESET};
  //   end else begin
  //     begin
  //       decode_state <= '{pc: f_pc_current, insn: f_insn, cycle_status: f_cycle_status};
  //     end
  //   end
  // end
  // wire [255:0] d_disasm;
  // Disasm #(
  //     .PREFIX("D")
  // ) disasm_1decode (
  //     .insn  (decode_state.insn),
  //     .disasm(d_disasm)
  // );

  // TODO: your code here, though you will also need to modify some of the code above
  // TODO: the testbench requires that your register file instance is named `rf`

  // Register containing state passed from Fetch to Decode
  stage_decode_t decode_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{pc: 0, insn: 0, cycle_status: CYCLE_RESET};
    end else if (e_branch_taken) begin
      // Insert bubble if branch taken (pipeline flush)
      decode_state <= '{pc: 0, insn: 0, cycle_status: CYCLE_TAKEN_BRANCH};
    end else if (load_use_hazard) begin
      // Keep current decode state during stall (do not update)
      decode_state <= decode_state;
    end else begin
      decode_state <= '{pc: f_pc_current, insn: f_insn, cycle_status: f_cycle_status};
    end
  end

  // Disassembly for debugging
  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );

  // Extract instruction fields
  logic [4:0] d_rs1;
  logic [4:0] d_rs2;
  logic [4:0] d_rd;
  logic [2:0] d_funct3;
  logic [6:0] d_funct7;
  logic [`OPCODE_SIZE] d_opcode;
  logic [`REG_SIZE] d_imm_i;
  logic [`REG_SIZE] d_imm_s;
  logic [`REG_SIZE] d_imm_b;
  logic [`REG_SIZE] d_imm_u;
  logic [`REG_SIZE] d_imm_j;

  always_comb begin
    // Parse instruction fields
    d_opcode = decode_state.insn[6:0];
    d_rd = decode_state.insn[11:7];
    d_funct3 = decode_state.insn[14:12];
    d_rs1 = decode_state.insn[19:15];
    d_rs2 = decode_state.insn[24:20];
    d_funct7 = decode_state.insn[31:25];

    // Generate immediates for different instruction formats
    d_imm_i = {{20{decode_state.insn[31]}}, decode_state.insn[31:20]};
    d_imm_s = {{20{decode_state.insn[31]}}, decode_state.insn[31:25], decode_state.insn[11:7]};
    d_imm_b = {
      {19{decode_state.insn[31]}},
      decode_state.insn[31],
      decode_state.insn[7],
      decode_state.insn[30:25],
      decode_state.insn[11:8],
      1'b0
    };
    d_imm_u = {decode_state.insn[31:12], 12'b0};
    d_imm_j = {
      {11{decode_state.insn[31]}},
      decode_state.insn[31],
      decode_state.insn[19:12],
      decode_state.insn[20],
      decode_state.insn[30:21],
      1'b0
    };
  end

  // Determine if we write to register file
  logic d_write_rd;
  always_comb begin
    case (d_opcode)
      OpcodeLoad, OpcodeRegImm, OpcodeRegReg, OpcodeLui, OpcodeJal, OpcodeJalr: d_write_rd = 1'b1;
      default: d_write_rd = 1'b0;
    endcase
  end

  // Register file connections and instance
  logic [`REG_SIZE] d_rs1_data, d_rs2_data;
  logic w_write_rd;
  logic [4:0] w_rd_addr;
  logic [`REG_SIZE] w_rd_data;

  // Register file instance as required by testbench
  RegFile rf (
      .clk(clk),
      .rst(rst),
      .we(w_write_rd),
      .rd(w_rd_addr),
      .rd_data(w_rd_data),
      .rs1(d_rs1),
      .rs1_data(d_rs1_data),
      .rs2(d_rs2),
      .rs2_data(d_rs2_data)
  );

  // Pass decoded state to execute stage
  stage_execute_t execute_state;
  always_ff @(posedge clk) begin
    if (rst || e_branch_taken) begin
      // Reset or branch
      execute_state <= '{
          pc: 0,
          insn: 0,
          rs1_data: 0,
          rs2_data: 0,
          rs1_addr: 0,
          rs2_addr: 0,
          rd_addr: 0,
          write_rd: 0,
          cycle_status: e_branch_taken ? CYCLE_TAKEN_BRANCH : CYCLE_RESET
      };
    end else if (load_use_hazard) begin
      // Insert bubble when load-use hazard detected
      execute_state <= '{
          pc: 0,
          insn: 0,
          rs1_data: 0,
          rs2_data: 0,
          rs1_addr: 0,
          rs2_addr: 0,
          rd_addr: 0,
          write_rd: 0,
          cycle_status: CYCLE_LOAD2USE
      };
    end else begin
      // Normal operation
      execute_state <= '{
          pc: decode_state.pc,
          insn: decode_state.insn,
          rs1_data: d_rs1_data,
          rs2_data: d_rs2_data,
          rs1_addr: d_rs1,
          rs2_addr: d_rs2,
          rd_addr: d_rd,
          write_rd: d_write_rd,
          cycle_status: decode_state.cycle_status
      };
    end
  end


  /*****************/
  /* EXECUTE STAGE */
  /*****************/

  // Execute stage signals
  logic [`REG_SIZE] e_rs1_data, e_rs2_data, e_alu_result;
  logic [4:0] e_rs1_addr, e_rs2_addr, e_rd_addr;
  logic e_write_rd;
  logic [`INSN_SIZE] e_insn;
  logic [`REG_SIZE] e_pc;
  logic [2:0] e_funct3;
  logic [6:0] e_funct7;
  logic [`OPCODE_SIZE] e_opcode;
  cycle_status_e e_cycle_status;
  logic [`REG_SIZE] e_imm_i, e_imm_b, e_imm_u, e_imm_s;

  // CLA adder signals
  logic [`REG_SIZE] e_cla_sum;
  logic [`REG_SIZE] e_cla_b_input;

  // CLA adder input logic
  always_comb begin
    case (e_opcode)
      OpcodeRegImm: begin
        // For I-type instructions
        e_cla_b_input = e_imm_i;
      end
      OpcodeRegReg: begin
        if (e_funct3 == 3'b000 && e_funct7[5])
          // For SUB instruction
          e_cla_b_input = ~e_rs2_data + 1;
        else
          // For ADD instruction
          e_cla_b_input = e_rs2_data;
      end
      OpcodeLoad: begin
        // For load address calculation
        e_cla_b_input = e_imm_i;
      end
      OpcodeStore: begin
        // For store address calculation
        e_cla_b_input = e_imm_s;
      end
      default: e_cla_b_input = 32'd0;
    endcase
  end

  // Instantiate the CLA adder
  cla cla_adder (
      .a  (e_rs1_data),
      .b  (e_cla_b_input),
      .cin(1'b0),
      .sum(e_cla_sum)
  );

  // Connect execute stage signals
  always_comb begin
    e_rs1_addr = execute_state.rs1_addr;
    e_rs2_addr = execute_state.rs2_addr;
    e_rd_addr = execute_state.rd_addr;
    e_write_rd = execute_state.write_rd;
    e_insn = execute_state.insn;
    e_pc = execute_state.pc;
    e_cycle_status = execute_state.cycle_status;

    // Parse instruction components
    e_opcode = e_insn[6:0];
    e_funct3 = e_insn[14:12];
    e_funct7 = e_insn[31:25];

    // Generate immediates
    e_imm_i = {{20{e_insn[31]}}, e_insn[31:20]};
    e_imm_b = {{19{e_insn[31]}}, e_insn[31], e_insn[7], e_insn[30:25], e_insn[11:8], 1'b0};
    e_imm_u = {e_insn[31:12], 12'b0};
    e_imm_s = {{20{e_insn[31]}}, e_insn[31:25], e_insn[11:7]};  // Add this line
  end

  // Bypass control logic
  logic use_m_x1_bypass, use_m_x2_bypass;
  logic use_w_x1_bypass, use_w_x2_bypass;
  logic [`REG_SIZE] m_bypass_data, w_bypass_data;

  // Determine if bypassing is needed
  always_comb begin
    // MX bypass: Memory to Execute
    use_m_x1_bypass = (e_rs1_addr != 0) && (e_rs1_addr == m_rd_addr) && m_write_rd;
    use_m_x2_bypass = (e_rs2_addr != 0) && (e_rs2_addr == m_rd_addr) && m_write_rd;

    // WX bypass: Writeback to Execute
    use_w_x1_bypass = (e_rs1_addr != 0) && (e_rs1_addr == w_rd_addr) && w_write_rd;
    use_w_x2_bypass = (e_rs2_addr != 0) && (e_rs2_addr == w_rd_addr) && w_write_rd;
  end

  // Apply bypass logic to get actual operand values
  always_comb begin
    // First operand (rs1)
    if (use_m_x1_bypass) e_rs1_data = m_bypass_data;
    else if (use_w_x1_bypass) e_rs1_data = w_bypass_data;
    else e_rs1_data = execute_state.rs1_data;

    // Second operand (rs2)
    if (use_m_x2_bypass) e_rs2_data = m_bypass_data;
    else if (use_w_x2_bypass) e_rs2_data = w_bypass_data;
    else e_rs2_data = execute_state.rs2_data;
  end

  // ALU second operand selection
  logic [`REG_SIZE] e_alu_op2;
  always_comb begin
    case (e_opcode)
      OpcodeRegImm: e_alu_op2 = e_imm_i;  // I-type
      OpcodeLui:    e_alu_op2 = e_imm_u;  // U-type
      default:      e_alu_op2 = e_rs2_data;  // R-type
    endcase
  end

  // ALU operation
  always_comb begin
    case (e_opcode)
      OpcodeLui: begin
        // LUI just passes immediate to result
        e_alu_result = e_imm_u;
      end

      OpcodeRegImm: begin
        // I-type ALU operations
        case (e_funct3)
          3'b000:  e_alu_result = e_cla_sum;  // ADDI
          3'b010:  e_alu_result = {31'b0, $signed(e_rs1_data) < $signed(e_imm_i)};  // SLTI
          3'b011:  e_alu_result = {31'b0, e_rs1_data < e_imm_i};  // SLTIU
          3'b100:  e_alu_result = e_rs1_data ^ e_imm_i;  // XORI
          3'b110:  e_alu_result = e_rs1_data | e_imm_i;  // ORI
          3'b111:  e_alu_result = e_rs1_data & e_imm_i;  // ANDI
          3'b001:  e_alu_result = e_rs1_data << e_imm_i[4:0];  // SLLI
          3'b101: begin
            if (e_funct7[5]) e_alu_result = $signed(e_rs1_data) >>> e_imm_i[4:0];  // SRAI
            else e_alu_result = e_rs1_data >> e_imm_i[4:0];  // SRLI
          end
          default: e_alu_result = 0;
        endcase
      end

      OpcodeRegReg: begin
        // R-type ALU operations
        case (e_funct3)
          3'b000: begin
            e_alu_result = e_cla_sum;  // ADD/SUB
          end
          3'b001:  e_alu_result = e_rs1_data << e_rs2_data[4:0];  // SLL
          3'b010:  e_alu_result = {31'b0, $signed(e_rs1_data) < $signed(e_rs2_data)};  // SLT
          3'b011:  e_alu_result = {31'b0, e_rs1_data < e_rs2_data};  // SLTU
          3'b100:  e_alu_result = e_rs1_data ^ e_rs2_data;  // XOR
          3'b101: begin
            if (e_funct7[5]) e_alu_result = $signed(e_rs1_data) >>> e_rs2_data[4:0];  // SRA
            else e_alu_result = e_rs1_data >> e_rs2_data[4:0];  // SRL
          end
          3'b110:  e_alu_result = e_rs1_data | e_rs2_data;  // OR
          3'b111:  e_alu_result = e_rs1_data & e_rs2_data;  // AND
          default: e_alu_result = 0;
        endcase
      end

      // PLACEHOLDER for Load instructions (Milestone 2)
      OpcodeLoad: begin
        e_alu_result = e_cla_sum;  // Calculate effective address
      end

      // PLACEHOLDER for Store instructions (Milestone 2)
      OpcodeStore: begin
        e_alu_result = e_cla_sum;  // Calculate effective address
      end

      // PLACEHOLDER for AUIPC (Milestone 2)
      OpcodeAuipc: begin
        e_alu_result = e_pc + e_imm_u;
      end

      // PLACEHOLDER for JAL (Milestone 2)
      OpcodeJal: begin
        e_alu_result = e_pc + 4;  // Return address
      end

      // PLACEHOLDER for JALR (Milestone 2)
      OpcodeJalr: begin
        e_alu_result = e_pc + 4;
      end

      // PLACEHOLDER for Miscellaneous Memory ops (Milestone 2)
      OpcodeMiscMem: begin
        e_alu_result = 0;
      end

      // PLACEHOLDER for Environment instructions (Milestone 2)
      OpcodeEnviron: begin
        e_alu_result = 0;
      end

      default: e_alu_result = 0;
    endcase
  end

  // Branch handling
  always_comb begin
    e_branch_taken  = 0;
    e_branch_target = e_pc + 4;  // Default next PC

    if (e_opcode == OpcodeBranch) begin
      // Branch instruction
      e_branch_target = e_pc + e_imm_b;  // Branch target

      case (e_funct3)
        3'b000:  e_branch_taken = (e_rs1_data == e_rs2_data);  // BEQ
        3'b001:  e_branch_taken = (e_rs1_data != e_rs2_data);  // BNE
        3'b100:  e_branch_taken = ($signed(e_rs1_data) < $signed(e_rs2_data));  // BLT
        3'b101:  e_branch_taken = ($signed(e_rs1_data) >= $signed(e_rs2_data));  // BGE
        3'b110:  e_branch_taken = (e_rs1_data < e_rs2_data);  // BLTU
        3'b111:  e_branch_taken = (e_rs1_data >= e_rs2_data);  // BGEU
        default: e_branch_taken = 0;
      endcase
    end
  end

  // Detect load-use hazard (load in Execute, dependent instruction in Decode)
  logic load_use_hazard;
  always_comb begin
    // Detect if Execute stage contains a load instruction
    logic execute_has_load = (e_opcode == OpcodeLoad);
    
    // Check if Decode reads a register that Execute's load will write
    logic decode_uses_execute_rd = (execute_has_load && e_write_rd && e_rd_addr != 0) && 
                                  ((d_rs1 == e_rd_addr) || (d_rs2 == e_rd_addr));
    
    // Load-use hazard detected
    load_use_hazard = execute_has_load && decode_uses_execute_rd;
  end

  // Disassembly for debugging
  wire [255:0] e_disasm;
  Disasm #(
      .PREFIX("E")
  ) disasm_2execute (
      .insn  (e_insn),
      .disasm(e_disasm)
  );

  // Pass execute results to memory stage
  stage_memory_t memory_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
          pc: 0,
          insn: 0,
          alu_result: 0,
          rs2_data: 0,  // Initialize rs2_data field
          rd_addr: 0,
          write_rd: 0,
          cycle_status: CYCLE_RESET
      };
    end else begin
      memory_state <= '{
          pc: e_pc,
          insn: e_insn,
          alu_result: e_alu_result,
          rs2_data: e_rs2_data,  // Pass rs2_data from Execute to Memory
          rd_addr: e_rd_addr,
          write_rd: e_write_rd,
          cycle_status: e_branch_taken ? CYCLE_TAKEN_BRANCH : e_cycle_status
      };
    end
  end

  /****************/
  /* MEMORY STAGE */
  /****************/

  // Memory stage signals
  logic [`REG_SIZE] m_pc, m_alu_result, m_rs2_data;
  logic [`INSN_SIZE] m_insn;
  logic [4:0] m_rd_addr;
  logic m_write_rd;
  cycle_status_e m_cycle_status;
  logic use_w_m_bypass;
  logic [`OPCODE_SIZE] m_opcode;


  // Connect memory stage signals
  always_comb begin
    m_pc = memory_state.pc;
    m_insn = memory_state.insn;
    m_alu_result = memory_state.alu_result;
    m_rs2_data = memory_state.rs2_data;  // Connect rs2_data

    m_rd_addr = memory_state.rd_addr;
    m_write_rd = memory_state.write_rd;
    m_cycle_status = memory_state.cycle_status;
    m_opcode = m_insn[6:0];

    // Data for MX bypass
    m_bypass_data = m_alu_result;

  // WM Bypass: Detect if we need to forward data from Writeback to Memory
  // This matters for store instructions where rs2 contains the data to store
  use_w_m_bypass = (m_opcode == OpcodeStore) && 
                   (memory_state.insn[24:20] != 0) && // rs2 != x0
                   (memory_state.insn[24:20] == w_rd_addr) && 
                   w_write_rd;
  end

  // Passing ALU result through to Writeback stage

  // Disassembly for debugging
  wire [255:0] m_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_3memory (
      .insn  (m_insn),
      .disasm(m_disasm)
  );

  // Pass memory stage results to writeback stage
  stage_writeback_t writeback_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      writeback_state <= '{
          pc: 0,
          insn: 0,
          result: 0,
          rd_addr: 0,
          write_rd: 0,
          cycle_status: CYCLE_RESET
      };
    end else begin
      // Determine result based on instruction type
    logic [`REG_SIZE] result_value;
    if (m_opcode == OpcodeLoad) 
      result_value = load_data_from_dmem;  // Load result from memory
    else
      result_value = m_alu_result;  // ALU result for other instructions
      
    writeback_state <= '{
        pc: m_pc,
        insn: m_insn,
        result: result_value,
        rd_addr: m_rd_addr,
        write_rd: m_write_rd,
        cycle_status: (m_cycle_status == CYCLE_TAKEN_BRANCH && m_insn != 0) ? 
                      CYCLE_NO_STALL : m_cycle_status
    };
    end
  end

  /*******************/
  /* WRITEBACK STAGE */
  /*******************/

  // Writeback stage signals
  logic [`REG_SIZE] w_pc, w_result;
  logic [`INSN_SIZE] w_insn;
  cycle_status_e w_cycle_status;

  // Connect writeback stage signals
  always_comb begin
    w_pc = writeback_state.pc;
    w_insn = writeback_state.insn;
    w_result = writeback_state.result;
    w_rd_addr = writeback_state.rd_addr;
    w_write_rd = writeback_state.write_rd;
    w_cycle_status = writeback_state.cycle_status;

    // Data for WX bypass
    w_bypass_data = w_result;
  end

  // Connect to register file for writeback
  assign w_rd_data = w_result;

  // Disassembly for debugging
  wire [255:0] w_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_4writeback (
      .insn  (w_insn),
      .disasm(w_disasm)
  );

  // Connect trace outputs
  assign trace_writeback_pc = w_pc;
  assign trace_writeback_insn = w_insn;
  assign trace_writeback_cycle_status = w_cycle_status;

  // Detect halt condition when ECALL is encountered
  assign halt = (w_insn[6:0] == OpcodeEnviron) && (w_pc != 0);

  // Memory stage connections to data memory /////////
  always_comb begin
    // Default values
    addr_to_dmem = 0;
    store_data_to_dmem = 0;
    store_we_to_dmem = 4'b0000;
    
    if (m_opcode == OpcodeLoad) begin
      // Load instruction: send address to data memory
      addr_to_dmem = m_alu_result;
    end else if (m_opcode == OpcodeStore) begin
      // Store instruction
      addr_to_dmem = m_alu_result;
      
      // Store data with WM bypassing
      if (use_w_m_bypass) 
        store_data_to_dmem = w_result;  // Use data from Writeback
      else
        store_data_to_dmem = m_rs2_data;  // Use data from Memory stage
      
      // Generate appropriate byte enable signals based on funct3
      case (m_insn[14:12])
        3'b000: begin // SB
          case (m_alu_result[1:0])
            2'b00: store_we_to_dmem = 4'b0001;
            2'b01: store_we_to_dmem = 4'b0010;
            2'b10: store_we_to_dmem = 4'b0100;
            2'b11: store_we_to_dmem = 4'b1000;
          endcase
        end
        
        3'b001: begin // SH
          case (m_alu_result[1])
            1'b0: store_we_to_dmem = 4'b0011;
            1'b1: store_we_to_dmem = 4'b1100;
          endcase
        end
        
        3'b010: begin // SW
          store_we_to_dmem = 4'b1111;
        end
        
        default: store_we_to_dmem = 4'b0000;
      endcase
    end
  end
  //////////////////////////////////////////////

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
