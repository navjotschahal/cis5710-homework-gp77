`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
  `include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "../hw5-pipelined/cycle_status.sv"
`include "AxilCache.sv"

module Disasm #(
    PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef RISCV_FORMAL
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
`endif
endmodule

// TODO: copy over your RegFile and pipeline structs from HW5

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

module DatapathPipelinedCache (
    input wire clk,
    input wire rst,

    // AXIL interface to insn memory
    axi_if.manager icache,
    // AXIL interface to data memory/cache
    axi_if.manager dcache,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  localparam bit True = 1'b1;
  localparam bit False = 1'b0;

  // cycle counter
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  // TODO: copy in your HW5B datapath as a starting point


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

  // for FENCE handelling
  logic fence_stall_condition;  // Master stall signal for FENCE
  logic d_is_fence;  // Is FENCE instruction in Decode?
  logic e_is_fence;  // Is FENCE instruction in Execute?
  logic m_is_fence;  // Is FENCE instruction in Memory?

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
    if (load_use_hazard || div_data_hazard || fence_stall_condition)
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

  always_comb begin
  // Default values
  icache.ARVALID = False;
  icache.ARADDR = 32'b0;
  icache.RREADY = False;
  
  // Only perform instruction fetch when not in reset or stall
  if (!rst && !(load_use_hazard || div_data_hazard || fence_stall_condition)) begin
    // Request address
    icache.ARVALID = True;
    icache.ARADDR = f_pc_current;
    icache.RREADY = True;
  end
end

// Define instruction fetch result in decode stage
logic [`INSN_SIZE] insn_from_imem;
assign insn_from_imem = icache.RDATA;

// Remove this disassembly for fetch stage since instruction bits aren't available yet
// wire [255:0] f_disasm;
// Disasm #(
//     .PREFIX("F")
// ) disasm_0fetch (
//     .insn  (f_insn),
//     .disasm(f_disasm)
// );

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


  // TODO: your code here, though you will also need to modify some of the code above
  // TODO: the testbench requires that your register file instance is named `rf`

  // Register containing state passed from Fetch to Decode
stage_decode_t decode_state_stall_reg;
always_ff @(posedge clk) begin
  if (rst) begin
    decode_state_stall_reg <= '{pc: 0, insn: 0, cycle_status: CYCLE_RESET};
  end else if (load_use_hazard || div_data_hazard || fence_stall_condition) begin
    decode_state_stall_reg <= decode_state;
  end
end

stage_decode_t decode_state;
always_comb begin
    decode_state = '{pc: f_pc_current, insn: insn_from_imem, cycle_status: f_cycle_status};
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
  logic d_is_div;
  logic d_is_mul;


  always_comb begin
    // Parse instruction fields
    d_opcode = decode_state.insn[6:0];
    d_rd = decode_state.insn[11:7];
    d_funct3 = decode_state.insn[14:12];
    d_rs1 = decode_state.insn[19:15];
    d_rs2 = decode_state.insn[24:20];
    d_funct7 = decode_state.insn[31:25];

    d_is_div = (d_opcode == OpcodeRegReg && d_funct7 == 7'b0000001 && 
             (d_funct3 == 3'b100 || d_funct3 == 3'b101 || 
              d_funct3 == 3'b110 || d_funct3 == 3'b111));

    d_is_fence = (d_opcode == OpcodeMiscMem && d_funct3 == 3'b000 && decode_state.insn != 0);

    // Detect multiply instructions (MUL, MULH, MULHSU, MULHU)
    d_is_mul = (d_opcode == OpcodeRegReg && d_funct7 == 7'b0000001 && 
           (d_funct3 == 3'b000 || d_funct3 == 3'b001 || 
            d_funct3 == 3'b010 || d_funct3 == 3'b011));


    // Generate immediates for different instruction formats
    d_imm_i = {{20{decode_state.insn[31]}}, decode_state.insn[31:20]};
    d_imm_s = {{20{decode_state.insn[31]}}, decode_state.insn[31:25], decode_state.insn[11:7]};
    d_imm_b = {
      {19{decode_state.insn[31]}},
      decode_state.insn[31],
      decode_state.insn[7],
      decode_state.insn[30:25],
      decode_state.insn[11:8],
      False
    };
    d_imm_u = {decode_state.insn[31:12], 12'b0};
    d_imm_j = {
      {11{decode_state.insn[31]}},
      decode_state.insn[31],
      decode_state.insn[19:12],
      decode_state.insn[20],
      decode_state.insn[30:21],
      False
    };
  end

  // Determine if we write to register file
  logic d_write_rd;
  always_comb begin
    case (d_opcode)
      OpcodeLoad, OpcodeRegImm, OpcodeRegReg, OpcodeLui, OpcodeJal, OpcodeJalr, OpcodeAuipc:
      d_write_rd = True;
      default: d_write_rd = False;
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

    end else if (div_data_hazard) begin
      // Insert bubble for division hazards or when starting a division
      execute_state <= '{
          pc: 0,
          insn: 0,
          rs1_data: 0,
          rs2_data: 0,
          rs1_addr: 0,
          rs2_addr: 0,
          rd_addr: 0,
          write_rd: 0,
          cycle_status: CYCLE_DIV
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
  logic [`REG_SIZE] e_imm_i, e_imm_b, e_imm_u, e_imm_s, e_imm_j;

  // CLA adder signals
  logic [`REG_SIZE] e_cla_sum;
  logic [`REG_SIZE] e_cla_b_input;

  // Add execute stage detection for division and multiply instructions

  // CLA adder input logic
  always_comb begin
    // detect if FENCE
    e_is_fence = (e_opcode == OpcodeMiscMem && e_funct3 == 3'b000 && e_insn != 0);

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
      .cin(False),
      .sum(e_cla_sum)
  );

  logic div_data_hazard;
  // Divider inputs and outputs
  logic [`REG_SIZE] div_i_dividend;  // Dividend for the divider
  logic [`REG_SIZE] div_i_divisor;  // Divisor for the divider
  logic [`REG_SIZE] div_o_quotient;  // Quotient output from the divider
  logic [`REG_SIZE] div_o_remainder;  // Remainder output from the divider

  // Structure to track in-flight division operations
  typedef struct packed {
    logic              valid;     // Is this entry valid?
    logic [4:0]        rd_addr;   // Destination register
    logic              write_rd;  // Write to register file?
    logic [`REG_SIZE]  pc;        // PC of the instruction
    logic [`INSN_SIZE] insn;      // Instruction bits
    logic [3:0]        div_op;    // Operation type (DIV, DIVU, REM, REMU)
    logic              rs1_sign;  // Sign of rs1 (for signed ops)
    logic              rs2_sign;  // Sign of rs2 (for signed ops)
    logic [`REG_SIZE]  rs1_data;  // Original dividend value

  } div_tracker_t;

  // Shift register for tracking up to 8 in-flight divisions
  div_tracker_t div_tracker[8];



  // Instantiate the pipelined divider
  DividerUnsignedPipelined divider (
      .clk(clk),
      .rst(rst),
      .stall(False),  // We don't stall the divider internally
      .i_dividend(div_i_dividend),
      .i_divisor(div_i_divisor),
      .o_quotient(div_o_quotient),
      .o_remainder(div_o_remainder)
  );

  // Division operation detector and tracker
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int i = 0; i < 8; i++) begin
        div_tracker[i] <= '{
            valid: 0,
            rd_addr: 0,
            write_rd: 0,
            pc: 0,
            insn: 0,
            div_op: 0,
            rs1_sign: 0,
            rs2_sign: 0,
            rs1_data: 0
        };
      end
      div_i_dividend <= 0;
      div_i_divisor  <= 0;

    end else begin
      // Shift the tracker entries
      for (int i = 7; i > 0; i--) begin
        div_tracker[i] <= div_tracker[i-1];
      end

      // Check for new division operation
      if (e_is_div) begin
        div_tracker[0].valid <= 1;
        div_tracker[0].rd_addr <= e_rd_addr;
        div_tracker[0].write_rd <= e_write_rd;
        div_tracker[0].pc <= e_pc;
        div_tracker[0].insn <= e_insn;
        div_tracker[0].rs1_data <= e_rs1_data;  // Store original dividend value

        // Check for division by zero
        if (e_rs2_data == 0) begin
          // Handle division by zero according to RISC-V spec
          case (e_funct3)
            3'b100:  div_tracker[0].div_op <= 4'b1111;  // DIV - special flag for div by zero
            3'b101:  div_tracker[0].div_op <= 4'b1110;  // DIVU - special flag for div by zero
            3'b110:  div_tracker[0].div_op <= 4'b1101;  // REM - special flag for rem by zero
            3'b111:  div_tracker[0].div_op <= 4'b1100;  // REMU - special flag for rem by zero
            default: div_tracker[0].div_op <= 4'b0000;
          endcase
        end else begin
          // Normal operation codes
          case (e_funct3)
            3'b100:  div_tracker[0].div_op <= 4'b0001;  // DIV
            3'b101:  div_tracker[0].div_op <= 4'b0010;  // DIVU
            3'b110:  div_tracker[0].div_op <= 4'b0100;  // REM
            3'b111:  div_tracker[0].div_op <= 4'b1000;  // REMU
            default: div_tracker[0].div_op <= 4'b0000;
          endcase
        end

        // Store signs for signed operations
        if (e_funct3 == 3'b100 || e_funct3 == 3'b110) begin
          div_tracker[0].rs1_sign <= e_rs1_data[31];
          div_tracker[0].rs2_sign <= e_rs2_data[31];
        end else begin
          div_tracker[0].rs1_sign <= 0;
          div_tracker[0].rs2_sign <= 0;
        end

        // Feed the divider with sign-corrected inputs
        case (e_funct3)
          3'b100, 3'b110: begin  // DIV, REM (signed)
            div_i_dividend <= e_rs1_data[31] ? (~e_rs1_data + 1) : e_rs1_data;
            div_i_divisor  <= e_rs2_data[31] ? (~e_rs2_data + 1) : e_rs2_data;
          end
          default: begin  // DIVU, REMU (unsigned)
            div_i_dividend <= e_rs1_data;
            div_i_divisor  <= e_rs2_data;
          end
        endcase
      end else begin
        div_tracker[0].valid <= 0;
        div_tracker[0].rd_addr <= 0;
        div_tracker[0].write_rd <= 0;
        div_tracker[0].pc <= 0;
        div_tracker[0].insn <= 0;
        div_tracker[0].div_op <= 0;
        div_tracker[0].rs1_sign <= 0;
        div_tracker[0].rs2_sign <= 0;
        div_tracker[0].rs1_data <= 0;
      end
    end
  end

  always_comb begin
    div_data_hazard = 0;


    // Check for any valid division operations in progress
    for (int i = 0; i < 6; i++) begin
      if (div_tracker[i].valid) begin
        if (!d_is_div) begin
          // Case 1: Non-division instruction after division

          div_data_hazard = 1;
          break;
        end else if ((d_rs1 != 0 && d_rs1 == div_tracker[i].rd_addr) || 
                         (d_rs2 != 0 && d_rs2 == div_tracker[i].rd_addr)) begin
          // Case 2: Division instruction that depends on previous division

          div_data_hazard = 1;
          break;
        end
        // Case 3: Independent back-to-back division - no stall needed
      end
    end

    // Also check for division in execute stage that hasn't entered tracker yet
    if (e_opcode == OpcodeRegReg && e_funct7 == 7'b0000001 && 
        (e_funct3 == 3'b100 || e_funct3 == 3'b101 || 
         e_funct3 == 3'b110 || e_funct3 == 3'b111)) begin

      if (!d_is_div) begin
        // Non-division after division

        div_data_hazard = 1;
      end else if ((d_rs1 != 0 && d_rs1 == e_rd_addr) || (d_rs2 != 0 && d_rs2 == e_rd_addr)) begin
        // Dependent division
        div_data_hazard = 1;
      end
    end
  end

  // Add division bypass logic
  logic use_div_x1_bypass, use_div_x2_bypass;
  logic [`REG_SIZE] div_bypass_data;

  // Fix bypass logic to check all tracker stages
  always_comb begin
    use_div_x1_bypass = 0;
    use_div_x2_bypass = 0;
    div_bypass_data   = 0;

    for (int i = 0; i < 8; i++) begin
      if (div_tracker[i].valid && div_tracker[i].write_rd) begin
        if (e_rs1_addr != 0 && e_rs1_addr == div_tracker[i].rd_addr) begin
          use_div_x1_bypass = 1;
        end
        if (e_rs2_addr != 0 && e_rs2_addr == div_tracker[i].rd_addr) begin
          use_div_x2_bypass = 1;
        end
        if (use_div_x1_bypass || use_div_x2_bypass) begin
          case (div_tracker[i].div_op)
            4'b0001:
            div_bypass_data = (div_tracker[i].rs1_sign != div_tracker[i].rs2_sign) ? 
                                             ~div_o_quotient + 1 : div_o_quotient;
            4'b0010: div_bypass_data = div_o_quotient;
            4'b0100:
            div_bypass_data = div_tracker[i].rs1_sign ? ~div_o_remainder + 1 : div_o_remainder;
            4'b1000: div_bypass_data = div_o_remainder;
            default: div_bypass_data = 0;
          endcase
          break;  // Use most recent result
        end
      end
    end
  end

  logic e_is_div;
  logic e_is_mul;

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
    e_imm_b = {{19{e_insn[31]}}, e_insn[31], e_insn[7], e_insn[30:25], e_insn[11:8], False};
    e_imm_u = {e_insn[31:12], 12'b0};
    e_imm_s = {{20{e_insn[31]}}, e_insn[31:25], e_insn[11:7]};  // Add this line
    e_imm_j = {{11{e_insn[31]}}, e_insn[31], e_insn[19:12], e_insn[20], e_insn[30:21], False};

    // Detect division instructions in execute stage
    e_is_div = (e_opcode == OpcodeRegReg && e_funct7 == 7'b0000001 && 
             (e_funct3 == 3'b100 || e_funct3 == 3'b101 || 
              e_funct3 == 3'b110 || e_funct3 == 3'b111));

    // Detect multiply instructions in execute stage
    e_is_mul = (e_opcode == OpcodeRegReg && e_funct7 == 7'b0000001 && 
             (e_funct3 == 3'b000 || e_funct3 == 3'b001 || 
              e_funct3 == 3'b010 || e_funct3 == 3'b011));
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
  // Update operand selection with proper priority
  always_comb begin
    // First operand (rs1)
    if (use_div_x1_bypass) e_rs1_data = div_bypass_data;
    else if (use_m_x1_bypass) e_rs1_data = m_bypass_data;
    else if (use_w_x1_bypass) e_rs1_data = w_bypass_data;
    else e_rs1_data = execute_state.rs1_data;

    // Second operand (rs2)
    if (use_div_x2_bypass) e_rs2_data = div_bypass_data;
    else if (use_m_x2_bypass) e_rs2_data = m_bypass_data;
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
    e_branch_taken  = 0;
    e_branch_target = e_pc + 4;  // Default next PC

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
            if (e_funct7 == 7'b0000001) begin
              // MUL - lower 32 bits of signed x signed product
              e_alu_result = $signed(e_rs1_data) * $signed(e_rs2_data);
            end else begin
              e_alu_result = e_cla_sum;  // ADD/SUB
            end
          end
          3'b001: begin
            if (e_funct7 == 7'b0000001) begin
              // MULH - upper 32 bits of signed x signed product
              e_alu_result = 32'(((64'($signed(e_rs1_data)) * 64'($signed(e_rs2_data))) >> 32));
            end else begin
              e_alu_result = e_rs1_data << e_rs2_data[4:0];  // SLL
            end
          end
          3'b010: begin
            if (e_funct7 == 7'b0000001) begin
              // MULHSU - upper 32 bits of signed x unsigned product
              e_alu_result = 32'(((64'($signed(e_rs1_data)) * 64'($unsigned(e_rs2_data))) >> 32));
            end else begin
              e_alu_result = {31'b0, $signed(e_rs1_data) < $signed(e_rs2_data)};  // SLT
            end
          end
          3'b011: begin
            if (e_funct7 == 7'b0000001) begin
              // MULHU - upper 32 bits of unsigned x unsigned product
              e_alu_result = 32'(((64'($unsigned(e_rs1_data)) * 64'($unsigned(e_rs2_data))) >> 32));
            end else begin
              e_alu_result = {31'b0, e_rs1_data < e_rs2_data};  // SLTU
            end
          end
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
        e_branch_taken = 1;
        e_branch_target = e_pc + e_imm_j;
      end

      // PLACEHOLDER for JALR (Milestone 2)
      OpcodeJalr: begin
        e_alu_result = e_pc + 4;  // Return address
        e_branch_taken = 1;  // Indicate that a branch is taken
        e_branch_target = (e_rs1_data + e_imm_i) & ~32'b1;  // Calculate target address
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
    // end


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
    end else if (e_opcode == OpcodeJal) begin
      // JAL instruction
      e_branch_taken  = 1;
      e_branch_target = e_pc + e_imm_j;  // Jump target
    end
  end

  // Detect load-use hazard (load in Execute, dependent instruction in Decode)
  logic load_use_hazard;
  logic insn_uses_rs1;
  logic insn_uses_rs2;
  logic decode_uses_execute_rd;
  always_comb begin
    // Detect if Execute stage contains a load instruction
    logic execute_has_load = (e_opcode == OpcodeLoad);

    case (d_opcode)
      OpcodeRegReg: begin
        insn_uses_rs1 = (d_funct3 == 3'b000 || d_funct3 == 3'b001 || d_funct3 == 3'b010 || 
                      d_funct3 == 3'b011 || d_funct3 == 3'b100 || d_funct3 == 3'b101 || 
                      d_funct3 == 3'b110 || d_funct3 == 3'b111);
        insn_uses_rs2 = (d_funct3 != 3'b001);
      end
      OpcodeRegImm: begin
        insn_uses_rs1 = 1;
        insn_uses_rs2 = 0;
      end
      OpcodeLoad: begin
        insn_uses_rs1 = 1;
        insn_uses_rs2 = 0;
      end
      OpcodeStore: begin
        insn_uses_rs1 = 1;
        insn_uses_rs2 = 1;
      end
      OpcodeJal: begin
        insn_uses_rs1 = 0;
        insn_uses_rs2 = 0;
      end
      OpcodeJalr: begin
        insn_uses_rs1 = 1;
        insn_uses_rs2 = 0;
      end
      OpcodeAuipc: begin
        insn_uses_rs1 = 0;
        insn_uses_rs2 = 0;
      end
      OpcodeLui: begin
        insn_uses_rs1 = 0;
        insn_uses_rs2 = 0;
      end
      OpcodeBranch: begin
        insn_uses_rs1 = 1;
        insn_uses_rs2 = 1;
      end
      OpcodeMiscMem: begin
        insn_uses_rs1 = 0;
        insn_uses_rs2 = 0;
      end
      OpcodeEnviron: begin
        insn_uses_rs1 = 0;
        insn_uses_rs2 = 0;
      end
      default: begin
        insn_uses_rs1 = 0;
        insn_uses_rs2 = 0;
      end
    endcase

    decode_uses_execute_rd = (execute_has_load && e_write_rd && e_rd_addr != 0) && 
                                ((d_rs1 == e_rd_addr && d_rs1 != 0 && insn_uses_rs1) || 
                                 (d_rs2 == e_rd_addr && d_rs2 != 0 && insn_uses_rs2) && d_opcode != OpcodeStore);

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
  // Pass execute results to memory stage
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
          pc: 0,
          insn: 0,
          alu_result: 0,
          rs2_data: 0,
          rd_addr: 0,
          write_rd: 0,
          cycle_status: CYCLE_RESET
      };
    end else begin
      memory_state <= '{
          pc: e_pc,
          insn: e_insn,
          alu_result: e_alu_result,
          rs2_data: e_rs2_data,
          rd_addr: e_rd_addr,
          write_rd: e_write_rd,
          cycle_status: e_branch_taken ? CYCLE_TAKEN_BRANCH : e_cycle_status
      };
      // end
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

    m_is_fence = (m_opcode == OpcodeMiscMem && m_insn[14:12] == 3'b000 && m_insn != 0);

    // Data for MX bypass
    m_bypass_data = m_alu_result;

    // WM Bypass: Detect if we need to forward data from Writeback to Memory
    // This matters for store instructions where rs2 contains the data to store
    use_w_m_bypass = (m_opcode == OpcodeStore) && (memory_state.insn[24:20] != 0) &&  // rs2 != x0
    (memory_state.insn[24:20] == w_rd_addr) && w_write_rd;
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
  
  // Define load data signal
logic [`REG_SIZE] load_data_from_dmem;
assign load_data_from_dmem = dcache.RDATA;

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
      logic div_in_progress;

      // Check if there's a division in progress but not yet completed
      div_in_progress = 0;
      for (int i = 0; i < 7; i++) begin
        if (div_tracker[i].valid) begin
          div_in_progress = 1;
          break;
        end
      end

      if (div_tracker[7].valid) begin
        // Division result is ready
        case (div_tracker[7].div_op)
          4'b0001:
          result_value = (div_tracker[7].rs1_sign != div_tracker[7].rs2_sign) ? 
                         ~div_o_quotient + 1 : div_o_quotient;  // DIV
          4'b0010: result_value = div_o_quotient;  // DIVU
          4'b0100:
          result_value = div_tracker[7].rs1_sign ? ~div_o_remainder + 1 : div_o_remainder;  // REM
          4'b1000: result_value = div_o_remainder;  // REMU
          4'b1111: result_value = 32'hFFFFFFFF;  // DIV by zero
          4'b1110: result_value = 32'hFFFFFFFF;  // DIVU by zero
          4'b1101: result_value = div_tracker[7].rs1_data;  // REM by zero
          4'b1100: result_value = div_tracker[7].rs1_data;  // REMU by zero
          default: result_value = div_o_quotient;
        endcase

        writeback_state <= '{
            pc: div_tracker[7].pc,
            insn: div_tracker[7].insn,
            result: result_value,
            rd_addr: div_tracker[7].rd_addr,
            write_rd: div_tracker[7].write_rd,
            cycle_status: CYCLE_NO_STALL
        };
      end else if (div_in_progress) begin
        // Insert bubble during division calculation
        writeback_state <= '{
            pc: 0,
            insn: 0,
            result: 0,
            rd_addr: 0,
            write_rd: 0,
            cycle_status: CYCLE_DIV
        };
      end else begin
        // Normal operation
        // In the writeback stage, where load data is processed:
        if (m_opcode == OpcodeLoad) begin
  case (m_insn[14:12])  // funct3 field
    3'b000: begin  // LB (load byte)
      case (m_alu_result[1:0])  // Check the byte offset
        2'b00: result_value = {{24{load_data_from_dmem[7]}}, load_data_from_dmem[7:0]};  // Byte 0
        2'b01: result_value = {{24{load_data_from_dmem[15]}}, load_data_from_dmem[15:8]};  // Byte 1
        2'b10: result_value = {{24{load_data_from_dmem[23]}}, load_data_from_dmem[23:16]};  // Byte 2
        2'b11: result_value = {{24{load_data_from_dmem[31]}}, load_data_from_dmem[31:24]};  // Byte 3
      endcase
    end
    3'b001: begin  // LH (load halfword)
      case (m_alu_result[1])  // Check the halfword offset
        False: result_value = {{16{load_data_from_dmem[15]}}, load_data_from_dmem[15:0]};  // Lower halfword
        True: result_value = {{16{load_data_from_dmem[31]}}, load_data_from_dmem[31:16]};  // Upper halfword
      endcase
    end
    3'b010: begin  // LW (load word)
      result_value = load_data_from_dmem;  // No extraction needed for word loads
    end
    3'b011: begin  // LD (load doubleword) - not used in RV32I
      result_value = load_data_from_dmem; // Default behavior for unsupported instruction
    end
    3'b100: begin  // LBU (load byte unsigned)
      case (m_alu_result[1:0])
        2'b00: result_value = {24'b0, load_data_from_dmem[7:0]};  // Byte 0
        2'b01: result_value = {24'b0, load_data_from_dmem[15:8]};  // Byte 1
        2'b10: result_value = {24'b0, load_data_from_dmem[23:16]};  // Byte 2
        2'b11: result_value = {24'b0, load_data_from_dmem[31:24]};  // Byte 3
      endcase
    end
    3'b101: begin  // LHU (load halfword unsigned)
      case (m_alu_result[1])
        False: result_value = {16'b0, load_data_from_dmem[15:0]};  // Lower halfword
        True: result_value = {16'b0, load_data_from_dmem[31:16]};  // Upper halfword
      endcase
    end
    3'b110: begin  // LWU (load word unsigned) - not used in RV32I
      result_value = load_data_from_dmem; // Default behavior for unsupported instruction
    end
    3'b111: begin  // Reserved
      result_value = load_data_from_dmem; // Default behavior for reserved instruction
    end
    default: result_value = load_data_from_dmem;
  endcase
end else begin
          result_value = m_alu_result;  // ALU result for non-load instructions
        end
        writeback_state <= '{
            pc: m_pc,
            insn: m_insn,
            result: result_value,
            rd_addr: m_rd_addr,
            write_rd: m_write_rd,
            cycle_status:
            (
            m_cycle_status == CYCLE_TAKEN_BRANCH && m_insn != 0
            ) ?
            CYCLE_NO_STALL
            :
            m_cycle_status
        };
      end
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
  // assign halt = (w_insn[6:0] == OpcodeEnviron) && (w_pc != 0);
  // Use explicit equality comparison for opcode
  wire is_environ_opcode = (w_insn[6:0] == 7'b1110011);
  wire pc_not_zero = |w_pc;  // OR reduction is faster than comparison
  assign halt = is_environ_opcode & pc_not_zero;


 // Memory stage connections to data memory via AXIL
always_comb begin
  // Default values (write channel)
  dcache.AWVALID = False;
  dcache.AWADDR = 32'b0;
  dcache.WVALID = False;
  dcache.WDATA = 32'b0;
  dcache.WSTRB = 4'b0000;
  dcache.BREADY = True;
  
  // Default values (read channel)
  dcache.ARVALID = False;
  dcache.ARADDR = 32'b0;
  dcache.RREADY = True;
  
  // Handle load instructions
  if (m_opcode == OpcodeLoad) begin
    // Initiate read request
    dcache.ARVALID = True;
    dcache.ARADDR = {m_alu_result[31:2], 2'b00}; // Align to word boundary
  end
  
  // Handle store instructions
  else if (m_opcode == OpcodeStore) begin
    logic [`REG_SIZE] store_data;
    
    // Determine store data with proper bypassing
    if (use_w_m_bypass) 
      store_data = w_result;
    else 
      store_data = m_rs2_data;
    
    // Initiate write request
    dcache.AWVALID = True;
    dcache.AWADDR = {m_alu_result[31:2], 2'b00}; // Align to word boundary
    dcache.WVALID = True;
    
    // Generate appropriate byte enable signals based on funct3
    case (m_insn[14:12])
      3'b000: begin  // SB - store byte
        case (m_alu_result[1:0])
          2'b00: begin
            dcache.WDATA = {24'b0, store_data[7:0]};
            dcache.WSTRB = 4'b0001;
          end
          2'b01: begin
            dcache.WDATA = {16'b0, store_data[7:0], 8'b0};
            dcache.WSTRB = 4'b0010;
          end
          2'b10: begin
            dcache.WDATA = {8'b0, store_data[7:0], 16'b0};
            dcache.WSTRB = 4'b0100;
          end
          2'b11: begin
            dcache.WDATA = {store_data[7:0], 24'b0};
            dcache.WSTRB = 4'b1000;
          end
        endcase
      end
      3'b001: begin  // SH - store halfword
        case (m_alu_result[1])
          False: begin
            dcache.WDATA = {16'b0, store_data[15:0]};
            dcache.WSTRB = 4'b0011;
          end
          True: begin
            dcache.WDATA = {store_data[15:0], 16'b0};
            dcache.WSTRB = 4'b1100;
          end
        endcase
      end
      3'b010: begin  // SW - store word
        dcache.WDATA = store_data;
        dcache.WSTRB = 4'b1111;
      end
      default: begin
        // Invalid store type - don't perform a write
        dcache.WDATA = 32'b0;
        dcache.WSTRB = 4'b0000;
      end
    endcase
  end
end
  //////////////////////////////////////////////

  always_comb begin
    // Detect stores in Execute and Memory stages
    logic e_has_store = (e_opcode == OpcodeStore && e_insn != 0);
    logic m_has_store = (m_opcode == OpcodeStore && m_insn != 0);

    // Stall only when FENCE is in Decode and there are pending stores
    fence_stall_condition = d_is_fence && (e_has_store || m_has_store);

  end


endmodule // DatapathPipelinedCache

module Processor (
    input wire                       clk,
    input wire                       rst,
    output logic                     halt,
    output wire [`REG_SIZE]          trace_writeback_pc,
    output wire [`INSN_SIZE]         trace_writeback_insn,
    output                           cycle_status_e trace_writeback_cycle_status
);

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  axi_if axi_data_cache ();
  axi_if axi_insn_cache ();
  // memory is dual-ported, to connect to both I$ and D$
  axi_if axi_mem_ro ();
  axi_if axi_mem_rw ();

AxilMemory #(.NUM_WORDS(8192)) memory (
  .ACLK(clk),
  .ARESETn(~rst),
  .port_ro(axi_mem_ro.subord),
  .port_rw(axi_mem_rw.subord)
);

`ifdef ENABLE_INSN_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) icache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_insn_cache.subord),
    .mem(axi_mem_ro.manager)
  );
`endif
`ifdef ENABLE_DATA_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) dcache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_data_cache.subord),
    .mem(axi_mem_rw.manager)
  );
`endif

  DatapathPipelinedCache datapath (
      .clk(clk),
      .rst(rst),
`ifdef ENABLE_INSN_CACHE
      .icache(axi_insn_cache.manager),
`else
      .icache(axi_mem_ro.manager),
`endif
`ifdef ENABLE_DATA_CACHE
      .dcache(axi_data_cache.manager),
`else
      .dcache(axi_mem_rw.manager),
`endif
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule
