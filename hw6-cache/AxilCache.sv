`timescale 1ns / 1ns

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

interface axi_if #(
      parameter int ADDR_WIDTH = 32
    , parameter int DATA_WIDTH = 32
);
  logic                      ARREADY;
  logic                      ARVALID;
  logic [    ADDR_WIDTH-1:0] ARADDR;
  logic [               2:0] ARPROT;

  logic                      RREADY;
  logic                      RVALID;
  logic [    DATA_WIDTH-1:0] RDATA;
  logic [               1:0] RRESP;

  logic                      AWREADY;
  logic                      AWVALID;
  logic [    ADDR_WIDTH-1:0] AWADDR;
  logic [               2:0] AWPROT;

  logic                      WREADY;
  logic                      WVALID;
  logic [    DATA_WIDTH-1:0] WDATA;
  logic [(DATA_WIDTH/8)-1:0] WSTRB;

  logic                      BREADY;
  logic                      BVALID;
  logic [               1:0] BRESP;

  modport manager(
      input ARREADY, RVALID, RDATA, RRESP, AWREADY, WREADY, BVALID, BRESP,
      output ARVALID, ARADDR, ARPROT, RREADY, AWVALID, AWADDR, AWPROT, WVALID, WDATA, WSTRB, BREADY
  );
  modport subord(
      input ARVALID, ARADDR, ARPROT, RREADY, AWVALID, AWADDR, AWPROT, WVALID, WDATA, WSTRB, BREADY,
      output ARREADY, RVALID, RDATA, RRESP, AWREADY, WREADY, BVALID, BRESP
  );
endinterface

// [BR]RESP codes, from Section A 3.4.4 of AXI4 spec
`define RESP_OK 2'b00
`define RESP_SUBORDINATE_ERROR 2'b10
`define RESP_DECODE_ERROR 2'b11

/** This is a simple memory that uses the AXI-Lite interface. */
module AxilMemory #(
    parameter int NUM_WORDS = 1024
) (
    input wire ACLK,
    input wire ARESETn,
    axi_if.subord port_ro,
    axi_if.subord port_rw
);
  localparam bit True = 1'b1;
  localparam bit False = 1'b0;
  localparam int AddrLsb = 2;  // since memory elements are 4B
  localparam int AddrMsb = $clog2(NUM_WORDS) + AddrLsb - 1;

  logic [31:0] mem_array[NUM_WORDS];
  logic [31:0] ro_araddr;
  logic ro_araddr_valid;

  initial begin
`ifdef SYNTHESIS
    $readmemh("mem_initial_contents.hex", mem_array);
`endif
  end

  assign port_ro.RRESP = `RESP_OK;
  assign port_ro.BRESP = `RESP_OK;
  assign port_rw.RRESP = `RESP_OK;
  assign port_rw.BRESP = `RESP_OK;

  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin
      ro_araddr <= 0;
      ro_araddr_valid <= False;

      port_ro.ARREADY <= True;
      port_ro.AWREADY <= False;
      port_ro.WREADY <= False;
      port_ro.RVALID <= False;
      port_ro.RDATA <= 0;

      port_rw.ARREADY <= True;
      port_rw.AWREADY <= True;
      port_rw.WREADY <= True;
      port_rw.RVALID <= False;
      port_rw.RDATA <= 0;
    end else begin

      // port_ro is read-only

      if (ro_araddr_valid) begin
        // there is a buffered read request
        if (port_ro.RREADY) begin
          // manager accepted our response, we generate next response
          port_ro.RVALID <= True;
          port_ro.RDATA <= mem_array[ro_araddr[AddrMsb:AddrLsb]];
          ro_araddr <= 0;
          ro_araddr_valid <= False;
          port_ro.ARREADY <= True;
        end
      end else if (port_ro.ARVALID && port_ro.ARREADY) begin
        // we have accepted a read request
        if (port_ro.RVALID && !port_ro.RREADY) begin
          // We have sent a response but manager has not accepted it. Buffer the new read request.
          ro_araddr <= port_ro.ARADDR;
          ro_araddr_valid <= True;
          port_ro.ARREADY <= False;
        end else begin
          // We have sent a response and manager has accepted it. Or, we were not already sending a response.
          // Either way, send a response to the request we just accepted.
          port_ro.RVALID <= True;
          port_ro.RDATA  <= mem_array[port_ro.ARADDR[AddrMsb:AddrLsb]];
        end
      end else if (port_ro.RVALID && port_ro.RREADY) begin
        // No incoming request. We have sent a response and manager has accepted it
        port_ro.RVALID  <= False;
        port_ro.RDATA   <= 0;
        port_ro.ARREADY <= True;
      end

      // port_rw is read-write

      // NB: we take a shortcut on port_rw because the manager will always be RREADY/BREADY
      // as 1) the datapath never stalls in the W stage and 2) the cache is always ready
      if (port_rw.ARVALID && port_rw.ARREADY) begin
        port_rw.RVALID <= True;
        port_rw.RDATA  <= mem_array[port_rw.ARADDR[AddrMsb:AddrLsb]];
      end else if (port_rw.RVALID) begin
        port_rw.RVALID <= False;
        port_rw.RDATA  <= 0;
      end

      if (port_rw.AWVALID && port_rw.AWREADY && port_rw.WVALID && port_rw.WREADY) begin
        if (port_rw.WSTRB[0]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][7:0] <= port_rw.WDATA[7:0];
        end
        if (port_rw.WSTRB[1]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][15:8] <= port_rw.WDATA[15:8];
        end
        if (port_rw.WSTRB[2]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][23:16] <= port_rw.WDATA[23:16];
        end
        if (port_rw.WSTRB[3]) begin
          mem_array[port_rw.AWADDR[AddrMsb:AddrLsb]][31:24] <= port_rw.WDATA[31:24];
        end
        port_rw.BVALID <= True;
      end else if (port_rw.BVALID) begin
        port_rw.BVALID <= False;
      end
    end
  end

endmodule

// States for cache state machine. You can change these if you want.
typedef enum {
  // cache can respond to an incoming request
  CACHE_AVAILABLE = 0,
  // cache miss, waiting for fill from memory
  CACHE_AWAIT_FILL_RESPONSE = 1,
  // cache miss, waiting for writeback to memory
  CACHE_AWAIT_WRITEBACK_RESPONSE = 2,
  // cache waiting for manager to accept response
  CACHE_AWAIT_MANAGER_READY = 3
} cache_state_t;

module AxilCache #(
    /** size of each cache block, in bits */
    parameter int BLOCK_SIZE_BITS = 32,
    /** number of blocks in each way of the cache */
    parameter int NUM_SETS = 4
) (
    input wire ACLK,
    input wire ARESETn,
    axi_if.subord proc,
    axi_if.manager mem
);

  // TODO: calculate these
  // localparam int BlockOffsetBits = 0;
  // localparam int IndexBits = 0;
  // localparam int TagBits = 0;

  // Calculate addressing constants
  localparam int BlockOffsetBits = 2;  // log2(4) for 4B blocks
  localparam int IndexBits = $clog2(NUM_SETS);  // bits needed to address NUM_SETS
  localparam int TagBits = 32 - IndexBits - BlockOffsetBits;  // rest of address bits

  // cache state
  cache_state_t current_state;
  // main cache structures: do not rename as tests reference these names
  logic [BLOCK_SIZE_BITS-1:0] data[NUM_SETS];
  logic [TagBits-1:0] tag[NUM_SETS];
  logic [0:0] valid[NUM_SETS];
  logic [0:0] dirty[NUM_SETS];

  // Address parsing
  logic [IndexBits-1:0] index;
  logic [TagBits-1:0] addr_tag;

  // For handling cache misses
  logic [31:0] miss_addr;
  logic miss_is_read;
  logic [31:0] miss_wdata;
  logic [3:0] miss_wstrb;

  // Buffer for read response
  logic buffered_read_valid;
  logic [31:0] buffered_read_addr;

  // For write handling during misses
  logic need_writeback;
  logic [31:0] writeback_addr;

  // For read hit timing control
  logic read_hit_detected;  // detect a hit in the current cycle
  logic read_hit_waiting;  // indicates we're waiting to respond to a hit
  logic [IndexBits-1:0] read_hit_index;  // store the index for delayed response

  // logic [31:0] next_rdata;  // intermediate signal for RDATA assignment

  // initialize cache state to all zeroes
  genvar seti;
  for (seti = 0; seti < NUM_SETS; seti = seti + 1) begin : gen_cache_init
    initial begin
      valid[seti] = '0;
      dirty[seti] = '0;
      data[seti]  = 0;
      tag[seti]   = 0;
    end
  end

  always_comb begin
    // addresses should always be 4B-aligned
    assert (!proc.ARVALID || proc.ARADDR[1:0] == 2'b00);
    assert (proc.ARPROT == 3'd0);
    assert (!proc.AWVALID || proc.AWADDR[1:0] == 2'b00);
    assert (proc.AWPROT == 3'd0);
    // cache is single-ported
    assert (!(proc.ARVALID && (proc.AWVALID || proc.WVALID)));
  end
  // the cache never raises any errors
  assign proc.RRESP = `RESP_OK;
  assign proc.BRESP = `RESP_OK;

  // TODO: the rest of your changes will go below

  // Calculate index and tag based on current request
  always_comb begin
    // Default address parsing for AVAILABLE state
    if (proc.ARVALID) begin
      index = proc.ARADDR[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = proc.ARADDR[31:BlockOffsetBits+IndexBits];
    end else if (proc.AWVALID) begin
      index = proc.AWADDR[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = proc.AWADDR[31:BlockOffsetBits+IndexBits];
    end else if (buffered_read_valid) begin
      index = buffered_read_addr[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = buffered_read_addr[31:BlockOffsetBits+IndexBits];
    end else if (current_state != CACHE_AVAILABLE) begin
      // For miss handling
      index = miss_addr[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = miss_addr[31:BlockOffsetBits+IndexBits];
    end else begin
      index = '0;
      addr_tag = '0;
    end

    // Calculate writeback address when needed
    writeback_addr = {tag[index], index, {BlockOffsetBits{1'b0}}};

    // Determine if writeback is needed on a miss
    need_writeback = valid[index] && dirty[index] && (tag[index] != addr_tag);
  end

  // Handle RDATA combinational assignment
  // always_comb begin
  //   // For miss responses: direct connection between memory and processor
  //   if (current_state == CACHE_AWAIT_FILL_RESPONSE && mem.RVALID) begin
  //     proc.RDATA = mem.RDATA;  // Combinational assignment for miss responses
  //   end else if (read_hit_waiting) begin
  //     proc.RDATA = data[read_hit_index];  // Delayed response for hits
  //   end else begin
  //     proc.RDATA = '0;
  //   end
  // end

  // Handle RDATA combinational assignment
  // always_comb begin
  //   // For miss responses: direct connection between memory and processor
  //   if (current_state == CACHE_AWAIT_FILL_RESPONSE && mem.RVALID) begin
  //     next_rdata = mem.RDATA;  // Combinational assignment for miss responses
  //   end else if (read_hit_waiting) begin
  //     next_rdata = data[read_hit_index];  // Delayed response for hits
  //   end else begin
  //     next_rdata = '0;
  //   end
  // end

  // always_ff @(posedge ACLK) begin
  //   if (!ARESETn) begin  // NB: reset when ARESETn == 0
  //     current_state <= CACHE_AVAILABLE;
  //   end
  // end

  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin  // Reset state
      current_state <= CACHE_AVAILABLE;

      // Reset buffers
      buffered_read_valid <= 0;
      buffered_read_addr <= 0;
      miss_addr <= 0;
      miss_is_read <= 0;
      miss_wdata <= 0;
      miss_wstrb <= 0;

      // Reset proc interface signals
      proc.ARREADY <= 1;
      proc.RVALID <= 0;
      proc.RDATA <= 0;
      // proc.RDATA <= next_rdata;
      proc.AWREADY <= 1;
      proc.WREADY <= 1;
      proc.BVALID <= 0;

      // Reset mem interface signals
      mem.ARVALID <= 0;
      mem.ARADDR <= 0;
      mem.ARPROT <= 0;
      mem.RREADY <= 0;
      mem.AWVALID <= 0;
      mem.AWADDR <= 0;
      mem.AWPROT <= 0;
      mem.WVALID <= 0;
      mem.WDATA <= 0;
      mem.WSTRB <= 0;
      mem.BREADY <= 0;

      read_hit_detected <= 0;
      read_hit_waiting <= 0;
      read_hit_index <= 0;

      // next_rdata <= 0;

      // For RDATA generation 
      // logic [31:0] next_rdata;  // intermediate signal for RDATA assignment

    end else begin
      case (current_state)
        CACHE_AVAILABLE: begin

          // Handle read hits with proper timing
          if (proc.ARVALID && proc.ARREADY) begin
            if (valid[index] && (tag[index] == addr_tag)) begin
              // // Detect a hit but delay the response by one cycle
              // read_hit_detected <= 1;
              // read_hit_index <= index;
              // proc.ARREADY <= proc.RREADY;  // Can accept another request if ready

              // if (!proc.RREADY) begin
              //   // Buffer the read request if manager not ready
              //   buffered_read_valid <= 1;
              //   buffered_read_addr <= proc.ARADDR;
              //   current_state <= CACHE_AWAIT_MANAGER_READY;
              // end

              // For cache hit, immediately set RVALID and RDATA
              proc.RVALID  <= 1;
              proc.RDATA   <= data[index];

              // Can accept another request if manager ready for response
              proc.ARREADY <= proc.RREADY;

              if (!proc.RREADY) begin
                // Manager not ready for response, transition to wait state
                current_state <= CACHE_AWAIT_MANAGER_READY;
              end
            end else begin
              // Cache miss - need to fetch from memory
              miss_addr <= proc.ARADDR;
              miss_is_read <= 1;
              proc.ARREADY <= 0;

              if (need_writeback) begin
                // Need to writeback dirty data first
                current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE;
                mem.AWVALID <= 1;
                mem.AWADDR <= writeback_addr;
                mem.AWPROT <= 0;
                mem.WVALID <= 1;
                mem.WDATA <= data[index];
                mem.WSTRB <= 4'b1111;  // Write all bytes
                mem.BREADY <= 1;
              end else begin
                // Can directly fetch from memory
                current_state <= CACHE_AWAIT_FILL_RESPONSE;
                mem.ARVALID <= 1;
                mem.ARADDR <= proc.ARADDR;
                mem.ARPROT <= 0;
                mem.RREADY <= 1;
              end
            end
          end

          // // Handle delayed read hit response
          // if (read_hit_detected) begin
          //   read_hit_detected <= 0;
          //   read_hit_waiting <= 1;
          //   proc.RVALID <= 1;
          //   proc.RDATA <= data[read_hit_index];  // Directly use the cached data
          // end

          // Clean up responses when manager has accepted them
          if (proc.RVALID && proc.RREADY) begin
            proc.RVALID <= 0;
            read_hit_waiting <= 0;
            proc.ARREADY <= 1;
          end  // Handle write hits
          else if (proc.AWVALID && proc.AWREADY && proc.WVALID && proc.WREADY) begin
            if (valid[index] && (tag[index] == addr_tag)) begin
              // Cache hit - update data and mark dirty
              if (proc.WSTRB[0]) data[index][7:0] <= proc.WDATA[7:0];
              if (proc.WSTRB[1]) data[index][15:8] <= proc.WDATA[15:8];
              if (proc.WSTRB[2]) data[index][23:16] <= proc.WDATA[23:16];
              if (proc.WSTRB[3]) data[index][31:24] <= proc.WDATA[31:24];

              dirty[index] <= 1;
              proc.BVALID  <= 1;

              // Can accept another request if consumer is ready
              proc.AWREADY <= proc.BREADY;
              proc.WREADY  <= proc.BREADY;

              if (!proc.BREADY) begin
                // Transition to wait state if manager not ready
                current_state <= CACHE_AWAIT_MANAGER_READY;
              end
            end else begin
              // Cache miss - need to fetch from memory
              miss_addr <= proc.AWADDR;
              miss_is_read <= 0;
              miss_wdata <= proc.WDATA;
              miss_wstrb <= proc.WSTRB;
              proc.AWREADY <= 0;
              proc.WREADY <= 0;

              if (need_writeback) begin
                // Need to writeback dirty data first
                current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE;
                mem.AWVALID <= 1;
                mem.AWADDR <= writeback_addr;
                mem.AWPROT <= 0;
                mem.WVALID <= 1;
                mem.WDATA <= data[index];
                mem.WSTRB <= 4'b1111;  // Write all bytes
                mem.BREADY <= 1;
              end else begin
                // Can directly fetch from memory
                current_state <= CACHE_AWAIT_FILL_RESPONSE;
                mem.ARVALID <= 1;
                mem.ARADDR <= proc.AWADDR;
                mem.ARPROT <= 0;
                mem.RREADY <= 1;
              end
            end
          end  // Handle buffered read
          else if (buffered_read_valid) begin
            // Process the buffered read now
            if (valid[index] && (tag[index] == addr_tag)) begin
              // Cache hit for buffered read
              proc.RVALID <= 1;
              // proc.RDATA <= next_rdata;
              proc.RDATA <= data[index];  // Directly use the cached data
              read_hit_waiting <= 1;
              read_hit_index <= index;

              if (proc.RREADY) begin
                // Manager ready, clear buffer
                buffered_read_valid <= 0;
                buffered_read_addr <= 0;
                read_hit_waiting <= 0;
              end else begin
                // Manager still not ready
                current_state <= CACHE_AWAIT_MANAGER_READY;
              end
            end else begin
              // Cache miss for buffered read
              miss_addr <= buffered_read_addr;
              miss_is_read <= 1;
              buffered_read_valid <= 0;

              if (need_writeback) begin
                // Need to writeback dirty data first
                current_state <= CACHE_AWAIT_WRITEBACK_RESPONSE;
                mem.AWVALID <= 1;
                mem.AWADDR <= writeback_addr;
                mem.AWPROT <= 0;
                mem.WVALID <= 1;
                mem.WDATA <= data[index];
                mem.WSTRB <= 4'b1111;
                mem.BREADY <= 1;
              end else begin
                // Can directly fetch from memory
                current_state <= CACHE_AWAIT_FILL_RESPONSE;
                mem.ARVALID <= 1;
                mem.ARADDR <= buffered_read_addr;
                mem.ARPROT <= 0;
                mem.RREADY <= 1;
              end
            end
          end

        end

        CACHE_AWAIT_WRITEBACK_RESPONSE: begin
          // Wait for memory to accept writeback request
          if (mem.AWREADY && mem.AWVALID) begin
            mem.AWVALID <= 0;
          end

          if (mem.WREADY && mem.WVALID) begin
            mem.WVALID <= 0;
          end

          // Wait for memory to confirm writeback completion
          if (mem.BVALID && mem.BREADY) begin
            mem.BREADY <= 0;

            // Proceed to fetch data for the miss
            current_state <= CACHE_AWAIT_FILL_RESPONSE;
            mem.ARVALID <= 1;
            mem.ARADDR <= miss_addr;
            mem.ARPROT <= 0;
            mem.RREADY <= 1;
          end
        end

        CACHE_AWAIT_FILL_RESPONSE: begin
          // Wait for memory to accept read request
          if (mem.ARREADY && mem.ARVALID) begin
            mem.ARVALID <= 0;
          end

          // Wait for memory to return data
          if (mem.RVALID && mem.RREADY) begin
            // Update cache with fetched data
            data[index]  <= mem.RDATA;
            tag[index]   <= addr_tag;
            valid[index] <= 1;

            if (miss_is_read) begin
              // For read miss, return data to processor
              proc.RVALID  <= 1;
              // proc.RDATA   <= next_rdata;
              proc.RDATA   <= mem.RDATA;  // Directly use memory data

              // proc.RDATA is assigned in always_comb
              proc.ARREADY <= proc.RREADY;

              if (proc.RREADY) begin
                // Return to available state if manager ready
                current_state <= CACHE_AVAILABLE;
                mem.RREADY <= 0;
              end else begin
                // Manager not ready for response
                current_state <= CACHE_AWAIT_MANAGER_READY;
                mem.RREADY <= 0;
              end
            end else begin
              // For write miss, update cache with write data
              if (miss_wstrb[0]) data[index][7:0] <= miss_wdata[7:0];
              if (miss_wstrb[1]) data[index][15:8] <= miss_wdata[15:8];
              if (miss_wstrb[2]) data[index][23:16] <= miss_wdata[23:16];
              if (miss_wstrb[3]) data[index][31:24] <= miss_wdata[31:24];

              dirty[index] <= 1;
              proc.BVALID  <= 1;

              if (proc.BREADY) begin
                // Return to available state if manager ready
                current_state <= CACHE_AVAILABLE;
                proc.AWREADY <= 1;
                proc.WREADY <= 1;
                mem.RREADY <= 0;
              end else begin
                // Manager not ready for response
                current_state <= CACHE_AWAIT_MANAGER_READY;
                mem.RREADY <= 0;
              end
            end
          end
        end

        CACHE_AWAIT_MANAGER_READY: begin


          // Ensure ARREADY is deasserted while buffering a response
          proc.ARREADY <= 0;

          if (proc.RVALID && proc.RREADY) begin
            // Read response accepted
            proc.RVALID <= 0;
            read_hit_waiting <= 0;
            buffered_read_valid <= 0;
            current_state <= CACHE_AVAILABLE;
            proc.ARREADY <= 1;
          end

          if (proc.BVALID && proc.BREADY) begin
            // Write response accepted
            proc.BVALID   <= 0;
            current_state <= CACHE_AVAILABLE;
            proc.AWREADY  <= 1;
            proc.WREADY   <= 1;
          end

        end
      endcase
    end
  end

endmodule  // AxilCache

`ifndef SYNTHESIS
/** This is used for testing AxilCache in simulation. Since Verilator doesn't allow
SV interfaces in a top-level module, we wrap the interfaces with plain wires. */
module AxilCacheTester #(
    // these parameters are for the AXIL interface
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    // these parameters are for the cache
    parameter int BLOCK_SIZE_BITS = 32,
    parameter int NUM_SETS = 4
) (
    input wire ACLK,
    input wire ARESETn,

    input  wire                       CACHE_ARVALID,
    output logic                      CACHE_ARREADY,
    input  wire  [    ADDR_WIDTH-1:0] CACHE_ARADDR,
    input  wire  [               2:0] CACHE_ARPROT,
    output logic                      CACHE_RVALID,
    input  wire                       CACHE_RREADY,
    output logic [    ADDR_WIDTH-1:0] CACHE_RDATA,
    output logic [               1:0] CACHE_RRESP,
    input  wire                       CACHE_AWVALID,
    output logic                      CACHE_AWREADY,
    input  wire  [    ADDR_WIDTH-1:0] CACHE_AWADDR,
    input  wire  [               2:0] CACHE_AWPROT,
    input  wire                       CACHE_WVALID,
    output logic                      CACHE_WREADY,
    input  wire  [    DATA_WIDTH-1:0] CACHE_WDATA,
    input  wire  [(DATA_WIDTH/8)-1:0] CACHE_WSTRB,
    output logic                      CACHE_BVALID,
    input  wire                       CACHE_BREADY,
    output logic [               1:0] CACHE_BRESP,

    output wire                       MEM_ARVALID,
    input  logic                      MEM_ARREADY,
    output wire  [    ADDR_WIDTH-1:0] MEM_ARADDR,
    output wire  [               2:0] MEM_ARPROT,
    input  logic                      MEM_RVALID,
    output wire                       MEM_RREADY,
    input  logic [    ADDR_WIDTH-1:0] MEM_RDATA,
    input  logic [               1:0] MEM_RRESP,
    output wire                       MEM_AWVALID,
    input  logic                      MEM_AWREADY,
    output wire  [    ADDR_WIDTH-1:0] MEM_AWADDR,
    output wire  [               2:0] MEM_AWPROT,
    output wire                       MEM_WVALID,
    input  logic                      MEM_WREADY,
    output wire  [    DATA_WIDTH-1:0] MEM_WDATA,
    output wire  [(DATA_WIDTH/8)-1:0] MEM_WSTRB,
    input  logic                      MEM_BVALID,
    output wire                       MEM_BREADY,
    input  logic [               1:0] MEM_BRESP
);

  axi_if #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) cache_axi ();
  assign cache_axi.manager.ARVALID = CACHE_ARVALID;
  assign CACHE_ARREADY = cache_axi.manager.ARREADY;
  assign cache_axi.manager.ARADDR = CACHE_ARADDR;
  assign cache_axi.manager.ARPROT = CACHE_ARPROT;
  assign CACHE_RVALID = cache_axi.manager.RVALID;
  assign cache_axi.manager.RREADY = CACHE_RREADY;
  assign CACHE_RRESP = cache_axi.manager.RRESP;
  assign CACHE_RDATA = cache_axi.manager.RDATA;
  assign cache_axi.manager.AWVALID = CACHE_AWVALID;
  assign CACHE_AWREADY = cache_axi.manager.AWREADY;
  assign cache_axi.manager.AWADDR = CACHE_AWADDR;
  assign cache_axi.manager.AWPROT = CACHE_AWPROT;
  assign cache_axi.manager.WVALID = CACHE_WVALID;
  assign CACHE_WREADY = cache_axi.manager.WREADY;
  assign cache_axi.manager.WDATA = CACHE_WDATA;
  assign cache_axi.manager.WSTRB = CACHE_WSTRB;
  assign CACHE_BVALID = cache_axi.manager.BVALID;
  assign cache_axi.manager.BREADY = CACHE_BREADY;
  assign CACHE_BRESP = cache_axi.manager.BRESP;

  axi_if #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
  ) mem_axi ();
  assign MEM_ARVALID = mem_axi.subord.ARVALID;
  assign mem_axi.subord.ARREADY = MEM_ARREADY;
  assign MEM_ARADDR = mem_axi.subord.ARADDR;
  assign MEM_ARPROT = mem_axi.subord.ARPROT;
  assign mem_axi.subord.RVALID = MEM_RVALID;
  assign MEM_RREADY = mem_axi.subord.RREADY;
  assign mem_axi.subord.RRESP = MEM_RRESP;
  assign mem_axi.subord.RDATA = MEM_RDATA;
  assign MEM_AWVALID = mem_axi.subord.AWVALID;
  assign mem_axi.subord.AWREADY = MEM_AWREADY;
  assign MEM_AWADDR = mem_axi.subord.AWADDR;
  assign MEM_AWPROT = mem_axi.subord.AWPROT;
  assign MEM_WVALID = mem_axi.subord.WVALID;
  assign mem_axi.subord.WREADY = MEM_WREADY;
  assign MEM_WDATA = mem_axi.subord.WDATA;
  assign MEM_WSTRB = mem_axi.subord.WSTRB;
  assign mem_axi.subord.BVALID = MEM_BVALID;
  assign MEM_BREADY = mem_axi.subord.BREADY;
  assign mem_axi.subord.BRESP = MEM_BRESP;

  AxilCache #(
      .BLOCK_SIZE_BITS(BLOCK_SIZE_BITS),
      .NUM_SETS(NUM_SETS)
  ) cache (
      .ACLK(ACLK),
      .ARESETn(ARESETn),
      .proc(cache_axi.subord),
      .mem(mem_axi.manager)
  );
endmodule  // AxilCacheTester
`endif
