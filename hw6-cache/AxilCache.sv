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

  // Calculate addressing constants
  localparam int BlockOffsetBits = 2;
  localparam int IndexBits = $clog2(NUM_SETS);
  localparam int TagBits = 32 - IndexBits - BlockOffsetBits;  // rest of address bits

  // cache state
  cache_state_t current_state, next_state;

  // Buffer for read response data while waiting for manager
  logic [31:0] buffered_proc_rdata, next_buffered_proc_rdata;
  // Flag to indicate if the pending response is read (vs write)
  logic pending_response_is_read, next_pending_response_is_read;


  // main cache structures: do not rename as tests reference these names
  logic [BLOCK_SIZE_BITS-1:0] data[NUM_SETS];
  logic [TagBits-1:0] tag[NUM_SETS];
  logic [0:0] valid[NUM_SETS];
  logic [0:0] dirty[NUM_SETS];

  // Address parsing registers
  logic [IndexBits-1:0] index, next_index;
  logic [TagBits-1:0] addr_tag, next_addr_tag;

  // For handling cache misses
  logic [31:0] miss_addr, next_miss_addr;
  logic miss_is_read, next_miss_is_read;
  logic [31:0] miss_wdata, next_miss_wdata;
  logic [3:0] miss_wstrb, next_miss_wstrb;

  logic                 read_pending;
  logic [         31:0] pending_read_addr;
  logic [IndexBits-1:0] pending_index;
  logic [  TagBits-1:0] pending_tag;

  logic                 next_read_pending;
  logic [         31:0] next_pending_read_addr;
  logic [IndexBits-1:0] next_pending_read_index;
  logic [  TagBits-1:0] next_pending_tag;

  // Buffer for read response
  logic buffered_read_valid, next_buffered_read_valid;
  logic [31:0] buffered_read_addr, next_buffered_read_addr;

  // For write handling during misses
  logic need_writeback;
  logic [31:0] writeback_addr;

  // AXI signals
  logic next_proc_arready;
  logic next_proc_rvalid;
  logic [31:0] next_proc_rdata;
  logic next_proc_awready;
  logic next_proc_wready;
  logic next_proc_bvalid;

  logic next_mem_arvalid;
  logic [31:0] next_mem_araddr;
  logic next_mem_rready;
  logic next_mem_awvalid;
  logic [31:0] next_mem_awaddr;
  logic next_mem_wvalid;
  logic [31:0] next_mem_wdata;
  logic [3:0] next_mem_wstrb;
  logic next_mem_bready;

  // Data signals
  logic [BLOCK_SIZE_BITS-1:0] next_data[NUM_SETS];
  logic [TagBits-1:0] next_tag[NUM_SETS];
  logic [0:0] next_valid[NUM_SETS];
  logic [0:0] next_dirty[NUM_SETS];

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

  // Calculate index and tag based on current request OR miss state
  always_comb begin
    if (current_state == CACHE_AVAILABLE && proc.ARVALID) begin
      index = proc.ARADDR[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = proc.ARADDR[31:BlockOffsetBits+IndexBits];
    end else if (current_state == CACHE_AVAILABLE && proc.AWVALID) begin
      index = proc.AWADDR[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = proc.AWADDR[31:BlockOffsetBits+IndexBits];
    end else if (current_state != CACHE_AVAILABLE) begin
      index = miss_addr[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
      addr_tag = miss_addr[31:BlockOffsetBits+IndexBits];
    end else begin
      index = '0;
      addr_tag = '0;
    end

    // Calculate writeback address based on the current data at 'index'
    writeback_addr = {tag[index], index, {BlockOffsetBits{1'b0}}};

    // Determine if writeback is needed based on the current data at 'index'
    need_writeback = valid[index] && dirty[index];  // CHANGE: Removed 'logic' and fixed condition
  end

  // Main FSM combinational logic
  always_comb begin
    // Default values for all signals
    next_state = current_state;

    // Default processor interface signals - mostly not ready/valid
    next_proc_arready = 1'b0;
    next_proc_rvalid = 1'b0;  // Default RVALID low
    next_proc_rdata = '0;  // Default RDATA zero
    next_proc_awready = 1'b0;
    next_proc_wready = 1'b0;
    next_proc_bvalid = 1'b0;  // Default BVALID low

    next_pending_response_is_read = pending_response_is_read;  // Default hold flag
    next_buffered_proc_rdata = buffered_proc_rdata;

    // Default memory interface signals - mostly not ready/valid
    next_mem_arvalid = 1'b0;
    next_mem_araddr = miss_addr;
    next_mem_rready = 1'b0;
    next_mem_awvalid = 1'b0;
    next_mem_awaddr = writeback_addr;
    next_mem_wvalid = 1'b0;
    next_mem_wdata = data[index];
    next_mem_wstrb = 4'b1111;  // Full word write for writebacks
    next_mem_bready = 1'b0;

    // Default next state values for internal registers
    next_read_pending = read_pending;
    next_pending_read_addr = pending_read_addr;
    next_pending_read_index = pending_index;
    next_pending_tag = pending_tag;
    next_buffered_read_valid = buffered_read_valid;
    next_buffered_read_addr = buffered_read_addr;
    next_miss_addr = miss_addr;
    next_miss_is_read = miss_is_read;
    next_miss_wdata = miss_wdata;
    next_miss_wstrb = miss_wstrb;

    // Default next state for cache data structures
    for (int i = 0; i < NUM_SETS; i++) begin
      next_data[i]  = data[i];
      next_tag[i]   = tag[i];
      next_valid[i] = valid[i];
      next_dirty[i] = dirty[i];
    end

    // FSM logic
    case (current_state)
      CACHE_AVAILABLE: begin
        // Cache is ready to accept new requests in this state.
        next_proc_arready = 1'b1;
        next_proc_awready = 1'b1;
        next_proc_wready  = 1'b1;

        // Handle clearing potentially completed responses from previous cycle
        if (proc.RVALID && proc.RREADY) begin
          next_proc_rvalid  = 1'b0;  // Ensure response clear if handshake happened last cycle
          // Keep readiness high
          next_proc_arready = 1'b1;
          next_proc_awready = 1'b1;
          next_proc_wready  = 1'b1;
        end
        if (proc.BVALID && proc.BREADY) begin
          next_proc_bvalid  = 1'b0;  // Ensure response clear if handshake happened last cycle
          // Keep readiness high
          next_proc_arready = 1'b1;
          next_proc_awready = 1'b1;
          next_proc_wready  = 1'b1;
        end

        // Process New Read Request
        if (proc.ARVALID && proc.ARREADY) begin
          // Parse request
          logic [IndexBits-1:0] req_index = proc.ARADDR[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
          logic [TagBits-1:0] req_tag = proc.ARADDR[31:BlockOffsetBits+IndexBits];

          // Check for hit/miss (using current cache state)
          if (valid[req_index] && tag[req_index] == req_tag) begin
            // Read Hit
            next_proc_rvalid = 1'b1;  // Drive response combinatorially THIS cycle
            next_proc_rdata  = data[req_index];  // Drive response combinatorially THIS cycle

            /////// al;dfkjalkjf

            if (!proc.RREADY) begin  // Check if manager is ready THIS cycle
              // Manager NOT ready, need to wait
              next_state = CACHE_AWAIT_MANAGER_READY;
              next_pending_response_is_read = 1'b1;  // Flag as pending read
              next_buffered_proc_rdata = data[req_index];  // Buffer data for wait state
              // Block new requests NEXT cycle while waiting
              next_proc_arready = 1'b0;
              next_proc_awready = 1'b0;
              next_proc_wready = 1'b0;
            end else begin
              // Manager IS ready, handshake completes THIS cycle. Stay AVAILABLE
              next_state = CACHE_AVAILABLE;
              next_buffered_proc_rdata = '0;
              next_pending_response_is_read = 1'bx;
            end
          end else begin
            // --- Read Miss ---
            next_miss_addr = proc.ARADDR;
            next_miss_is_read = 1'b1;
            // Block new requests NEXT cycle
            next_proc_arready = 1'b0;
            next_proc_awready = 1'b0;
            next_proc_wready = 1'b0;

            // Check if the block we need to replace is dirty, requires writeback
            if (need_writeback) begin
              next_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
              // Initiate writeback to memory
              next_mem_awvalid = 1'b1;
              next_mem_awaddr = writeback_addr;  // Address of block being evicted
              next_mem_wvalid = 1'b1;
              next_mem_wdata = data[index];
              next_mem_wstrb = '1;
              next_mem_bready = 1'b1;
            end else begin
              // No writeback needed, just fetch the data
              next_state = CACHE_AWAIT_FILL_RESPONSE;
              // Initiate read from memory
              next_mem_arvalid = 1'b1;
              next_mem_araddr = proc.ARADDR;
              next_mem_rready = 1'b1;
            end
          end
        end // End Read Request Processing

        // Process New Write Request, only process if no read request was processed this cycle
        else if (proc.AWVALID && proc.AWREADY && proc.WVALID && proc.WREADY) begin
          // Parse request
          logic [IndexBits-1:0] req_index = proc.AWADDR[BlockOffsetBits+IndexBits-1:BlockOffsetBits];
          logic [TagBits-1:0] req_tag = proc.AWADDR[31:BlockOffsetBits+IndexBits];

          // Check for hit/miss (using current cache state)
          if (valid[req_index] && (tag[req_index] == req_tag)) begin
            // --- Write Hit ---
            for (int i = 0; i < (BLOCK_SIZE_BITS / 8); i++) begin
              if (proc.WSTRB[i]) begin
                next_data[req_index][8*i+:8] = proc.WDATA[8*i+:8];
              end
            end
            next_dirty[req_index] = 1'b1;

            // Drive response combinatorially THIS cycle
            next_proc_bvalid = 1'b1;

            if (!proc.BREADY) begin  // Check if manager is ready THIS cycle
              // Manager NOT ready, need to wait
              next_state = CACHE_AWAIT_MANAGER_READY;
              next_pending_response_is_read = 1'b0;  // Flag as pending write response
              next_buffered_proc_rdata = data[req_index];  // Buffer data for wait state
              // Block new requests NEXT cycle while waiting
              next_proc_arready = 1'b0;
              next_proc_awready = 1'b0;
              next_proc_wready = 1'b0;
            end else begin
              // Manager IS ready, handshake completes THIS cycle. Stay AVAILABLE.
              next_state = CACHE_AVAILABLE;

              next_pending_response_is_read = 1'bx;
            end
          end else begin
            // Write Miss
            // Store miss information for later states
            next_miss_addr = proc.AWADDR;
            next_miss_is_read = 1'b0;
            next_miss_wdata = proc.WDATA;  // Buffer write data
            next_miss_wstrb = proc.WSTRB;  // Buffer write strobe
            // Block new requests NEXT cycle
            next_proc_arready = 1'b0;
            next_proc_awready = 1'b0;
            next_proc_wready = 1'b0;

            // Check if the block we need to replace is dirty (requires writeback)
            if (need_writeback) begin
              next_state = CACHE_AWAIT_WRITEBACK_RESPONSE;
              // Initiate writeback to memory
              next_mem_awvalid = 1'b1;
              next_mem_awaddr = writeback_addr;
              next_mem_wvalid = 1'b1;
              next_mem_wdata = data[index];
              next_mem_wstrb = '1;
              next_mem_bready = 1'b1;
            end else begin
              next_state = CACHE_AWAIT_FILL_RESPONSE;
              next_mem_arvalid = 1'b1;
              next_mem_araddr = proc.AWADDR;  // Address needed for the fill
              next_mem_rready = 1'b1;
            end
          end
        end
      end  // End CACHE_AVAILABLE case

      CACHE_AWAIT_FILL_RESPONSE: begin

        next_mem_araddr = miss_addr;

        // Keep memory request signals asserted until the request is accepted
        if (!mem.ARREADY || (mem.ARVALID && !mem.ARREADY)) begin
          next_mem_arvalid = 1'b1;
        end

        // Always be ready to receive memory data
        next_mem_rready = 1'b1;

        // Process memory response
        if (mem.RVALID && mem.RREADY) begin
          // Update cache with memory data (for NEXT cycle)
          next_data[index]  = mem.RDATA;
          next_tag[index]   = addr_tag;  // Use addr_tag calculated from miss_addr
          next_valid[index] = 1'b1;
          next_dirty[index] = 1'b0;  // Filled line is initially clean

          if (miss_is_read) begin
            next_state = CACHE_AWAIT_MANAGER_READY;  // Go to wait state for handshake
            next_pending_response_is_read = 1'b1;  // It's a read response
            next_buffered_proc_rdata = mem.RDATA;  // Buffer the data
          end else begin  // Write miss fill complete, now apply the write
            for (int i = 0; i < 4; i++) begin
              if (miss_wstrb[i]) begin
                // Apply write to the data we just fetched
                next_data[index][8*i+:8] = miss_wdata[8*i+:8];
              end
            end
            next_dirty[index] = 1'b1;  // Set dirty bit for the write

            next_state = CACHE_AWAIT_MANAGER_READY;  // Go to wait state for handshake
            next_pending_response_is_read = 1'b0;  // It's a write response
          end
        end
      end

      CACHE_AWAIT_MANAGER_READY: begin
        // Always keep request channels disabled in this state
        next_proc_arready = 1'b0;
        next_proc_awready = 1'b0;
        next_proc_wready  = 1'b0;

        if (pending_response_is_read) begin  // Waiting for RREADY
          // Check if handshake complete THIS cycle
          if (proc.RVALID && proc.RREADY) begin
            // Handshake complete, immediately deassert RVALID for next cycle
            next_proc_rvalid = 1'b0;
            next_proc_rdata = '0;
            // Transition to AVAILABLE for NEXT cycle
            next_state = CACHE_AVAILABLE;
            // Clear buffer and flag
            next_buffered_proc_rdata = '0;
            next_pending_response_is_read = 1'bx;
            // Set processor readiness for NEXT cycle
            next_proc_arready = 1'b1;
            next_proc_awready = 1'b1;
            next_proc_wready = 1'b1;
          end else begin
            // No handshake yet, keep asserting RVALID
            next_proc_rvalid = 1'b1;
            next_proc_rdata  = buffered_proc_rdata;
          end
        end else begin  // Waiting for BREADY
          // Similar change for BVALID
          if (proc.BVALID && proc.BREADY) begin
            // Handshake complete, immediately deassert BVALID
            next_proc_bvalid = 1'b0;
            next_state = CACHE_AVAILABLE;
            next_pending_response_is_read = 1'b0;
            next_proc_arready = 1'b1;
            next_proc_awready = 1'b1;
            next_proc_wready = 1'b1;
          end else begin
            // No handshake yet, keep asserting BVALID
            next_proc_bvalid = 1'b1;
          end
        end
      end  // End of CACHE_AWAIT_MANAGER_READY

      CACHE_AWAIT_WRITEBACK_RESPONSE: begin

        next_mem_awvalid = 1'b1;
        next_mem_awaddr  = writeback_addr;  // Address of block being evicted
        next_mem_wvalid  = 1'b1;
        next_mem_wdata   = data[index];  // Data from block being evicted
        next_mem_wstrb   = '1;

        next_mem_bready  = 1'b1;
        if (mem.BVALID) begin
          next_state       = CACHE_AWAIT_FILL_RESPONSE;

          // Start the memory read request for the original miss address.
          next_mem_arvalid = 1'b1;
          next_mem_araddr  = miss_addr;  // Use the stored miss address
          next_mem_rready  = 1'b1;  // Be ready to accept read data next

          // Deassert write/response signals for the next cycle.
          next_mem_awvalid = 1'b0;
          next_mem_wvalid  = 1'b0;
          next_mem_bready  = 1'b0;
        end
      end  // End of CACHE_AWAIT_WRITEBACK_RESPONSE
    endcase
  end

  // Sequential logic - update state registers
  always_ff @(posedge ACLK) begin
    if (!ARESETn) begin
      // Reset state
      current_state            <= CACHE_AVAILABLE;

      miss_addr                <= 0;
      miss_is_read             <= 0;
      miss_wdata               <= 0;
      miss_wstrb               <= 0;

      buffered_proc_rdata      <= '0;
      pending_response_is_read <= 1'b0;


      // Reset proc interface signals
      proc.ARREADY             <= 1;
      proc.RVALID              <= 0;
      proc.RDATA               <= 0;
      proc.AWREADY             <= 1;
      proc.WREADY              <= 1;
      proc.BVALID              <= 0;

      // Reset mem interface signals
      mem.ARVALID              <= 0;
      mem.ARADDR               <= 0;
      mem.ARPROT               <= 0;
      mem.RREADY               <= 0;
      mem.AWVALID              <= 0;
      mem.AWADDR               <= 0;
      mem.AWPROT               <= 0;
      mem.WVALID               <= 0;
      mem.WDATA                <= 0;
      mem.WSTRB                <= 0;
      mem.BREADY               <= 0;

    end else begin
      proc.RDATA          <= next_proc_rdata;
      current_state       <= next_state;

      // Update registers
      read_pending        <= next_read_pending;
      pending_read_addr   <= next_pending_read_addr;
      pending_index       <= next_pending_read_index;
      pending_tag         <= next_pending_tag;

      // Update internal registers
      buffered_read_valid <= next_buffered_read_valid;
      buffered_read_addr  <= next_buffered_read_addr;
      miss_addr           <= next_miss_addr;
      miss_is_read        <= next_miss_is_read;
      miss_wdata          <= next_miss_wdata;
      miss_wstrb          <= next_miss_wstrb;

      // Update data
      for (int i = 0; i < NUM_SETS; i++) begin
        data[i]  <= next_data[i];
        tag[i]   <= next_tag[i];
        valid[i] <= next_valid[i];
        dirty[i] <= next_dirty[i];
      end

      buffered_proc_rdata <= next_buffered_proc_rdata;
      pending_response_is_read <= next_pending_response_is_read;

      // Update output signals
      proc.ARREADY <= next_proc_arready;
      proc.RVALID <= next_proc_rvalid;
      proc.RDATA <= next_proc_rdata;
      proc.AWREADY <= next_proc_awready;
      proc.WREADY <= next_proc_wready;
      proc.BVALID <= next_proc_bvalid;

      mem.ARVALID <= next_mem_arvalid;
      mem.ARADDR <= next_mem_araddr;
      mem.ARPROT <= 0;
      mem.RREADY <= next_mem_rready;
      mem.AWVALID <= next_mem_awvalid;
      mem.AWADDR <= next_mem_awaddr;
      mem.AWPROT <= 0;
      mem.WVALID <= next_mem_wvalid;
      mem.WDATA <= next_mem_wdata;
      mem.WSTRB <= next_mem_wstrb;
      mem.BREADY <= next_mem_bready;
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
