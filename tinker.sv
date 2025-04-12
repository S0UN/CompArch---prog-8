// Top-Level Module - MODIFIED
module tinker_core (
    input logic clk,
    input logic reset,
    output logic hlt // Output signal
);
    // --- Constants ---
    localparam HALT_OPCODE = 5'b11111;

    // --- State Machine Type ---
    typedef enum logic [2:0] {
        S_FETCH, S_DECODE, S_EXECUTE, S_MEMORY, S_WRITEBACK, S_HALTED
    } state_t;

    // --- Internal Signal Declarations (GROUPED AND PLACED FIRST) ---

    // State Variables (assigned in always blocks)
    state_t current_state, next_state;

    // PC Signals
    logic [63:0] pc_current;        // Output from fetch_unit
    logic [63:0] pc_next;           // Output from control_unit, input to fetch_unit
    logic        pc_write_enable;   // Control signal (assigned in always @(*))

    // Instruction Decode Signals
    logic [31:0] instr_word;        // Output from memory_unit (instruction port)
    logic [31:0] instr_reg;         // Latched instruction (assigned in always @(posedge clk))
    logic [4:0]  dest_reg;          // Output from inst_decoder
    logic [4:0]  src_reg1;          // Output from inst_decoder
    logic [4:0]  src_reg2;          // Output from inst_decoder
    logic [4:0]  opcode;            // Output from inst_decoder
    logic [63:0] imm_value;         // Output from inst_decoder

	logic [63:0] return_addr_reg;

    // Register File Signals
    logic [63:0] dest_val;          // Output from reg_file (data_dest port)
    logic [63:0] src_val1;          // Output from reg_file (data1 port)
    logic [63:0] src_val2;          // Output from reg_file (data2 port)
    logic [63:0] stack_ptr;         // Output from reg_file (stack port)
    logic        reg_write;         // Control signal (assigned in always @(*))
    logic        mem_to_reg;        // Control signal (assigned in always @(*))

    // ALU Signals
    logic [63:0] alu_operand2;      // Output from reg_lit_mux
    logic [63:0] alu_output;        // Output from alu_unit
    logic [63:0] alu_out_reg;       // Latched ALU output (assigned in always @(posedge clk))

    // Memory Access Signals
    logic [63:0] mem_addr;          // Output from mem_handler (addr_out port)
    logic [63:0] mem_data_in;       // Output from mem_handler (data_out port), input to memory_unit
    logic [63:0] mem_data_out;      // Output from memory_unit (data_out port)
    logic [63:0] mem_data_reg;      // Latched memory data (assigned in always @(posedge clk))
    logic        mem_read;          // Control signal (assigned in always @(*))
    logic        mem_write;         // Control signal (assigned in always @(*))
    logic        ir_write;          // Control signal (assigned in always @(*))
    logic [63:0] memory_unit_address; // Intermediate signal for memory address (assigned in always @(*))

    // Control/Branch Signals
    logic        is_branch_instr;   // Output from control_unit
    logic        branch_taken;      // Output from control_unit

    // Unused Signals (as noted in original code) - Declare them anyway if needed to avoid implicit wires elsewhere
    // logic reg_dst, alu_src; // If truly unused, can omit, but declaring is safer.

    // --- State Register ---
    // This block correctly implements the state register logic
    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= S_FETCH;
        else if (current_state == S_HALTED)
             current_state <= S_HALTED; // Remain halted
        else
            current_state <= next_state; // Update state
    end

	// Return Address Register
	always @(posedge clk or posedge reset) begin
        if (reset) begin
            return_addr_reg <= 64'b0; // Reset the register
        end else if (current_state == S_DECODE && opcode == 5'b01100) begin // Latch on CALL decode
            return_addr_reg <= pc_current + 64'd4; // Latch PC + 4 for call
        end
    end


    // --- Next State Logic ---
    // This block correctly calculates the next state based on the current state and opcode
    always @(*) begin
        next_state = current_state; // Default: stay in current state
        case (current_state)
            S_FETCH:   next_state = S_DECODE;
            S_DECODE: begin
                if (opcode == HALT_OPCODE) begin
                    next_state = S_HALTED;
                end else begin
                    next_state = S_EXECUTE;
                end
            end
            S_EXECUTE: begin
                 if (is_memory_operation(opcode)) begin
                     next_state = S_MEMORY;
                 end else if (is_branch_instr && branch_taken) begin // Branch taken, go fetch next instr
                     next_state = S_FETCH;
                 // NOTE: Removed non-taken branch logic here; handled in WRITEBACK/default
                 end else if (opcode == 5'b01100) begin // CALL needs memory access
                    next_state = S_MEMORY;
                 end else begin // Non-memory, non-branch instructions go to Writeback
                     next_state = S_WRITEBACK;
                 end
            end
            S_MEMORY:    next_state = S_WRITEBACK; // After memory access, go to writeback (for loads/returns) or fetch (for stores/calls)
             // Correction: CALL and STORE don't write back to reg file in WB stage like LD/RET
             //             They transition directly to FETCH after MEMORY
            S_WRITEBACK: next_state = S_FETCH;     // After writeback, fetch next instruction
            S_HALTED:    next_state = S_HALTED;    // Stay halted
            default:     next_state = S_FETCH;     // Default recovery state
        endcase

       // Refined Next State Logic for Memory Operations:
       if (current_state == S_MEMORY) begin
            if (is_store_operation(opcode) || opcode == 5'b01100) begin // STORE or CALL
                 next_state = S_FETCH;
            end else begin // LOAD or RETURN
                 next_state = S_WRITEBACK;
            end
        end
    end

    // --- Control Signal Logic ---
    always @(*) begin
        // --- Default values MUST be assigned FIRST ---
        ir_write = 1'b0;
        pc_write_enable = 1'b0;
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0;
        hlt = 1'b0;
        memory_unit_address = mem_addr; // Default to mem_handler output

        case (current_state)
            S_FETCH: begin
                ir_write = 1'b1;            // Enable instruction register write
                mem_read = 1'b1;            // Enable memory read for instruction fetch
                memory_unit_address = pc_current; // Use PC as address for instruction fetch
            end
            S_DECODE: begin
                // No control signals asserted in decode stage typically (just combinational logic)
            end
            S_EXECUTE: begin
                // PC write for taken branches (handled by control_unit outputting pc_next)
                // Conditional branches decide pc_next in control_unit based on flags/comparisons.
                // Unconditional jumps (br, brr, call) also calculate pc_next in control_unit.
                // If a branch IS taken (branch_taken=1), pc_write_enable gets set here.
                 if (is_branch_instr && branch_taken) begin // Check branch_taken AFTER control_unit logic runs
                    pc_write_enable = 1'b1;
                end
            end
            S_MEMORY: begin
                // Assert memory read/write based on opcode
                if (is_load_operation(opcode)) begin // Load / Return
                    mem_read = 1'b1;
                    memory_unit_address = mem_addr; // Address from mem_handler (e.g., stack ptr)
                end else if (is_store_operation(opcode)) begin // Store / Call
                    mem_write = 1'b1;
                     memory_unit_address = mem_addr; // Address from mem_handler (e.g., stack ptr)
                     // For CALL, mem_data_in comes from mem_handler (return address)
                     // For STORE, mem_data_in comes from mem_handler (source register value)
                end
                 // Address for memory operations comes from mem_handler (mem_addr)
                 // data_in for store/call also comes from mem_handler (mem_data_in)
            end
            S_WRITEBACK: begin
                 // REG WRITE ENABLE - Enable for instructions that write to the register file
                 // Excludes: Stores, Branches that don't write back (like simple branches), HALT
                 if (!is_store_operation(opcode) && !is_branch_no_writeback(opcode) && opcode != HALT_OPCODE && opcode != 5'b01100 /*CALL*/) begin
                     reg_write = 1'b1;
                 end

                 // WRITEBACK SOURCE SELECT (mem_to_reg)
                 // Select memory data for loads/returns, ALU output otherwise
                 if (is_load_operation(opcode)) begin // Load/Return
                     mem_to_reg = 1'b1;
                 end else begin // ALU result (or move result passed through ALU)
                     mem_to_reg = 1'b0;
                 end

                 // PC WRITE ENABLE (Sequential or Return)
                 // Update PC if it wasn't updated by a taken branch in EXECUTE
                 // Also update PC for RETURN instruction
                 if (opcode == 5'b01101) begin // Return instruction
                     pc_write_enable = 1'b1; // Write PC with value from stack (via mem_data_reg -> control_unit -> pc_next)
                 end else if (!is_branch_instr || !branch_taken) begin // Not a branch OR branch not taken
                     // Only update PC sequentially if it wasn't a taken branch
                     // HALT also stops PC update here.
                     if (opcode != HALT_OPCODE) begin
                          pc_write_enable = 1'b1; // Write PC with PC+4 (calculated by control_unit -> pc_next)
                     end
                 end
            end
            S_HALTED: begin
                hlt = 1'b1; // Assert halt signal
            end
            default: begin
                // Default case: might reset signals or go to a safe state if needed
            end
        endcase
    end


    // --- Helper Functions ---
     function logic is_memory_operation(logic [4:0] op);
        // Includes Load, Store, Call, Return
        return (op == 5'b10000 || op == 5'b10011 || op == 5'b01100 || op == 5'b01101);
     endfunction

     function logic is_load_operation(logic [4:0] op);
        // Includes Load (mov rd, (rs)(L)) and Return
        // Return reads from memory (stack) into PC, potentially needs WB stage if modelled that way
        return (op == 5'b10000 || op == 5'b01101);
     endfunction

     function logic is_store_operation(logic [4:0] op);
         // Includes Store (mov (rd)(L), rs) and Call
         // Call writes return address to memory (stack)
         return (op == 5'b10011 || op == 5'b01100);
     endfunction

     function logic is_branch_no_writeback(logic [4:0] op);
          // Identifies branches/control flow changes that DON'T write a result to the GPR file
          // Includes all branches, call, return, halt
          return (op >= 5'b01000 && op <= 5'b01110) // br, brr, brr L, brnz, call, return, brgt
                 || op == HALT_OPCODE; // halt is 0xf, but L=0 -> opcode field is 0xf
                                       // Need accurate HALT opcode representation
                                       // Assuming HALT uses opcode 5'b11111 based on constant
     endfunction

    // Instruction Register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_reg <= 32'b0;
        end else if (ir_write) begin // Write only when ir_write is asserted (in Fetch)
            instr_reg <= instr_word;
        end
    end

    // Memory Data Register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_data_reg <= 64'b0;
        end else if (current_state == S_MEMORY && mem_read) begin // Latch data read from memory
             mem_data_reg <= mem_data_out;
        end
    end

    // ALU Output Register
    always @(posedge clk or posedge reset) begin
       if (reset) begin
           alu_out_reg <= 64'b0;
       end else if (current_state == S_EXECUTE) begin // Latch ALU result during Execute stage
            alu_out_reg <= alu_output;
       end
    end

    // --- Instantiations ---
    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_next),           // Input from control unit's calculation
        .pc_out(pc_current),       // Output to rest of pipeline
        .pc_write(pc_write_enable) // Controlled by main FSM logic
    );

    memory_unit memory (
        // .program_counter(pc_current), // program_counter input removed as it's not used internally
        .clk(clk),
        .reset(reset),
        .read_en(mem_read),            // Controlled by main FSM
        .write_en(mem_write),          // Controlled by main FSM
        .data_in(mem_data_in),         // Data to write (from mem_handler)
        .address(memory_unit_address), // Address for access (from PC in Fetch, mem_handler in Memory)
        .data_out(mem_data_out),       // Data read from memory (to mem_data_reg)
        .instruction(instr_word)       // Instruction fetched (to instr_reg)
    );

    control_unit ctrl (
        .operation(opcode),            // From instruction decoder
        .dest_in(dest_val),            // Value read from Rd port (for branches using Rd value)
        .src_in1(src_val1),            // Value read from Rs1 port (for conditions/ALU)
        .src_in2(src_val2),            // Value read from Rs2 port (for conditions/ALU)
        .immediate(imm_value),         // From instruction decoder
        .current_pc(pc_current),       // From fetch unit
        .memory_data(mem_data_reg),    // Data read from memory (used for RETURN)
        .next_pc(pc_next),             // Output: Calculated next PC value (to fetch unit pc_in)
        .is_branch(is_branch_instr),   // Output: Indicates if instruction is a branch type
        .branch_taken(branch_taken)    // Output: Indicates if a conditional/unconditional branch is taken
    );

	mem_handler mem_mgr (
		.op(opcode),                   // From instruction decoder
		.dest(dest_val),               // Value read from Rd port (used for store address base)
		.src(src_val1),                // Value read from Rs1 port (used for load address base, store data)
		.imm(imm_value),               // From instruction decoder (used for address offset)
		.pc(pc_current),               // From fetch unit (potentially used if base+offset needed pc) - Unused currently based on logic
		.r31(stack_ptr),               // Value read from R31 port (stack pointer)
		.return_addr(return_addr_reg), // Latched PC+4 for call instruction
		.addr_out(mem_addr),           // Output: Calculated memory address for load/store/call/return
		.data_out(mem_data_in)         // Output: Data to be written to memory for store/call
	);

    inst_decoder dec (
        .instruction(instr_reg),       // Input: Latched instruction word
        .imm(imm_value),               // Output: Decoded immediate value
        .dest(dest_reg),               // Output: Destination register address
        .src1(src_reg1),               // Output: Source 1 register address
        .src2(src_reg2),               // Output: Source 2 register address
        .opcode(opcode)                // Output: Decoded opcode
    );

    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .write_en(reg_write),         // Controlled by main FSM
        .write_data(mem_to_reg ? mem_data_reg : alu_out_reg), // Data to write (from Mem or ALU)
        .addr1(src_reg1),             // Read address 1 (from decoder)
        .addr2(src_reg2),             // Read address 2 (from decoder)
        .write_addr(dest_reg),        // Write address (from decoder)
        .data1(src_val1),             // Output: Data read from addr1
        .data2(src_val2),             // Output: Data read from addr2
        .data_dest(dest_val),         // Output: Data read from write_addr port (used for branches/stores)
        .stack(stack_ptr)             // Output: Data read from R31 (stack pointer)
    );

    reg_lit_mux mux (
        .op(opcode),                   // From decoder
        .reg_val(src_val2),            // Input: Data from Rs2
        .lit_val(imm_value),           // Input: Immediate value
        .out(alu_operand2)             // Output: Second operand for ALU
    );

    alu_unit alu (
        .ctrl(opcode),                 // From decoder
        .in1(src_val1),                // Input: Data from Rs1
        .in2(alu_operand2),            // Input: Second operand (from Mux)
        .out(alu_output)               // Output: Result of ALU operation (to alu_out_reg)
    );

endmodule // End of tinker_core

//------------------------------------------------------------------------------
// Supporting Modules (MUST ALSO BE MODIFIED for always_comb/always_ff)
//------------------------------------------------------------------------------

// Modified Fetch Unit (Using always @)
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic pc_write,
    input logic [63:0] pc_in,
    output logic [63:0] pc_out
);
    logic [63:0] pc_reg;
    assign pc_out = pc_reg;

    always @(posedge clk or posedge reset) begin // Changed from always_ff
        if (reset) begin
            pc_reg <= 64'h2000;
        end else if (pc_write) begin
            pc_reg <= pc_in;
        end
    end
endmodule



// Memory Unit (Unchanged)
module memory_unit (
    // input logic [63:0] program_counter, // Removed unused input
    input logic clk,
    input logic reset,
    input logic read_en,
    input logic write_en,
    input logic [63:0] data_in,
    input logic [63:0] address,
    output logic [63:0] data_out,
    output logic [31:0] instruction
);
    localparam MEM_SIZE_BYTES = 524288;
    logic [7:0] bytes [0:MEM_SIZE_BYTES-1];
    integer j, k;

    // Instruction read (combinational) - Handle potential out-of-bounds
    assign instruction[7:0]   = (address < MEM_SIZE_BYTES) ? bytes[address] : 8'h0;
    assign instruction[15:8]  = (address + 1 < MEM_SIZE_BYTES) ? bytes[address + 1] : 8'h0;
    assign instruction[23:16] = (address + 2 < MEM_SIZE_BYTES) ? bytes[address + 2] : 8'h0;
    assign instruction[31:24] = (address + 3 < MEM_SIZE_BYTES) ? bytes[address + 3] : 8'h0;

    // Data read (combinational) - Handle potential out-of-bounds
    assign data_out[7:0]   = (address < MEM_SIZE_BYTES) ? bytes[address] : 8'h0;
    assign data_out[15:8]  = (address + 1 < MEM_SIZE_BYTES) ? bytes[address + 1] : 8'h0;
    assign data_out[23:16] = (address + 2 < MEM_SIZE_BYTES) ? bytes[address + 2] : 8'h0;
    assign data_out[31:24] = (address + 3 < MEM_SIZE_BYTES) ? bytes[address + 3] : 8'h0;
    assign data_out[39:32] = (address + 4 < MEM_SIZE_BYTES) ? bytes[address + 4] : 8'h0;
    assign data_out[47:40] = (address + 5 < MEM_SIZE_BYTES) ? bytes[address + 5] : 8'h0;
    assign data_out[55:48] = (address + 6 < MEM_SIZE_BYTES) ? bytes[address + 6] : 8'h0;
    assign data_out[63:56] = (address + 7 < MEM_SIZE_BYTES) ? bytes[address + 7] : 8'h0;


    // Memory write (sequential)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize memory (optional, can be slow for large memories in simulation)
            for (j = 0; j < MEM_SIZE_BYTES; j = j + 1)
                bytes[j] <= 8'h0;
            // Consider using $readmemh or $readmemb for initialization if needed
        end
        else if (write_en) begin
            // Write 8 bytes (64 bits) - Handle potential out-of-bounds
            for (k = 0; k < 8; k = k + 1) begin
                 if ((address + k) < MEM_SIZE_BYTES) begin
                    bytes[address + k] <= data_in[k*8 +: 8]; // Corrected slice indexing
                 end
            end
        end
    end
endmodule
module control_unit (
    input logic [4:0] operation,
    input logic [63:0] dest_in,     // Value of Rd register (used as target addr for br, brnz, brgt, call; offset for brr rd)
    input logic [63:0] src_in1,     // Value of Rs1/Rs register (used for conditions brnz, brgt)
    input logic [63:0] src_in2,     // Value of Rs2/Rt register (used for condition brgt)
    input logic [63:0] immediate,   // Decoded immediate (sign-extended, used for brr L)
    input logic [63:0] current_pc,  // Current PC value from fetch stage
    input logic [63:0] memory_data, // Data read from stack for return instruction
    output logic [63:0] next_pc,    // Calculated address for next instruction
    output logic is_branch,         // Signal indicating instruction might alter PC flow
    output logic branch_taken       // Signal indicating PC is not PC+4
);
    logic branch_condition_met;

    // Explicit check for individual branch-related opcodes
    always @(*) begin
        is_branch = 1'b0;
        if (operation == 5'b01000 || // br
            operation == 5'b01001 || // brr
            operation == 5'b01010 || // brr L
            operation == 5'b01011 || // brnz
            operation == 5'b01110)   // brgt
        begin
            is_branch = 1'b1;
        end
    end

    always @(*) begin
        // Determine if condition is met (only relevant for conditional branches)
        case(operation)
            5'b01011: branch_condition_met = (src_in1 != 0); // brnz
            5'b01110: branch_condition_met = ($signed(src_in1) > $signed(src_in2)); // brgt
            default: branch_condition_met = 1'b1; // Assume true (unconditional or not applicable)
        endcase

        // Default next PC and taken signal
        branch_taken = 1'b0;
        next_pc = current_pc + 4;

        case (operation)
            5'b01000: begin // br $rd
                next_pc = dest_in;
                branch_taken = 1'b1;
            end
            5'b01100: begin // call $rd
                next_pc = dest_in;
                branch_taken = 1'b1;
            end
            5'b01101: begin // return
                next_pc = memory_data;
                branch_taken = 1'b1;
            end
            5'b01001: begin // brr $rd
                next_pc = current_pc + dest_in;
                branch_taken = 1'b1;
            end
            5'b01010: begin // brr L
                next_pc = current_pc + immediate;
                branch_taken = 1'b1;
            end
            5'b01011: begin // brnz $rd, $rs
                if (branch_condition_met) begin
                    next_pc = dest_in;
                    branch_taken = 1'b1;
                end
            end
            5'b01110: begin // brgt $rd, $rs, $rt
                if (branch_condition_met) begin
                    next_pc = dest_in;
                    branch_taken = 1'b1;
                end
            end
            default: begin
                // default next_pc and branch_taken already set above
            end
        endcase
    end
endmodule



// Modified Memory Handler (Using always @(*))
module mem_handler (
		input logic [4:0] op,
		input logic [63:0] dest,
		input logic [63:0] src,
		input logic [63:0] imm,
		input logic [63:0] pc,
		input logic [63:0] r31,
		input logic [63:0] return_addr, // New input
		output logic [63:0] addr_out,
		output logic [63:0] data_out
);
    always @(*) begin // Changed from always_comb
        addr_out = 64'h0;
        data_out = 64'h0;
        case (op)
			5'b01100: begin // Call instruction
				addr_out = r31 - 8; // Stack pointer adjustment (match your design)
				data_out = return_addr; // Use latched return address
			end
            5'b01101: begin addr_out = r31 - 8; data_out = 64'h0; end
            5'b10000: begin addr_out = src + imm; data_out = 64'h0; end
            5'b10011: begin addr_out = dest + imm; data_out = src; end
            default: begin 
				addr_out = 64'b0;
            	data_out = 64'b0; 
				end
        endcase
    end
endmodule



// *** MODIFIED Register File Bank ***
module reg_file_bank (
    input logic clk,
    input logic reset,
    input logic write_en,         // Write enable signal from FSM
    input logic [63:0] write_data, // Data to be written (from Mux: Mem or ALU)
    input logic [4:0] addr1,      // Read address for port 1 (src1)
    input logic [4:0] addr2,      // Read address for port 2 (src2)
    input logic [4:0] write_addr, // Write address (dest)
    output logic [63:0] data1,    // Data read from port 1
    output logic [63:0] data2,    // Data read from port 2
    output logic [63:0] data_dest,// Data read from port connected to write_addr (for store base, branch target)
    output logic [63:0] stack     // Data read from register 31 (stack pointer)
);
    logic [63:0] registers [0:31]; // 32 registers, 64 bits each
    integer i;
    localparam MEMSIZE = 64'd524288; // Initial stack pointer value

    // Combinational Read Logic: Outputs reflect current register values based on addresses
    // R0 is readable like any other register.
    assign data1 = registers[addr1];
    assign data2 = registers[addr2];
    assign data_dest = registers[write_addr]; // Provide value of register targeted by write_addr for immediate use (e.g., store base)
    assign stack = registers[31];           // Dedicated output for stack pointer R31

    // Sequential Write Logic: Update registers on clock edge if enabled
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize all registers to 0, except stack pointer
            for (i = 0; i < 31; i = i + 1) begin
                registers[i] <= 64'h0;
            end
            registers[31] <= MEMSIZE; // Initialize stack pointer
        end else if (write_en) begin
            // *** MODIFICATION: Removed "&& write_addr != 5'b0" check ***
            // Now allows writes to register 0 if write_en is high and write_addr is 0
            registers[write_addr] <= write_data;
        end
    end
endmodule

module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic [63:0] out
);
    always @(*) begin
        case (ctrl)
            5'b11000: out = in1 + in2;         // add
            5'b11001: out = in1 + in2;         // addi (Opcode 0x19)
            // 5'b01010: // REMOVED - Belongs in control_unit (brr L)
            5'b11010: out = in1 - in2;         // sub
            5'b11011: out = in1 - in2;         // subi (Opcode 0x1B)
            // 5'b01011: // REMOVED - Belongs in control_unit (brnz)
            5'b11100: out = in1 * in2;         // mul
            5'b11101: out = (in2 == 0) ? 64'h0 : in1 / in2; // div
            5'b00000: out = in1 & in2;         // and
            5'b00001: out = in1 | in2;         // or
            5'b00010: out = in1 ^ in2;         // xor
            5'b00011: out = ~in1;              // not
            5'b00100: out = in1 >> in2;        // shftr
            5'b00101: out = $signed(in1) >>> in2; // shftri
            5'b00110: out = in1 << in2;        // shftl
            5'b00111: out = in1 << in2;        // shftli
            5'b10001: out = in1;               // mov $r_d, $r_s
            5'b10010: out = in2;               // mov $r_d, L
            default: out = 64'h0;
        endcase
    end
endmodule
// Instruction Decoder (Modified src1 mapping for ADDI/SUBI/SHFTRI/SHFTLI)
module inst_decoder (
    input logic [31:0] instruction,
    output logic [63:0] imm,
    output logic [4:0] dest,         // Assigned normally from dest field
    output logic [4:0] src1,         // Now conditionally assigned
    output logic [4:0] src2,         // Assigned normally from src2 field
    output logic [4:0] opcode
);
    logic [11:0] imm_raw;
    logic [4:0] opcode_internal; // Internal signal for opcode

    // Decode fields that are always the same position or mapping
    assign imm_raw = instruction[11:0];
    assign opcode_internal = instruction[31:27];
    assign opcode = opcode_internal; // Assign to output port
    assign imm = {{52{imm_raw[11]}}, imm_raw}; // Sign extend immediate
    assign dest = instruction[26:22];          // Destination register is always field [26:22]
    assign src2 = instruction[16:12];          // src2 output always comes from src2 field [16:12]

    // Determine src1 register address based on opcode
    always @(*) begin
        // Default mapping: src1 normally comes from field [21:17]
        src1 = instruction[21:17];

        // Override mapping specifically for certain I-type ALU instructions
        if (opcode_internal == 5'b11001 || // addi (Op1)
            opcode_internal == 5'b11011 || // subi (Op1)
            opcode_internal == 5'b01010 || // addi (Op2)
            opcode_internal == 5'b01011 || // subi (Op2)
            opcode_internal == 5'b00101 || // shftri <-- ADDED
            opcode_internal == 5'b00111 )  // shftli <-- ADDED
        begin
            // For these I-type ops: Use the destination register field [26:22] as src1 address
            src1 = instruction[26:22];
        end
        // For all other opcodes (R-type, branches, loads, stores, etc.), the default mapping applies.
    end

endmodule

module reg_lit_mux (
    input logic [4:0] op,
    input logic [63:0] reg_val,
    input logic [63:0] lit_val,
    output logic [63:0] out
);
    always @(*) begin
        if (op == 5'b11001 || op == 5'b11011 ||
            op == 5'b00101 || op == 5'b00111 ||
            op == 5'b10010 ||
            op == 5'b10000 || op == 5'b10011 ||
            op == 5'b01010 || op == 5'b01011)
            out = lit_val;
        else
            out = reg_val;
    end
endmodule