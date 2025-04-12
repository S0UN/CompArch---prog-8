
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
    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= S_FETCH;
        else if (current_state == S_HALTED)
             current_state <= S_HALTED;
        else
            current_state <= next_state;
    end

	always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
        return_addr_reg <= 64'b0; // Reset the register
    end else if (current_state == S_DECODE && opcode == 5'b01100) begin
        return_addr_reg <= pc_current + 64'd4; // Latch PC + 4 for call
    end
end

    // --- Next State Logic ---
    always @(*) begin
        // ... (logic as before, using direct enum assignments) ...
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
                 end else if (is_branch_instr)  begin
                     next_state = S_FETCH;
                 end else begin
                     next_state = S_WRITEBACK;
                 end
            end
            S_MEMORY:    next_state = S_WRITEBACK;
            S_WRITEBACK: next_state = S_FETCH;
            S_HALTED:    next_state = S_HALTED;
            default:     next_state = S_FETCH;
        endcase
    end
    always @(*) begin // CHANGE BACK TO THIS
        // --- Default values MUST be assigned FIRST ---
        ir_write = 1'b0;
        pc_write_enable = 1'b0;
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0;
        hlt = 1'b0;
        memory_unit_address = mem_addr;

        case (current_state)
            S_FETCH: begin // OK
                ir_write = 1'b1;
                mem_read = 1'b1;
                memory_unit_address = pc_current;
            end
            S_DECODE: begin // OK
            end
            S_EXECUTE: begin // OK?
                if (is_branch_instr && branch_taken && opcode != 5'b01101) begin
                    pc_write_enable = 1'b1; // PC update for taken branch
                end
            end
            S_MEMORY: begin // OK?
                if (is_load_operation(opcode)) begin
                    mem_read = 1'b1;
                end else if (is_store_operation(opcode)) begin
                    mem_write = 1'b1;
                end
            end
            S_WRITEBACK: begin
                // REG WRITE ENABLE
                if (!is_store_operation(opcode) && !is_branch_no_writeback(opcode) && opcode != HALT_OPCODE) begin
                    reg_write = 1'b1; // Should be TRUE for ADD/AND
                end
                // WRITEBACK SOURCE SELECT
                if (is_load_operation(opcode)) begin
                    mem_to_reg = 1'b1;
                end else begin
                    mem_to_reg = 1'b0; // Should be TRUE for ADD/AND
                end
                // PC WRITE ENABLE (Sequential)
				if (opcode == 5'b01101) begin // Return: update PC with return address
                pc_write_enable = 1'b1;
           		end else if (!branch_taken) begin // branch_taken should be FALSE for ADD/AND
                    pc_write_enable = 1'b1; // Should be TRUE for ADD/AND
                end
               
            end
            S_HALTED: begin // OK
                hlt = 1'b1;
            end
            default: begin // OK
            end
        endcase
    end

    // --- Helper Functions --- (Unchanged)
    // ... (functions as before) ...
    function logic is_memory_operation(logic [4:0] op);
         return (op == 5'b10000 || op == 5'b10011 || op == 5'b01100 || op == 5'b01101);
    endfunction
    function logic is_load_operation(logic [4:0] op);
         return (op == 5'b10000 || op == 5'b01101); // Load/Ret
    endfunction
    function logic is_store_operation(logic [4:0] op);
        return (op == 5'b10011 || op == 5'b01100); // Store/Call
    endfunction
    function logic is_branch_no_writeback(logic [4:0] op);
         return (op >= 5'b01000 && op <= 5'b01110) || op == HALT_OPCODE;
    endfunction

    // Instruction Register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_reg <= 32'b0; // Reset to 0
        end else if (ir_write) begin
            instr_reg <= instr_word;
        end
    end

    // Memory Data Register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_data_reg <= 64'b0; // Reset to 0
        end else if (current_state == S_MEMORY && mem_read) begin
             mem_data_reg <= mem_data_out;
        end
    end

// ALU Output Register - Revert to simpler latch condition
    always @(posedge clk or posedge reset) begin
       if (reset) begin
           alu_out_reg <= 64'b0; // Reset to 0
       end else if (current_state == S_EXECUTE) begin // Latch whenever in EXECUTE state
            alu_out_reg <= alu_output;
       end
    end

    // --- Instantiations --- (Connections should now match declared signal widths)
    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_next),           // Should be logic [63:0]
        .pc_out(pc_current),       // Should be logic [63:0]
        .pc_write(pc_write_enable) // Should be logic
    );

    memory_unit memory (
        .program_counter(pc_current),  // Should be logic [63:0]
        .clk(clk),
        .reset(reset),
        .read_en(mem_read),            // Should be logic
        .write_en(mem_write),          // Should be logic
        .data_in(mem_data_in),         // Should be logic [63:0]
        .address(memory_unit_address), // Should be logic [63:0]
        .data_out(mem_data_out),       // Should be logic [63:0]
        .instruction(instr_word)       // Should be logic [31:0]
    );

    control_unit ctrl (
        .operation(opcode),            // Should be logic [4:0]
        .dest_in(dest_val),            // Should be logic [63:0]
        .src_in1(src_val1),            // Should be logic [63:0]
        .src_in2(src_val2),            // Should be logic [63:0]
        .immediate(imm_value),         // Should be logic [63:0]
        .current_pc(pc_current),       // Should be logic [63:0]
        .memory_data(mem_data_reg),    // Should be logic [63:0]
        .next_pc(pc_next),             // Should be logic [63:0]
        .is_branch(is_branch_instr),   // Should be logic
        .branch_taken(branch_taken)    // Should be logic
    );

	mem_handler mem_mgr (
		.op(opcode),
		.dest(dest_val),
		.src(src_val1),
		.imm(imm_value),
		.pc(pc_current),
		.r31(stack_ptr),
		.return_addr(return_addr_reg), // New connection
		.addr_out(mem_addr),
		.data_out(mem_data_in)
	);
    inst_decoder dec (
        .instruction(instr_reg),       // Should be logic [31:0]
        .imm(imm_value),               // Should be logic [63:0]
        .dest(dest_reg),               // Should be logic [4:0]
        .src1(src_reg1),               // Should be logic [4:0]
        .src2(src_reg2),               // Should be logic [4:0]
        .opcode(opcode)                // Should be logic [4:0]
    );

    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .write_en(reg_write),         // Should be logic
        .write_data(mem_to_reg ? mem_data_reg : alu_out_reg), // Input data width check
        .addr1(src_reg1),             // Should be logic [4:0]
        .addr2(src_reg2),             // Should be logic [4:0]
        .write_addr(dest_reg),        // Should be logic [4:0]
        .data1(src_val1),             // Should be logic [63:0]
        .data2(src_val2),             // Should be logic [63:0]
        .data_dest(dest_val),         // Should be logic [63:0]
        .stack(stack_ptr)             // Should be logic [63:0]
    );

    reg_lit_mux mux (
        .op(opcode),                   // Should be logic [4:0]
        .reg_val(src_val2),            // Should be logic [63:0]
        .lit_val(imm_value),           // Should be logic [63:0]
        .out(alu_operand2)             // Should be logic [63:0]
    );

    alu_unit alu (
        .ctrl(opcode),                 // Should be logic [4:0]
        .in1(src_val1),                // Should be logic [63:0]
        .in2(alu_operand2),            // Should be logic [63:0]
        .out(alu_output)               // Should be logic [63:0]
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

// Modified Memory Unit (Using always @)
module memory_unit (
    input logic [63:0] program_counter,
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

    assign instruction[7:0]   = (address + 0 < MEM_SIZE_BYTES) ? bytes[address + 0] : 8'h0;
    // ... other assign statements for instruction/data_out ...
    assign instruction[15:8]  = (address + 1 < MEM_SIZE_BYTES) ? bytes[address + 1] : 8'h0;
    assign instruction[23:16] = (address + 2 < MEM_SIZE_BYTES) ? bytes[address + 2] : 8'h0;
    assign instruction[31:24] = (address + 3 < MEM_SIZE_BYTES) ? bytes[address + 3] : 8'h0;
    assign data_out[7:0]   = (address + 0 < MEM_SIZE_BYTES) ? bytes[address + 0] : 8'h0;
    assign data_out[15:8]  = (address + 1 < MEM_SIZE_BYTES) ? bytes[address + 1] : 8'h0;
    assign data_out[23:16] = (address + 2 < MEM_SIZE_BYTES) ? bytes[address + 2] : 8'h0;
    assign data_out[31:24] = (address + 3 < MEM_SIZE_BYTES) ? bytes[address + 3] : 8'h0;
    assign data_out[39:32] = (address + 4 < MEM_SIZE_BYTES) ? bytes[address + 4] : 8'h0;
    assign data_out[47:40] = (address + 5 < MEM_SIZE_BYTES) ? bytes[address + 5] : 8'h0;
    assign data_out[55:48] = (address + 6 < MEM_SIZE_BYTES) ? bytes[address + 6] : 8'h0;
    assign data_out[63:56] = (address + 7 < MEM_SIZE_BYTES) ? bytes[address + 7] : 8'h0;


    always @(posedge clk or posedge reset) begin // Changed from always_ff
        if (reset) begin
            for (j = 0; j < MEM_SIZE_BYTES; j = j + 1)
                bytes[j] <= 8'h0;
        end
        else if (write_en) begin
            for (k = 0; k < 8; k = k + 1) begin
                 if ((address + k) < MEM_SIZE_BYTES) begin
                    bytes[address + k] <= data_in[8*k +: 8];
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

// Modified Register File (Using always @)
// Modified Register File (Using always @) - R0 NOT hardwired
module reg_file_bank (
    input logic clk,
    input logic reset,
    input logic write_en,
    input logic [63:0] write_data,
    input logic [4:0] addr1,
    input logic [4:0] addr2,
    input logic [4:0] write_addr,
    output logic [63:0] data1,
    output logic [63:0] data2,
    output logic [63:0] data_dest,
    output logic [63:0] stack
);
    logic [63:0] registers [0:31];
    integer i;
    localparam MEMSIZE = 64'd524288;

    // Modified assignments to read actual register values
    assign data1 = registers[addr1];
    assign data2 = registers[addr2];
    assign data_dest = registers[write_addr];
    assign stack = registers[31];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) begin registers[i] <= 64'h0; end
            registers[31] <= MEMSIZE;
        end else if (write_en) begin
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