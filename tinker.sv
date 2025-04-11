
// Top-Level Module
module tinker_core (
    input logic clk,
    input logic reset
    // Add halt output if needed: output logic hlt
);
    // State machine states
    typedef enum logic [2:0] {
        FETCH,
        DECODE,
        EXECUTE,
        MEMORY,
        WRITEBACK
    } state_t;

    state_t current_state, next_state;

    // Instruction components
    logic [4:0] dest_reg, src_reg1, src_reg2, opcode;
    logic [31:0] instr_word, instr_reg; // instr_word from mem, instr_reg latched

    // Program counter signals
    logic [63:0] pc_current; // Output from fetch_unit
    logic [63:0] pc_next;    // Input to fetch_unit (calculated by control_unit)
    logic pc_write_enable;   // Enable signal for fetch_unit PC latch

    // Register values
    logic [63:0] imm_value, dest_val, src_val1, src_val2, alu_operand2;
    logic [63:0] alu_output, alu_out_reg; // alu_output combinational, alu_out_reg latched

    // Memory signals
    logic [63:0] stack_ptr, mem_data_in, mem_addr, mem_data_out, mem_data_reg; // mem_data_out combinational, mem_data_reg latched
    logic mem_read, mem_write;

    // Control signals
    logic ir_write, reg_write;
    logic reg_dst, alu_src; // Note: reg_dst, alu_src seem unused in this version's logic
    logic mem_to_reg;
    logic is_branch_instr, branch_taken; // From control_unit

    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= FETCH;
        else
            current_state <= next_state;
    end

    // Next state logic
    always_comb begin
        // Default to current state (for safety, though most states transition)
        next_state = current_state;
        case (current_state)
            FETCH:    next_state = DECODE;
            DECODE:   next_state = EXECUTE;
            EXECUTE:  next_state = state_t'(is_memory_operation(opcode) ? MEMORY :
                                           (is_branch_instr ? FETCH : WRITEBACK)); // Added cast
            MEMORY:   next_state = WRITEBACK;
            WRITEBACK: next_state = FETCH;
            default:  next_state = FETCH; // Default back to FETCH
        endcase
    end

    // Control signals based on state
    always_comb begin
        // Default values
        ir_write = 1'b0;
        pc_write_enable = 1'b0; // PC write enable default off
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0; // Default writeback from ALU

        case (current_state)
            FETCH: begin
                // Latch instruction read from memory address PC
                ir_write = 1'b1;
                // Tell memory unit to read instruction at PC
                mem_read = 1'b1; // Use read port for instruction fetch too
            end

            DECODE: begin
                // Combinational decode using instr_reg
                // Read registers combinatorially via reg_file instance
            end

            EXECUTE: begin
                // ALU performs operation combinatorially
                // Branch decision made by control unit combinatorially
                if (is_branch_instr && branch_taken) begin
                    pc_write_enable = 1'b1; // Update PC immediately with branch target
                end
            end

            MEMORY: begin
                // Perform Load or Store based on opcode
                if (is_load_operation(opcode)) begin
                    mem_read = 1'b1; // Read from data memory at mem_addr
                end else if (is_store_operation(opcode)) begin
                    mem_write = 1'b1; // Write to data memory at mem_addr
                end
            end

            WRITEBACK: begin
                // Enable register write if needed
                if (!is_store_operation(opcode) && !is_branch_no_writeback(opcode)) begin
                    reg_write = 1'b1;
                end

                // Determine source for writeback data
                if (is_load_operation(opcode)) begin
                    mem_to_reg = 1'b1; // Write back from memory data register
                end else begin
                    mem_to_reg = 1'b0; // Write back from ALU output register
                end

                // Update PC for the *next* instruction if no branch was taken earlier
                // The actual value pc_next comes from control_unit (PC+4)
                if (!branch_taken) begin
                    pc_write_enable = 1'b1;
                end
            end
            default: begin // Handles potential X state during reset/init
                ir_write = 1'b0;
                pc_write_enable = 1'b0;
                reg_write = 1'b0;
                mem_read = 1'b0;
                mem_write = 1'b0;
                mem_to_reg = 1'b0;
            end
        endcase
    end

    // Helper functions (can be placed outside always_comb)
    function logic is_memory_operation(logic [4:0] op);
        // mov $rd, ($rs)(L) OR mov ($rd)(L), $rs
        return (op == 5'b10000 || op == 5'b10011 || op == 5'b01100 || op == 5'b01101); // Add Call/Ret
    endfunction

    function logic is_load_operation(logic [4:0] op);
        // mov $rd, ($rs)(L) OR return (reads stack)
        return (op == 5'b10000 || op == 5'b01101);
    endfunction

    function logic is_store_operation(logic [4:0] op);
        // mov ($rd)(L), $rs OR call (writes stack)
        return (op == 5'b10011 || op == 5'b01100);
    endfunction

    function logic is_branch_no_writeback(logic [4:0] op);
        // Any branch/jump/call/ret instruction that doesn't write Rd normally
        return (op >= 5'b01000 && op <= 5'b01110); // All branch/jump/call/ret instructions
    endfunction

    // --- Datapath Components and Registers ---

    // Instruction Register
    always_ff @(posedge clk) begin
        // Latch instruction fetched from memory during FETCH state
        if (ir_write) begin // Only write when ir_write is asserted in FETCH
            instr_reg <= instr_word;
        end
    end

    // Memory Data Register
    always_ff @(posedge clk) begin
        // Latch data read from memory during MEMORY state (for Loads/Returns)
        // Check if mem_read was active in the *previous* cycle (MEMORY state)
        // Simpler: Latch whenever mem_read is high and it's a load/ret op? No, latch in MEMORY state.
        if (current_state == MEMORY && mem_read) begin // Latch if reading in MEMORY state
             mem_data_reg <= mem_data_out;
        end
    end

    // ALU Output Register
    always_ff @(posedge clk) begin
         // Latch ALU result at the end of EXECUTE state
        if (current_state == EXECUTE) begin
            alu_out_reg <= alu_output;
        end
    end

    // Fetch unit (Instantiated PC register)
    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        // .stack(stack_ptr), // stack input removed from fetch_unit
        .pc_in(pc_next),          // Value for next PC (from control_unit)
        .pc_out(pc_current),      // Current PC value
        .pc_write(pc_write_enable) // Enable PC update (from FSM)
    );

    // Memory unit with read/write control
    memory_unit memory (
        .program_counter(pc_current), // Not strictly needed if address is always used
        .clk(clk),
        .reset(reset),
        .read_en(mem_read),         // Read Enable from FSM
        .write_en(mem_write),       // Write Enable from FSM
        .data_in(mem_data_in),      // Data source for stores (from mem_handler)
        .address(current_state == FETCH ? pc_current : mem_addr), // Use PC for fetch, mem_addr otherwise
        .data_out(mem_data_out),    // Raw data output from memory
        .instruction(instr_word)    // Raw instruction output from memory
    );

    // Control unit for managing branching and next PC value
    control_unit ctrl (
        .operation(opcode),         // Decoded opcode
        .dest_in(dest_val),         // Value of Rd (read from regfile) - For BR/CALL $rd target
        .src_in1(src_val1),         // Value of Rs1 (read from regfile) - For condition check
        .src_in2(src_val2),         // Value of Rs2 (read from regfile) - For condition check
        .immediate(imm_value),      // Decoded immediate
        .current_pc(pc_current),    // Current PC
        .memory_data(mem_data_reg), // Use latched memory data for RET target
        .next_pc(pc_next),          // OUTPUT: Calculated next PC value (PC+4 or target)
        .is_branch(is_branch_instr),// OUTPUT: True if instruction is branch/jump type
        .branch_taken(branch_taken) // OUTPUT: True if conditional branch is taken
    );

    // Memory handler computes address/data for memory ops
    mem_handler mem_mgr (
        // .state(current_state), // State no longer needed? Maybe for CALL/RET logic
        .op(opcode),
        .dest(dest_val),         // Value of Rd register (for store addr base)
        .src(src_val1),          // Value of Rs1 register (for load addr base, or store data)
        .imm(imm_value),         // Immediate offset
        .pc(pc_current),         // Needed for CALL return address calculation
        .r31(stack_ptr),         // Stack pointer value from regfile
        .addr_out(mem_addr),     // OUTPUT: Address for MEMORY stage
        .data_out(mem_data_in)   // OUTPUT: Data for MEMORY stage (stores/call)
        // Removed reg_en output as writeback source decided by FSM (mem_to_reg)
    );

    // Instruction decoder uses the instruction register
    inst_decoder dec (
        .instruction(instr_reg), // Use latched instruction register
        .imm(imm_value),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    // Register file with control signals for write enable
    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .write_en(reg_write), // Controlled directly by FSM reg_write signal
        .write_data(mem_to_reg ? mem_data_reg : alu_out_reg), // Select data source via FSM mem_to_reg
        .addr1(src_reg1),         // Read address 1 (from instr_reg)
        .addr2(src_reg2),         // Read address 2 (from instr_reg)
        .write_addr(dest_reg),    // Write address (from instr_reg)
        .data1(src_val1),         // OUTPUT: Read data 1
        .data2(src_val2),         // OUTPUT: Read data 2
        .data_dest(dest_val),     // OUTPUT: Current value of dest reg (read addr = write addr)
        .stack(stack_ptr)         // OUTPUT: Current value of R31
    );

    // Multiplexer for ALU second operand
    reg_lit_mux mux (
        .op(opcode),
        .reg_val(src_val2),      // Value of Rs2 register
        .lit_val(imm_value),     // Decoded immediate
        .out(alu_operand2)       // OUTPUT: Second operand for ALU
    );

    // ALU unit - computes result combinatorially during EXECUTE
    alu_unit alu (
        .ctrl(opcode),           // Decoded opcode
        .in1(src_val1),          // Value of Rs1 register
        .in2(alu_operand2),      // Output of reg_lit_mux
        .out(alu_output)         // OUTPUT: Combinational ALU result
    );

endmodule


//------------------------------------------------------------------------------
// Supporting Modules (Modified where needed)
//------------------------------------------------------------------------------


// Modified Fetch Unit with PC write enable
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic pc_write,        // Enable signal from FSM
    // input logic [63:0] stack, // Removed stack input
    input logic [63:0] pc_in,     // Next PC value from control_unit
    output logic [63:0] pc_out    // Current PC value output
);
    // Internal PC register
    logic [63:0] pc_reg;
    assign pc_out = pc_reg; // Output the current PC

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_reg <= 64'h2000; // Default starting address for program
        end else if (pc_write) begin
            pc_reg <= pc_in;   // Update PC with the calculated next PC value when enabled
        end
        // If pc_write is low, PC holds its value
    end
endmodule

// Modified Memory Unit with read enable
module memory_unit (
    input logic [63:0] program_counter, // Unused if address port handles fetch too
    input logic clk,
    input logic reset,
    input logic read_en,         // FSM controls read
    input logic write_en,        // FSM controls write
    input logic [63:0] data_in,
    input logic [63:0] address,     // Used for both Inst Fetch and Data Access
    output logic [63:0] data_out,   // Combinational data output
    output logic [31:0] instruction // Combinational instruction output
);
    // Reduced memory size for faster simulation init if needed
    localparam MEM_SIZE_BYTES = 32768; // 32KB
    logic [7:0] bytes [0:MEM_SIZE_BYTES-1];
    integer j, k;

    // Instruction and data outputs (Combinational Reads based on address)
    // Use address input directly for both fetch and data reads
    assign instruction[7:0]   = (address + 0 < MEM_SIZE_BYTES) ? bytes[address + 0] : 8'h0;
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

    // Memory write logic (Synchronous)
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (j = 0; j < MEM_SIZE_BYTES; j = j + 1)
                bytes[j] <= 8'h0;
        end
        else if (write_en) begin
            for (k = 0; k < 8; k = k + 1) begin
                 if ((address + k) < MEM_SIZE_BYTES) begin // Check bounds
                    bytes[address + k] <= data_in[8*k +: 8];
                 end
            end
        end
    end
endmodule

// Modified Control Unit with branch detection
module control_unit (
    input logic [4:0] operation,
    input logic [63:0] dest_in,     // Value of Rd (for BR/CALL target)
    input logic [63:0] src_in1,     // Value of Rs1 (for condition)
    input logic [63:0] src_in2,     // Value of Rs2 (for condition)
    input logic [63:0] immediate,   // Immediate value
    input logic [63:0] current_pc,  // Current PC value
    input logic [63:0] memory_data, // Latched data from memory (for RET target)
    output logic [63:0] next_pc,    // Calculated next PC value
    output logic is_branch,         // Is it a branch/jump type?
    output logic branch_taken       // Is the branch condition met?
);
    logic branch_condition_met;

    assign is_branch = (operation >= 5'b01000 && operation <= 5'b01110);

    always_comb begin
        // Determine if condition is met (only relevant for conditional branches)
        case(operation)
            5'b01011: branch_condition_met = (src_in1 != 0); // brnz $rd, $rs (check Rs1)
            5'b01110: branch_condition_met = ($signed(src_in1) > $signed(src_in2)); // brgt $rd, $rs1, $rs2 (check Rs1 > Rs2)
            default: branch_condition_met = 1'b1; // Assume true for unconditional jumps/calls/rets
        endcase

        // Determine next PC value and if branch is effectively taken
        branch_taken = 1'b0; // Default not taken
        next_pc = current_pc + 4; // Default next sequential PC

        case (operation)
            // Unconditional Jumps / Call / Ret
            5'b01000: begin  // br $rd
                next_pc = dest_in;      // Target is value of Rd
                branch_taken = 1'b1;
            end
            5'b01100: begin  // call $rd
                next_pc = dest_in;      // Target is value of Rd (PC update happens in MEMORY state)
                branch_taken = 1'b1;    // Mark as taken for FSM logic
            end
            5'b01101: begin  // return
                next_pc = memory_data;  // Target is read from stack (using latched MDR)
                branch_taken = 1'b1;    // Mark as taken for FSM logic
            end
             // Relative Jumps
            5'b01001: begin  // brr $r_d (PC + Reg[Rd]) - Assuming Rd holds offset
                next_pc = current_pc + dest_in;
                branch_taken = 1'b1;
            end
            5'b01010: begin  // brr L (PC + Imm)
                next_pc = current_pc + $signed(immediate);
                branch_taken = 1'b1;
            end
             // Conditional Branches
            5'b01011: begin  // brnz $rd, $rs (Target is Rd value if Rs1 != 0)
                if (branch_condition_met) begin
                    next_pc = dest_in;  // Target is value of Rd
                    branch_taken = 1'b1;
                end else begin
                    next_pc = current_pc + 4; // Not taken
                    branch_taken = 1'b0;
                end
            end
            5'b01110: begin  // brgt $rd, $rs1, $rs2 (Target is Rd value if Rs1 > Rs2)
                if (branch_condition_met) begin
                    next_pc = dest_in;  // Target is value of Rd
                    branch_taken = 1'b1;
                end else begin
                    next_pc = current_pc + 4; // Not taken
                    branch_taken = 1'b0;
                end
            end
             // Default for non-branch instructions
            default: begin
                next_pc = current_pc + 4;
                branch_taken = 0;
            end
        endcase
    end
endmodule

// Modified Memory Handler
module mem_handler (
    // input state_t state, // Removed state input
    input logic [4:0] op,
    input logic [63:0] dest, // Value of Rd (for store base)
    input logic [63:0] src,  // Value of Rs1 (for load base, or store data)
    input logic [63:0] imm,
    input logic [63:0] pc,
    input logic [63:0] r31,  // Stack pointer value
    output logic [63:0] addr_out, // Address for memory stage
    output logic [63:0] data_out  // Data to write in memory stage
);
    always_comb begin
        // Default assignments
        addr_out = 64'h0; // Default address
        data_out = 64'h0; // Default data

        case (op)
            5'b01100: begin  // call $rd
                addr_out = r31 - 8; // Stack address
                data_out = pc + 4;  // Data to write (Return Address)
            end
            5'b01101: begin  // return
                addr_out = r31 - 8; // Stack address (to read from)
                data_out = 64'h0;   // No data to write
            end
            5'b10000: begin  // mov $r_d, ($r_s)(L) ; Load
                addr_out = src + imm; // Calculate address: Reg[Rs1] + Immediate
                data_out = 64'h0;   // No data to write
            end
            5'b10011: begin  // mov ($r_d)(L), $r_s ; Store
                addr_out = dest + imm; // Calculate address: Reg[Rd] + Immediate
                data_out = src;      // Data to write comes from Reg[Rs1]
            end
            default: begin
                // Keep defaults
            end
        endcase
    end
endmodule

// Simplified Register File (Interface Adjusted)
module reg_file_bank (
    input logic clk,
    input logic reset,
    input logic write_en,      // From FSM
    input logic [63:0] write_data, // From MUX controlled by FSM
    input logic [4:0] addr1,     // Read Address 1 (Rs1)
    input logic [4:0] addr2,     // Read Address 2 (Rs2)
    input logic [4:0] write_addr,  // Write Address (Rd)
    output logic [63:0] data1,     // Output Read Data 1
    output logic [63:0] data2,     // Output Read Data 2
    output logic [63:0] data_dest, // Output value currently at write_addr (for potential forwarding/use)
    output logic [63:0] stack      // Output R31 value
);
    logic [63:0] registers [0:31];
    integer i;

    // Combinational Read Ports (R0 hardwired to 0)
    assign data1 = (addr1 == 5'b0) ? 64'b0 : registers[addr1];
    assign data2 = (addr2 == 5'b0) ? 64'b0 : registers[addr2];
    assign data_dest = (write_addr == 5'b0) ? 64'b0 : registers[write_addr];
    assign stack = registers[31];

    // Synchronous Write Port
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) begin
                registers[i] <= 64'h0;
            end
            registers[31] <= 64'd32768;  // Smaller default stack pointer if memory size reduced
        end
        // R0 is hardwired to 0, cannot be written.
        else if (write_en && write_addr != 5'b0) begin
            registers[write_addr] <= write_data;
        end
    end
endmodule

// ALU Unit (Interface unchanged, logic simplified if float removed)
module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic [63:0] out // Removed 'valid' output
);
    // --- Floating Point Removed for better synthesizability/simplicity ---
    // real f1, f2, fres;
    // assign f1 = $bitstoreal(in1);
    // assign f2 = $bitstoreal(in2);

    always_comb begin
        case (ctrl)
            5'b11000: out = in1 + in2;         // add
            5'b11001: out = in1 + in2;         // addi
            5'b11010: out = in1 - in2;         // sub
            5'b11011: out = in1 - in2;         // subi
            5'b11100: out = in1 * in2;         // mul
            5'b11101: out = (in2 == 0) ? 64'h0 : in1 / in2; // div (handle div by zero)
            5'b00000: out = in1 & in2;         // and
            5'b00001: out = in1 | in2;         // or
            5'b00010: out = in1 ^ in2;         // xor
            5'b00011: out = ~in1;              // not
            5'b00100: out = in1 >> in2;        // shftr (logical)
            5'b00101: out = $signed(in1) >>> in2; // shftri (arithmetic)
            5'b00110: out = in1 << in2;        // shftl
            5'b00111: out = in1 << in2;        // shftli
            5'b10001: out = in1;               // mov $r_d, $r_s
            // mov $r_d, L: ALU just passes immediate (in2) through
            5'b10010: out = in2;
            // --- Floating Point Removed ---
            // 5'b10100: begin fres = f1 + f2; out = $realtobits(fres); end // addf
            // 5'b10101: begin fres = f1 - f2; out = $realtobits(fres); end // subf
            // 5'b10110: begin fres = f1 * f2; out = $realtobits(fres); end // mulf
            // 5'b10111: begin fres = (f2==0.0) ? 64'h0 : f1 / f2; out = $realtobits(fres); end // divf
            default: out = 64'h0; // Default for non-ALU ops (branches, mem ops, etc.)
        endcase
    end
endmodule

// Instruction Decoder (Unchanged)
module inst_decoder (
    input logic [31:0] instruction,
    output logic [63:0] imm,
    output logic [4:0] dest,
    output logic [4:0] src1,
    output logic [4:0] src2,
    output logic [4:0] opcode
);
    logic [11:0] imm_raw;
    assign imm_raw = instruction[11:0];
    assign opcode = instruction[31:27];
    assign dest = instruction[26:22];
    assign src1 = instruction[21:17];
    assign src2 = instruction[16:12];
    assign imm = {{52{imm_raw[11]}}, imm_raw}; // Sign extend
    // Removed src1=dest modification - assumes ISA handles this if needed
endmodule

// Register/Literal Mux (Unchanged)
module reg_lit_mux (
    input logic [4:0] op,
    input logic [63:0] reg_val, // from src2 read port
    input logic [63:0] lit_val, // from immediate decoder
    output logic [63:0] out    // to alu operand 2
);
    always_comb begin
        // Select immediate for I-type ALU ops, Loads, Stores, Mov immediate
        // Note: Load/Store address calculation may also use immediate
        if (op == 5'b11001 || op == 5'b11011 || // addi, subi
            op == 5'b00101 || op == 5'b00111 || // shftri, shftli
            op == 5'b10010 ||                   // mov $rd, L
            op == 5'b10000 || op == 5'b10011 || // Load/Store base+offset
            op == 5'b01010                     // brr L
           )
            out = lit_val;
        else
            out = reg_val; // Otherwise use register value src2
    end
endmodule