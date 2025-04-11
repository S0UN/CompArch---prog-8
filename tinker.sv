// Top-Level Module - MODIFIED with call/return fix and branch FSM fix
module tinker_core (
    input logic clk,
    input logic reset,
    output logic hlt // Output signal
);
    // --- Constants ---
    localparam HALT_OPCODE = 5'b11111;
    localparam CALL_OPCODE = 5'b01100; // Opcode for call
    localparam RET_OPCODE  = 5'b01101; // Opcode for return

    // --- State Machine Type ---
    typedef enum logic [2:0] {
        S_FETCH, S_DECODE, S_EXECUTE, S_MEMORY, S_WRITEBACK, S_HALTED
    } state_t;

    // --- Internal Signal Declarations ---
    state_t current_state, next_state;
    logic [63:0] pc_current;
    logic [63:0] pc_next;
    logic        pc_write_enable;
    logic [31:0] instr_word;
    logic [31:0] instr_reg;
    logic [4:0]  dest_reg;
    logic [4:0]  src_reg1;
    logic [4:0]  src_reg2;
    logic [4:0]  opcode;
    logic [63:0] imm_value;
    logic [63:0] dest_val;
    logic [63:0] src_val1;
    logic [63:0] src_val2;
    logic [63:0] stack_ptr;
    logic        reg_write;
    logic        mem_to_reg;
    logic [63:0] alu_operand2;
    logic [63:0] alu_output;
    logic [63:0] alu_out_reg;
    logic [63:0] mem_addr;
    logic [63:0] mem_data_in;
    logic [63:0] mem_data_out;
    logic [63:0] mem_data_reg;
    logic        mem_read;
    logic        mem_write;
    logic        ir_write;
    logic [63:0] memory_unit_address;
    logic        is_branch_instr;
    logic        branch_taken;
    logic [63:0] return_addr_reg; // Register to store return address for call

    // --- State Register ---
    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= S_FETCH;
        else if (current_state == S_HALTED)
             current_state <= S_HALTED;
        else
            current_state <= next_state;
    end

    // --- Next State Logic (Corrected Execute Transition for Branches) ---
    always @(*) begin
        next_state = current_state;
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
                     next_state = S_MEMORY; // Memory ops (load, store, call, return) go to MEMORY
                 end else if (is_branch_instr) begin
                     // Branches: If taken, go FETCH. If not taken, go WB.
                     if (branch_taken) begin
                         next_state = S_FETCH;
                     end else begin
                         next_state = S_WRITEBACK; // Non-taken branches go to WB
                     end
                 end else begin // Non-branch, non-memory ALU/move ops
                     next_state = S_WRITEBACK;
                 end
            end
            S_MEMORY:    next_state = S_WRITEBACK;
            S_WRITEBACK: next_state = S_FETCH; // All ops completing WB go to FETCH
            S_HALTED:    next_state = S_HALTED;
            default:     next_state = S_FETCH;
        endcase
    end

    // --- Control Signals (Modified for call/return PC update) ---
    always @(*) begin
        ir_write = 1'b0;
        pc_write_enable = 1'b0;
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0;
        hlt = 1'b0;
        memory_unit_address = mem_addr; // Default unless overridden

        case (current_state)
            S_FETCH: begin
                ir_write = 1'b1;
                mem_read = 1'b1;
                memory_unit_address = pc_current;
            end
            S_DECODE: begin
            end
            S_EXECUTE: begin
                // Enable PC write for TAKEN branches/jumps/call, BUT NOT for return
                if (is_branch_instr && branch_taken && (opcode != RET_OPCODE)) begin
                    pc_write_enable = 1'b1;
                end
            end
            S_MEMORY: begin
                // is_load_operation covers Load (10000) and Return (01101)
                // is_store_operation covers Store (10011) and Call (01100)
                if (is_load_operation(opcode)) begin
                    mem_read = 1'b1;
                end else if (is_store_operation(opcode)) begin
                    mem_write = 1'b1;
                end
            end
            S_WRITEBACK: begin
                // REG WRITE ENABLE
                if (!is_store_operation(opcode) && !is_branch_no_writeback(opcode) && opcode != HALT_OPCODE) begin
                    reg_write = 1'b1;
                end
                // WRITEBACK SOURCE SELECT
                if (is_load_operation(opcode)) begin // Includes return
                    mem_to_reg = 1'b1;
                end else begin
                    mem_to_reg = 1'b0;
                end
                // PC WRITE ENABLE
                // Enable for sequential instructions OR non-taken branches OR return instruction
                if (!branch_taken || (opcode == RET_OPCODE)) begin
                    pc_write_enable = 1'b1;
                end
            end
            S_HALTED: begin
                hlt = 1'b1;
            end
            default: begin
            end
        endcase
    end

    // --- Helper Functions ---
    function logic is_memory_operation(logic [4:0] op);
         return (op == 5'b10000 || op == 5'b10011 || op == 5'b01100 || op == 5'b01101); // load, store, call, return
    endfunction
    function logic is_load_operation(logic [4:0] op);
         return (op == 5'b10000 || op == 5'b01101); // Load, Ret
    endfunction
    function logic is_store_operation(logic [4:0] op);
        return (op == 5'b10011 || op == 5'b01100); // Store, Call
    endfunction
    function logic is_branch_no_writeback(logic [4:0] op);
         // Branches/Jumps/Call/Return usually don't write back like ALU ops
         return (op >= 5'b01000 && op <= 5'b01110) || op == HALT_OPCODE; // Opcodes 8-14 and Halt
    endfunction

    // --- Pipeline Registers ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_reg <= 32'b0;
        end else if (ir_write) begin
            instr_reg <= instr_word;
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_data_reg <= 64'b0;
        end else if (current_state == S_MEMORY && mem_read) begin
             mem_data_reg <= mem_data_out;
        end
    end

    always @(posedge clk or posedge reset) begin
       if (reset) begin
           alu_out_reg <= 64'b0;
       end else if (current_state == S_EXECUTE) begin
            alu_out_reg <= alu_output;
       end
    end

    // NEW: Return Address Register for CALL
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            return_addr_reg <= 64'b0;
        // Latch PC+4 during EXECUTE stage if it's a CALL instruction
        // Use PC *current* in Execute stage, which is the address of the CALL instr.
        end else if (current_state == S_EXECUTE && opcode == CALL_OPCODE) begin
            return_addr_reg <= pc_current + 4;
        end
        // Holds value otherwise
    end

    // --- Instantiations ---
    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_next),
        .pc_out(pc_current),
        .pc_write(pc_write_enable)
    );

    memory_unit memory (
        .program_counter(pc_current),
        .clk(clk),
        .reset(reset),
        .read_en(mem_read),
        .write_en(mem_write),
        .data_in(mem_data_in),
        .address(memory_unit_address),
        .data_out(mem_data_out),
        .instruction(instr_word)
    );

    // Use the CORRECTED control unit
    control_unit ctrl (
        .operation(opcode),
        .dest_in(dest_val),
        .src_in1(src_val1),
        .src_in2(src_val2),
        .immediate(imm_value),
        .current_pc(pc_current),
        .memory_data(mem_data_reg), // Used by return
        .next_pc(pc_next),
        .is_branch(is_branch_instr),
        .branch_taken(branch_taken)
    );

    // Use the MODIFIED memory handler
    mem_handler mem_mgr (
        .op(opcode),
        .dest(dest_val),
        .src(src_val1),
        .imm(imm_value),
        .pc(pc_current),
        .r31(stack_ptr),
        .return_addr_in(return_addr_reg), // Changed input name
        .addr_out(mem_addr),
        .data_out(mem_data_in)
    );

    // Use the standard instruction decoder
    inst_decoder dec (
        .instruction(instr_reg),
        .imm(imm_value),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    // Use the NON-HARDWIRED R0 register file
    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .write_en(reg_write),
        .write_data(mem_to_reg ? mem_data_reg : alu_out_reg),
        .addr1(src_reg1),
        .addr2(src_reg2),
        .write_addr(dest_reg),
        .data1(src_val1),
        .data2(src_val2),
        .data_dest(dest_val),
        .stack(stack_ptr)
    );

    reg_lit_mux mux (
        .op(opcode),
        .reg_val(src_val2),
        .lit_val(imm_value),
        .out(alu_operand2)
    );

    // Use the ALU unit without branch opcodes
    alu_unit alu (
        .ctrl(opcode),
        .in1(src_val1),
        .in2(alu_operand2),
        .out(alu_output)
    );

endmodule

// ================================================
// Supporting Modules Definitions
// ================================================

// Fetch Unit (Unchanged)
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic pc_write,
    input logic [63:0] pc_in,
    output logic [63:0] pc_out
);
    logic [63:0] pc_reg;
    assign pc_out = pc_reg;
    always @(posedge clk or posedge reset) begin
        if (reset) pc_reg <= 64'h2000;
        else if (pc_write) pc_reg <= pc_in;
    end
endmodule

// Memory Unit (Unchanged)
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
    always @(posedge clk or posedge reset) begin
        if (reset) for (j = 0; j < MEM_SIZE_BYTES; j = j + 1) bytes[j] <= 8'h0;
        else if (write_en) for (k = 0; k < 8; k = k + 1) if ((address + k) < MEM_SIZE_BYTES) bytes[address + k] <= data_in[8*k +: 8];
    end
endmodule

// Control Unit (Corrected based on Manual)
module control_unit (
    input logic [4:0] operation,
    input logic [63:0] dest_in,     // Value of Rd register
    input logic [63:0] src_in1,     // Value of Rs1/Rs register
    input logic [63:0] src_in2,     // Value of Rs2/Rt register
    input logic [63:0] immediate,   // Decoded immediate
    input logic [63:0] current_pc,  // Current PC value
    input logic [63:0] memory_data, // Data read from stack for return
    output logic [63:0] next_pc,
    output logic is_branch,         // Is it a branch/jump/call/return?
    output logic branch_taken       // Is the branch taken (PC != PC+4)?
);
    logic branch_condition_met;

    assign is_branch = (operation >= 5'b01000 && operation <= 5'b01110); // Opcodes 8 through 14 inclusive

    always @(*) begin
        case(operation)
            5'b01011: branch_condition_met = (src_in1 != 0); // brnz check Rs1
            5'b01110: branch_condition_met = ($signed(src_in1) > $signed(src_in2)); // brgt check Rs1 > Rs2
            default: branch_condition_met = 1'b1;
        endcase

        branch_taken = 1'b0;
        next_pc = current_pc + 4;

        case (operation)
            5'b01000: begin next_pc = dest_in;     branch_taken = 1'b1; end // br $rd
            5'b01001: begin next_pc = current_pc + dest_in;    branch_taken = 1'b1; end // brr $rd
            5'b01010: begin next_pc = current_pc + immediate;  branch_taken = 1'b1; end // brr L
            5'b01011: begin // brnz $rd, $rs
                if (branch_condition_met) begin next_pc = dest_in; branch_taken = 1'b1; end
            end
            5'b01100: begin next_pc = dest_in;     branch_taken = 1'b1; end // call $rd
            5'b01101: begin next_pc = memory_data; branch_taken = 1'b1; end // return
            5'b01110: begin // brgt $rd, $rs, $rt
                if (branch_condition_met) begin next_pc = dest_in; branch_taken = 1'b1; end
            end
            default: begin end // Keep defaults
        endcase
    end
endmodule

// Memory Handler (Modified for call)
module mem_handler (
    input logic [4:0] op,
    input logic [63:0] dest,
    input logic [63:0] src,
    input logic [63:0] imm,
    input logic [63:0] pc,
    input logic [63:0] r31,          // Value of R31 (stack pointer)
    input logic [63:0] return_addr_in, // Latched return address for call
    output logic [63:0] addr_out,
    output logic [63:0] data_out
);
    always @(*) begin
        addr_out = 64'h0;
        data_out = 64'h0;
        case (op)
            5'b01100: begin addr_out = r31 - 8; data_out = return_addr_in; end // call: Use latched RA
            5'b01101: begin addr_out = r31 - 8; data_out = 64'h0; end // return: Provide read address
            5'b10000: begin addr_out = src + imm; data_out = 64'h0; end // load: Provide read address
            5'b10011: begin addr_out = dest + imm; data_out = src; end // store: Provide write address and data
            default: begin end
        endcase
    end
endmodule

// Register File (R0 NOT Hardwired)
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

    assign data1 = registers[addr1]; // Read directly
    assign data2 = registers[addr2]; // Read directly
    assign data_dest = registers[write_addr]; // Read directly
    assign stack = registers[31];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize ALL registers r0-r30 to 0
            for (i = 0; i < 31; i = i + 1) begin registers[i] <= 64'h0; end
            registers[31] <= MEMSIZE;
        end else if (write_en) begin // Allow writing to r0
            registers[write_addr] <= write_data;
        end
    end
endmodule

// ALU Unit (Corrected - No branch opcodes)
module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic [63:0] out
);
    always @(*) begin
        case (ctrl)
            5'b11000: out = in1 + in2;         // add
            5'b11001: out = in1 + in2;         // addi
            5'b11010: out = in1 - in2;         // sub
            5'b11011: out = in1 - in2;         // subi
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

// Instruction Decoder (Standard Version)
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
    assign imm = {{52{imm_raw[11]}}, imm_raw};
endmodule

// Register/Literal Mux (Corrected based on ALU opcodes)
module reg_lit_mux (
    input logic [4:0] op,
    input logic [63:0] reg_val, // Connected to src_val2
    input logic [63:0] lit_val, // Connected to imm_value
    output logic [63:0] out    // Connected to alu_operand2
);
    always @(*) begin
        // Select Literal for I-type ALU, Load/Store offset, mov L
        // Note: Does NOT include branch immediates like brr L
        if (op == 5'b11001 || // addi
            op == 5'b11011 || // subi
            op == 5'b00101 || // shftri
            op == 5'b00111 || // shftli
            op == 5'b10010 || // mov $rd, L
            op == 5'b10000 || // Load base+offset
            op == 5'b10011 )  // Store base+offset
            out = lit_val;
        else
            out = reg_val; // Use register value (src2) otherwise
    end
endmodule