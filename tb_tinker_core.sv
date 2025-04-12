// Register File Module (Standalone)
module register_file #(
    parameter R31_INIT = 524288  // Parameter for initializing r31
) (
    input logic clk,
    input logic reset,
    input logic [4:0] read_addr1,   // First read address
    input logic [4:0] read_addr2,   // Second read address
    input logic [4:0] write_addr,   // Write address
    input logic write_en,           // Write enable
    input logic [63:0] write_data,  // Data to write
    output logic [63:0] read_data1, // First read data
    output logic [63:0] read_data2, // Second read data
    output logic [63:0] r31         // Value of register 31 (stack pointer)
);
    logic [63:0] registers [0:31];  // 32 registers, 64 bits each

    // Register File Outputs (Combinational Reads)
    assign read_data1 = registers[read_addr1];
    assign read_data2 = registers[read_addr2];
    assign r31 = registers[31];

    // Register File Logic (Sequential Writes)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 31; i++) registers[i] <= 64'h0;  // Reset registers 0-30 to 0
            registers[31] <= R31_INIT;  // Initialize r31 with parameter
        end else if (write_en) begin
            registers[write_addr] <= write_data;  // Write data to specified register
        end
    end
endmodule

// Memory Unit Module (Standalone)
module memory_unit #(
    parameter MEMSIZE = 524288  // Parameter for memory size
) (
    input logic clk,
    input logic reset,
    input logic [63:0] addr,    // Memory address
    input logic [1:0] size,     // Access size (2: 32-bit, 3: 64-bit)
    input logic read_en,        // Read enable
    input logic write_en,       // Write enable
    input logic [63:0] data_in, // Data to write
    output logic [63:0] data_out // Data read
);
    logic [7:0] bytes [0:MEMSIZE-1];  // Byte-addressable memory

    // Memory Write Logic (Sequential)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Memory could be initialized here if needed (left empty for simplicity)
        end else if (write_en) begin
            case (size)
                3: begin  // 64-bit write
                    bytes[addr]   <= data_in[7:0];
                    bytes[addr+1] <= data_in[15:8];
                    bytes[addr+2] <= data_in[23:16];
                    bytes[addr+3] <= data_in[31:24];
                    bytes[addr+4] <= data_in[39:32];
                    bytes[addr+5] <= data_in[47:40];
                    bytes[addr+6] <= data_in[55:48];
                    bytes[addr+7] <= data_in[63:56];
                end
            endcase
        end
    end

    // Memory Read Logic (Combinational)
    always @(*) begin
        data_out = 64'h0;  // Default output
        if (read_en) begin
            case (size)
                2: data_out = {32'h0, bytes[addr+3], bytes[addr+2], bytes[addr+1], bytes[addr]};  // 32-bit read
                3: data_out = {bytes[addr+7], bytes[addr+6], bytes[addr+5], bytes[addr+4],
                               bytes[addr+3], bytes[addr+2], bytes[addr+1], bytes[addr]};  // 64-bit read
            endcase
        end
    end
endmodule

// Tinker Core Module (Top-Level)
module tinker_core (
    input logic clk,            // Clock input
    input logic reset,          // Reset input
    output logic hlt            // Halt output signal
);
    // Parameters
    parameter MEMSIZE = 524288;  // Memory size in bytes (example value)

    // State Enumeration
    typedef enum logic [2:0] {
        FETCH,      // Fetch instruction from memory
        DECODE,     // Decode the instruction
        EXECUTE,    // Execute ALU operation or compute address
        MEMORY,     // Perform memory read/write
        WRITEBACK,  // Write result back to register
        PC_UPDATE,  // Update program counter (e.g., for return)
        HALT        // Halt state
    } state_t;

    // Internal Signals
    logic [63:0] PC, next_pc;           // Program counter and next PC value
    logic [31:0] IR;                    // Instruction register
    logic [63:0] A, B;                  // Source operand registers
    logic [63:0] ALUOut;                // ALU output
    logic [63:0] MDR;                   // Memory data register
    state_t state, next_state;          // Current and next state
    logic pc_write;                     // PC update enable
    logic hlt_reg;                      // Halt signal register
    assign hlt = hlt_reg;               // Output halt signal

    // Instruction Fields
    logic [4:0] opcode, dest, src1, src2; // Opcode and register fields
    logic [11:0] L_field;                 // Immediate field
    logic [63:0] imm;                     // Extended immediate value

    // Control Signals
    logic [4:0] read_reg1, read_reg2, write_reg; // Register file addresses
    logic reg_write;                             // Register write enable
    logic [63:0] reg_data1, reg_data2, r31;     // Register file data
    logic [1:0] size;                            // Memory access size (2: 32-bit, 3: 64-bit)
    logic mem_read, mem_write;                   // Memory read/write enables
    logic [63:0] mem_addr, mem_data_in, mem_data_out; // Memory interface signals

    // Instruction Type Flags
    logic is_alu_reg, is_alu_imm, is_load, is_store, is_branch, is_call, is_return, is_halt;
    logic B_src;                        // Source for B (0: register, 1: immediate)

    // Instantiate Register File with Parameter
    register_file #(.R31_INIT(MEMSIZE)) reg_file (
        .clk(clk),
        .reset(reset),
        .read_addr1(read_reg1),
        .read_addr2(read_reg2),
        .write_addr(write_reg),
        .write_en(reg_write),
        .write_data(is_load ? MDR : ALUOut), // Select write data: MDR for load, ALUOut otherwise
        .read_data1(reg_data1),
        .read_data2(reg_data2),
        .r31(r31)
    );

    // Instantiate Memory Unit with Parameter
    memory_unit #(.MEMSIZE(MEMSIZE)) mem (
        .clk(clk),
        .reset(reset),
        .addr(mem_addr),
        .size(size),
        .read_en(mem_read),
        .write_en(mem_write),
        .data_in(mem_data_in),
        .data_out(mem_data_out)
    );

    // Sequential Logic (State and PC Updates)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= FETCH;         // Start in FETCH state
            PC <= 64'h2000;         // Initial PC value
            hlt_reg <= 0;           // Clear halt signal
            IR <= 32'h0;            // Clear instruction register
            A <= 64'h0;             // Clear operand A
            B <= 64'h0;             // Clear operand B
            ALUOut <= 64'h0;        // Clear ALU output
            MDR <= 64'h0;           // Clear memory data register
        end else begin
            state <= next_state;    // Update state
            if (state == FETCH && mem_read) IR <= mem_data_out[31:0]; // Load instruction
            if (state == DECODE) begin
                A <= reg_data1;     // Load first operand
                B <= (B_src == 0) ? reg_data2 : imm; // Load second operand (register or immediate)
            end
            if (state == MEMORY && mem_read) MDR <= mem_data_out; // Load memory data
            if (state == EXECUTE) ALUOut <= ALUOut_next; // Latch ALU result
            if (pc_write) PC <= next_pc; // Update PC
            if (state == DECODE && is_halt) hlt_reg <= 1; // Set halt signal
        end
    end

    // ALU Output (Combinational Logic)
    logic [63:0] ALUOut_next;
    always @(*) begin
        ALUOut_next = ALUOut; // Default to current value
        if (state == EXECUTE) begin
            case (opcode)
                5'h18, 5'h19: ALUOut_next = A + B; // add, addi
                5'h1a, 5'h1b: ALUOut_next = A - B; // sub, subi
                5'h1c: ALUOut_next = A * B; // mul
                5'h1d: ALUOut_next = A / B; // div
                5'h00: ALUOut_next = A & B; // and
                5'h01: ALUOut_next = A | B; // or
                5'h02: ALUOut_next = A ^ B; // xor
                5'h03: ALUOut_next = ~A;    // not
                5'h04, 5'h05: ALUOut_next = A >> B; // shftr, shftri
                5'h06, 5'h07: ALUOut_next = A << B; // shftl, shftli
                5'h11: ALUOut_next = A;     // mov $r_d, $r_s
                5'h12: ALUOut_next = B;     // mov $r_d, L (immediate)
                // Simplified floating-point operations
                5'h14: ALUOut_next = $realtobits($bitstoreal(A) + $bitstoreal(B)); // addf
                5'h15: ALUOut_next = $realtobits($bitstoreal(A) - $bitstoreal(B)); // subf
                5'h16: ALUOut_next = $realtobits($bitstoreal(A) * $bitstoreal(B)); // mulf
                5'h17: ALUOut_next = $realtobits($bitstoreal(A) / $bitstoreal(B)); // divf
                default: ALUOut_next = A + B; // Default to add
            endcase
            // Memory address calculation for load/store/call/return
            if (is_load || is_store || is_call || is_return) ALUOut_next = A + imm;
        end
    end

    // State Machine (Combinational Logic)
    always @(*) begin
        // Default Assignments
        next_state = state;
        pc_write = 0;
        mem_read = 0;
        mem_write = 0;
        reg_write = 0;
        size = 3; // Default to 64-bit access
        mem_addr = 64'h0;
        mem_data_in = 64'h0;
        read_reg1 = 5'h0;
        read_reg2 = 5'h0;
        write_reg = 5'h0;
        next_pc = PC + 4; // Default PC increment
        B_src = 1'b1; // Use 1-bit binary '1'
        imm = 64'h0;

        // Instruction Decoding
        opcode = IR[31:27]; // 5-bit opcode
        dest = IR[26:22];   // Destination register
        src1 = IR[21:17];   // First source register
        src2 = IR[16:12];   // Second source register
        L_field = IR[11:0]; // Immediate field

        // Instruction Type Detection
        is_alu_reg = (opcode == 5'h18 || opcode == 5'h1a || opcode == 5'h1c || opcode == 5'h1d ||
                      opcode == 5'h00 || opcode == 5'h01 || opcode == 5'h02 || opcode == 5'h03 ||
                      opcode == 5'h04 || opcode == 5'h06);
        is_alu_imm = (opcode == 5'h19 || opcode == 5'h1b || opcode == 5'h05 || opcode == 5'h07 ||
                      opcode == 5'h12);
        is_load = (opcode == 5'h10);
        is_store = (opcode == 5'h13);
        is_branch = (opcode >= 5'h08 && opcode <= 5'h0b) || (opcode == 5'h0e);
        is_call = (opcode == 5'h0c);
        is_return = (opcode == 5'h0d);
        is_halt = (opcode == 5'h0f && L_field == 12'h0); // Halt instruction

        // State Transitions
        case (state)
            FETCH: begin
                mem_addr = PC;
                size = 2; // 32-bit instruction fetch
                mem_read = 1;
                next_state = DECODE;
            end

            DECODE: begin
                imm = {52'h0, L_field}; // Zero-extend by default
                if (opcode == 5'h0a) imm = {{52{L_field[11]}}, L_field}; // Sign-extend for brr L

                // Set Register Read Addresses
                if (is_alu_reg) begin
                    read_reg1 = src1;
                    read_reg2 = src2;
                end else if (is_alu_imm) begin
                    read_reg1 = dest;
                    B_src = 1;
                end else if (is_load) begin
                    read_reg1 = src1;
                    B_src = 1;
                end else if (is_store) begin
                    read_reg1 = src1; // Data to store
                    read_reg2 = dest; // Base address
                end else if (is_branch) begin
                    case (opcode)
                        5'h08: read_reg1 = dest; // br $r_d
                        5'h09: read_reg1 = dest; // brr $r_d
                        5'h0a: ; // brr L (immediate)
                        5'h0b: begin read_reg1 = src1; read_reg2 = dest; end // brnz
                        5'h0e: begin read_reg1 = src1; read_reg2 = src2; end // brgt
                    endcase
                    if (opcode == 5'h0a) B_src = 1;
                end else if (is_call) begin
                    read_reg1 = dest; // Target address
                    read_reg2 = 5'h1f; // r31 (stack pointer)
                end else if (is_return) begin
                    read_reg1 = 5'h1f; // r31 (stack pointer)
                    B_src = 1'b1;
                    imm = -64'd8; // Offset for return address
                end

                if (is_halt) begin
                    next_state = HALT;
                end else begin
                    next_state = EXECUTE;
                end
            end

            EXECUTE: begin
                if (is_alu_reg || is_alu_imm) begin
                    write_reg = dest;
                    next_state = WRITEBACK;
                end else if (is_load || is_store) begin
                    next_state = MEMORY;
                end else if (is_branch) begin
                    case (opcode)
                        5'h08: next_pc = A; // br $r_d
                        5'h09: next_pc = PC + A; // brr $r_d
                        5'h0a: next_pc = PC + B; // brr L
                        5'h0b: next_pc = (A != 0) ? reg_data2 : PC; // brnz
                        5'h0e: next_pc = (A > B) ? reg_data2 : PC; // brgt
                    endcase
                    pc_write = 1;
                    next_state = FETCH;
                end else if (is_call) begin
                    mem_addr = r31 - 8; // Stack pointer - 8
                    mem_data_in = PC + 4; // Save return address
                    next_state = MEMORY;
                end else if (is_return) begin
                    mem_addr = A - 8; // Read return address from stack
                    next_state = MEMORY;
                end
            end

            MEMORY: begin
                if (is_load) begin
                    mem_addr = ALUOut;
                    mem_read = 1;
                    next_state = WRITEBACK;
                end else if (is_store) begin
                    mem_addr = ALUOut;
                    mem_write = 1;
                    mem_data_in = A;
                    next_state = FETCH;
                    pc_write = 1;
                end else if (is_call) begin
                    mem_write = 1;
                    next_pc = A; // Jump to target
                    pc_write = 1;
                    next_state = FETCH;
                end else if (is_return) begin
                    mem_read = 1;
                    next_state = PC_UPDATE;
                end
            end

            WRITEBACK: begin
                reg_write = 1;
                write_reg = dest;
                next_state = FETCH;
                pc_write = 1;
            end

            PC_UPDATE: begin
                next_pc = MDR; // Load return address
                pc_write = 1;
                next_state = FETCH;
            end

            HALT: begin
                next_state = HALT; // Remain halted until reset
            end
        endcase
    end
endmodule