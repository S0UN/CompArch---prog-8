//############################################################################
//## tinker_core (Top Level Module)
//############################################################################
module tinker_core (
    input clk,
    input reset,
    output hlt
);
    // --- Pipeline Stage Signals ---
    // IF Stage
    logic [63:0] pc_if;
    logic [31:0] instruction_if;

    // IF/DE Register Outputs
    logic [63:0] pc_de;
    logic [31:0] instruction_de;

    // DE Stage Outputs (Inputs to DE/EX Register)
    logic [63:0] operand_a_de;
    // CHANGED: Intermediate wire for RegFile output before Mux
    logic [63:0] regfile_operand_b_de;
    logic [63:0] operand_b_de; // Output of RegLitMux
    logic [63:0] literal_de;
    logic [4:0]  rd_addr_de;
    logic [4:0]  rs_addr_de;
    logic [4:0]  rt_addr_de;
    logic [4:0]  opcode_de;
    logic [63:0] stack_ptr_de;
    logic        alu_enable_de, mem_read_de, mem_write_de, reg_write_de, mem_to_reg_de, branch_taken_ctrl_de, mem_pc_de; // Control signals

    // DE/EX Register Outputs -> Inputs to EX Stage
    logic [63:0] pc_ex;
    logic [63:0] operand_a_ex;
    logic [63:0] operand_b_ex;
    logic [63:0] literal_ex;
    logic [4:0]  rd_addr_ex;
    logic [4:0]  rs_addr_ex;
    logic [4:0]  rt_addr_ex;
    logic [4:0]  opcode_ex;
    logic [63:0] stack_ptr_ex;
    logic        alu_enable_ex, mem_read_ex, mem_write_ex, reg_write_ex, mem_to_reg_ex, mem_pc_ex; // Control signals

    // EX Stage Outputs -> Inputs to EX/MEM Register
    logic [63:0] alu_result_ex;
    logic [63:0] alu_mem_addr_ex;
    logic [63:0] alu_mem_data_ex;
    logic [63:0] alu_branch_pc_ex;
    logic        branch_taken_ex;
    logic        hlt_ex;

    // EX/MEM Register Outputs -> Inputs to MEM Stage
    logic [63:0] alu_result_mem;
    logic [63:0] mem_addr_mem;
    logic [63:0] mem_wdata_mem;
    logic [63:0] branch_pc_mem;
    logic [4:0]  rd_addr_mem;
    logic        mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, branch_taken_mem, mem_pc_mem; // Control
    logic        hlt_mem;

    // MEM Stage Outputs
    logic [63:0] mem_rdata_mem;
    logic [63:0] return_pc_mem;

    // MEM/WB Stage
    logic [63:0] reg_wdata_wb;

    // --- Control Signals ---
    logic flush_de, flush_ex;
    logic take_return_pc_fetch;

    // --- Module Instantiations ---

    fetch instruction_fetcher (
        .clk(clk),
        .reset(reset),
        .branch_taken(branch_taken_mem),
        .branch_pc(branch_pc_mem),
        .take_return_pc(take_return_pc_fetch),
        .return_pc(return_pc_mem),
        .pc_out(pc_if)
    );

    memory memory (
        .clk(clk),
        .reset(reset),
        .inst_addr(pc_if),
        .instruction_out(instruction_if),
        .data_addr(mem_addr_mem),
        .data_wdata(mem_wdata_mem),
        .mem_read(mem_read_mem),
        .mem_write(mem_write_mem),
        .data_rdata(mem_rdata_mem)
    );

    if_de_register if_de_reg (
        .clk(clk),
        .flush(flush_de),
        .pc_in(pc_if),
        .instruction_in(instruction_if),
        .pc_out(pc_de),
        .instruction_out(instruction_de)
    );

    instructionDecoder instruction_parser (
        .instructionLine(instruction_de),
        .literal(literal_de),
        .rd(rd_addr_de),
        .rs(rs_addr_de),
        .rt(rt_addr_de),
        .opcode(opcode_de),
        .alu_enable(alu_enable_de),
        .mem_read(mem_read_de),
        .mem_write(mem_write_de),
        .reg_write(reg_write_de),
        .mem_to_reg(mem_to_reg_de),
        .branch_taken(branch_taken_ctrl_de),
        .mem_pc(mem_pc_de)
    );

    registerFile reg_file (
        .clk(clk),
        .reset(reset),
        .write_addr(rd_addr_mem),
        .write_data(reg_wdata_wb),
        .write_enable(reg_write_mem),
        .read_addr1(rs_addr_de),
        .read_data1(operand_a_de),
        .read_addr2(rt_addr_de),
        // CHANGED: Output to intermediate wire
        .read_data2(regfile_operand_b_de),
        .stack_ptr_out(stack_ptr_de)
    );

    // CHANGED: Input from intermediate wire, output drives operand_b_de
    reglitmux input_selector (
        .sel(opcode_de),
        .reg_in(regfile_operand_b_de), // Input from RegFile output wire
        .lit_in(literal_de),
        .out(operand_b_de)           // Output drives the signal going to DE/EX reg
    );

    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush_ex),
        .pc_in(pc_de),
        .operand_a_in(operand_a_de),
        .operand_b_in(operand_b_de), // Input from RegLitMux output
        .literal_in(literal_de),
        .rd_addr_in(rd_addr_de),
        .rs_addr_in(rs_addr_de),
        .rt_addr_in(rt_addr_de),
        .opcode_in(opcode_de),
        .stack_ptr_in(stack_ptr_de),
        .alu_enable_in(alu_enable_de),
        .mem_read_in(mem_read_de),
        .mem_write_in(mem_write_de),
        .reg_write_in(reg_write_de),
        .mem_to_reg_in(mem_to_reg_de),
        .branch_taken_ctrl_in(branch_taken_ctrl_de),
        .mem_pc_in(mem_pc_de),
        .pc_out(pc_ex),
        .operand_a_out(operand_a_ex),
        .operand_b_out(operand_b_ex),
        .literal_out(literal_ex),
        .rd_addr_out(rd_addr_ex),
        .rs_addr_out(rs_addr_ex),
        .rt_addr_out(rt_addr_ex),
        .opcode_out(opcode_ex),
        .stack_ptr_out(stack_ptr_ex),
        .alu_enable_out(alu_enable_ex),
        .mem_read_out(mem_read_ex),
        .mem_write_out(mem_write_ex),
        .reg_write_out(reg_write_ex),
        .mem_to_reg_out(mem_to_reg_ex),
        .branch_taken_ctrl_out(),
        .mem_pc_out(mem_pc_ex)
    );

    // CHANGED: Removed pass-through output connections for control signals
    alu calculation_unit (
        .alu_enable(alu_enable_ex),
        .opcode(opcode_ex),
        .input1(operand_a_ex),
        .input2(operand_b_ex),
        .rd_addr(rd_addr_ex),
        .literal(literal_ex),
        .pc_in(pc_ex),
        .stack_ptr(stack_ptr_ex),
        .result(alu_result_ex),
        .mem_addr(alu_mem_addr_ex),
        .mem_wdata(alu_mem_data_ex),
        .branch_pc(alu_branch_pc_ex),
        .branch_taken(branch_taken_ex),
        .hlt_out(hlt_ex),
        // Pass-through INPUTS (still needed for ALU internal logic if it were more complex)
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .mem_pc_in(mem_pc_ex)
        // Pass-through OUTPUTS REMOVED
    );

    // CHANGED: Control signal inputs connected directly from de_ex_reg outputs
    ex_mem_register ex_mem_reg (
        .clk(clk),
        .result_in(alu_result_ex),
        .mem_addr_in(alu_mem_addr_ex),
        .mem_wdata_in(alu_mem_data_ex),
        .branch_pc_in(alu_branch_pc_ex),
        .rd_addr_in(rd_addr_ex),
        .hlt_in(hlt_ex),
        .mem_read_in(mem_read_ex), // From de_ex_reg output
        .mem_write_in(mem_write_ex), // From de_ex_reg output
        .reg_write_in(reg_write_ex), // From de_ex_reg output
        .mem_to_reg_in(mem_to_reg_ex), // From de_ex_reg output
        .branch_taken_in(branch_taken_ex), // From ALU output
        .mem_pc_in(mem_pc_ex),       // From de_ex_reg output
        .result_out(alu_result_mem),
        .mem_addr_out(mem_addr_mem),
        .mem_wdata_out(mem_wdata_mem),
        .branch_pc_out(branch_pc_mem),
        .rd_addr_out(rd_addr_mem),
        .hlt_out(hlt_mem),
        .mem_read_out(mem_read_mem),
        .mem_write_out(mem_write_mem),
        .reg_write_out(reg_write_mem),
        .mem_to_reg_out(mem_to_reg_mem),
        .branch_taken_out(branch_taken_mem),
        .mem_pc_out(mem_pc_mem)
    );

    aluMemMux return_pc_selector (
        .mem_pc(mem_pc_mem),
        .memData(mem_rdata_mem),
        .aluOut(branch_pc_mem),
        .newPc(return_pc_mem)
    );

    memRegMux data_source_selector (
        .mem_to_reg(mem_to_reg_mem),
        .readData(mem_rdata_mem),
        .aluResult(alu_result_mem),
        .regWriteData(reg_wdata_wb)
    );

    assign flush_de = branch_taken_mem;
    assign flush_ex = branch_taken_mem;
    assign take_return_pc_fetch = mem_pc_mem;
    assign hlt = hlt_mem;

endmodule


//############################################################################
//## registerFile
//## CHANGED: Combinational Reads, Synchronous Write
//############################################################################
module registerFile (
    input clk, // ADDED
    input reset,
    // Write Port
    input [4:0] write_addr, // Renamed from write
    input [63:0] write_data, // Renamed from data
    input write_enable, // Renamed from regWriteFlag
    // Read Port 1
    input [4:0] read_addr1, // Renamed from read1
    output logic [63:0] read_data1, // Renamed from output1
    // Read Port 2
    input [4:0] read_addr2, // Renamed from read2
    output logic [63:0] read_data2, // Renamed from output2
    // Stack Pointer Output
    output logic [63:0] stack_ptr_out // Renamed from stackPtr
);
    reg [63:0] registers [0:31];
    integer idx;

    // Initialize registers
    initial begin
        for (idx = 0; idx < 31; idx = idx + 1) begin
            registers[idx] = 64'b0; // Use blocking assignment for initialization
        end
        // Initialize Stack Pointer (R31) - Adjust address as needed
        registers[31] = 64'h0008_0000; // Example: 512KB stack space
    end

    // Combinational Read Port 1
    assign read_data1 = (read_addr1 == 5'd31) ? registers[31] :
                       (read_addr1 == write_addr && write_enable) ? write_data : // Internal forwarding for write-during-read
                       registers[read_addr1];

    // Combinational Read Port 2
    assign read_data2 = (read_addr2 == 5'd31) ? registers[31] :
                       (read_addr2 == write_addr && write_enable) ? write_data : // Internal forwarding for write-during-read
                       registers[read_addr2];

    // Stack Pointer Output (combinational)
    assign stack_ptr_out = registers[31];

    // Synchronous Write Port
    always @(posedge clk) begin
        if (reset) begin // Asynchronous reset handled by initial block, sync reset optional
            // Could re-initialize here if sync reset needed, but initial is common
        end else begin
            if (write_enable && write_addr != 5'd0) begin // Assuming R0 is not writable
                registers[write_addr] <= write_data;
                // Special handling if stack pointer (R31) is written
                // if (write_addr == 5'd31) begin
                //     // Add stack pointer checks if necessary
                // end
            end
        end
    end

    // Removed regReadFlag logic
    // Removed output3 (unused)

endmodule

//############################################################################
//## fetch
//## CHANGED: Added inputs/logic for RETURN PC handling
//############################################################################
module fetch (
    input clk,
    input reset,
    input branch_taken,         // Branch decision from MEM stage
    input [63:0] branch_pc,     // Branch target PC from MEM stage
    input take_return_pc,       // Control for RETURN from MEM stage
    input [63:0] return_pc,     // Return address from MEM stage (aluMemMux output)
    output logic [63:0] pc_out  // Renamed from pc
);
    localparam INITIAL_PC = 64'h2000; // Standard starting address

    reg [63:0] current_pc;

    // Output assignment
    assign pc_out = current_pc;

    // PC update logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_pc <= INITIAL_PC;
        end else begin
            if (take_return_pc) begin // Highest priority: RETURN instruction
                current_pc <= return_pc;
            end else if (branch_taken) begin // Next priority: Taken Branch
                current_pc <= branch_pc;
            end else begin // Default: Increment PC
                current_pc <= current_pc + 64'd4;
            end
        end
    end
endmodule


//############################################################################
//## if_de_register
//## No functional changes needed, but renamed signals for clarity
//############################################################################
module if_de_register (
    input clk,
    input flush,
    input [63:0] pc_in,
    input [31:0] instruction_in,
    output reg [63:0] pc_out,
    output reg [31:0] instruction_out
);
    always @(posedge clk) begin
        if (flush) begin
            pc_out <= 64'b0; // Or initial PC? Depends on flush strategy
            instruction_out <= 32'b0; // NOP instruction often preferred
        end else begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end
endmodule


//############################################################################
//## instructionDecoder
//## CHANGED: Updated RETURN logic, removed direct rs=rd override for MOV_LIT
//## CHANGED: Renamed outputs for clarity
//############################################################################
module instructionDecoder (
    input [31:0] instructionLine,
    output reg [63:0] literal, // was constant_value
    output reg [4:0] rd, // was target_register
    output reg [4:0] rs, // was src_reg_a
    output reg [4:0] rt, // was src_reg_b
    output reg [4:0] opcode, // was instruction_type
    // Control Signals
    output reg alu_enable,
    output reg mem_read,
    output reg mem_write,
    output reg reg_write,
    output reg mem_to_reg,
    output reg branch_taken, // Is this needed? ALU makes decision. Maybe for CALL type?
    output reg mem_pc        // Control signal for RETURN PC source
);
    // Opcodes (ensure these match localparams in ALU)
    localparam AND     = 5'h00; localparam OR      = 5'h01; localparam XOR     = 5'h02;
    localparam NOT     = 5'h03; localparam SHFTR   = 5'h04; localparam SHFTRI  = 5'h05;
    localparam SHFTL   = 5'h06; localparam SHFTLI  = 5'h07; localparam BR      = 5'h08;
    localparam BRR     = 5'h09; localparam BRRI    = 5'h0A; localparam BRNZ    = 5'h0B;
    localparam CALL    = 5'h0C; localparam RETURN  = 5'h0D; localparam BRGT    = 5'h0E;
    localparam PRIV    = 5'h0F; localparam MOV_MEM = 5'h10; localparam MOV_REG = 5'h11;
    localparam MOV_LIT = 5'h12; localparam MOV_STR = 5'h13; localparam ADDF    = 5'h14;
    localparam SUBF    = 5'h15; localparam MULF    = 5'h16; localparam DIVF    = 5'h17;
    localparam ADD     = 5'h18; localparam ADDI    = 5'h19; localparam SUB     = 5'h1A;
    localparam SUBI    = 5'h1B; localparam MUL     = 5'h1C; localparam DIV     = 5'h1D;

    always @(*) begin // Use always_comb for combinational logic
        // Field extraction
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        // Literal Extraction (Sign or Zero Extension depends on ISA - Assuming Zero Extend based on code)
        literal = {52'b0, instructionLine[11:0]}; // Zero-extend 12-bit literal

        // Default control signal values
        alu_enable = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        reg_write = 1'b0;
        mem_to_reg = 1'b0;
        branch_taken = 1'b0; // This likely indicates branch *type*, not decision
        mem_pc = 1'b0;

        // Decode based on opcode
        case (opcode)
            // R-Type Arithmetic/Logic/Shift
            ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL: begin
                alu_enable = 1'b1;
                reg_write = 1'b1;
                mem_to_reg = 1'b0; // Result from ALU
            end
            // I-Type Arithmetic/Logic/Shift
            ADDI, SUBI, SHFTRI, SHFTLI: begin
                alu_enable = 1'b1;
                reg_write = 1'b1;
                mem_to_reg = 1'b0; // Result from ALU
            end
            // Load
            MOV_MEM: begin // mov rd, (rs)(L) // [cite: 60]
                alu_enable = 1'b1; // ALU calculates address rs+L
                mem_read = 1'b1;
                reg_write = 1'b1;
                mem_to_reg = 1'b1; // Result from Memory
            end
            // Store
            MOV_STR: begin // mov (rd)(L), rs // [cite: 67]
                alu_enable = 1'b1; // ALU calculates address rd+L
                mem_write = 1'b1;
                reg_write = 1'b0; // No register write
            end
            // Register Moves
            MOV_REG: begin // mov rd, rs // [cite: 63]
                alu_enable = 1'b1; // Pass rs through ALU
                reg_write = 1'b1;
                mem_to_reg = 1'b0; // Result from ALU
            end
            // Literal Move (Special Handling in ALU)
            MOV_LIT: begin // mov rd, L // [cite: 65]
                alu_enable = 1'b1; // ALU performs specific operation
                reg_write = 1'b1;
                mem_to_reg = 1'b0; // Result from ALU
            end
            // Floating Point (Assuming treated like integer R-Type for control)
            ADDF, SUBF, MULF, DIVF: begin
                alu_enable = 1'b1;
                reg_write = 1'b1;
                mem_to_reg = 1'b0; // Result from ALU
            end
            // Control Flow - Branches (ALU calculates target & decides)
            BR, BRR, BRRI, BRNZ, BRGT: begin
                alu_enable = 1'b1; // ALU calculates target PC, compares if needed
                reg_write = 1'b0;
                branch_taken = 1'b1; // Indicates it *could* be a branch
            end
            // Control Flow - Call
            CALL: begin // call rd, rs, rt // [cite: 40] (Manual seems wrong, likely call Target Addr)
                       // Assuming 'call TargetAddr' (like BR), saves PC+4 to stack
                alu_enable = 1'b1; // ALU calculates stack addr, gets PC+4
                mem_write = 1'b1;  // Write PC+4 to stack
                reg_write = 1'b0;  // No register write (unless saving link register?) - R31 is implicit
                branch_taken = 1'b1; // This is an unconditional jump type
            end
            // Control Flow - Return
            RETURN: begin // return // [cite: 43]
                alu_enable = 1'b1; // ALU calculates stack address to read from
                mem_read = 1'b1;   // Read return address from stack
                reg_write = 1'b0;
                mem_pc = 1'b1;     // Use memory data as next PC
                branch_taken = 1'b1; // Treat like a branch for flushing etc.
            end
            // Privileged (HALT)
            PRIV: begin
                // Assuming L=0 is HALT // [cite: 51, 52]
                // Other PRIV functions (Trap, RTE, Input, Output) need specific decoding if implemented
                if (literal[11:0] == 12'h0) begin // Check if L field is 0 for HALT
                    alu_enable = 1'b1; // ALU sets hlt flag
                    reg_write = 1'b0;
                end else begin
                    // Handle other PRIV calls or treat as illegal instruction
                    alu_enable = 1'b0; // Or generate exception
                end
            end
            default: begin
                // Treat as NOP or illegal instruction
                // Set safe defaults (mostly done above)
            end
        endcase

        // Modify rs/rt fields based on instruction type for clarity before reg file read
        case (opcode)
             // For I-types using rd as source and literal
             ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: begin
                 rs = rd; // Use rd field as the source register address
                 // rt field is ignored for these, literal is used instead
             end
             // For NOT using only rs
             NOT: begin
                 // rt field is ignored
             end
              // For Store using rd as base address and rs as source data
             MOV_STR: begin
                 // Use rd as base address register (passed to ALU)
                 // Use rs as source data register (passed to ALU)
                 // rt field is ignored
             end
             // For Branches using rd as target or comparison
             BR, BRR, BRNZ, BRGT: begin
                 // Operands depend on specific branch, ALU handles use of rs/rt/rd fields
             end
             // For Call/Return/Priv - fields used differently
             CALL, RETURN, PRIV: begin
                 // Specific handling in ALU
             end
             // Default R-Type and Load uses rs, rt as sources
             default: begin
                 // rs, rt used as source registers
             end
        endcase

    end
endmodule


//############################################################################
//## de_ex_register
//## CHANGED: Added ports for pipelined operands, PC, stack pointer
//## CHANGED: Fixed typo in literal_out width
//############################################################################
module de_ex_register (
    input clk,
    input flush,
    // Inputs from Decode Stage
    input [63:0] pc_in, // ADDED
    input [63:0] operand_a_in, // ADDED
    input [63:0] operand_b_in, // ADDED (Output of RegLitMux)
    input [63:0] literal_in,
    input [4:0] rd_addr_in, // Renamed from rd_in
    input [4:0] rs_addr_in, // Renamed from rs_in
    input [4:0] rt_addr_in, // Renamed from rt_in
    input [4:0] opcode_in,
    input [63:0] stack_ptr_in, // ADDED
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,
    input mem_to_reg_in,
    input branch_taken_ctrl_in, // Renamed from branch_taken_in
    input mem_pc_in,
    // Outputs to Execute Stage
    output reg [63:0] pc_out, // ADDED
    output reg [63:0] operand_a_out, // ADDED
    output reg [63:0] operand_b_out, // ADDED
    output reg [63:0] literal_out, // Fixed width typo [63:00] -> [63:0]
    output reg [4:0] rd_addr_out, // Renamed from rd_out
    output reg [4:0] rs_addr_out, // Renamed from rs_out
    output reg [4:0] rt_addr_out, // Renamed from rt_out
    output reg [4:0] opcode_out,
    output reg [63:0] stack_ptr_out, // ADDED
    output reg alu_enable_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_ctrl_out, // Renamed from branch_taken_out
    output reg mem_pc_out
);
    always @(posedge clk) begin
        if (flush) begin
            // Flush Pipeline Register (Set to NOP or safe values)
            pc_out <= 64'b0;
            operand_a_out <= 64'b0;
            operand_b_out <= 64'b0;
            literal_out <= 64'b0;
            rd_addr_out <= 5'b0;
            rs_addr_out <= 5'b0;
            rt_addr_out <= 5'b0;
            opcode_out <= 5'b0; // NOP opcode if defined, else 0
            stack_ptr_out <= 64'b0;
            alu_enable_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            branch_taken_ctrl_out <= 1'b0;
            mem_pc_out <= 1'b0;
        end else begin
            // Latch Inputs to Outputs
            pc_out <= pc_in;
            operand_a_out <= operand_a_in;
            operand_b_out <= operand_b_in;
            literal_out <= literal_in;
            rd_addr_out <= rd_addr_in;
            rs_addr_out <= rs_addr_in;
            rt_addr_out <= rt_addr_in;
            opcode_out <= opcode_in;
            stack_ptr_out <= stack_ptr_in;
            alu_enable_out <= alu_enable_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            branch_taken_ctrl_out <= branch_taken_ctrl_in;
            mem_pc_out <= mem_pc_in;
        end
    end
endmodule

//############################################################################
//## alu
//## CHANGED: Removed unnecessary pass-through output ports for control signals
//## CHANGED: Replaced always_comb with always @(*)
//############################################################################
module alu (
    // Control Inputs
    input logic alu_enable,
    input logic [4:0] opcode,
    // Data Inputs
    input logic [63:0] input1,
    input logic [63:0] input2,
    input logic [4:0] rd_addr,
    input logic [63:0] literal,
    input logic [63:0] pc_in,
    input logic [63:0] stack_ptr,
    // Outputs
    output logic [63:0] result,
    output logic [63:0] mem_addr,
    output logic [63:0] mem_wdata,
    output logic [63:0] branch_pc,
    output logic branch_taken,
    output logic hlt_out,
    // Control Signal Inputs (Needed if ALU logic depends on them - currently not)
    // Keep inputs for potential future use, but remove corresponding outputs
    input logic mem_read_in,
    input logic mem_write_in,
    input logic reg_write_in,
    input logic mem_to_reg_in,
    input logic mem_pc_in
    // CHANGED: Removed output logic mem_read_out, mem_write_out, etc.
);

    // Opcodes (ensure these match localparams in Decoder)
    localparam AND     = 5'h00; localparam OR      = 5'h01; localparam XOR     = 5'h02;
    localparam NOT     = 5'h03; localparam SHFTR   = 5'h04; localparam SHFTRI  = 5'h05;
    localparam SHFTL   = 5'h06; localparam SHFTLI  = 5'h07; localparam BR      = 5'h08;
    localparam BRR     = 5'h09; localparam BRRI    = 5'h0A; localparam BRNZ    = 5'h0B;
    localparam CALL    = 5'h0C; localparam RETURN  = 5'h0D; localparam BRGT    = 5'h0E;
    localparam PRIV    = 5'h0F; localparam MOV_MEM = 5'h10; localparam MOV_REG = 5'h11;
    localparam MOV_LIT = 5'h12; localparam MOV_STR = 5'h13; localparam ADDF    = 5'h14;
    localparam SUBF    = 5'h15; localparam MULF    = 5'h16; localparam DIVF    = 5'h17;
    localparam ADD     = 5'h18; localparam ADDI    = 5'h19; localparam SUB     = 5'h1A;
    localparam SUBI    = 5'h1B; localparam MUL     = 5'h1C; localparam DIV     = 5'h1D;

    logic [63:0] fp_result; // For simulation only

    // Default assignments in combinational block
    always @(*) begin // CHANGED from always_comb
        // Default Output Values
        result = 64'b0;
        mem_addr = 64'b0;
        mem_wdata = 64'b0;
        branch_pc = pc_in + 4;
        branch_taken = 1'b0;
        hlt_out = 1'b0;

        // Perform ALU operation if enabled
        if (alu_enable) begin
            case (opcode)
                // Integer Arithmetic
                ADD, ADDI: result = $signed(input1) + $signed(input2);
                SUB, SUBI: result = $signed(input1) - $signed(input2);
                MUL: result = $signed(input1) * $signed(input2);
                DIV: result = $signed(input1) / $signed(input2); // Consider division by zero

                // Logical
                AND: result = input1 & input2;
                OR:  result = input1 | input2;
                XOR: result = input1 ^ input2;
                NOT: result = ~input1;

                // Shift
                SHFTR, SHFTRI: result = input1 >> input2[5:0];
                SHFTL, SHFTLI: result = input1 << input2[5:0];

                // Data Movement - Load
                MOV_MEM: begin
                    mem_addr = input1 + $signed(input2); // rs + L
                end
                // Data Movement - Register to Register
                MOV_REG: begin
                    result = input1; // Pass rs
                end
                 // Data Movement - Literal
                MOV_LIT: begin
                    result = {input1[63:12], input2[11:0]}; // Manual is weird, using this interpretation
                end
                // Data Movement - Store
                MOV_STR: begin
                    // Needs rd_data + L. Using placeholder. Requires pipeline pass-through of rd_data.
                    mem_addr = {{59{1'b0}}, rd_addr} + $signed(literal); // PLACEHOLDER
                    mem_wdata = input1; // rs_data
                end

                // Floating Point (Simulation only)
                ADDF: begin fp_result = $realtobits($bitstoreal(input1) + $bitstoreal(input2)); result = fp_result; end
                SUBF: begin fp_result = $realtobits($bitstoreal(input1) - $bitstoreal(input2)); result = fp_result; end
                MULF: begin fp_result = $realtobits($bitstoreal(input1) * $bitstoreal(input2)); result = fp_result; end
                DIVF: begin fp_result = $realtobits($bitstoreal(input1) / $bitstoreal(input2)); result = fp_result; end

                // Control Flow - Branches
                BR: begin
                    branch_pc = input1; // target is rs/rd data
                    branch_taken = 1'b1;
                end
                BRR: begin
                    branch_pc = pc_in + $signed(input1); // target is PC + rs/rd data
                    branch_taken = 1'b1;
                end
                BRRI: begin
                    branch_pc = pc_in + $signed(input2); // target is PC + literal
                    branch_taken = 1'b1;
                end
                BRNZ: begin
                    // Needs target address (rd_data). Using placeholder.
                    if ($signed(input1) != 0) begin // check rs_data
                        branch_pc = input2; // PLACEHOLDER for target rd_data
                        branch_taken = 1'b1;
                    end else begin
                        branch_taken = 1'b0;
                    end
                end
                BRGT: begin
                     // Needs target address (rd_data). Using placeholder.
                    if ($signed(input1) > $signed(input2)) begin // if rs > rt
                        branch_pc = stack_ptr; // PLACEHOLDER for target rd_data
                        branch_taken = 1'b1;
                    end else begin
                        branch_taken = 1'b0;
                    end
                end

                // Control Flow - Subroutines
                CALL: begin
                    // Needs target address (rd_data). Using placeholder.
                    branch_pc = input1; // PLACEHOLDER for target rd_data
                    mem_addr = stack_ptr - 8;
                    mem_wdata = pc_in + 4;
                    branch_taken = 1'b1;
                end
                RETURN: begin
                    mem_addr = stack_ptr - 8;
                    branch_taken = 1'b0; // PC comes from memory via aluMemMux->fetch
                end

                // Privileged - Halt
                PRIV: begin
                    if (literal[11:0] == 12'h0) begin // L=0 for HALT
                         hlt_out = 1'b1;
                    end
                end

                default: result = 64'b0;
            endcase
        end // if (alu_enable)
    end // always@(*)

endmodule


//############################################################################
//## ex_mem_register
//## CHANGED: Added rd_addr pipelining, fixed typo in rwAddress_out width
//############################################################################
module ex_mem_register (
    input clk,
    // Inputs from Execute Stage
    input logic [63:0] result_in,
    input logic [63:0] mem_addr_in, // Renamed from rwAddress_in
    input logic [63:0] mem_wdata_in, // Renamed from writeData_in
    input logic [63:0] branch_pc_in, // Renamed from pc_in
    input logic [4:0] rd_addr_in, // ADDED: Pipeline write address
    input logic hlt_in,
    input logic mem_read_in,
    input logic mem_write_in,
    input logic reg_write_in,
    input logic mem_to_reg_in,
    input logic branch_taken_in, // Branch decision from ALU
    input logic mem_pc_in,
    // Outputs to Memory/Writeback Stage
    output logic [63:0] result_out,
    output logic [63:0] mem_addr_out, // Renamed from rwAddress_out, fixed width [63:00] -> [63:0]
    output logic [63:0] mem_wdata_out, // Renamed from writeData_out
    output logic [63:0] branch_pc_out, // Renamed from pc_out
    output logic [4:0] rd_addr_out, // ADDED: Pipelined write address
    output logic hlt_out,
    output logic mem_read_out,
    output logic mem_write_out,
    output logic reg_write_out,
    output logic mem_to_reg_out,
    output logic branch_taken_out, // Pipelined branch decision
    output logic mem_pc_out
);
    always @(posedge clk) begin
        // No flush input? Assume previous stage handles flush propagation if needed,
        // or add flush input if EX/MEM needs independent flushing.
        // Latching inputs to outputs
        result_out <= result_in;
        mem_addr_out <= mem_addr_in;
        mem_wdata_out <= mem_wdata_in;
        branch_pc_out <= branch_pc_in;
        rd_addr_out <= rd_addr_in; // Latch write address
        hlt_out <= hlt_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        branch_taken_out <= branch_taken_in; // Latch branch decision
        mem_pc_out <= mem_pc_in;
    end
endmodule


//############################################################################
//## memory
//## CHANGED: Combinational Read, Synchronous Write, Simplified Interface
//############################################################################
module memory (
    input clk,
    input reset,
    // Instruction Fetch Port (Read Only)
    input [63:0] inst_addr,
    output logic [31:0] instruction_out,
    // Data Port (Read/Write)
    input [63:0] data_addr,
    input [63:0] data_wdata,
    input mem_read,
    input mem_write,
    output logic [63:0] data_rdata
);
    localparam MEM_SIZE_BYTES = 524288; // 512 KB example size
    localparam MEM_ADDR_BITS = $clog2(MEM_SIZE_BYTES);

    // Memory array (byte addressable)
    reg [7:0] bytes [0:MEM_SIZE_BYTES-1];
    integer i;

    // Initialization / Reset
    initial begin
        for (i = 0; i < MEM_SIZE_BYTES; i = i + 1) begin
            bytes[i] = 8'b0;
        end
        // Preload memory here if needed for simulation
        // Example: bytes[16'h2000] = 8'hXX; bytes[16'h2001] = 8'hYY; ...
    end

    // --- Instruction Fetch Port (Combinational Read) ---
    // Assuming byte addressing and little-endian fetch
    // Ensure inst_addr is properly aligned if required by ISA
    assign instruction_out[7:0]   = bytes[inst_addr + 0];
    assign instruction_out[15:8]  = bytes[inst_addr + 1];
    assign instruction_out[23:16] = bytes[inst_addr + 2];
    assign instruction_out[31:24] = bytes[inst_addr + 3];

    // --- Data Port ---
    // Combinational Read
    // Assuming little-endian data access
    assign data_rdata[7:0]   = bytes[data_addr + 0];
    assign data_rdata[15:8]  = bytes[data_addr + 1];
    assign data_rdata[23:16] = bytes[data_addr + 2];
    assign data_rdata[31:24] = bytes[data_addr + 3];
    assign data_rdata[39:32] = bytes[data_addr + 4];
    assign data_rdata[47:40] = bytes[data_addr + 5];
    assign data_rdata[55:48] = bytes[data_addr + 6];
    assign data_rdata[63:56] = bytes[data_addr + 7];

    // Synchronous Write
    always @(posedge clk) begin
        if (mem_write) begin
             // Assuming little-endian write
             bytes[data_addr + 0] <= data_wdata[7:0];
             bytes[data_addr + 1] <= data_wdata[15:8];
             bytes[data_addr + 2] <= data_wdata[23:16];
             bytes[data_addr + 3] <= data_wdata[31:24];
             bytes[data_addr + 4] <= data_wdata[39:32];
             bytes[data_addr + 5] <= data_wdata[47:40];
             bytes[data_addr + 6] <= data_wdata[55:48];
             bytes[data_addr + 7] <= data_wdata[63:56];
         end
    end

    // Removed memFlag logic

endmodule


//############################################################################
//## aluMemMux
//## CHANGED: Renamed ports for clarity, functionality remains
//## Connects to Fetch Stage input for RETURN PC
//############################################################################
module aluMemMux (
    input mem_pc,           // Control signal from EX/MEM (indicates RETURN)
    input [63:0] memData,   // Data read from memory (Return Address)
    input [63:0] aluOut,    // Calculated branch target from ALU (passed via EX/MEM)
    output reg [63:0] newPc // Output PC value to be used by Fetch
);
    always @(*) begin // Use always_comb
        if (mem_pc) begin
            newPc = memData; // Select return address from memory
        end else begin
            newPc = aluOut; // Select calculated branch target (or PC+4 if not branch)
        end
    end
endmodule


//############################################################################
//## reglitmux
//## CHANGED: Renamed ports, using localparams for selection (more readable)
//############################################################################
module reglitmux (
    input [4:0] sel, // Opcode input
    input [63:0] reg_in, // Data from Register File (rt)
    input [63:0] lit_in, // Data from Decoder (literal)
    output reg [63:0] out // Output to DE/EX register (Operand B)
);
    // Opcodes using literal as second ALU operand
    localparam ADDI    = 5'h19; localparam SUBI    = 5'h1B;
    localparam SHFTRI  = 5'h05; localparam SHFTLI  = 5'h07;
    localparam BRRI    = 5'h0A;
    localparam MOV_MEM = 5'h10; // Uses literal for offset calculation
    localparam MOV_LIT = 5'h12; // Uses literal value directly
    localparam MOV_STR = 5'h13; // Uses literal for offset calculation

    always @(*) begin // Use always_comb
        // Select literal if opcode matches instructions that use it as input2
        case (sel)
            ADDI, SUBI, SHFTRI, SHFTLI, BRRI, MOV_MEM, MOV_LIT, MOV_STR: begin
                out = lit_in; // Select literal
            end
            default: begin
                out = reg_in; // Select register value (rt)
            end
        endcase
    end
endmodule


//############################################################################
//## memRegMux
//## No functional changes needed, renamed ports for clarity
//############################################################################
module memRegMux (
    input mem_to_reg, // Control signal from EX/MEM
    input [63:0] readData, // Data from Memory Read (MEM Stage)
    input [63:0] aluResult, // Result from ALU (passed via EX/MEM)
    output reg [63:0] regWriteData // Data to be written back to Register File
);
    always @(*) begin // Use always_comb
        if (mem_to_reg) begin
            regWriteData = readData; // Select data from memory (for Loads)
        end else begin
            regWriteData = aluResult; // Select data from ALU result
        end
    end
endmodule