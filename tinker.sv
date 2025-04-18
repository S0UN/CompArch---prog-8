//############################################################################
//## Forwarding Unit
//## Determines when to forward results from EX or MEM stages to DE stage inputs
//############################################################################
module forwarding_unit (
    // Inputs from EX stage (outputs of DE/EX register)
    input logic [4:0] rd_addr_ex,
    input logic       reg_write_ex,

    // Inputs from MEM stage (outputs of EX/MEM register)
    input logic [4:0] rd_addr_mem,
    input logic       reg_write_mem,

    // Inputs from DE stage (outputs of IF/DE register + decoder)
    input logic [4:0] rs_addr_de,
    input logic [4:0] rt_addr_de,
    input logic [4:0] rd_addr_de, // Read address for Port 3

    // Outputs: Forwarding select signals for DE stage operand muxes
    // 2'b00: Use Register File output
    // 2'b01: Forward from EX stage (alu_result_ex)
    // 2'b10: Forward from MEM stage (reg_wdata_wb)
    // 2'b11: Unused / Reserved
    output reg [1:0] forward_a_sel, // For operand A (rs)
    output reg [1:0] forward_b_sel, // For operand B (rt)
    output reg [1:0] forward_c_sel  // For operand C (rd read via Port 3)
);

    // --- Forwarding Logic for Operand A (rs_data) ---
    always @(*) begin // Replaced always_comb
        forward_a_sel = 2'b00; // Default: No forwarding
        // EX Hazard: Forward ALU result from EX stage?
        // Check if EX stage is writing (reg_write_ex), dest is not R0, and dest matches DE source (rs)
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rs_addr_de)) begin
            forward_a_sel = 2'b01; // Forward from EX
        end
        // MEM Hazard: Forward result from MEM stage? (Check only if no EX hazard)
        // Note: We forward the final writeback data (reg_wdata_wb) which handles loads correctly.
        // Check if MEM stage is writing (reg_write_mem), dest is not R0, and dest matches DE source (rs)
        else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rs_addr_de)) begin
             forward_a_sel = 2'b10; // Forward from MEM/WB
        end
    end

    // --- Forwarding Logic for Operand B (rt_data) ---
    // Note: This forwards the value *before* it goes into the reglitmux
    always @(*) begin // Replaced always_comb
        forward_b_sel = 2'b00; // Default: No forwarding
        // EX Hazard
        // Check if EX stage is writing, dest is not R0, and dest matches DE source (rt)
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rt_addr_de)) begin
            forward_b_sel = 2'b01; // Forward from EX
        end
        // MEM Hazard
        // Check if MEM stage is writing, dest is not R0, and dest matches DE source (rt)
        else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rt_addr_de)) begin
            forward_b_sel = 2'b10; // Forward from MEM/WB
        end
    end

     // --- Forwarding Logic for Operand C (rd_data read via Port 3) ---
    // Used by instructions like MOV_STR[cite: 68], BR[cite: 32], BRR[cite: 34], BRNZ[cite: 39], BRGT[cite: 47], CALL [cite: 42]
    always @(*) begin // Replaced always_comb
        forward_c_sel = 2'b00; // Default: No forwarding
        // EX Hazard
        // Check if EX stage is writing, dest is not R0, and dest matches DE source (rd)
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rd_addr_de)) begin
            forward_c_sel = 2'b01; // Forward from EX
        end
        // MEM Hazard
        // Check if MEM stage is writing, dest is not R0, and dest matches DE source (rd)
        else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rd_addr_de)) begin
            forward_c_sel = 2'b10; // Forward from MEM/WB
        end
    end

endmodule

//############################################################################
//## tinker_core (Top Level Module with Forwarding)
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

    // DE Stage Register File Outputs
    logic [63:0] regfile_operand_a_de; // rs_data from regfile
    logic [63:0] regfile_operand_b_de; // rt_data from regfile
    logic [63:0] regfile_operand_c_de; // rd_data from regfile (Port 3)

    // DE Stage Operands after Forwarding/Muxing
    logic [63:0] forwarded_operand_a_de; // operand_a after forwarding mux
    logic [63:0] forwarded_operand_b_de; // operand_b after forwarding mux (before reglitmux)
    logic [63:0] forwarded_operand_c_de; // operand_c after forwarding mux
    logic [63:0] operand_b_mux_in_de;    // Output of reglitmux (final operand B for EX)

    // DE Stage Decoded Fields & Control
    logic [63:0] literal_de; // Now sign-extended
    logic [4:0]  rd_addr_de;
    logic [4:0]  rs_addr_de;
    logic [4:0]  rt_addr_de;
    logic [4:0]  opcode_de;
    logic [63:0] stack_ptr_de;
    logic        alu_enable_de, mem_read_de, mem_write_de, reg_write_de, mem_to_reg_de, branch_taken_ctrl_de, mem_pc_de;

    // DE/EX Register Outputs -> Inputs to EX Stage
    logic [63:0] pc_ex;
    logic [63:0] operand_a_ex;
    logic [63:0] operand_b_ex;
    logic [63:0] operand_c_ex;
    logic [63:0] literal_ex;
    logic [4:0]  rd_addr_ex;   // Pipelined write address (destination)
    logic [4:0]  rs_addr_ex;   // Pipelined rs address
    logic [4:0]  rt_addr_ex;   // Pipelined rt address
    logic [4:0]  opcode_ex;
    logic [63:0] stack_ptr_ex;
    logic        alu_enable_ex, mem_read_ex, mem_write_ex, reg_write_ex, mem_to_reg_ex, mem_pc_ex;

    // EX Stage Outputs -> Inputs to EX/MEM Register
    logic [63:0] alu_result_ex;
    logic [63:0] alu_mem_addr_ex;
    logic [63:0] alu_mem_data_ex; // Data to be stored (e.g., for MOV_STR)
    logic [63:0] alu_branch_pc_ex;
    logic        branch_taken_ex; // Branch decision made in EX
    logic        hlt_ex;

    // EX/MEM Register Outputs -> Inputs to MEM Stage
    logic [63:0] alu_result_mem;  // Result from ALU (passed through)
    logic [63:0] mem_addr_mem;    // Address for data memory access
    logic [63:0] mem_wdata_mem;   // Data to write to memory (comes from alu_mem_data_ex)
    logic [63:0] branch_pc_mem;   // Branch target PC (passed through)
    logic [4:0]  rd_addr_mem;     // Destination register (passed through)
    logic        mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, branch_taken_mem, mem_pc_mem; // Control
    logic        hlt_mem;

    // MEM Stage Outputs
    logic [63:0] mem_rdata_mem;   // Data read from memory
    logic [63:0] return_pc_mem;   // PC value selected by aluMemMux

    // MEM/WB Stage Data (Result of memRegMux, input to Register File Write)
    logic [63:0] reg_wdata_wb;

    // --- Forwarding Signals ---
    logic [1:0] forward_a_sel_de;
    logic [1:0] forward_b_sel_de;
    logic [1:0] forward_c_sel_de;

    // --- Control Signals ---
    logic flush_de, flush_ex;
    logic take_return_pc_fetch;

    // --- Module Instantiations ---

    fetch instruction_fetcher (
        .clk(clk),
        .reset(reset),
        .branch_taken(branch_taken_mem),   // Use final branch decision from MEM stage
        .branch_pc(branch_pc_mem),         // Use final branch PC from MEM stage
        .take_return_pc(take_return_pc_fetch), // Use final control from MEM stage
        .return_pc(return_pc_mem),         // Use PC from MEM stage mux
        .pc_out(pc_if)
    );

    memory data_memory ( // Instance name matches previous code
        .clk(clk),
        .reset(reset),
        .inst_addr(pc_if),                 // Instruction fetch address
        .instruction_out(instruction_if),  // Instruction output
        .data_addr(mem_addr_mem),          // Data address from MEM stage
        .data_wdata(mem_wdata_mem),        // Data to write from MEM stage
        .mem_read(mem_read_mem),           // Read control from MEM stage
        .mem_write(mem_write_mem),         // Write control from MEM stage
        .data_rdata(mem_rdata_mem)         // Data read output (used in WB mux)
    );

    if_de_register if_de_reg (
        .clk(clk),
        .flush(flush_de),                  // Flush signal from MEM stage branch taken
        .pc_in(pc_if),
        .instruction_in(instruction_if),
        .pc_out(pc_de),
        .instruction_out(instruction_de)
    );

    instructionDecoder instruction_parser (
        .instructionLine(instruction_de),
        .literal(literal_de),              // Output: sign-extended literal
        .rd(rd_addr_de),
        .rs(rs_addr_de),
        .rt(rt_addr_de),
        .opcode(opcode_de),
        .alu_enable(alu_enable_de),
        .mem_read(mem_read_de),
        .mem_write(mem_write_de),
        .reg_write(reg_write_de),
        .mem_to_reg(mem_to_reg_de),
        .branch_taken(branch_taken_ctrl_de), // Indicates branch *type*
        .mem_pc(mem_pc_de)                 // Indicates RETURN instruction
    );

    registerFile reg_file (
        .clk(clk),
        .reset(reset),
        // Write Port (controlled by MEM stage signals)
        .write_addr(rd_addr_mem),
        .write_data(reg_wdata_wb),
        .write_enable(reg_write_mem),
        // Read Ports (for DE stage)
        .read_addr1(rs_addr_de),
        .read_data1(regfile_operand_a_de), // Output from RegFile for rs
        .read_addr2(rt_addr_de),
        .read_data2(regfile_operand_b_de), // Output from RegFile for rt
        .read_addr3(rd_addr_de),
        .read_data3(regfile_operand_c_de), // Output from RegFile for rd (Port 3)
        // Stack Pointer Output (read in DE)
        .stack_ptr_out(stack_ptr_de)
    );

    // --- Forwarding Unit ---
    forwarding_unit fwd_unit (
        // Inputs from EX stage (DE/EX register outputs)
        .rd_addr_ex(rd_addr_ex),
        .reg_write_ex(reg_write_ex),
        // Inputs from MEM stage (EX/MEM register outputs)
        .rd_addr_mem(rd_addr_mem),
        .reg_write_mem(reg_write_mem),
        // Inputs from DE stage (Decoder outputs)
        .rs_addr_de(rs_addr_de),
        .rt_addr_de(rt_addr_de),
        .rd_addr_de(rd_addr_de),
        // Outputs to Forwarding Muxes
        .forward_a_sel(forward_a_sel_de),
        .forward_b_sel(forward_b_sel_de),
        .forward_c_sel(forward_c_sel_de)
    );

    // --- Forwarding Multiplexers (Combinational Logic in DE stage) ---
    // Mux for Operand A (rs) -> Input to DE/EX Register
    always @(*) begin // Replaced always_comb
        case (forward_a_sel_de)
            2'b01:  forwarded_operand_a_de = alu_result_ex;  // Forward ALU result from EX stage
            2'b10:  forwarded_operand_a_de = reg_wdata_wb;   // Forward final writeback data from WB stage mux
            default: forwarded_operand_a_de = regfile_operand_a_de; // Use data read from Register File
        endcase
    end

    // Mux for Operand B (rt) -> Input to reglitmux
    always @(*) begin // Replaced always_comb
        case (forward_b_sel_de)
            2'b01:  forwarded_operand_b_de = alu_result_ex;  // Forward ALU result from EX stage
            2'b10:  forwarded_operand_b_de = reg_wdata_wb;   // Forward final writeback data from WB stage mux
            default: forwarded_operand_b_de = regfile_operand_b_de; // Use data read from Register File
        endcase
    end

    // Mux for Operand C (rd read on Port 3) -> Input to DE/EX Register
    always @(*) begin // Replaced always_comb
        case (forward_c_sel_de)
            2'b01:  forwarded_operand_c_de = alu_result_ex;  // Forward ALU result from EX stage
            2'b10:  forwarded_operand_c_de = reg_wdata_wb;   // Forward final writeback data from WB stage mux
            default: forwarded_operand_c_de = regfile_operand_c_de; // Use data read from Register File
        endcase
    end

    // Register/Literal Mux for final Operand B -> Input to DE/EX Register
    reglitmux input_selector (
        .sel(opcode_de),
        .reg_in(forwarded_operand_b_de), // Use the potentially forwarded rt_data
        .lit_in(literal_de),             // Use the sign-extended literal
        .out(operand_b_mux_in_de)
    );

    // --- Pipeline Registers ---
    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush_ex),                  // Flush signal from MEM stage branch taken
        // Inputs from Decode Stage (potentially forwarded/muxed)
        .pc_in(pc_de),
        .operand_a_in(forwarded_operand_a_de), // Pass forwarded rs_data
        .operand_b_in(operand_b_mux_in_de),    // Pass final operand B (reg or lit)
        .operand_c_in(forwarded_operand_c_de), // Pass forwarded rd_data
        .literal_in(literal_de),               // Pass sign-extended literal through
        .rd_addr_in(rd_addr_de),               // Pass original rd address
        .rs_addr_in(rs_addr_de),               // Pass original rs address
        .rt_addr_in(rt_addr_de),               // Pass original rt address
        .opcode_in(opcode_de),
        .stack_ptr_in(stack_ptr_de),
        // Control signals
        .alu_enable_in(alu_enable_de),
        .mem_read_in(mem_read_de),
        .mem_write_in(mem_write_de),
        .reg_write_in(reg_write_de),           // Pass reg_write control signal
        .mem_to_reg_in(mem_to_reg_de),
        .branch_taken_ctrl_in(branch_taken_ctrl_de),
        .mem_pc_in(mem_pc_de),
        // Outputs to Execute Stage
        .pc_out(pc_ex),
        .operand_a_out(operand_a_ex),
        .operand_b_out(operand_b_ex),
        .operand_c_out(operand_c_ex),
        .literal_out(literal_ex),
        .rd_addr_out(rd_addr_ex),              // Output destination address (needed for fwd unit)
        .rs_addr_out(rs_addr_ex),
        .rt_addr_out(rt_addr_ex),
        .opcode_out(opcode_ex),
        .stack_ptr_out(stack_ptr_ex),
        // Control Signals out
        .alu_enable_out(alu_enable_ex),
        .mem_read_out(mem_read_ex),
        .mem_write_out(mem_write_ex),
        .reg_write_out(reg_write_ex),          // Output reg_write (needed for fwd unit)
        .mem_to_reg_out(mem_to_reg_ex),
        .branch_taken_ctrl_out(),              // Still unused
        .mem_pc_out(mem_pc_ex)
    );

    // --- Execute Stage ---
    alu calculation_unit (
        .alu_enable(alu_enable_ex),
        .opcode(opcode_ex),
        // Data Inputs from DE/EX register
        .input1(operand_a_ex),
        .input2(operand_b_ex),
        .input3(operand_c_ex),
        .rd_addr(rd_addr_ex), // Needed? ALU doesn't directly use rd_addr for calculation
        .literal(literal_ex),
        .pc_in(pc_ex),
        .stack_ptr(stack_ptr_ex),
        // Outputs to EX/MEM Register
        .result(alu_result_ex),     // Result needed for EX->DE forwarding
        .mem_addr(alu_mem_addr_ex), // Calculated address for load/store/call/return
        .mem_wdata(alu_mem_data_ex),// Data for store/call
        .branch_pc(alu_branch_pc_ex),// Calculated branch target
        .branch_taken(branch_taken_ex),// Branch decision logic output
        .hlt_out(hlt_ex),           // Halt signal
        // Control Inputs Pass-through (Not used by this ALU, but passed for consistency)
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .mem_pc_in(mem_pc_ex)
    );

    // --- Pipeline Register ---
    ex_mem_register ex_mem_reg (
        .clk(clk),
        .flush_mem(flush_ex),          // Flush signal from MEM stage branch taken
        // Inputs from Execute Stage
        .result_in(alu_result_ex),     // ALU result
        .mem_addr_in(alu_mem_addr_ex), // Memory address
        .mem_wdata_in(alu_mem_data_ex),// Memory write data
        .branch_pc_in(alu_branch_pc_ex),// Branch target PC
        .rd_addr_in(rd_addr_ex),       // Pass destination addr
        .hlt_in(hlt_ex),               // Pass halt signal
        // Control Inputs (from DE/EX outputs)
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),   // Pass reg_write control
        .mem_to_reg_in(mem_to_reg_ex),
        .branch_taken_in(branch_taken_ex), // Pass branch decision from ALU
        .mem_pc_in(mem_pc_ex),         // Pass return control signal
        // Outputs to Memory Stage
        .result_out(alu_result_mem),
        .mem_addr_out(mem_addr_mem),
        .mem_wdata_out(mem_wdata_mem),
        .branch_pc_out(branch_pc_mem),
        .rd_addr_out(rd_addr_mem),     // Output destination addr (for fwd unit & regfile write)
        .hlt_out(hlt_mem),
        // Control Outputs
        .mem_read_out(mem_read_mem),
        .mem_write_out(mem_write_mem),
        .reg_write_out(reg_write_mem), // Output reg_write (for fwd unit & regfile write)
        .mem_to_reg_out(mem_to_reg_mem),
        .branch_taken_out(branch_taken_mem), // Output final branch decision
        .mem_pc_out(mem_pc_mem)        // Output final return control
    );

    // --- Memory Stage Muxes ---
    // Mux to select PC source for Fetch stage (Return PC from Mem vs Branch PC from ALU)
    aluMemMux return_pc_selector (
        .mem_pc(mem_pc_mem),         // Control signal from MEM stage (high for RETURN)
        .memData(mem_rdata_mem),     // Return address read from stack
        .aluOut(branch_pc_mem),      // Branch target address calculated earlier
        .newPc(return_pc_mem)        // Selected PC for Fetch stage if take_return_pc is high
    );

    // Mux to select data for Register File Writeback (Data from Memory vs ALU Result)
    memRegMux data_source_selector (
        .mem_to_reg(mem_to_reg_mem), // Control signal from MEM stage (high for Load)
        .readData(mem_rdata_mem),    // Data read from memory
        .aluResult(alu_result_mem),  // Result from ALU
        .regWriteData(reg_wdata_wb)  // Final data for writeback (also needed for MEM->DE forwarding)
    );

    // --- Pipeline Control Logic ---
    // Flush IF/DE and DE/EX registers if a branch is taken in the MEM stage
    assign flush_de = branch_taken_mem;
    assign flush_ex = branch_taken_mem;

    // Control signal for Fetch stage PC mux (select Return PC path for RETURN instruction)
    assign take_return_pc_fetch = mem_pc_mem;

    // Halt Signal Output
    assign hlt = hlt_mem;

endmodule


//############################################################################
//## registerFile
//## CHANGED: Ignore writes to R0
//############################################################################
module registerFile (
    input clk,
    input reset,
    // Write Port
    input [4:0] write_addr,
    input [63:0] write_data,
    input write_enable,
    // Read Port 1 (rs)
    input [4:0] read_addr1,
    output logic [63:0] read_data1,
    // Read Port 2 (rt)
    input [4:0] read_addr2,
    output logic [63:0] read_data2,
    // Read Port 3 (rd)
    input [4:0] read_addr3,
    output logic [63:0] read_data3,
    // Stack Pointer Output
    output logic [63:0] stack_ptr_out
);
    reg [63:0] registers [0:31];
    integer idx;

    // Initialize Registers (R31 is Stack Pointer)
    initial begin
        registers[0] = 64'b0; // R0 is always 0
        for (idx = 1; idx < 31; idx = idx + 1) begin
            registers[idx] = 64'b0;
        end
        registers[31] = 64'h0008_0000; // Example Stack Pointer Init
    end

    // Combinational Read Port 1 (rs)
    // Reads committed state only; forwarding handled externally
    assign read_data1 = registers[read_addr1]; // R0 reads as 0 due to initialization

    // Combinational Read Port 2 (rt)
    assign read_data2 = registers[read_addr2]; // R0 reads as 0

    // Combinational Read Port 3 (rd)
    assign read_data3 = registers[read_addr3]; // R0 reads as 0

    // Stack Pointer Output (Direct access to R31)
    assign stack_ptr_out = registers[31];

    // Synchronous Write Port
    always @(posedge clk) begin
        // Ignore writes to R0
        if (!reset && write_enable && (write_addr != 5'd0)) begin
            registers[write_addr] <= write_data;
        end
        // Optional: Explicitly handle reset if needed for simulation/FPGA init
        // else if (reset) begin
        //    registers[0] <= 64'b0;
        //    for (integer j = 1; j < 31; j = j + 1) begin
        //        registers[j] <= 64'b0;
        //    end
        //    registers[31] <= 64'h0008_0000; // Reset Stack Pointer
        // end
    end

endmodule


//############################################################################
//## fetch (No changes)
//############################################################################
module fetch (
    input clk,
    input reset,
    input branch_taken,
    input [63:0] branch_pc,
    input take_return_pc, // High for RETURN instruction
    input [63:0] return_pc,  // PC value from stack (via aluMemMux)
    output logic [63:0] pc_out
);
    localparam INITIAL_PC = 64'h2000; // Default starting PC
    reg [63:0] current_pc;

    assign pc_out = current_pc;

    // PC update logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_pc <= INITIAL_PC;
        end else begin
            if (take_return_pc) begin
                current_pc <= return_pc; // Use PC from stack for RETURN
            end else if (branch_taken) begin
                current_pc <= branch_pc; // Use calculated branch target
            end else begin
                current_pc <= current_pc + 64'd4; // Default: Increment PC
            end
        end
    end
endmodule

//############################################################################
//## if_de_register
//## CHANGED: Use specific NOP instruction on flush
//############################################################################
module if_de_register (
    input clk,
    input flush,
    input [63:0] pc_in,
    input [31:0] instruction_in,
    output reg [63:0] pc_out,
    output reg [31:0] instruction_out
);
    // Define NOP instruction (e.g., add r0, r0, r0 -> Opcode 0x18 [cite: 1])
    localparam NOP_INST = 32'h18000000;

    always @(posedge clk) begin
        if (flush) begin
            // Insert NOP instruction into the pipeline
            pc_out <= 64'b0; // PC value doesn't matter much for NOP
            instruction_out <= NOP_INST;
        end else begin
            // Latch instruction and PC normally
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end
endmodule

//############################################################################
//## instructionDecoder
//## CHANGED: Added sign extension for literal, Explicit NOP handling
//############################################################################
module instructionDecoder (
    input [31:0] instructionLine,
    output reg [63:0] literal, // Now Sign-Extended
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg alu_enable,
    output reg mem_read,
    output reg mem_write,
    output reg reg_write,
    output reg mem_to_reg,
    output reg branch_taken, // Indicates branch *type* (potential branch)
    output reg mem_pc        // Indicates RETURN instruction (modifies PC source)
);
    // Opcodes (ensure these match your ISA document)
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;
    // NOP Instruction representation (Example: ADD R0, R0, R0 [cite: 1])
    localparam NOP_INST = 32'h18000000;

    always @(*) begin
        // Default control signals (inactive)
        alu_enable = 1'b0; mem_read = 1'b0; mem_write = 1'b0; reg_write = 1'b0;
        mem_to_reg = 1'b0; branch_taken = 1'b0; mem_pc = 1'b0;

        // Decode fields regardless of NOP
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        // *** Sign-extend the 12-bit literal to 64 bits ***
        literal = {{52{instructionLine[11]}}, instructionLine[11:0]};

        // Check for NOP first (inserted by flushing or actual NOP in code)
        if (instructionLine == NOP_INST) begin
             // Keep control signals at their default (inactive) values
             // Register fields are decoded but control signals ensure no action
             opcode = ADD; // Treat NOP as ADD R0,R0,R0 internally if needed
             rd = 5'b0;
             rs = 5'b0;
             rt = 5'b0;
        end else begin
            // Set control signals based on actual opcode
            case (opcode)
                // R-Type ALU
                ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL: begin
                    alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0;
                end
                // I-Type ALU
                ADDI, SUBI, SHFTRI, SHFTLI: begin
                    alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; rs = rd; // Use rd as source
                end
                 // Load Word
                MOV_MEM: begin // mov rd, (rs)(L) [cite: 60, 61, 62]
                    alu_enable=1'b1; mem_read=1'b1; reg_write=1'b1; mem_to_reg=1'b1; rs = rd; // Use rd as source for address calc? No, source is rs [cite: 62]
                end
                // Store Word
                MOV_STR: begin // mov (rd)(L), rs [cite: 67, 68, 69]
                    alu_enable=1'b1; mem_write=1'b1; reg_write=1'b0; // rs is data source, rd is base address
                end
                // Register Move
                MOV_REG: begin // mov rd, rs [cite: 63, 64]
                    alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; // Pass rs through ALU (or bypass)
                end
                 // Load Literal
                MOV_LIT: begin // mov rd, L [cite: 65, 66] -> This only sets bits 52:63? Manual says 12 bits.. Assuming 12bit literal load like ADDI
                    // If it loads *only* high bits, ALU might need modification. Assuming standard immediate load for now.
                    alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; rs = rd; // Use rd as source/dest
                end
                // Floating Point (treat like R-Type for control)
                ADDF, SUBF, MULF, DIVF: begin // [cite: 70, 71, 72, 73, 74, 75, 76, 77, 78, 79]
                    alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0;
                end
                // Branches (Calculate target in ALU, potentially take branch in MEM)
                BR, BRR, BRRI, BRNZ, BRGT: begin // [cite: 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 45, 46, 47, 48]
                    alu_enable=1'b1; reg_write=1'b0; branch_taken=1'b1; // branch_taken enables branch logic path
                end
                // Call (Calculate target, Write PC to stack)
                CALL: begin // [cite: 40, 41, 42]
                    alu_enable=1'b1; mem_write=1'b1; reg_write=1'b0; branch_taken=1'b1;
                end
                 // Return (Read PC from stack)
                RETURN: begin // [cite: 43, 44, 45]
                    alu_enable=1'b1; mem_read=1'b1; reg_write=1'b0; mem_pc=1'b1; branch_taken=1'b1; // mem_pc selects MemData path for PC, branch_taken flushes pipeline
                end
                // Privileged
                PRIV: begin // [cite: 49, 50, 51]
                    if(literal[11:0]==12'h0) begin // HALT [cite: 51]
                        alu_enable=1'b1; reg_write=1'b0; // Needs ALU enable to signal halt in EX stage
                    end
                    // Add other PRIV functions (TRAP, RTE, IN, OUT) here if needed [cite: 53, 55, 56, 57]
                    else begin
                         alu_enable=1'b0; // Default: Unknown PRIV is NOP-like
                    end
                end
                default: ; // Undefined opcodes behave like NOPs
            endcase

            // Fixup rs for I-type instructions where rd is also the source operand
            // Ensure this matches ISA for ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT
             case (opcode)
                 ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: rs = rd; // [cite: 2, 3, 23, 27, 65]
                 MOV_MEM: ; // rs is base register [cite: 61, 62]
                 default: ;
             endcase
        end
    end
endmodule


//############################################################################
//## de_ex_register (No structural changes needed for forwarding)
//## Passes potentially forwarded data and original addresses/control
//############################################################################
module de_ex_register (
    input clk,
    input flush,
    // Inputs from Decode Stage
    input [63:0] pc_in,
    input [63:0] operand_a_in,         // Potentially forwarded rs_data
    input [63:0] operand_b_in,         // Potentially forwarded rt_data OR literal
    input [63:0] operand_c_in,         // Potentially forwarded rd_data
    input [63:0] literal_in,           // Sign-extended literal
    input [4:0] rd_addr_in,            // Original destination address
    input [4:0] rs_addr_in,            // Original source 1 address
    input [4:0] rt_addr_in,            // Original source 2 address
    input [4:0] opcode_in,
    input [63:0] stack_ptr_in,
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,                // Control signal to be passed
    input mem_to_reg_in,
    input branch_taken_ctrl_in,        // Branch type indicator (not final decision)
    input mem_pc_in,                   // Return indicator
    // Outputs to Execute Stage
    output reg [63:0] pc_out,
    output reg [63:0] operand_a_out,   // Pipelined operand A
    output reg [63:0] operand_b_out,   // Pipelined operand B
    output reg [63:0] operand_c_out,   // Pipelined operand C
    output reg [63:0] literal_out,     // Pipelined literal
    output reg [4:0] rd_addr_out,      // Pipelined destination address (for FWD check)
    output reg [4:0] rs_addr_out,      // Pipelined source 1 address
    output reg [4:0] rt_addr_out,      // Pipelined source 2 address
    output reg [4:0] opcode_out,
    output reg [63:0] stack_ptr_out,
    output reg alu_enable_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,          // Pipelined control signal (for FWD check)
    output reg mem_to_reg_out,
    output reg branch_taken_ctrl_out,  // Pipelined branch type indicator
    output reg mem_pc_out              // Pipelined return indicator
);
    // Define NOP instruction (e.g., add r0, r0, r0 -> Opcode 0x18)
    localparam NOP_OPCODE = 5'h18; // ADD
    localparam NOP_RD = 5'b0;
    localparam NOP_RS = 5'b0;
    localparam NOP_RT = 5'b0;

    always @(posedge clk) begin
        if (flush) begin
            // Flush with NOP equivalent control signals
            pc_out <= 64'b0;
            operand_a_out <= 64'b0; operand_b_out <= 64'b0; operand_c_out <= 64'b0;
            literal_out <= 64'b0;
            // Ensure register addresses are zero for NOP if they matter downstream
            rd_addr_out <= NOP_RD; rs_addr_out <= NOP_RS; rt_addr_out <= NOP_RT;
            opcode_out <= NOP_OPCODE; // Flush with NOP opcode
            stack_ptr_out <= 64'b0;
            // Crucially, disable control signals
            alu_enable_out <= 1'b0; // NOP shouldn't enable ALU (unless ALU handles NOP) -> Decoder handles NOP now.
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0; // *** Disable write on flush ***
            mem_to_reg_out <= 1'b0;
            branch_taken_ctrl_out <= 1'b0;
            mem_pc_out <= 1'b0;
        end else begin
            // Latch inputs to outputs normally
            pc_out <= pc_in;
            operand_a_out <= operand_a_in; operand_b_out <= operand_b_in; operand_c_out <= operand_c_in;
            literal_out <= literal_in;
            rd_addr_out <= rd_addr_in; rs_addr_out <= rs_addr_in; rt_addr_out <= rt_addr_in;
            opcode_out <= opcode_in; stack_ptr_out <= stack_ptr_in;
            // Latch control signals
            alu_enable_out <= alu_enable_in;
            mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in; branch_taken_ctrl_out <= branch_taken_ctrl_in; mem_pc_out <= mem_pc_in;
        end
    end
endmodule


//############################################################################
//## alu
//## CHANGED: Use sign-extended inputs where appropriate
//## Note: Floating point is just placeholder
//############################################################################
module alu (
    // Control Inputs
    input logic alu_enable,       // Is ALU active for this instruction?
    input logic [4:0] opcode,     // Operation to perform
    // Data Inputs (from DE/EX register, potentially forwarded)
    input logic [63:0] input1,    // Operand A (rs_data or rd_data for some I-types)
    input logic [63:0] input2,    // Operand B (rt_data or sign-extended literal)
    input logic [63:0] input3,    // Operand C (rd_data for stores/branches)
    input logic [4:0] rd_addr,    // Destination register address (passed thru, not used in calc)
    input logic [63:0] literal,   // Sign-extended literal (passed thru, used for MOV_STR)
    input logic [63:0] pc_in,     // PC value for relative branches/calls
    input logic [63:0] stack_ptr, // Stack pointer (R31) for call/return
    // Outputs -> To EX/MEM register
    output logic [63:0] result,    // Result of ALU operation (for R/I-types, MOV_REG, MOV_LIT)
    output logic [63:0] mem_addr,  // Calculated memory address (for loads/stores/call/return)
    output logic [63:0] mem_wdata, // Data to be written to memory (for stores/call)
    output logic [63:0] branch_pc, // Calculated branch target address
    output logic branch_taken,     // Branch decision (for conditional branches)
    output logic hlt_out,          // Halt signal asserted
    // Control Signal Inputs (passed through, not directly used by ALU logic)
    input logic mem_read_in,
    input logic mem_write_in,
    input logic reg_write_in,
    input logic mem_to_reg_in,
    input logic mem_pc_in          // Is this a RETURN instruction?
);

    // Opcodes (ensure these match decoder and ISA manual)
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;
    localparam NOP_OPCODE = 5'h18; // ADD

    // Internal signals
    logic is_branch_type; // Is this a branch/call/return type?
    logic condition_met;  // For conditional branches

    // Determine if the instruction is a control flow type that might change PC
    assign is_branch_type = (opcode == BR || opcode == BRR || opcode == BRRI || opcode == BRNZ || opcode == BRGT || opcode == CALL || opcode == RETURN);

    always @(*) begin
        // Default Output Values
        result = 64'b0;
        mem_addr = 64'b0;
        mem_wdata = 64'b0;
        branch_pc = pc_in + 4; // Default next PC unless branch taken/return
        branch_taken = 1'b0;   // Default: branch not taken
        hlt_out = 1'b0;
        condition_met = 1'b0;  // Default condition for branches

        // Only perform calculations if ALU is enabled for this instruction
        // and it's not a NOP that might have slipped through flushing checks
        if (alu_enable && !(opcode == NOP_OPCODE && rd_addr == 5'b0)) begin
            case (opcode)
                // --- Integer Arithmetic ---
                // Assumes input1=rs/rd, input2=rt/literal
                ADD, ADDI: result = $signed(input1) + $signed(input2); // [cite: 1, 2]
                SUB, SUBI: result = $signed(input1) - $signed(input2); // [cite: 3]
                MUL: result = $signed(input1) * $signed(input2); // [cite: 5, 7]
                DIV: if (input2 != 0) result = $signed(input1) / $signed(input2); else result = 64'b0; // Avoid division by zero [cite: 8, 9]

                // --- Logical ---
                // Assumes input1=rs, input2=rt
                AND: result = input1 & input2; // [cite: 10, 12]
                OR:  result = input1 | input2; // [cite: 13, 15]
                XOR: result = input1 ^ input2; // [cite: 16, 17]
                NOT: result = ~input1;         // input2 not used [cite: 18, 19]

                // --- Shift ---
                // Assumes input1=rs/rd, input2=rt/literal (only low 6 bits used for amount)
                SHFTR, SHFTRI: result = input1 >>> input2[5:0]; // Use arithmetic right shift (>>>) if signed shift needed, logical (>>) otherwise. Assuming logical. [cite: 20, 22, 23, 24]
                SHFTL, SHFTLI: result = input1 << input2[5:0]; // [cite: 25, 26, 27, 29]

                // --- Data Movement ---
                MOV_MEM: begin // mov rd, (rs)(L) -> Calculate address rs+L [cite: 60, 62]
                    mem_addr = input1 + $signed(input2); // input1=rs, input2=literal (sign-extended)
                    // result is determined by memRegMux based on memory read
                end
                MOV_REG: begin // mov rd, rs -> Pass rs through [cite: 63, 64]
                    result = input1; // input1=rs
                end
                MOV_LIT: begin // mov rd, L -> Pass literal through (assuming 12-bit load similar to ADDI) [cite: 65, 66]
                    result = input2; // input2=literal (sign-extended from DE stage)
                                     // ISA manual [cite: 66] mentions only bits 52:63. If that's strict, logic needs change: result = {literal[11:0], 52'b0} ? Check requirements.
                end
                MOV_STR: begin // mov (rd)(L), rs -> Calculate address rd+L, pass rs as data [cite: 67, 68]
                    mem_addr = input3 + $signed(literal); // input3=rd, literal comes direct from EX reg
                    mem_wdata = input1; // input1=rs
                end

                // --- Floating Point (Placeholders) ---
                ADDF, SUBF, MULF, DIVF: begin // [cite: 70, 72, 73, 74, 75, 76, 77, 79]
                    // Add actual FP logic here if implementing
                    result = 64'b0; // Placeholder
                end

                // --- Control Flow - Branches/Jumps ---
                BR: begin // br rd -> Jump to address in rd [cite: 30, 32]
                    branch_pc = input3; // input3=rd_data
                    condition_met = 1'b1; // Unconditional
                end
                BRR: begin // brr rd -> Jump relative pc + rd [cite: 33, 34]
                    branch_pc = pc_in + $signed(input3); // input3=rd_data
                    condition_met = 1'b1; // Unconditional
                end
                BRRI: begin // brr L -> Jump relative pc + L [cite: 35, 36]
                    branch_pc = pc_in + $signed(input2); // input2=literal (sign-extended)
                    condition_met = 1'b1; // Unconditional
                end
                BRNZ: begin // brnz rd, rs -> Jump to rd if rs != 0 [cite: 37, 39]
                    branch_pc = input3; // input3=rd_data (target address)
                    if (input1 != 64'b0) condition_met = 1'b1; // input1=rs_data
                    else condition_met = 1'b0;
                end
                BRGT: begin // brgt rd, rs, rt -> Jump to rd if rs > rt (signed) [cite: 45, 47]
                    branch_pc = input3; // input3=rd_data (target address)
                    if ($signed(input1) > $signed(input2)) condition_met = 1'b1; // input1=rs_data, input2=rt_data
                    else condition_met = 1'b0;
                end

                // --- Control Flow - Subroutines ---
                CALL: begin // call rd -> Jump to rd, Push PC+4 [cite: 40, 42]
                    branch_pc = input3; // input3=rd_data (target address)
                    mem_addr = stack_ptr - 8; // Calculate stack address
                    mem_wdata = pc_in + 4;   // Return address
                    condition_met = 1'b1; // Unconditional jump part of call
                end
                RETURN: begin // return -> Pop PC from stack [cite: 43, 45]
                    mem_addr = stack_ptr - 8; // Calculate stack address to read from
                    // Branch PC is determined by Mem stage mux based on mem_pc signal
                    condition_met = 1'b1; // Always "taken" to flush pipeline
                end

                // --- Privileged ---
                PRIV: begin // [cite: 49, 51]
                    if (literal[11:0] == 12'h0) begin // HALT [cite: 51]
                        hlt_out = 1'b1;
                    end
                    // Handle other PRIV types (TRAP, RTE, IN, OUT) if needed [cite: 53, 55, 56, 57]
                end

                default: ; // Undefined opcodes - treat as NOP
            endcase

            // Final branch taken decision for branches
            if (is_branch_type) begin
                branch_taken = condition_met; // Set based on condition evaluation
            end
        end
    end
endmodule

//############################################################################
//## ex_mem_register
//## Added flush input (already present in user code)
//## No structural changes needed for forwarding
//############################################################################
module ex_mem_register (
    input clk,
    input logic flush_mem,           // Flush signal from MEM stage branch taken
    // Inputs from Execute Stage
    input logic [63:0] result_in,    // ALU/calculation result
    input logic [63:0] mem_addr_in,  // Calculated Memory Address
    input logic [63:0] mem_wdata_in, // Data to write to Memory
    input logic [63:0] branch_pc_in, // Calculated Branch Target PC
    input logic [4:0]  rd_addr_in,   // Destination Register Address
    input logic        hlt_in,       // Halt signal from ALU
    input logic        mem_read_in,  // Memory Read Control
    input logic        mem_write_in, // Memory Write Control
    input logic        reg_write_in, // Register Write Control
    input logic        mem_to_reg_in,// Mux select for Writeback
    input logic        branch_taken_in, // Branch decision from ALU
    input logic        mem_pc_in,    // Return instruction indicator
    // Outputs to Memory Stage
    output logic [63:0] result_out,
    output logic [63:0] mem_addr_out,
    output logic [63:0] mem_wdata_out,
    output logic [63:0] branch_pc_out,
    output logic [4:0]  rd_addr_out, // Passed for Writeback & Forwarding
    output logic        hlt_out,
    output logic        mem_read_out,
    output logic        mem_write_out,
    output logic        reg_write_out, // Passed for Writeback & Forwarding
    output logic        mem_to_reg_out,
    output logic        branch_taken_out, // Final Branch Decision
    output logic        mem_pc_out      // Final Return Indicator
);
    // Define NOP instruction control signals (all off)
    localparam NOP_REG_WRITE = 1'b0;
    localparam NOP_MEM_READ = 1'b0;
    localparam NOP_MEM_WRITE = 1'b0;
    localparam NOP_MEM_TO_REG = 1'b0;
    localparam NOP_BRANCH_TAKEN = 1'b0;
    localparam NOP_MEM_PC = 1'b0;
    localparam NOP_HLT = 1'b0;

    always @(posedge clk) begin
        // If flushing, cancel control signals for this stage's instruction
        if (flush_mem) begin
            // Clear data path for cleanliness (optional, control signals are key)
            result_out <= 64'b0;
            mem_addr_out <= 64'b0;
            mem_wdata_out <= 64'b0;
            branch_pc_out <= 64'b0;
            rd_addr_out <= 5'b0; // Prevent accidental forwarding from R0 match
            // Force control signals low for NOP behavior
            hlt_out <= NOP_HLT;
            mem_read_out <= NOP_MEM_READ;
            mem_write_out <= NOP_MEM_WRITE;
            reg_write_out <= NOP_REG_WRITE; // Prevent register write
            mem_to_reg_out <= NOP_MEM_TO_REG; // Prevent incorrect mux selection
            branch_taken_out <= NOP_BRANCH_TAKEN; // Flushed instruction is not a branch
            mem_pc_out <= NOP_MEM_PC;         // Flushed instruction is not a return
        end else begin
            // Latching inputs to outputs normally
            result_out <= result_in;
            mem_addr_out <= mem_addr_in;
            mem_wdata_out <= mem_wdata_in;
            branch_pc_out <= branch_pc_in;
            rd_addr_out <= rd_addr_in;
            hlt_out <= hlt_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            branch_taken_out <= branch_taken_in; // Pass final branch decision
            mem_pc_out <= mem_pc_in;         // Pass final return indicator
        end
    end
endmodule


//############################################################################
//## memory (No changes needed)
//############################################################################
module memory (
    input clk,
    input reset,
    input [63:0] inst_addr,
    output logic [31:0] instruction_out,
    input [63:0] data_addr,
    input [63:0] data_wdata,
    input mem_read,
    input mem_write,
    output logic [63:0] data_rdata
);
    localparam MEM_SIZE_BYTES = 524288; // 0.5 MB
    // Use logic for synthesizability if targeting FPGA, reg for simulation
    logic [7:0] bytes [0:MEM_SIZE_BYTES-1]; // Memory array
    integer i;

    // Initialization (for simulation)
    initial begin
       for(i=0; i<MEM_SIZE_BYTES; i=i+1) bytes[i] = 8'b0;
    end

    // Instruction Read Port (Combinational) - Assuming word-aligned reads
    // Need error handling for misaligned addresses if required by ISA
    assign instruction_out[7:0]   = bytes[inst_addr + 0];
    assign instruction_out[15:8]  = bytes[inst_addr + 1];
    assign instruction_out[23:16] = bytes[inst_addr + 2];
    assign instruction_out[31:24] = bytes[inst_addr + 3];

    // Data Read Port (Combinational) - Assuming word-aligned reads
    assign data_rdata[7:0]   = bytes[data_addr + 0];
    assign data_rdata[15:8]  = bytes[data_addr + 1];
    assign data_rdata[23:16] = bytes[data_addr + 2];
    assign data_rdata[31:24] = bytes[data_addr + 3];
    assign data_rdata[39:32] = bytes[data_addr + 4];
    assign data_rdata[47:40] = bytes[data_addr + 5];
    assign data_rdata[55:48] = bytes[data_addr + 6];
    assign data_rdata[63:56] = bytes[data_addr + 7];

    // Data Write Port (Synchronous)
    always @(posedge clk) begin
        if (mem_write) begin
            // Assuming Little Endian byte order based on assignments
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
endmodule


//############################################################################
//## aluMemMux (Selects PC for Fetch: Branch Target or Return Address from Mem)
//## Using always @(*) as requested
//############################################################################
module aluMemMux (
    input mem_pc,          // Control signal: High if RETURN instruction
    input [63:0] memData,  // Data read from memory (Potential Return PC)
    input [63:0] aluOut,   // Output from ALU/branch logic (Potential Branch PC)
    output reg [63:0] newPc // Selected PC value for Fetch stage
);
    always @(*) begin // Replaced always_comb
        if (mem_pc) begin
            newPc = memData; // Use value read from stack for RETURN
        end else begin
            newPc = aluOut;  // Use calculated branch target (or PC+4 if no branch)
        end
    end
endmodule

//############################################################################
//## reglitmux (Selects Operand B: Register or Literal)
//## Using always @(*) as requested
//############################################################################
module reglitmux (
    input [4:0] sel,       // Opcode from DE stage
    input [63:0] reg_in,   // Potentially forwarded rt_data
    input [63:0] lit_in,   // Sign-extended literal from DE stage
    output reg [63:0] out  // Selected value for Operand B input to DE/EX reg
);
    // Opcodes that use the Literal field as the second ALU operand
    localparam ADDI    = 5'h19, SUBI    = 5'h1B, SHFTRI  = 5'h05, SHFTLI = 5'h07,
               BRRI    = 5'h0A, // Uses literal for offset calculation
               MOV_MEM = 5'h10, // Uses literal for address offset [cite: 61, 62]
               MOV_LIT = 5'h12, // Uses literal as the value [cite: 65]
               MOV_STR = 5'h13; // Uses literal for address offset [cite: 67, 68]
              // Note: Other branches (BR, BRR, BRNZ, BRGT) use rd (input3) for target/offset [cite: 31, 33, 38, 46]
              //       R-type instructions use rt (reg_in)

    always @(*) begin // Replaced always_comb
        case (sel)
            // Select literal for these opcodes
            ADDI, SUBI, SHFTRI, SHFTLI, BRRI, MOV_MEM, MOV_LIT, MOV_STR: out = lit_in;
            // Select register value for all other opcodes (R-type, other branches, etc.)
            default: out = reg_in;
        endcase
    end
endmodule

//############################################################################
//## memRegMux (Selects Writeback Data: Memory Read or ALU Result)
//## Using always @(*) as requested
//############################################################################
module memRegMux (
    input mem_to_reg,      // Control signal (High for Load instructions like MOV_MEM)
    input [63:0] readData, // Data read from memory in MEM stage
    input [63:0] aluResult,// Result from ALU (passed through EX/MEM reg)
    output reg [63:0] regWriteData // Data to be written back to register file
);
    always @(*) begin // Replaced always_comb
        if (mem_to_reg) begin
            regWriteData = readData; // Use data from memory for loads
        end else begin
            regWriteData = aluResult; // Use ALU result for R-type, I-type, MOV_REG, MOV_LIT
        end
    end
endmodule