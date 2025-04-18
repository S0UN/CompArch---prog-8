//############################################################################
//## tinker_core (Top Level Module)
//## CHANGED: Added Forwarding Logic
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
    logic [63:0] regfile_operand_a_de; // rs_data from reg file
    logic [63:0] regfile_operand_b_de; // rt_data from reg file (before mux)
    logic [63:0] regfile_operand_c_de; // rd_data from reg file (read via Port 3)
    logic [63:0] operand_a_de;         // rs_data potentially forwarded
    logic [63:0] operand_b_de;         // rt_data or literal (after reglitmux)
    logic [63:0] operand_c_de;         // rd_data potentially forwarded
    logic [63:0] operand_b_reg_maybe_fwd; // rt_data potentially forwarded (input to reglitmux)

    logic [63:0] literal_de;
    logic [4:0]  rd_addr_de;
    logic [4:0]  rs_addr_de;
    logic [4:0]  rt_addr_de;
    logic [4:0]  opcode_de;
    logic [63:0] stack_ptr_de;
    logic        alu_enable_de, mem_read_de, mem_write_de, reg_write_de, mem_to_reg_de, branch_taken_ctrl_de, mem_pc_de; // Control signals

    // DE/EX Register Outputs -> Inputs to EX Stage
    logic [63:0] pc_ex;
    logic [63:0] operand_a_ex; // Pipelined rs_data (forwarded or from regfile)
    logic [63:0] operand_b_ex; // Pipelined rt_data or literal (operand_b_reg potentially forwarded)
    logic [63:0] operand_c_ex; // Pipelined rd_data (forwarded or from regfile)
    logic [63:0] literal_ex;
    logic [4:0]  rd_addr_ex; // Pipelined write address (destination)
    logic [4:0]  rs_addr_ex; // Pipelined rs address
    logic [4:0]  rt_addr_ex; // Pipelined rt address
    logic [4:0]  opcode_ex;
    logic [63:0] stack_ptr_ex;
    logic        alu_enable_ex, mem_read_ex, mem_write_ex, reg_write_ex, mem_to_reg_ex, mem_pc_ex; // Control signals

    // EX Stage Outputs -> Inputs to EX/MEM Register
    logic [63:0] alu_result_ex;
    logic [63:0] alu_mem_addr_ex;
    logic [63:0] alu_mem_data_ex; // Data to be stored (for MOV_STR)
    logic [63:0] alu_branch_pc_ex;
    logic        branch_taken_ex;
    logic        hlt_ex;

    // EX/MEM Register Outputs -> Inputs to MEM Stage
    logic [63:0] alu_result_mem;
    logic [63:0] mem_addr_mem;
    logic [63:0] mem_wdata_mem; // Pipelined data to be stored
    logic [63:0] branch_pc_mem;
    logic [4:0]  rd_addr_mem; // Pipelined write address (destination)
    logic        mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, branch_taken_mem, mem_pc_mem; // Control
    logic        hlt_mem;

    // MEM Stage Outputs
    logic [63:0] mem_rdata_mem;
    logic [63:0] return_pc_mem;

    // MEM/WB Stage (Writeback Data)
    logic [63:0] reg_wdata_wb; // Data selected for register write

    // --- Forwarding Signals --- // FORWARDING ADDED
    logic [1:0] forward_a_select;
    logic [1:0] forward_b_select;
    logic [1:0] forward_c_select;

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

    memory memory ( // Renamed instance for clarity
        .clk(clk),
        .reset(reset),
        .inst_addr(pc_if),
        .instruction_out(instruction_if),
        .data_addr(mem_addr_mem),
        .data_wdata(mem_wdata_mem), // Use pipelined write data from EX/MEM
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
        .branch_taken(branch_taken_ctrl_de), // Control signal indicating potential branch type
        .mem_pc(mem_pc_de)
    );

    registerFile reg_file (
        .clk(clk),
        .reset(reset),
        // Write Port (from MEM stage completion)
        .write_addr(rd_addr_mem),     // Destination register address from MEM stage
        .write_data(reg_wdata_wb),   // Data selected for write back
        .write_enable(reg_write_mem), // Write enable signal from MEM stage
        // Read Port 1 (rs) - Output to Forwarding Mux A
        .read_addr1(rs_addr_de),
        .read_data1(regfile_operand_a_de),
        // Read Port 2 (rt) - Output to Forwarding Mux B
        .read_addr2(rt_addr_de),
        .read_data2(regfile_operand_b_de),
        // Read Port 3 (rd) - Output to Forwarding Mux C
        .read_addr3(rd_addr_de),
        .read_data3(regfile_operand_c_de),
        // Stack Pointer
        .stack_ptr_out(stack_ptr_de)
    );

    // --- Forwarding Unit --- // FORWARDING ADDED
    forwardingUnit fwd_unit (
        .rs_addr_de(rs_addr_de),
        .rt_addr_de(rt_addr_de),
        .rd_addr_de(rd_addr_de), // Source address for operand C
        .rd_addr_ex(rd_addr_ex),
        .reg_write_ex(reg_write_ex),
        .rd_addr_mem(rd_addr_mem),
        .reg_write_mem(reg_write_mem),
        .forward_a_select(forward_a_select),
        .forward_b_select(forward_b_select),
        .forward_c_select(forward_c_select)
    );

    // --- Forwarding Multiplexers --- // FORWARDING ADDED

    // Forwarding Mux for Operand A (rs_data)
    forwardingMux forwardingMuxA (
        .select(forward_a_select),
        .data_regfile(regfile_operand_a_de),
        .data_ex(alu_result_ex),       // Forward ALU result from EX
        .data_mem(reg_wdata_wb),       // Forward Writeback data from MEM/WB
        .forwarded_data(operand_a_de)  // Output to DE/EX register input A
    );

    // Forwarding Mux for Operand B (rt_data - before reglitmux)
    forwardingMux forwardingMuxB (
        .select(forward_b_select),
        .data_regfile(regfile_operand_b_de),
        .data_ex(alu_result_ex),       // Forward ALU result from EX
        .data_mem(reg_wdata_wb),       // Forward Writeback data from MEM/WB
        .forwarded_data(operand_b_reg_maybe_fwd) // Output to reglitmux input
    );

    // Forwarding Mux for Operand C (rd_data - read port 3)
    forwardingMux forwardingMuxC (
        .select(forward_c_select),
        .data_regfile(regfile_operand_c_de),
        .data_ex(alu_result_ex),       // Forward ALU result from EX
        .data_mem(reg_wdata_wb),       // Forward Writeback data from MEM/WB
        .forwarded_data(operand_c_de)  // Output to DE/EX register input C
    );

    // Multiplexer to select between register data (potentially forwarded) and literal
    reglitmux input_selector (
        .sel(opcode_de),
        .reg_in(operand_b_reg_maybe_fwd), // Use potentially forwarded rt_data
        .lit_in(literal_de),
        .out(operand_b_de)            // Output to DE/EX register input B
    );

    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush_ex),
        // Inputs from Decode Stage (potentially forwarded)
        .pc_in(pc_de),
        .operand_a_in(operand_a_de), // From Forwarding Mux A
        .operand_b_in(operand_b_de), // From reglitmux (using forwarded B reg data)
        .operand_c_in(operand_c_de), // From Forwarding Mux C
        .literal_in(literal_de),
        .rd_addr_in(rd_addr_de),     // Destination address
        .rs_addr_in(rs_addr_de),     // Source address 1
        .rt_addr_in(rt_addr_de),     // Source address 2
        .opcode_in(opcode_de),
        .stack_ptr_in(stack_ptr_de),
        .alu_enable_in(alu_enable_de),
        .mem_read_in(mem_read_de),
        .mem_write_in(mem_write_de),
        .reg_write_in(reg_write_de), // Control signals passed through
        .mem_to_reg_in(mem_to_reg_de),
        .branch_taken_ctrl_in(branch_taken_ctrl_de),
        .mem_pc_in(mem_pc_de),
        // Outputs to Execute Stage
        .pc_out(pc_ex),
        .operand_a_out(operand_a_ex),
        .operand_b_out(operand_b_ex),
        .operand_c_out(operand_c_ex),
        .literal_out(literal_ex),
        .rd_addr_out(rd_addr_ex),     // Pipelined destination address
        .rs_addr_out(rs_addr_ex),
        .rt_addr_out(rt_addr_ex),
        .opcode_out(opcode_ex),
        .stack_ptr_out(stack_ptr_ex),
        .alu_enable_out(alu_enable_ex),
        .mem_read_out(mem_read_ex),
        .mem_write_out(mem_write_ex),
        .reg_write_out(reg_write_ex), // Control signals passed through
        .mem_to_reg_out(mem_to_reg_ex),
        .branch_taken_ctrl_out(),      // This output seems unused now
        .mem_pc_out(mem_pc_ex)
    );

    alu calculation_unit (
        .alu_enable(alu_enable_ex),
        .opcode(opcode_ex),
        // Data Inputs from DE/EX (potentially forwarded values)
        .input1(operand_a_ex), // Pipelined rs_data
        .input2(operand_b_ex), // Pipelined rt_data or literal
        .input3(operand_c_ex), // Pipelined rd_data
        .rd_addr(rd_addr_ex),  // Pipelined destination register address
        .literal(literal_ex),
        .pc_in(pc_ex),
        .stack_ptr(stack_ptr_ex),
        // Outputs to EX/MEM Register
        .result(alu_result_ex),
        .mem_addr(alu_mem_addr_ex),
        .mem_wdata(alu_mem_data_ex), // For MOV_STR
        .branch_pc(alu_branch_pc_ex),
        .branch_taken(branch_taken_ex), // ALU determines if branch is actually taken
        .hlt_out(hlt_ex),
        // Control Inputs Pass-through (needed if ALU logic used them - not strictly needed here but good practice)
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .mem_pc_in(mem_pc_ex)
    );

    ex_mem_register ex_mem_reg (
        .clk(clk),
        .flush_mem(flush_ex),
        // Inputs from Execute Stage
        .result_in(alu_result_ex),
        .mem_addr_in(alu_mem_addr_ex),
        .mem_wdata_in(alu_mem_data_ex), // Pass store data
        .branch_pc_in(alu_branch_pc_ex),
        .rd_addr_in(rd_addr_ex), // Pass destination addr
        .hlt_in(hlt_ex),
        // Control Inputs (from DE/EX outputs)
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .branch_taken_in(branch_taken_ex), // Decision from ALU
        .mem_pc_in(mem_pc_ex),
        // Outputs to Memory Stage
        .result_out(alu_result_mem),
        .mem_addr_out(mem_addr_mem),
        .mem_wdata_out(mem_wdata_mem), // Output store data
        .branch_pc_out(branch_pc_mem),
        .rd_addr_out(rd_addr_mem), // Output destination addr
        .hlt_out(hlt_mem),
        .mem_read_out(mem_read_mem),
        .mem_write_out(mem_write_mem),
        .reg_write_out(reg_write_mem),
        .mem_to_reg_out(mem_to_reg_mem),
        .branch_taken_out(branch_taken_mem), // Pipelined branch decision used for flush/fetch
        .mem_pc_out(mem_pc_mem)
    );

    // Mux for RETURN instruction PC source
    aluMemMux return_pc_selector (
        .mem_pc(mem_pc_mem),       // Is it a RETURN instruction?
        .memData(mem_rdata_mem),  // PC from Memory (stack)
        .aluOut(branch_pc_mem),   // PC from ALU (for normal branches)
        .newPc(return_pc_mem)     // Selected PC for Fetch stage mux
    );

    // Mux for Register Writeback Data Source
    memRegMux data_source_selector (
        .mem_to_reg(mem_to_reg_mem), // Control signal from MEM stage
        .readData(mem_rdata_mem),   // Data from Memory Read
        .aluResult(alu_result_mem), // Data from ALU result (pipelined)
        .regWriteData(reg_wdata_wb) // Output to Register File Write Port
    );

    // --- Flush Logic ---
    // Flush DE and EX stages if a branch is taken in the MEM stage
    assign flush_de = branch_taken_mem;
    assign flush_ex = branch_taken_mem;

    // --- Fetch PC Select Logic ---
    // Select return PC if RETURN instruction is in MEM stage
    assign take_return_pc_fetch = mem_pc_mem; // Use the control signal passed from MEM

    // --- Halt Signal ---
    assign hlt = hlt_mem; // Halt signal comes from MEM stage

endmodule

//############################################################################
//## forwardingUnit (NEW MODULE)
//## Detects hazards and determines forwarding paths.
//############################################################################
module forwardingUnit (
    // Inputs from DE stage (read addresses)
    input logic [4:0] rs_addr_de,
    input logic [4:0] rt_addr_de,
    input logic [4:0] rd_addr_de, // For read port 3 usage (e.g., MOV_STR base, BR target)

    // Inputs from EX stage (write address & control)
    input logic [4:0] rd_addr_ex,
    input logic       reg_write_ex,

    // Inputs from MEM stage (write address & control)
    input logic [4:0] rd_addr_mem,
    input logic       reg_write_mem,

    // Outputs: Mux select signals (2'b00: RegFile, 2'b01: EX, 2'b10: MEM)
    output logic [1:0] forward_a_select, // For operand A (rs)
    output logic [1:0] forward_b_select, // For operand B (rt)
    output logic [1:0] forward_c_select  // For operand C (rd read port 3)
);

    // Use always @(*) for combinational logic as requested
    always @(*) begin
        // Default: No forwarding (select from Register File)
        forward_a_select = 2'b00;
        forward_b_select = 2'b00;
        forward_c_select = 2'b00;

        // --- Forwarding Logic for Operand A (rs_addr_de) ---

        // EX Hazard Check for A: Forward from EX if EX writes to a non-zero register
        // matching rs_addr_de
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rs_addr_de)) begin
            forward_a_select = 2'b01; // Forward EX result (ALU output)
        end
        // MEM Hazard Check for A: Forward from MEM if MEM writes to a non-zero register
        // matching rs_addr_de, AND no EX hazard exists for the same register
        else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rs_addr_de)) begin
             forward_a_select = 2'b10; // Forward MEM result (Writeback data)
        end

        // --- Forwarding Logic for Operand B (rt_addr_de) ---

        // EX Hazard Check for B: Forward from EX if EX writes to a non-zero register
        // matching rt_addr_de
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rt_addr_de)) begin
            forward_b_select = 2'b01; // Forward EX result
        end
        // MEM Hazard Check for B: Forward from MEM if MEM writes to a non-zero register
        // matching rt_addr_de, AND no EX hazard exists for the same register
        else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rt_addr_de)) begin
            forward_b_select = 2'b10; // Forward MEM result
        end

        // --- Forwarding Logic for Operand C (rd_addr_de - used as source) ---
        // This handles cases like MOV_STR where rd is a source base address,
        // or branches like BR, BRNZ, BRGT where rd holds the target address.

        // EX Hazard Check for C: Forward from EX if EX writes to a non-zero register
        // matching rd_addr_de (when rd is used as a source in DE)
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rd_addr_de)) begin
            forward_c_select = 2'b01; // Forward EX result
        end
        // MEM Hazard Check for C: Forward from MEM if MEM writes to a non-zero register
        // matching rd_addr_de, AND no EX hazard exists for the same register
        else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rd_addr_de)) begin
           forward_c_select = 2'b10; // Forward MEM result
        end
    end

endmodule

//############################################################################
//## forwardingMux (NEW MODULE)
//## Generic 3-input mux for forwarding data.
//############################################################################
module forwardingMux (
    input logic [1:0] select,        // 00: RegFile, 01: EX, 10: MEM
    input logic [63:0] data_regfile, // Data from Register File read port
    input logic [63:0] data_ex,      // Data from EX stage (ALU result)
    input logic [63:0] data_mem,     // Data from MEM/WB stage (Writeback data)
    output logic [63:0] forwarded_data // Selected data
);

    // Use always @(*) for combinational logic as requested
    always @(*) begin
        case (select)
            2'b00: forwarded_data = data_regfile; // Select Register File data
            2'b01: forwarded_data = data_ex;      // Select EX stage data
            2'b10: forwarded_data = data_mem;     // Select MEM/WB stage data
            default: forwarded_data = data_regfile; // Default to RegFile data
        endcase
    end

endmodule

//############################################################################
//## registerFile (No changes needed based on request)
//## Reads committed state only, forwarding happens outside.
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
    initial begin
        for (idx = 0; idx < 31; idx = idx + 1) begin
            registers[idx] = 64'b0;
        end
        registers[31] = 64'h0008_0000; // Example Stack Pointer Init
    end

    // Combinational Read Port 1 (rs)
    assign read_data1 = (read_addr1 == 5'd31) ? registers[31] : registers[read_addr1];

    // Combinational Read Port 2 (rt)
    assign read_data2 = (read_addr2 == 5'd31) ? registers[31] : registers[read_addr2];

    // Combinational Read Port 3 (rd)
    assign read_data3 = (read_addr3 == 5'd31) ? registers[31] : registers[read_addr3];

    // Stack Pointer Output
    assign stack_ptr_out = registers[31];

    // Synchronous Write Port
    always @(posedge clk) begin
        if (!reset && write_enable) begin // R0 is writable as requested
            registers[write_addr] <= write_data;
        end
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
    input take_return_pc,
    input [63:0] return_pc,
    output logic [63:0] pc_out
);
    localparam INITIAL_PC = 64'h2000;
    reg [63:0] current_pc;
    assign pc_out = current_pc;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_pc <= INITIAL_PC;
        end else begin
            if (take_return_pc) current_pc <= return_pc;       // Priority for RETURN
            else if (branch_taken) current_pc <= branch_pc;    // Then normal branches
            else current_pc <= current_pc + 64'd4;             // Default increment
        end
    end
endmodule

//############################################################################
//## if_de_register (No changes)
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
            pc_out <= 64'b0; // Flush PC
            instruction_out <= 32'b0; // Flush instruction (NOP)
        end else begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end
endmodule

//############################################################################
//## instructionDecoder (No changes)
//############################################################################
module instructionDecoder (
    input [31:0] instructionLine,
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg alu_enable,
    output reg mem_read,
    output reg mem_write,
    output reg reg_write,
    output reg mem_to_reg,
    output reg branch_taken, // Indicates potential branch type
    output reg mem_pc        // Indicates RETURN type
);
    // Opcodes
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;

    // Use always @(*) for combinational logic as requested
    always @(*) begin
        // Default control signals
        alu_enable = 1'b0; mem_read = 1'b0; mem_write = 1'b0; reg_write = 1'b0;
        mem_to_reg = 1'b0; branch_taken = 1'b0; mem_pc = 1'b0;

        // Decode fields
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        literal = {{52{instructionLine[11]}}, instructionLine[11:0]}; // Sign-extend literal

        // Decode control signals based on opcode
        case (opcode)
            ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end // R-type ALU
            ADDI, SUBI, SHFTRI, SHFTLI: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; rs = rd; end // I-type ALU (rs<-rd)
            MOV_MEM: begin alu_enable=1'b1; mem_read=1'b1; reg_write=1'b1; mem_to_reg=1'b1; end // Load Word (uses rs, literal)
            MOV_STR: begin alu_enable=1'b1; mem_write=1'b1; reg_write=1'b0; end // Store Word (uses rd, rs, literal)
            MOV_REG: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end // Register Move (uses rs)
            MOV_LIT: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; rs = rd; end // Literal Move (uses rd, literal)
            ADDF, SUBF, MULF, DIVF: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end // FP (Treat as ALU for control)
            BR, BRR, BRRI, BRNZ, BRGT: begin alu_enable=1'b1; reg_write=1'b0; branch_taken=1'b1; end // Branches (ALU calculates target/condition)
            CALL: begin alu_enable=1'b1; mem_write=1'b1; reg_write=1'b0; branch_taken=1'b1; end // Call (ALU calc stack addr, uses rd for target)
            RETURN: begin alu_enable=1'b1; mem_read=1'b1; reg_write=1'b0; mem_pc=1'b1; branch_taken=1'b1; end // Return (ALU calc stack addr, mem_pc signals special fetch)
            PRIV: begin // Privileged
                      if(literal[11:0]==12'h0) begin // HALT
                          alu_enable=1'b1; reg_write=1'b0;
                      end else begin // Other PRIV codes (Trap, RTE, Input, Output) - simplified for now
                          alu_enable=1'b0; // Or handle specific PRIV ops if needed
                      end
                  end
            default: ; // NOP or undefined
        endcase
    end
endmodule

//############################################################################
//## de_ex_register (No changes)
//############################################################################
module de_ex_register (
    input clk,
    input flush,
    input [63:0] pc_in,
    input [63:0] operand_a_in, // Now potentially forwarded rs data
    input [63:0] operand_b_in, // Now potentially forwarded rt data or literal
    input [63:0] operand_c_in, // Now potentially forwarded rd data
    input [63:0] literal_in,
    input [4:0] rd_addr_in,
    input [4:0] rs_addr_in,
    input [4:0] rt_addr_in,
    input [4:0] opcode_in,
    input [63:0] stack_ptr_in,
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,
    input mem_to_reg_in,
    input branch_taken_ctrl_in,
    input mem_pc_in,
    output reg [63:0] pc_out,
    output reg [63:0] operand_a_out,
    output reg [63:0] operand_b_out,
    output reg [63:0] operand_c_out,
    output reg [63:0] literal_out,
    output reg [4:0] rd_addr_out,
    output reg [4:0] rs_addr_out,
    output reg [4:0] rt_addr_out,
    output reg [4:0] opcode_out,
    output reg [63:0] stack_ptr_out,
    output reg alu_enable_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_ctrl_out, // Unused output now
    output reg mem_pc_out
);
    always @(posedge clk) begin
        if (flush) begin
            // Flush pipeline register, importantly setting control signals to 0
            pc_out <= 64'b0; operand_a_out <= 64'b0; operand_b_out <= 64'b0; operand_c_out <= 64'b0;
            literal_out <= 64'b0; rd_addr_out <= 5'b0; rs_addr_out <= 5'b0; rt_addr_out <= 5'b0;
            opcode_out <= 5'b0; stack_ptr_out <= 64'b0; alu_enable_out <= 1'b0; mem_read_out <= 1'b0;
            mem_write_out <= 1'b0; reg_write_out <= 1'b0; mem_to_reg_out <= 1'b0;
            branch_taken_ctrl_out <= 1'b0; mem_pc_out <= 1'b0;
        end else begin
            // Latch inputs to outputs
            pc_out <= pc_in; operand_a_out <= operand_a_in; operand_b_out <= operand_b_in; operand_c_out <= operand_c_in;
            literal_out <= literal_in; rd_addr_out <= rd_addr_in; rs_addr_out <= rs_addr_in; rt_addr_out <= rt_addr_in;
            opcode_out <= opcode_in; stack_ptr_out <= stack_ptr_in; alu_enable_out <= alu_enable_in;
            mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in; branch_taken_ctrl_out <= branch_taken_ctrl_in; mem_pc_out <= mem_pc_in;
        end
    end
endmodule

//############################################################################
//## alu
//## CHANGED: Restored original MOV_LIT behavior
//############################################################################
module alu (
    // Control Inputs
    input logic alu_enable,
    input logic [4:0] opcode,
    // Data Inputs
    input logic [63:0] input1,    // Pipelined rs_data (potentially forwarded)
    input logic [63:0] input2,    // Pipelined rt_data or literal (potentially forwarded)
    input logic [63:0] input3,    // Pipelined rd_data (potentially forwarded)
    input logic [4:0] rd_addr,   // Pipelined destination register address
    input logic [63:0] literal,   // Pipelined literal value (sign-extended)
    input logic [63:0] pc_in,     // Pipelined PC value
    input logic [63:0] stack_ptr, // Pipelined R31 value
    // Outputs
    output logic [63:0] result,    // Result of ALU op (to EX/MEM & forwarding)
    output logic [63:0] mem_addr,  // Calculated memory address (to EX/MEM)
    output logic [63:0] mem_wdata, // Data to write to memory (MOV_STR, CALL)
    output logic [63:0] branch_pc, // Calculated branch target PC
    output logic branch_taken,     // Branch condition outcome
    output logic hlt_out,          // Halt signal
    // Control Signal Inputs (pass-through for potential use, not used currently)
    input logic mem_read_in,
    input logic mem_write_in,
    input logic reg_write_in,
    input logic mem_to_reg_in,
    input logic mem_pc_in
);
    // Opcodes (localparam for readability)
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;

    // Use always @(*) for combinational logic as requested
    always @(*) begin
        // Default Output Values
        result = 64'b0;
        mem_addr = 64'b0;
        mem_wdata = 64'b0;
        branch_pc = pc_in + 4; // Default next PC for non-branch/taken branch
        branch_taken = 1'b0;   // Default: branch not taken
        hlt_out = 1'b0;

        if (alu_enable) begin
            case (opcode)
                // Integer Arithmetic
                ADD, ADDI: result = $signed(input1) + $signed(input2);
                SUB, SUBI: result = $signed(input1) - $signed(input2);
                MUL: result = $signed(input1) * $signed(input2);
                DIV: if (input2 != 0) result = $signed(input1) / $signed(input2); else result = 64'b0; // Basic div by zero check

                // Logical
                AND: result = input1 & input2;
                OR:  result = input1 | input2;
                XOR: result = input1 ^ input2;
                NOT: result = ~input1;

                // Shift (Use lower 6 bits of shift amount)
                SHFTR, SHFTRI: result = $signed(input1) >>> input2[5:0]; // Arithmetic right shift
                SHFTL, SHFTLI: result = input1 << input2[5:0]; // Logical left shift

                // Data Movement - Calculate address or pass data
                MOV_MEM: mem_addr = input1 + $signed(input2); // Load: Addr = rs + L
                MOV_REG: result = input1;                   // Move Register: Result = rs
                // *** RESTORED ORIGINAL MOV_LIT LOGIC ***
                MOV_LIT: result = {input1[63:12], input2[11:0]}; // Combine upper bits of rd (input1) with literal (input2[11:0])
                MOV_STR: begin                              // Store: Addr = rd + L, Data = rs
                           mem_addr = input3 + $signed(literal); // Base address from input3 (rd)
                           mem_wdata = input1;                  // Data to store from input1 (rs)
                       end

                // Floating Point (Simulation only - treat as simple pass-through/zero for now)
                ADDF, SUBF, MULF, DIVF: result = 64'b0; // Placeholder

                // Control Flow - Branches (Calculate target PC and taken condition)
                BR: begin branch_pc = input3; branch_taken = 1'b1; end // Target is rd_data (input3)
                BRR: begin branch_pc = pc_in + $signed(input3); branch_taken = 1'b1; end // Target is PC + rd_data (input3)
                BRRI: begin branch_pc = pc_in + $signed(input2); branch_taken = 1'b1; end // Target is PC + literal (input2)
                BRNZ: begin if ($signed(input1) != 0) begin branch_pc = input3; branch_taken = 1'b1; end else branch_taken = 1'b0; end // Target rd_data (input3) if rs_data (input1) != 0
                BRGT: begin if ($signed(input1) > $signed(input2)) begin branch_pc = input3; branch_taken = 1'b1; end else branch_taken = 1'b0; end // Target rd_data (input3) if rs_data (input1) > rt_data (input2)

                // Control Flow - Subroutines
                CALL: begin
                          branch_pc = input3;               // Target is rd_data (input3)
                          mem_addr = stack_ptr - 8;         // Stack address
                          mem_wdata = pc_in + 4;            // Return address
                          branch_taken = 1'b1;              // Always taken
                      end
                RETURN: begin
                           mem_addr = stack_ptr - 8;         // Stack address to read PC from
                           branch_taken = 1'b1;              // Set taken for flush, actual PC comes from mem
                           // branch_pc is ignored by fetch stage due to mem_pc signal
                       end

                // Privileged - Halt
                PRIV: if (literal[11:0] == 12'h0) hlt_out = 1'b1; // HALT
                      // Other PRIV codes would be handled here if implemented

                default: result = 64'b0; // Default case
            endcase
        end
    end
endmodule

//############################################################################
//## ex_mem_register (No changes)
//############################################################################
module ex_mem_register (
    input clk,
    input logic flush_mem, // Use generic name 'flush' maybe?
    // Inputs from Execute Stage
    input logic [63:0] result_in,   // ALU result
    input logic [63:0] mem_addr_in, // Memory address calculated
    input logic [63:0] mem_wdata_in,// Data for memory write (from ALU)
    input logic [63:0] branch_pc_in,// Branch target PC calculated
    input logic [4:0] rd_addr_in,   // Destination register address
    input logic hlt_in,
    // Control Inputs (from DE/EX outputs)
    input logic mem_read_in,
    input logic mem_write_in,
    input logic reg_write_in,
    input logic mem_to_reg_in,
    input logic branch_taken_in, // Branch decision from ALU
    input logic mem_pc_in,       // RETURN instruction indicator
    // Outputs to Memory Stage
    output logic [63:0] result_out,
    output logic [63:0] mem_addr_out,
    output logic [63:0] mem_wdata_out, // Pass store data to memory module
    output logic [63:0] branch_pc_out,
    output logic [4:0] rd_addr_out,
    output logic hlt_out,
    output logic mem_read_out,
    output logic mem_write_out,
    output logic reg_write_out,
    output logic mem_to_reg_out,
    output logic branch_taken_out, // Pipelined branch decision used for flush/fetch
    output logic mem_pc_out        // Pipelined RETURN indicator
);
    always @(posedge clk) begin
        if (flush_mem) begin
            // Flush: Clear data paths and control signals
            result_out <= 64'b0;
            mem_addr_out <= 64'b0;
            mem_wdata_out <= 64'b0;
            branch_pc_out <= 64'b0;
            rd_addr_out <= 5'b0;
            hlt_out <= 1'b0;
            // Crucially clear control signals
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            branch_taken_out <= 1'b0;
            mem_pc_out <= 1'b0;
        end else begin
            // Latch inputs to outputs normally
            result_out <= result_in;
            mem_addr_out <= mem_addr_in;
            mem_wdata_out <= mem_wdata_in; // Latch store data
            branch_pc_out <= branch_pc_in;
            rd_addr_out <= rd_addr_in;
            hlt_out <= hlt_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            branch_taken_out <= branch_taken_in; // Pass branch decision
            mem_pc_out <= mem_pc_in;         // Pass RETURN indicator
        end
    end
endmodule


//############################################################################
//## memory (No changes)
//############################################################################
module memory (
    input clk,
    input reset,
    // Instruction Fetch Port
    input [63:0] inst_addr,
    output logic [31:0] instruction_out,
    // Data Access Port
    input [63:0] data_addr,    // Address from MEM stage (result of ALU calc in EX)
    input [63:0] data_wdata,   // Write data from MEM stage (pipelined from EX/ALU)
    input mem_read,            // Read enable from MEM stage
    input mem_write,           // Write enable from MEM stage
    output logic [63:0] data_rdata // Read data output to MEM stage (for WB mux)
);
    localparam MEM_SIZE_BYTES = 524288; // 0.5 MB example size
    localparam ADDR_BITS = $clog2(MEM_SIZE_BYTES);

    // Byte-addressable memory array
    reg [7:0] bytes [0:MEM_SIZE_BYTES-1];
    integer i;

    // Initialize memory to 0 (optional, good practice)
    initial begin
        for (i = 0; i < MEM_SIZE_BYTES; i = i + 1) begin
            bytes[i] = 8'b0;
        end
    end

    // Combinational Instruction Read (Byte addressing, Little Endian assumed)
    wire [ADDR_BITS-1:0] safe_inst_addr = inst_addr[$left(inst_addr) - 1 : 0]; // Adjust based on ADDR_BITS if needed
    assign instruction_out[7:0]   = bytes[safe_inst_addr + 0];
    assign instruction_out[15:8]  = bytes[safe_inst_addr + 1];
    assign instruction_out[23:16] = bytes[safe_inst_addr + 2];
    assign instruction_out[31:24] = bytes[safe_inst_addr + 3];

    // Combinational Data Read (Byte addressing, Little Endian assumed)
    wire [ADDR_BITS-1:0] safe_data_addr = data_addr[$left(data_addr) - 1 : 0]; // Adjust based on ADDR_BITS if needed
    assign data_rdata[7:0]   = bytes[safe_data_addr + 0];
    assign data_rdata[15:8]  = bytes[safe_data_addr + 1];
    assign data_rdata[23:16] = bytes[safe_data_addr + 2];
    assign data_rdata[31:24] = bytes[safe_data_addr + 3];
    assign data_rdata[39:32] = bytes[safe_data_addr + 4];
    assign data_rdata[47:40] = bytes[safe_data_addr + 5];
    assign data_rdata[55:48] = bytes[safe_data_addr + 6];
    assign data_rdata[63:56] = bytes[safe_data_addr + 7];

    // Synchronous Data Write (Byte addressing, Little Endian assumed)
    always @(posedge clk) begin
        if (mem_write) begin
            // Ensure address is within bounds (optional)
            bytes[safe_data_addr + 0] <= data_wdata[7:0];
            bytes[safe_data_addr + 1] <= data_wdata[15:8];
            bytes[safe_data_addr + 2] <= data_wdata[23:16];
            bytes[safe_data_addr + 3] <= data_wdata[31:24];
            bytes[safe_data_addr + 4] <= data_wdata[39:32];
            bytes[safe_data_addr + 5] <= data_wdata[47:40];
            bytes[safe_data_addr + 6] <= data_wdata[55:48];
            bytes[safe_data_addr + 7] <= data_wdata[63:56];
        end
    end
endmodule


//############################################################################
//## aluMemMux (No changes)
//## Selects PC source for Fetch stage (Branch Target or Return Address from Mem)
//############################################################################
module aluMemMux (
    input mem_pc,             // Control signal (is it RETURN?) from MEM stage
    input [63:0] memData,     // Data read from memory (potential return PC)
    input [63:0] aluOut,      // Branch PC calculated by ALU (passed through EX/MEM)
    output reg [63:0] newPc   // Selected PC for Fetch stage mux
);
    // Use always @(*) for combinational logic as requested
    always @(*) begin
        if (mem_pc) begin
            newPc = memData; // RETURN instruction: Use PC read from memory
        end else begin
            newPc = aluOut;  // Other branch/no branch: Use ALU calculated branch PC
        end
    end
endmodule

//############################################################################
//## reglitmux (No changes)
//## Selects ALU operand B source (Register or Literal)
//############################################################################
module reglitmux (
    input [4:0] sel,      // Opcode from DE stage
    input [63:0] reg_in,  // Register data (potentially forwarded rt)
    input [63:0] lit_in,  // Literal value (sign-extended) from DE stage
    output reg [63:0] out // Selected value for ALU input B (via DE/EX)
);
    // Opcodes requiring literal as second ALU operand
    localparam ADDI=5'h19, SUBI=5'h1B, SHFTRI=5'h05, SHFTLI=5'h07, BRRI=5'h0A,
               MOV_MEM=5'h10, MOV_LIT=5'h12, MOV_STR=5'h13; // Note: MOV_STR uses literal with rd (input3) in ALU

    // Use always @(*) for combinational logic as requested
    always @(*) begin
        case (sel)
            // Select Literal for these opcodes
            ADDI, SUBI, SHFTRI, SHFTLI, BRRI, MOV_MEM, MOV_LIT: begin // MOV_STR uses literal elsewhere
                out = lit_in;
            end
            // Default: Select Register Data (rt)
            default: begin
                out = reg_in;
            end
        endcase
    end
endmodule

//############################################################################
//## memRegMux (No changes)
//## Selects Writeback data source (Memory Read Data or ALU Result)
//############################################################################
module memRegMux (
    input mem_to_reg,         // Control signal from MEM stage
    input [63:0] readData,    // Data read from memory in MEM stage
    input [63:0] aluResult,   // ALU result passed from MEM stage
    output reg [63:0] regWriteData // Data to be written back to Register File
);
    // Use always @(*) for combinational logic as requested
    always @(*) begin
        if (mem_to_reg) begin
            regWriteData = readData; // Load instruction (MOV_MEM)
        end else begin
            regWriteData = aluResult; // ALU instruction result
        end
    end
endmodule