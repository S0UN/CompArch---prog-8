//############################################################################
//## tinker_core (Top Level Module)
//## CHANGED: Added Forwarding Logic AND Load-Use Hazard Stall Logic
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

    // --- Forwarding Signals ---
    logic [1:0] forward_a_select;
    logic [1:0] forward_b_select;
    logic [1:0] forward_c_select;

    // --- Hazard/Stall Signals --- // HAZARD/STALL ADDED
    logic stall_de;           // Stall DE stage due to load-use hazard
    logic pc_write_enable;    // Enable PC update in Fetch
    logic if_de_write_enable; // Enable IF/DE register update
    logic de_ex_bubble_enable;// Insert bubble into DE/EX register

    // --- Control Signals ---
    logic flush_de_branch, flush_ex_branch; // Renamed branch flush signals
    logic take_return_pc_fetch;

    // --- Module Instantiations ---

    // Detect Load-Use Hazard // HAZARD/STALL ADDED
    hazardDetectionUnit hazard_unit (
        .rs_addr_de(rs_addr_de),
        .rt_addr_de(rt_addr_de),
        .rd_addr_ex(rd_addr_ex),
        .mem_read_ex(mem_read_ex), // Check if instr in EX is a memory read
        .stall_de(stall_de)
    );

    // Calculate Pipeline Control Signals // HAZARD/STALL ADDED
    assign pc_write_enable = !stall_de;
    assign if_de_write_enable = !stall_de;
    assign de_ex_bubble_enable = stall_de; // Insert bubble if stalled

    // Fetch Stage
    fetch instruction_fetcher (
        .clk(clk),
        .reset(reset),
        .pc_write_enable(pc_write_enable), // HAZARD/STALL ADDED
        .branch_taken(branch_taken_mem),
        .branch_pc(branch_pc_mem),
        .take_return_pc(take_return_pc_fetch),
        .return_pc(return_pc_mem),
        .pc_out(pc_if)
    );

    // Memory Module
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

    // IF/DE Register Stage
    if_de_register if_de_reg (
        .clk(clk),
        .write_enable(if_de_write_enable), // HAZARD/STALL ADDED
        .flush(flush_de_branch), // Flush takes priority over stall for this reg
        .pc_in(pc_if),
        .instruction_in(instruction_if),
        .pc_out(pc_de),
        .instruction_out(instruction_de)
    );

    // Decode Stage Components
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
        .read_data1(regfile_operand_a_de),
        .read_addr2(rt_addr_de),
        .read_data2(regfile_operand_b_de),
        .read_addr3(rd_addr_de),
        .read_data3(regfile_operand_c_de),
        .stack_ptr_out(stack_ptr_de)
    );

    // Forwarding Unit
    forwardingUnit fwd_unit (
        .rs_addr_de(rs_addr_de),
        .rt_addr_de(rt_addr_de),
        .rd_addr_de(rd_addr_de),
        .rd_addr_ex(rd_addr_ex),
        .reg_write_ex(reg_write_ex),
        .rd_addr_mem(rd_addr_mem),
        .reg_write_mem(reg_write_mem),
        .forward_a_select(forward_a_select),
        .forward_b_select(forward_b_select),
        .forward_c_select(forward_c_select)
    );

    // Forwarding Multiplexers
    forwardingMux forwardingMuxA (
        .select(forward_a_select),
        .data_regfile(regfile_operand_a_de),
        .data_ex(alu_result_ex),
        .data_mem(reg_wdata_wb),
        .forwarded_data(operand_a_de)
    );
    forwardingMux forwardingMuxB (
        .select(forward_b_select),
        .data_regfile(regfile_operand_b_de),
        .data_ex(alu_result_ex),
        .data_mem(reg_wdata_wb),
        .forwarded_data(operand_b_reg_maybe_fwd)
    );
     forwardingMux forwardingMuxC (
        .select(forward_c_select),
        .data_regfile(regfile_operand_c_de),
        .data_ex(alu_result_ex),
        .data_mem(reg_wdata_wb),
        .forwarded_data(operand_c_de)
    );

    // Register/Literal Mux for Operand B
    reglitmux input_selector (
        .sel(opcode_de),
        .reg_in(operand_b_reg_maybe_fwd),
        .lit_in(literal_de),
        .out(operand_b_de)
    );

    // DE/EX Register Stage
    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush_ex_branch),   // HAZARD/STALL ADDED: Controlled by branch flush
        .bubble_enable(de_ex_bubble_enable), // HAZARD/STALL ADDED: Controlled by stall
        // Inputs from Decode Stage (potentially forwarded)
        .pc_in(pc_de),
        .operand_a_in(operand_a_de),
        .operand_b_in(operand_b_de),
        .operand_c_in(operand_c_de),
        .literal_in(literal_de),
        .rd_addr_in(rd_addr_de),
        .rs_addr_in(rs_addr_de),
        .rt_addr_in(rt_addr_de),
        .opcode_in(opcode_de),
        .stack_ptr_in(stack_ptr_de),
        // Control signals
        .alu_enable_in(alu_enable_de),
        .mem_read_in(mem_read_de),
        .mem_write_in(mem_write_de),
        .reg_write_in(reg_write_de),
        .mem_to_reg_in(mem_to_reg_de),
        .branch_taken_ctrl_in(branch_taken_ctrl_de),
        .mem_pc_in(mem_pc_de),
        // Outputs to Execute Stage
        .pc_out(pc_ex),
        .operand_a_out(operand_a_ex),
        .operand_b_out(operand_b_ex),
        .operand_c_out(operand_c_ex),
        .literal_out(literal_ex),
        .rd_addr_out(rd_addr_ex),
        .rs_addr_out(rs_addr_ex),
        .rt_addr_out(rt_addr_ex),
        .opcode_out(opcode_ex),
        .stack_ptr_out(stack_ptr_ex),
        // Control signals out
        .alu_enable_out(alu_enable_ex),
        .mem_read_out(mem_read_ex),
        .mem_write_out(mem_write_ex),
        .reg_write_out(reg_write_ex),
        .mem_to_reg_out(mem_to_reg_ex),
        .branch_taken_ctrl_out(), // Unused
        .mem_pc_out(mem_pc_ex)
    );

    // Execute Stage
    alu calculation_unit (
        .alu_enable(alu_enable_ex),
        .opcode(opcode_ex),
        .input1(operand_a_ex),
        .input2(operand_b_ex),
        .input3(operand_c_ex),
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
        // Pass-through controls (optional)
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .mem_pc_in(mem_pc_ex)
    );

    // EX/MEM Register Stage
    ex_mem_register ex_mem_reg (
        .clk(clk),
        .flush_mem(flush_ex_branch), // Controlled by branch flush only
        .result_in(alu_result_ex),
        .mem_addr_in(alu_mem_addr_ex),
        .mem_wdata_in(alu_mem_data_ex),
        .branch_pc_in(alu_branch_pc_ex),
        .rd_addr_in(rd_addr_ex),
        .hlt_in(hlt_ex),
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .branch_taken_in(branch_taken_ex),
        .mem_pc_in(mem_pc_ex),
        // Outputs
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

    // MEM Stage Muxes
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

    // --- Overall Control Logic ---
    // Flush DE and EX stages if a branch is taken in the MEM stage
    // These signals only control flushing due to branches. Stall handles load-use bubble.
    assign flush_de_branch = branch_taken_mem;
    assign flush_ex_branch = branch_taken_mem;

    // Select return PC if RETURN instruction is in MEM stage
    assign take_return_pc_fetch = mem_pc_mem;

    // Halt Signal
    assign hlt = hlt_mem;

endmodule

//############################################################################
//## hazardDetectionUnit (NEW MODULE - Load-Use)
//############################################################################
module hazardDetectionUnit (
    // Inputs from DE stage (read addresses)
    input logic [4:0] rs_addr_de,
    input logic [4:0] rt_addr_de,

    // Inputs from EX stage (write address & load control)
    input logic [4:0] rd_addr_ex,
    input logic       mem_read_ex, // Is instruction in EX a memory read (load)?

    // Output: Stall signal
    output logic stall_de
);
    // Use always @(*) for combinational logic as requested
    always @(*) begin
        // Stall if instruction in EX is a load (mem_read_ex) AND
        // its destination register (rd_addr_ex) is non-zero AND
        // matches either source register read in DE (rs_addr_de or rt_addr_de)
        if (mem_read_ex && (rd_addr_ex != 5'b0) &&
            ((rd_addr_ex == rs_addr_de) || (rd_addr_ex == rt_addr_de)))
        begin
            stall_de = 1'b1; // Load-use hazard detected, stall DE stage
        end else begin
            stall_de = 1'b0; // No load-use hazard
        end
    end
endmodule


//############################################################################
//## forwardingUnit (No Changes from previous version)
//############################################################################
module forwardingUnit (
    // Inputs from DE stage (read addresses)
    input logic [4:0] rs_addr_de,
    input logic [4:0] rt_addr_de,
    input logic [4:0] rd_addr_de, // For read port 3 usage

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

    always @(*) begin
        forward_a_select = 2'b00; forward_b_select = 2'b00; forward_c_select = 2'b00; // Defaults

        // Operand A (rs) Forwarding
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rs_addr_de)) begin
            forward_a_select = 2'b01; // EX Priority
        end else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rs_addr_de)) begin
            forward_a_select = 2'b10; // MEM
        end

        // Operand B (rt) Forwarding
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rt_addr_de)) begin
            forward_b_select = 2'b01; // EX Priority
        end else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rt_addr_de)) begin
            forward_b_select = 2'b10; // MEM
        end

        // Operand C (rd used as source) Forwarding
        if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rd_addr_de)) begin
            forward_c_select = 2'b01; // EX Priority
        end else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rd_addr_de)) begin
           forward_c_select = 2'b10; // MEM
        end
    end

endmodule

//############################################################################
//## forwardingMux (No Changes)
//############################################################################
module forwardingMux (
    input logic [1:0] select,        // 00: RegFile, 01: EX, 10: MEM
    input logic [63:0] data_regfile, // Data from Register File read port
    input logic [63:0] data_ex,      // Data from EX stage (ALU result)
    input logic [63:0] data_mem,     // Data from MEM/WB stage (Writeback data)
    output logic [63:0] forwarded_data // Selected data
);

    always @(*) begin
        case (select)
            2'b00: forwarded_data = data_regfile;
            2'b01: forwarded_data = data_ex;
            2'b10: forwarded_data = data_mem;
            default: forwarded_data = data_regfile; // Default to RegFile data
        endcase
    end

endmodule

//############################################################################
//## registerFile (No Changes)
//############################################################################
module registerFile (
    input clk,
    input reset,
    input [4:0] write_addr,
    input [63:0] write_data,
    input write_enable,
    input [4:0] read_addr1,
    output logic [63:0] read_data1,
    input [4:0] read_addr2,
    output logic [63:0] read_data2,
    input [4:0] read_addr3,
    output logic [63:0] read_data3,
    output logic [63:0] stack_ptr_out
);
    reg [63:0] registers [0:31];
    integer idx;
    initial begin
        for (idx = 0; idx < 31; idx = idx + 1) registers[idx] = 64'b0;
        registers[31] = 64'h0008_0000;
    end
    assign read_data1 = (read_addr1 == 5'd31) ? registers[31] : registers[read_addr1];
    assign read_data2 = (read_addr2 == 5'd31) ? registers[31] : registers[read_addr2];
    assign read_data3 = (read_addr3 == 5'd31) ? registers[31] : registers[read_addr3];
    assign stack_ptr_out = registers[31];
    always @(posedge clk) begin
        if (!reset && write_enable) registers[write_addr] <= write_data;
    end
endmodule

//############################################################################
//## fetch
//## CHANGED: Added pc_write_enable input for stalling
//############################################################################
module fetch (
    input clk,
    input reset,
    input pc_write_enable, // HAZARD/STALL ADDED: Control PC update
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
        end else if (pc_write_enable) begin // Only update PC if not stalled
            if (take_return_pc) current_pc <= return_pc;
            else if (branch_taken) current_pc <= branch_pc;
            else current_pc <= current_pc + 64'd4;
        end
        // If pc_write_enable is low, current_pc retains its value (stalled)
    end
endmodule

//############################################################################
//## if_de_register
//## CHANGED: Added write_enable input for stalling
//############################################################################
module if_de_register (
    input clk,
    input write_enable, // HAZARD/STALL ADDED: Control register update
    input flush,        // Branch flush signal
    input [63:0] pc_in,
    input [31:0] instruction_in,
    output reg [63:0] pc_out,
    output reg [31:0] instruction_out
);
    parameter NOP_INSTRUCTION = 32'b0; // Define a NOP (could be specific like add r0,r0,r0)

    always @(posedge clk) begin
        if (flush) begin // Flush has higher priority than stall for this register
            pc_out <= 64'b0;
            instruction_out <= NOP_INSTRUCTION; // Flush with NOP
        end else if (write_enable) begin // Only latch if not stalled and not flushed
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
        // If write_enable is low (stalled) and not flushed, register retains value
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
    output reg branch_taken, // Potential branch type
    output reg mem_pc        // RETURN type
);
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;

    always @(*) begin
        alu_enable = 1'b0; mem_read = 1'b0; mem_write = 1'b0; reg_write = 1'b0;
        mem_to_reg = 1'b0; branch_taken = 1'b0; mem_pc = 1'b0;
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        literal = {{52{instructionLine[11]}}, instructionLine[11:0]};

        case (opcode)
            ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL: begin alu_enable=1'b1; reg_write=1'b1; end
            ADDI, SUBI, SHFTRI, SHFTLI: begin alu_enable=1'b1; reg_write=1'b1; rs = rd; end
            MOV_MEM: begin alu_enable=1'b1; mem_read=1'b1; reg_write=1'b1; mem_to_reg=1'b1; end
            MOV_STR: begin alu_enable=1'b1; mem_write=1'b1; end
            MOV_REG: begin alu_enable=1'b1; reg_write=1'b1; end
            MOV_LIT: begin alu_enable=1'b1; reg_write=1'b1; rs = rd; end
            ADDF, SUBF, MULF, DIVF: begin alu_enable=1'b1; reg_write=1'b1; end
            BR, BRR, BRRI, BRNZ, BRGT: begin alu_enable=1'b1; branch_taken=1'b1; end
            CALL: begin alu_enable=1'b1; mem_write=1'b1; branch_taken=1'b1; end
            RETURN: begin alu_enable=1'b1; mem_read=1'b1; mem_pc=1'b1; branch_taken=1'b1; end
            PRIV: if(literal[11:0]==12'h0) begin alu_enable=1'b1; end
            default: ;
        endcase
    end
endmodule

//############################################################################
//## de_ex_register
//## CHANGED: Added bubble_enable input for stalling (inserts NOP)
//## flush input now only handles branch mispredict flush
//############################################################################
module de_ex_register (
    input clk,
    input flush,        // Flush due to branch mispredict
    input bubble_enable,// Insert bubble due to load-use stall
    // Inputs from Decode Stage
    input [63:0] pc_in,
    input [63:0] operand_a_in,
    input [63:0] operand_b_in,
    input [63:0] operand_c_in,
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
    // Outputs to Execute Stage
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
    parameter NOP_OPCODE = 5'b0; // Define NOP opcode (or use specific like ADD)

    always @(posedge clk) begin
        // Priority: Flush > Bubble > Normal Latch
        if (flush || bubble_enable) begin // If flushed (branch) or bubbled (stall)
            // Insert NOP / Clear control signals
            pc_out <= 64'b0; // PC doesn't matter for NOP
            operand_a_out <= 64'b0; operand_b_out <= 64'b0; operand_c_out <= 64'b0;
            literal_out <= 64'b0;
            rd_addr_out <= 5'b0; rs_addr_out <= 5'b0; rt_addr_out <= 5'b0;
            opcode_out <= NOP_OPCODE; // Ensure opcode is NOP
            stack_ptr_out <= 64'b0; // Or previous value? Zero for safety.
            // Clear control signals to prevent side effects
            alu_enable_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            branch_taken_ctrl_out <= 1'b0;
            mem_pc_out <= 1'b0;
        end else begin
            // Normal operation: Latch inputs to outputs
            pc_out <= pc_in; operand_a_out <= operand_a_in; operand_b_out <= operand_b_in; operand_c_out <= operand_c_in;
            literal_out <= literal_in; rd_addr_out <= rd_addr_in; rs_addr_out <= rs_addr_in; rt_addr_out <= rt_addr_in;
            opcode_out <= opcode_in; stack_ptr_out <= stack_ptr_in; alu_enable_out <= alu_enable_in;
            mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in; branch_taken_ctrl_out <= branch_taken_ctrl_in; mem_pc_out <= mem_pc_in;
        end
    end
endmodule


//############################################################################
//## alu (No Changes from previous version)
//############################################################################
module alu (
    input logic alu_enable, input logic [4:0] opcode,
    input logic [63:0] input1, input logic [63:0] input2, input logic [63:0] input3,
    input logic [4:0] rd_addr, input logic [63:0] literal, input logic [63:0] pc_in, input logic [63:0] stack_ptr,
    output logic [63:0] result, output logic [63:0] mem_addr, output logic [63:0] mem_wdata,
    output logic [63:0] branch_pc, output logic branch_taken, output logic hlt_out,
    input logic mem_read_in, input logic mem_write_in, input logic reg_write_in, input logic mem_to_reg_in, input logic mem_pc_in
);
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;
    always @(*) begin
        result = 64'b0; mem_addr = 64'b0; mem_wdata = 64'b0;
        branch_pc = pc_in + 4; branch_taken = 1'b0; hlt_out = 1'b0;
        if (alu_enable) begin
            case (opcode)
                ADD, ADDI: result = $signed(input1) + $signed(input2);
                SUB, SUBI: result = $signed(input1) - $signed(input2);
                MUL: result = $signed(input1) * $signed(input2);
                DIV: if (input2 != 0) result = $signed(input1) / $signed(input2); else result = 64'b0;
                AND: result = input1 & input2; OR: result = input1 | input2; XOR: result = input1 ^ input2; NOT: result = ~input1;
                SHFTR, SHFTRI: result = $signed(input1) >>> input2[5:0];
                SHFTL, SHFTLI: result = input1 << input2[5:0];
                MOV_MEM: mem_addr = input1 + $signed(input2);
                MOV_REG: result = input1;
                MOV_LIT: result = {input1[63:12], input2[11:0]};
                MOV_STR: begin mem_addr = input3 + $signed(literal); mem_wdata = input1; end
                ADDF, SUBF, MULF, DIVF: result = 64'b0; // Placeholder
                BR: begin branch_pc = input3; branch_taken = 1'b1; end
                BRR: begin branch_pc = pc_in + $signed(input3); branch_taken = 1'b1; end
                BRRI: begin branch_pc = pc_in + $signed(input2); branch_taken = 1'b1; end
                BRNZ: if ($signed(input1) != 0) begin branch_pc = input3; branch_taken = 1'b1; end else branch_taken = 1'b0;
                BRGT: if ($signed(input1) > $signed(input2)) begin branch_pc = input3; branch_taken = 1'b1; end else branch_taken = 1'b0;
                CALL: begin branch_pc=input3; mem_addr=stack_ptr-8; mem_wdata=pc_in+4; branch_taken=1'b1; end
                RETURN: begin mem_addr=stack_ptr-8; branch_taken=1'b1; end
                PRIV: if (literal[11:0] == 12'h0) hlt_out = 1'b1;
                default: result = 64'b0;
            endcase
        end
    end
endmodule

//############################################################################
//## ex_mem_register (No Changes)
//############################################################################
module ex_mem_register (
    input clk, input logic flush_mem,
    input logic [63:0] result_in, input logic [63:0] mem_addr_in, input logic [63:0] mem_wdata_in,
    input logic [63:0] branch_pc_in, input logic [4:0] rd_addr_in, input logic hlt_in,
    input logic mem_read_in, input logic mem_write_in, input logic reg_write_in, input logic mem_to_reg_in,
    input logic branch_taken_in, input logic mem_pc_in,
    output logic [63:0] result_out, output logic [63:0] mem_addr_out, output logic [63:0] mem_wdata_out,
    output logic [63:0] branch_pc_out, output logic [4:0] rd_addr_out, output logic hlt_out,
    output logic mem_read_out, output logic mem_write_out, output logic reg_write_out, output logic mem_to_reg_out,
    output logic branch_taken_out, output logic mem_pc_out
);
    always @(posedge clk) begin
        if (flush_mem) begin
            result_out <= 64'b0; mem_addr_out <= 64'b0; mem_wdata_out <= 64'b0;
            branch_pc_out <= 64'b0; rd_addr_out <= 5'b0; hlt_out <= 1'b0;
            mem_read_out <= 1'b0; mem_write_out <= 1'b0; reg_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0; branch_taken_out <= 1'b0; mem_pc_out <= 1'b0;
        end else begin
            result_out <= result_in; mem_addr_out <= mem_addr_in; mem_wdata_out <= mem_wdata_in;
            branch_pc_out <= branch_pc_in; rd_addr_out <= rd_addr_in; hlt_out <= hlt_in;
            mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in; branch_taken_out <= branch_taken_in; mem_pc_out <= mem_pc_in;
        end
    end
endmodule


//############################################################################
//## memory (No changes)
//############################################################################
module memory (
    input clk, input reset, input [63:0] inst_addr, output logic [31:0] instruction_out,
    input [63:0] data_addr, input [63:0] data_wdata, input mem_read, input mem_write,
    output logic [63:0] data_rdata
);
    localparam MEM_SIZE_BYTES = 524288; localparam ADDR_BITS = $clog2(MEM_SIZE_BYTES);
    reg [7:0] bytes [0:MEM_SIZE_BYTES-1]; integer i;
    initial begin for(i=0;i<MEM_SIZE_BYTES;i=i+1) bytes[i]=8'b0; end
    wire [ADDR_BITS-1:0] safe_inst_addr = inst_addr[$left(inst_addr) - 1 : 0];
    assign instruction_out[7:0]=bytes[safe_inst_addr+0]; assign instruction_out[15:8]=bytes[safe_inst_addr+1];
    assign instruction_out[23:16]=bytes[safe_inst_addr+2]; assign instruction_out[31:24]=bytes[safe_inst_addr+3];
    wire [ADDR_BITS-1:0] safe_data_addr = data_addr[$left(data_addr) - 1 : 0];
    assign data_rdata[7:0]=bytes[safe_data_addr+0]; assign data_rdata[15:8]=bytes[safe_data_addr+1];
    assign data_rdata[23:16]=bytes[safe_data_addr+2]; assign data_rdata[31:24]=bytes[safe_data_addr+3];
    assign data_rdata[39:32]=bytes[safe_data_addr+4]; assign data_rdata[47:40]=bytes[safe_data_addr+5];
    assign data_rdata[55:48]=bytes[safe_data_addr+6]; assign data_rdata[63:56]=bytes[safe_data_addr+7];
    always @(posedge clk) begin if(mem_write) begin
        bytes[safe_data_addr+0]<=data_wdata[7:0]; bytes[safe_data_addr+1]<=data_wdata[15:8];
        bytes[safe_data_addr+2]<=data_wdata[23:16]; bytes[safe_data_addr+3]<=data_wdata[31:24];
        bytes[safe_data_addr+4]<=data_wdata[39:32]; bytes[safe_data_addr+5]<=data_wdata[47:40];
        bytes[safe_data_addr+6]<=data_wdata[55:48]; bytes[safe_data_addr+7]<=data_wdata[63:56]; end end
endmodule


//############################################################################
//## aluMemMux (No changes)
//############################################################################
module aluMemMux (input mem_pc, input [63:0] memData, input [63:0] aluOut, output reg [63:0] newPc);
    always @(*) begin
        if (mem_pc) newPc = memData; else newPc = aluOut;
    end
endmodule

//############################################################################
//## reglitmux (No changes)
//############################################################################
module reglitmux (input [4:0] sel, input [63:0] reg_in, input [63:0] lit_in, output reg [63:0] out);
    localparam ADDI=5'h19, SUBI=5'h1B, SHFTRI=5'h05, SHFTLI=5'h07, BRRI=5'h0A,
               MOV_MEM=5'h10, MOV_LIT=5'h12, MOV_STR=5'h13;
    always @(*) begin
        case (sel)
            ADDI, SUBI, SHFTRI, SHFTLI, BRRI, MOV_MEM, MOV_LIT: out = lit_in;
            default: out = reg_in;
        endcase
    end
endmodule

//############################################################################
//## memRegMux (No changes)
//############################################################################
module memRegMux (input mem_to_reg, input [63:0] readData, input [63:0] aluResult, output reg [63:0] regWriteData);
    always @(*) begin
        if (mem_to_reg) regWriteData = readData; else regWriteData = aluResult;
    end
endmodule