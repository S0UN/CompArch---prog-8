// Verilog code for Tinker CPU Pipeline
// Includes MEM->EX Forwarding
// Passes 29/29 single instruction tests (arith_forwarding benchmark fails due to missing load-use/more complex hazard handling)

`timescale 1ns / 1ps

//############################################################################
//## forwardingUnit Module (NEW)
//############################################################################
module forwardingUnit (
    // Inputs from EX stage (register sources being used)
    input logic [4:0] rs_addr_ex,
    input logic [4:0] rt_addr_ex,

    // Inputs from MEM stage (register destination being written)
    input logic [4:0] rd_addr_mem,    // Destination register of instruction in MEM
    input logic       reg_write_mem,  // Write enable for instruction in MEM

    // Outputs: Control signals for ALU input Muxes
    output logic      forwardA,       // Select forwarded data for ALU input A (rs)
    output logic      forwardB        // Select forwarded data for ALU input B (rt)
);

    // Check if MEM stage instruction writes to a register
    // Note: Assumes R0 is writable as per previous correction.
    // If R0 were hardwired zero, would add: && (rd_addr_mem != 5'b0)
    logic mem_writes_reg;
    assign mem_writes_reg = reg_write_mem;

    // Forwarding condition for ALU Input A (operand comes from rs)
    assign forwardA = mem_writes_reg && (rd_addr_mem == rs_addr_ex);

    // Forwarding condition for ALU Input B (operand comes from rt)
    assign forwardB = mem_writes_reg && (rd_addr_mem == rt_addr_ex);

endmodule

//############################################################################
//## tinker_core (Top Level Module)
//## CHANGED: Added Forwarding Unit and ALU Input Muxes
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
    logic [63:0] operand_a_de; // rs_data
    logic [63:0] regfile_operand_b_de; // rt_data (before mux)
    logic [63:0] operand_b_de; // rt_data or literal (after mux)
    logic [63:0] operand_c_de; // rd_data (read via Port 3)
    logic [63:0] literal_de;
    logic [4:0]  rd_addr_de;
    logic [4:0]  rs_addr_de;
    logic [4:0]  rt_addr_de;
    logic [4:0]  opcode_de;
    logic [63:0] stack_ptr_de;
    logic        alu_enable_de, mem_read_de, mem_write_de, reg_write_de, mem_to_reg_de, branch_taken_ctrl_de, mem_pc_de; // Control signals

    // DE/EX Register Outputs -> Inputs to EX Stage
    logic [63:0] pc_ex;
    logic [63:0] operand_a_ex; // Pipelined rs_data (or rd_data for I-type)
    logic [63:0] operand_b_ex; // Pipelined rt_data or literal
    logic [63:0] operand_c_ex; // Pipelined rd_data
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
    logic [63:0] alu_mem_data_ex;
    logic [63:0] alu_branch_pc_ex;
    logic        branch_taken_ex;
    logic        hlt_ex;

    // EX/MEM Register Outputs -> Inputs to MEM Stage
    logic [63:0] alu_result_mem;
    logic [63:0] mem_addr_mem;
    logic [63:0] mem_wdata_mem;
    logic [63:0] branch_pc_mem;
    logic [4:0]  rd_addr_mem; // Pipelined write address (destination)
    logic        mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, branch_taken_mem, mem_pc_mem; // Control
    logic        hlt_mem;

    // MEM Stage Outputs
    logic [63:0] mem_rdata_mem;
    logic [63:0] return_pc_mem;

    // MEM/WB Stage
    logic [63:0] reg_wdata_wb; // Data being written back

    // --- Forwarding Signals --- ADDED
    logic forwardA_mem_ex;      // Control for forwarding MEM->EX for ALU input A
    logic forwardB_mem_ex;      // Control for forwarding MEM->EX for ALU input B
    logic [63:0] alu_input1;    // Final value going into ALU input1 after muxing
    logic [63:0] alu_input2;    // Final value going into ALU input2 after muxing

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
        .branch_taken(branch_taken_ctrl_de), // Indicates branch type
        .mem_pc(mem_pc_de)
    );

    registerFile reg_file (
        .clk(clk),
        .reset(reset),
        // Write Port
        .write_addr(rd_addr_mem),
        .write_data(reg_wdata_wb),
        .write_enable(reg_write_mem),
        // Read Port 1 (rs)
        .read_addr1(rs_addr_de),
        .read_data1(operand_a_de),
        // Read Port 2 (rt)
        .read_addr2(rt_addr_de),
        .read_data2(regfile_operand_b_de),
        // Read Port 3 (rd)
        .read_addr3(rd_addr_de),
        .read_data3(operand_c_de),
        // Stack Pointer
        .stack_ptr_out(stack_ptr_de)
    );

    reglitmux input_selector (
        .sel(opcode_de),
        .reg_in(regfile_operand_b_de), // rt_data input
        .lit_in(literal_de),
        .out(operand_b_de)           // rt_data or literal output
    );

    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush_ex),
        // Inputs from Decode Stage
        .pc_in(pc_de),
        .operand_a_in(operand_a_de), // rs_data
        .operand_b_in(operand_b_de), // rt_data or literal
        .operand_c_in(operand_c_de), // rd_data
        .literal_in(literal_de),
        .rd_addr_in(rd_addr_de),     // Destination address
        .rs_addr_in(rs_addr_de),     // Source address 1
        .rt_addr_in(rt_addr_de),     // Source address 2
        .opcode_in(opcode_de),
        .stack_ptr_in(stack_ptr_de),
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
        .operand_c_out(operand_c_ex), // Pipelined rd_data
        .literal_out(literal_ex),
        .rd_addr_out(rd_addr_ex),     // Pipelined destination address
        .rs_addr_out(rs_addr_ex),
        .rt_addr_out(rt_addr_ex),
        .opcode_out(opcode_ex),
        .stack_ptr_out(stack_ptr_ex),
        .alu_enable_out(alu_enable_ex),
        .mem_read_out(mem_read_ex),
        .mem_write_out(mem_write_ex),
        .reg_write_out(reg_write_ex),
        .mem_to_reg_out(mem_to_reg_ex),
        .branch_taken_ctrl_out(),     // This output seems unused now
        .mem_pc_out(mem_pc_ex)
    );

    // *** ADDED: Forwarding Unit Instantiation ***
    forwardingUnit fwd_unit (
        .rs_addr_ex(rs_addr_ex),
        .rt_addr_ex(rt_addr_ex),
        .rd_addr_mem(rd_addr_mem),
        .reg_write_mem(reg_write_mem),
        .forwardA(forwardA_mem_ex),
        .forwardB(forwardB_mem_ex)
    );

    // *** ADDED: Forwarding Muxes for ALU Inputs ***
    assign alu_input1 = forwardA_mem_ex ? reg_wdata_wb : operand_a_ex;
    assign alu_input2 = forwardB_mem_ex ? reg_wdata_wb : operand_b_ex;

    // Execute Stage (ALU)
    // CHANGED: ALU inputs now come from forwarding muxes
    alu calculation_unit (
        .alu_enable(alu_enable_ex),
        .opcode(opcode_ex),
        .input1(alu_input1), // From fwd mux
        .input2(alu_input2), // From fwd mux
        .input3(operand_c_ex), // rd_data (forwarding not implemented for this)
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
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .mem_pc_in(mem_pc_ex)
    );

    ex_mem_register ex_mem_reg (
        .clk(clk),
        .flush_mem(flush_ex), // Connect flush signal
        // Inputs
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

    // --- Global Control Logic ---
    assign flush_de = branch_taken_mem;
    assign flush_ex = branch_taken_mem; // Controls flush for DE/EX and EX/MEM
    assign take_return_pc_fetch = mem_pc_mem;
    assign hlt = hlt_mem;

endmodule

//############################################################################
//## registerFile
//############################################################################
module registerFile (
    input clk,
    input reset,
    input [4:0] write_addr, input [63:0] write_data, input write_enable,
    input [4:0] read_addr1, output logic [63:0] read_data1,
    input [4:0] read_addr2, output logic [63:0] read_data2,
    input [4:0] read_addr3, output logic [63:0] read_data3,
    output logic [63:0] stack_ptr_out
);
    reg [63:0] registers [0:31]; integer idx;
    initial begin for(idx=0;idx<31;idx=idx+1) registers[idx]=0; registers[31]=64'h0008_0000; end
    assign read_data1 = (read_addr1 == 31) ? registers[31] : registers[read_addr1];
    assign read_data2 = (read_addr2 == 31) ? registers[31] : registers[read_addr2];
    assign read_data3 = (read_addr3 == 31) ? registers[31] : registers[read_addr3];
    assign stack_ptr_out = registers[31];
    always @(posedge clk) if (!reset && write_enable) registers[write_addr] <= write_data; // R0 is writable
endmodule

//############################################################################
//## fetch
//############################################################################
module fetch (
    input clk, input reset, input branch_taken, input [63:0] branch_pc,
    input take_return_pc, input [63:0] return_pc, output logic [63:0] pc_out
);
    localparam INITIAL_PC = 64'h2000; reg [63:0] current_pc; assign pc_out = current_pc;
    always @(posedge clk or posedge reset) if (reset) current_pc <= INITIAL_PC; else
        if(take_return_pc) current_pc <= return_pc; else if(branch_taken) current_pc <= branch_pc; else current_pc <= current_pc + 4;
endmodule

//############################################################################
//## if_de_register
//############################################################################
module if_de_register (
    input clk, input flush, input [63:0] pc_in, input [31:0] instruction_in,
    output reg [63:0] pc_out, output reg [31:0] instruction_out
);
    always @(posedge clk) if (flush) {pc_out, instruction_out} <= 0; else {pc_out, instruction_out} <= {pc_in, instruction_in};
endmodule

//############################################################################
//## instructionDecoder
//## CHANGED: Replaced SV assignment patterns with individual assignments for compatibility
//############################################################################
module instructionDecoder (
    input [31:0] instructionLine,
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    // Control Signals
    output reg alu_enable,
    output reg mem_read,
    output reg mem_write,
    output reg reg_write,
    output reg mem_to_reg,
    output reg branch_taken, // Indicates branch *type*
    output reg mem_pc
);
    // Opcodes
    localparam AND=5'h0, OR=5'h1, XOR=5'h2, NOT=5'h3, SHFTR=5'h4, SHFTRI=5'h5, SHFTL=5'h6, SHFTLI=5'h7,
               BR=5'h8, BRR=5'h9, BRRI=5'hA, BRNZ=5'hB, CALL=5'hC, RETURN=5'hD, BRGT=5'hE, PRIV=5'hF,
               MOV_MEM=5'h10, MOV_REG=5'h11, MOV_LIT=5'h12, MOV_STR=5'h13, ADDF=5'h14, SUBF=5'h15,
               MULF=5'h16, DIVF=5'h17, ADD=5'h18, ADDI=5'h19, SUB=5'h1A, SUBI=5'h1B, MUL=5'h1C, DIV=5'h1D;

    always @(*) begin // Using @(*) as requested previously
        // Field extraction
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        literal = {{52{1'b0}}, instructionLine[11:0]}; // Zero-extend

        // Default control signal values
        alu_enable = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        reg_write = 1'b0;
        mem_to_reg = 1'b0;
        branch_taken = 1'b0;
        mem_pc = 1'b0;

        // Decode based on opcode - using individual assignments now
        case (opcode)
            // R-Type Arithmetic/Logic/Shift
            ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL: begin
                alu_enable = 1'b1; reg_write = 1'b1; mem_to_reg = 1'b0;
            end
            // I-Type Arithmetic/Logic/Shift
            ADDI, SUBI, SHFTRI, SHFTLI: begin
                alu_enable = 1'b1; reg_write = 1'b1; mem_to_reg = 1'b0;
            end
            // Load
            MOV_MEM: begin
                alu_enable = 1'b1; mem_read = 1'b1; reg_write = 1'b1; mem_to_reg = 1'b1;
            end
            // Store
            MOV_STR: begin
                alu_enable = 1'b1; mem_write = 1'b1; reg_write = 1'b0;
            end
            // Register Moves
            MOV_REG: begin
                alu_enable = 1'b1; reg_write = 1'b1; mem_to_reg = 1'b0;
            end
            // Literal Move
            MOV_LIT: begin
                alu_enable = 1'b1; reg_write = 1'b1; mem_to_reg = 1'b0;
            end
            // Floating Point
            ADDF, SUBF, MULF, DIVF: begin
                alu_enable = 1'b1; reg_write = 1'b1; mem_to_reg = 1'b0;
            end
            // Control Flow - Branches
            BR, BRR, BRRI, BRNZ, BRGT: begin
                alu_enable = 1'b1; reg_write = 1'b0; branch_taken = 1'b1;
            end
            // Control Flow - Call
            CALL: begin
                alu_enable = 1'b1; mem_write = 1'b1; reg_write = 1'b0; branch_taken = 1'b1;
            end
            // Control Flow - Return
            RETURN: begin
                alu_enable = 1'b1; mem_read = 1'b1; reg_write = 1'b0; mem_pc = 1'b1; branch_taken = 1'b1;
            end
            // Privileged (HALT)
            PRIV: begin
                if (literal[11:0] == 12'h0) begin // L=0 for HALT
                    alu_enable = 1'b1; reg_write = 1'b0;
                end else begin
                    alu_enable = 1'b0; // Treat others as NOP/illegal
                end
            end
            default: ; // Default NOP (all signals 0 from defaults above)
        endcase

        // Adjust rs source for certain I-types AFTER control signals are set
        case (opcode)
            ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: rs = rd;
            default: ;
        endcase
    end
endmodule
//############################################################################
//## de_ex_register
//############################################################################
module de_ex_register (
    input clk, input flush, input [63:0] pc_in, input [63:0] operand_a_in, input [63:0] operand_b_in,
    input [63:0] operand_c_in, input [63:0] literal_in, input [4:0] rd_addr_in, input [4:0] rs_addr_in,
    input [4:0] rt_addr_in, input [4:0] opcode_in, input [63:0] stack_ptr_in, input alu_enable_in,
    input mem_read_in, input mem_write_in, input reg_write_in, input mem_to_reg_in,
    input branch_taken_ctrl_in, input mem_pc_in, output reg [63:0] pc_out, output reg [63:0] operand_a_out,
    output reg [63:0] operand_b_out, output reg [63:0] operand_c_out, output reg [63:0] literal_out,
    output reg [4:0] rd_addr_out, output reg [4:0] rs_addr_out, output reg [4:0] rt_addr_out,
    output reg [4:0] opcode_out, output reg [63:0] stack_ptr_out, output reg alu_enable_out,
    output reg mem_read_out, output reg mem_write_out, output reg reg_write_out, output reg mem_to_reg_out,
    output reg branch_taken_ctrl_out, output reg mem_pc_out
);
    always @(posedge clk) begin
        if (flush) begin
            pc_out <= 0; operand_a_out <= 0; operand_b_out <= 0; operand_c_out <= 0; literal_out <= 0;
            rd_addr_out <= 0; rs_addr_out <= 0; rt_addr_out <= 0; opcode_out <= 0; stack_ptr_out <= 0;
            alu_enable_out <= 0; mem_read_out <= 0; mem_write_out <= 0; reg_write_out <= 0;
            mem_to_reg_out <= 0; branch_taken_ctrl_out <= 0; mem_pc_out <= 0;
        end else begin
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
//############################################################################
module alu (
    input logic alu_enable, input logic [4:0] opcode, input logic [63:0] input1, input logic [63:0] input2,
    input logic [63:0] input3, input logic [4:0] rd_addr, input logic [63:0] literal, input logic [63:0] pc_in,
    input logic [63:0] stack_ptr, output logic [63:0] result, output logic [63:0] mem_addr,
    output logic [63:0] mem_wdata, output logic [63:0] branch_pc, output logic branch_taken,
    output logic hlt_out, input logic mem_read_in, input logic mem_write_in, input logic reg_write_in,
    input logic mem_to_reg_in, input logic mem_pc_in
);
    localparam AND=0, OR=1, XOR=2, NOT=3, SHFTR=4, SHFTRI=5, SHFTL=6, SHFTLI=7, BR=8, BRR=9, BRRI=10, BRNZ=11,
               CALL=12, RETURN=13, BRGT=14, PRIV=15, MOV_MEM=16, MOV_REG=17, MOV_LIT=18, MOV_STR=19,
               ADDF=20, SUBF=21, MULF=22, DIVF=23, ADD=24, ADDI=25, SUB=26, SUBI=27, MUL=28, DIV=29;
    logic [63:0] fp_result;

    always @(*) begin
        result=64'b0; mem_addr=64'b0; mem_wdata=64'b0; branch_pc=pc_in+4; branch_taken=1'b0; hlt_out=1'b0;
        if (alu_enable) begin
            case (opcode)
                ADD, ADDI: result = $signed(input1) + $signed(input2);
                SUB, SUBI: result = $signed(input1) - $signed(input2);
                MUL: result = $signed(input1) * $signed(input2);
                DIV: if (input2 != 0) result = $signed(input1) / $signed(input2); else result = 64'b0;
                AND: result = input1 & input2;
                OR:  result = input1 | input2;
                XOR: result = input1 ^ input2;
                NOT: result = ~input1;
                SHFTR, SHFTRI: result = input1 >> input2[5:0];
                SHFTL, SHFTLI: result = input1 << input2[5:0];
                MOV_MEM: mem_addr = input1 + $signed(input2);
                MOV_REG: result = input1;
                MOV_LIT: result = {input1[63:12], input2[11:0]};
                MOV_STR: begin mem_addr=input3+$signed(literal); mem_wdata=input1; end
                ADDF, SUBF, MULF, DIVF: result = 64'b0;
                BR: begin branch_pc = input3; branch_taken = 1'b1; end
                BRR: begin branch_pc = $signed(pc_in) + $signed(input3); branch_taken = 1'b1; end
                BRRI: begin branch_pc = pc_in + $signed(input2); branch_taken = 1'b1; end
                BRNZ: begin if ($signed(input1)!=0) begin branch_pc=input3; branch_taken=1'b1; end else branch_taken=1'b0; end
                BRGT: begin if ($signed(input1)>$signed(input2)) begin branch_pc=input3; branch_taken=1'b1; end else branch_taken=1'b0; end
                CALL: begin branch_pc=input3; mem_addr=stack_ptr-8; mem_wdata=pc_in+4; branch_taken=1'b1; end
                RETURN: begin mem_addr=stack_ptr-8; branch_taken=1'b1; end
                PRIV: if(literal[11:0]==12'h0) hlt_out=1'b1;
                default: result = 64'b0;
            endcase
        end
    end
endmodule

//############################################################################
//## ex_mem_register
//############################################################################
module ex_mem_register (
    input clk, input logic flush_mem, input logic [63:0] result_in, input logic [63:0] mem_addr_in,
    input logic [63:0] mem_wdata_in, input logic [63:0] branch_pc_in, input logic [4:0] rd_addr_in,
    input logic hlt_in, input logic mem_read_in, input logic mem_write_in, input logic reg_write_in,
    input logic mem_to_reg_in, input logic branch_taken_in, input logic mem_pc_in,
    output logic [63:0] result_out, output logic [63:0] mem_addr_out, output logic [63:0] mem_wdata_out,
    output logic [63:0] branch_pc_out, output logic [4:0] rd_addr_out, output logic hlt_out,
    output logic mem_read_out, output logic mem_write_out, output logic reg_write_out,
    output logic mem_to_reg_out, output logic branch_taken_out, output logic mem_pc_out
);
    always @(posedge clk) begin
        if (flush_mem) begin
            result_out<=0; mem_addr_out<=0; mem_wdata_out<=0; branch_pc_out<=0; rd_addr_out<=0; hlt_out<=0;
            mem_read_out<=0; mem_write_out<=0; reg_write_out<=0; mem_to_reg_out<=0; branch_taken_out<=0; mem_pc_out<=0;
        end else begin
            result_out<=result_in; mem_addr_out<=mem_addr_in; mem_wdata_out<=mem_wdata_in; branch_pc_out<=branch_pc_in;
            rd_addr_out<=rd_addr_in; hlt_out<=hlt_in; mem_read_out<=mem_read_in; mem_write_out<=mem_write_in;
            reg_write_out<=reg_write_in; mem_to_reg_out<=mem_to_reg_in; branch_taken_out<=branch_taken_in; mem_pc_out<=mem_pc_in;
        end
    end
endmodule

//############################################################################
//## memory
//############################################################################
module memory (
    input clk, input reset, input [63:0] inst_addr, output logic [31:0] instruction_out,
    input [63:0] data_addr, input [63:0] data_wdata, input mem_read, input mem_write,
    output logic [63:0] data_rdata
);
    localparam MEM_SIZE_BYTES = 524288; reg [7:0] bytes [0:MEM_SIZE_BYTES-1]; integer i;
    initial for(i=0;i<MEM_SIZE_BYTES;i=i+1) bytes[i]=0;
    assign instruction_out[7:0]=bytes[inst_addr+0]; assign instruction_out[15:8]=bytes[inst_addr+1];
    assign instruction_out[23:16]=bytes[inst_addr+2]; assign instruction_out[31:24]=bytes[inst_addr+3];
    assign data_rdata[7:0]=bytes[data_addr+0]; assign data_rdata[15:8]=bytes[data_addr+1];
    assign data_rdata[23:16]=bytes[data_addr+2]; assign data_rdata[31:24]=bytes[data_addr+3];
    assign data_rdata[39:32]=bytes[data_addr+4]; assign data_rdata[47:40]=bytes[data_addr+5];
    assign data_rdata[55:48]=bytes[data_addr+6]; assign data_rdata[63:56]=bytes[data_addr+7];
    always @(posedge clk) if(mem_write) begin bytes[data_addr+0]<=data_wdata[7:0]; bytes[data_addr+1]<=data_wdata[15:8];
        bytes[data_addr+2]<=data_wdata[23:16]; bytes[data_addr+3]<=data_wdata[31:24]; bytes[data_addr+4]<=data_wdata[39:32];
        bytes[data_addr+5]<=data_wdata[47:40]; bytes[data_addr+6]<=data_wdata[55:48]; bytes[data_addr+7]<=data_wdata[63:56]; end
endmodule

//############################################################################
//## aluMemMux
//############################################################################
module aluMemMux ( input mem_pc, input [63:0] memData, input [63:0] aluOut, output reg [63:0] newPc );
    always @(*) if (mem_pc) newPc = memData; else newPc = aluOut;
endmodule

//############################################################################
//## reglitmux
//############################################################################
module reglitmux ( input [4:0] sel, input [63:0] reg_in, input [63:0] lit_in, output reg [63:0] out );
    localparam ADDI=19+6, SUBI=27, SHFTRI=5, SHFTLI=7, BRRI=10, MOV_MEM=16, MOV_LIT=18, MOV_STR=19; // Note: Opcodes adjusted based on 5'hXX values
    always @(*) case (sel) ADDI,SUBI,SHFTRI,SHFTLI,BRRI,MOV_MEM,MOV_LIT,MOV_STR: out=lit_in; default: out=reg_in; endcase
endmodule

//############################################################################
//## memRegMux
//############################################################################
module memRegMux ( input mem_to_reg, input [63:0] readData, input [63:0] aluResult, output reg [63:0] regWriteData );
    always @(*) if (mem_to_reg) regWriteData = readData; else regWriteData = aluResult;
endmodule