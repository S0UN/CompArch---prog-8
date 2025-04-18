module tinker_core (
    input clk,
    input reset,
    output hlt
);
    logic [31:0] instruction_word, if_de_instruction;
    logic [4:0] instruction_type;
    logic [4:0] target_register, src_reg_a, src_reg_b;
    logic [63:0] current_address, next_address, alu_calculated_address, if_de_pc;
    logic [63:0] constant_value;
    logic [63:0] source_a_value, source_b_value, target_reg_value;
    logic [63:0] secondary_input, computation_result;
    logic [63:0] stack_address;
    logic [63:0] memory_write_value, memory_access_address, memory_read_value;
    logic [63:0] register_input_value;
    logic memory_write_flag, use_memory_address, memory_read_flag;
    logic computation_start, computation_complete;
    logic register_read_start, register_write_start;
    logic memory_operation_start;
    logic flush;

    // Wires for ALU outputs
    logic mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out, branch_taken_out, mem_pc_out;

    // Distinct EX/MEM register outputs
    logic [63:0] ex_mem_result_out, ex_mem_writeData_out, ex_mem_rwAddress_out, ex_mem_pc_out;
    logic ex_mem_hlt_out, ex_mem_mem_read_out, ex_mem_mem_write_out, ex_mem_reg_write_out;
    logic ex_mem_mem_to_reg_out, ex_mem_branch_taken_out, ex_mem_mem_pc_out;

    // DE/EX register outputs
    logic [63:0] de_ex_literal_out;
    logic [4:0] de_ex_rd_out, de_ex_rs_out, de_ex_rt_out, de_ex_opcode_out;
    logic de_ex_alu_enable_out, de_ex_mem_read_out, de_ex_mem_write_out, de_ex_reg_write_out;
    logic de_ex_mem_to_reg_out, de_ex_branch_taken_out, de_ex_mem_pc_out;

    // Register File
    registerFile reg_file(
        .data(register_input_value),
        .read1(src_reg_a),
        .read2(src_reg_b),
        .write(target_register),
        .reset(reset),
        .clk(clk),
        .regReadFlag(register_read_start),
        .regWriteFlag(ex_mem_reg_write_out),
        .output1(source_a_value),
        .output2(source_b_value),
        .output3(target_reg_value),
        .stackPtr(stack_address)
    );

    // Fetch
    fetch instruction_fetcher(
        .clk(clk),
        .reset(reset),
        .branch_taken(ex_mem_branch_taken_out),
        .branch_pc(ex_mem_pc_out),
        .pc(current_address)
    );

    // IF/DE Register
    if_de_register if_de_reg (
        .clk(clk),
        .flush(flush),
        .pc_in(current_address),
        .instruction_in(instruction_word),
        .pc_out(if_de_pc),
        .instruction_out(if_de_instruction)
    );

    // Instruction Decoder
    instructionDecoder instruction_parser(
        .instructionLine(if_de_instruction),
        .literal(constant_value),
        .rd(target_register),
        .rs(src_reg_a),
        .rt(src_reg_b),
        .opcode(instruction_type),
        .alu_enable(computation_start),
        .mem_read(memory_read_flag),
        .mem_write(memory_write_flag),
        .reg_write(register_write_start),
        .mem_to_reg(mem_to_reg_out),  // Note: renamed to avoid conflict
        .branch_taken(branch_taken_out),  // Note: renamed to avoid conflict
        .mem_pc(use_memory_address)
    );

    // RegLitMux
    reglitmux input_selector(
        .sel(instruction_type),
        .reg1(source_b_value),
        .lit(constant_value),
        .out(secondary_input)
    );

    // DE/EX Register
    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush),
        .literal_in(constant_value),
        .rd_in(target_register),
        .rs_in(src_reg_a),
        .rt_in(src_reg_b),
        .opcode_in(instruction_type),
        .alu_enable_in(computation_start),
        .mem_read_in(memory_read_flag),
        .mem_write_in(memory_write_flag),
        .reg_write_in(register_write_start),
        .mem_to_reg_in(mem_to_reg_out),
        .branch_taken_in(branch_taken_out),
        .mem_pc_in(use_memory_address),
        .literal_out(de_ex_literal_out),
        .rd_out(de_ex_rd_out),
        .rs_out(de_ex_rs_out),
        .rt_out(de_ex_rt_out),
        .opcode_out(de_ex_opcode_out),
        .alu_enable_out(de_ex_alu_enable_out),
        .mem_read_out(de_ex_mem_read_out),
        .mem_write_out(de_ex_mem_write_out),
        .reg_write_out(de_ex_reg_write_out),
        .mem_to_reg_out(de_ex_mem_to_reg_out),
        .branch_taken_out(de_ex_branch_taken_out),
        .mem_pc_out(de_ex_mem_pc_out)
    );

    // ALU (Execute Stage)
    alu calculation_unit(
        .alu_enable_in(de_ex_alu_enable_out),
        .mem_read_in(de_ex_mem_read_out),
        .mem_write_in(de_ex_mem_write_out),
        .reg_write_in(de_ex_reg_write_out),
        .mem_to_reg_in(de_ex_mem_to_reg_out),
        .mem_pc_in(de_ex_mem_pc_out),
        .opcode(de_ex_opcode_out),
        .input1(source_a_value),
        .input2(secondary_input),
        .rd({59'b0, de_ex_rd_out}),
        .inputPc(if_de_pc),
        .r31(stack_address),
        .result(computation_result),
        .writeData(memory_write_value),
        .rwAddress(memory_access_address),
        .pc(alu_calculated_address),
        .hlt(hlt),
        .mem_read_out(mem_read_out),
        .mem_write_out(mem_write_out),
        .reg_write_out(reg_write_out),
        .mem_to_reg_out(mem_to_reg_out),
        .branch_taken_out(branch_taken_out),
        .mem_pc_out(mem_pc_out)
    );

    // EX/MEM Register
    ex_mem_register ex_mem_reg (
        .clk(clk),
        .result_in(computation_result),
        .writeData_in(memory_write_value),
        .rwAddress_in(memory_access_address),
        .pc_in(alu_calculated_address),
        .hlt_in(hlt),
        .mem_read_in(mem_read_out),
        .mem_write_in(mem_write_out),
        .reg_write_in(reg_write_out),
        .mem_to_reg_in(mem_to_reg_out),
        .branch_taken_in(branch_taken_out),
        .mem_pc_in(mem_pc_out),
        .result_out(ex_mem_result_out),
        .writeData_out(ex_mem_writeData_out),
        .rwAddress_out(ex_mem_rwAddress_out),
        .pc_out(ex_mem_pc_out),
        .hlt_out(ex_mem_hlt_out),
        .mem_read_out(ex_mem_mem_read_out),
        .mem_write_out(ex_mem_mem_write_out),
        .reg_write_out(ex_mem_reg_write_out),
        .mem_to_reg_out(ex_mem_mem_to_reg_out),
        .branch_taken_out(ex_mem_branch_taken_out),
        .mem_pc_out(ex_mem_mem_pc_out)
    );

    // Memory
    memory memory(
        .pc(current_address),
        .clk(clk),
        .reset(reset),
        .writeFlag(ex_mem_mem_write_out),  // Fixed: Use write enable signal
        .memFlag(memory_operation_start),
        .memRead(ex_mem_mem_read_out),
        .writeData(ex_mem_writeData_out),  // Correct: Data to write
        .rwAddress(ex_mem_rwAddress_out),
        .readData(memory_read_value),
        .instruction(instruction_word)
    );

    // ALU Mem Mux
    aluMemMux address_source_selector(
        .mem_pc(ex_mem_mem_pc_out),  // Updated to use EX/MEM output
        .memData(memory_read_value),
        .aluOut(alu_calculated_address),
        .newPc(next_address)
    );

    // Mem Reg Mux
    memRegMux data_source_selector(
        .mem_to_reg(ex_mem_mem_to_reg_out),
        .readData(memory_read_value),
        .aluResult(ex_mem_result_out),
        .regWriteData(register_input_value)
    );

    // Control signals
    assign memory_operation_start = ex_mem_mem_read_out | ex_mem_mem_write_out;
    assign flush = ex_mem_branch_taken_out;
endmodule


module registerFile (
    input [63:0] data,
    input [4:0] read1,
    input [4:0] read2,
    input [4:0] write,
    input reset,
    input clk,
    input regReadFlag,
    input regWriteFlag,
    output reg [63:0] output1,
    output reg [63:0] output2,
    output reg [63:0] output3,
    output reg [63:0] stackPtr
);
    reg [63:0] registers [0:31];
    integer idx;

    // Initialize registers
    initial begin
        for (idx = 0; idx < 31; idx = idx + 1) begin
            registers[idx] <= 64'b0;
        end
        registers[31] <= 64'd524288;
    end

    // Continuous assignment for stack pointer
    assign stackPtr = registers[31];

    // Read and write operations
    always @(posedge clk) begin
        if (regReadFlag) begin
            output1 = registers[read1];
            output2 = registers[read2];
            output3 = registers[write];
        end
    end

    // Write operation 
    always @(*) begin
        if (regWriteFlag) begin
            registers[write] = data;
        end
    end
endmodule

module fetch (
    input branch_taken,
    input clk,
    input reset,
    input [63:0] branch_pc,
    output reg [63:0] pc
);
    localparam INITIAL_PC = 64'h2000;

    reg [63:0] address_register;

    always @(*) begin
        pc = address_register;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            address_register <= INITIAL_PC;
        end else begin
            if (branch_taken) begin
                address_register <= branch_pc;
            end else begin
                address_register <= address_register + 64'd4;
            end
        end
    end
endmodule

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
            pc_out <= 64'b0;
            instruction_out <= 32'b0;
        end else begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end
endmodule



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
    output reg branch_taken,
    output reg mem_pc
);
    localparam AND     = 5'h00;
    localparam OR      = 5'h01;
    localparam XOR     = 5'h02;
    localparam NOT     = 5'h03;
    localparam SHFTR   = 5'h04;
    localparam SHFTRI  = 5'h05;
    localparam SHFTL   = 5'h06;
    localparam SHFTLI  = 5'h07;
    localparam BR      = 5'h08;
    localparam BRR     = 5'h09;
    localparam BRRI    = 5'h0A;
    localparam BRNZ    = 5'h0B;
    localparam CALL    = 5'h0C;
    localparam RETURN  = 5'h0D;
    localparam BRGT    = 5'h0E;
    localparam PRIV    = 5'h0F;
    localparam MOV_MEM = 5'h10;
    localparam MOV_REG = 5'h11;
    localparam MOV_LIT = 5'h12;
    localparam MOV_STR = 5'h13;
    localparam ADDF    = 5'h14;
    localparam SUBF    = 5'h15;
    localparam MULF    = 5'h16;
    localparam DIVF    = 5'h17;
    localparam ADD     = 5'h18;
    localparam ADDI    = 5'h19;
    localparam SUB     = 5'h1A;
    localparam SUBI    = 5'h1B;
    localparam MUL     = 5'h1C;
    localparam DIV     = 5'h1D;

    always @(*) begin
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        literal = {52'b0, instructionLine[11:0]};
        alu_enable = 0;
        mem_read = 0;
        mem_write = 0;
        reg_write = 0;
        mem_to_reg = 0;
        branch_taken = 0;
        mem_pc = 0;

        case (opcode)
            ADD, ADDI, SUB, SUBI, MUL, DIV: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            AND, OR, XOR, NOT: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            SHFTR, SHFTRI, SHFTL, SHFTLI: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            MOV_MEM: begin
                alu_enable = 1;
                mem_read = 1;
                reg_write = 1;
                mem_to_reg = 1;
            end
            MOV_REG: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            MOV_LIT: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            MOV_STR: begin
                alu_enable = 1;
                mem_write = 1;
            end
            ADDF, SUBF, MULF, DIVF: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            BR, BRR, BRRI, BRNZ, BRGT: begin
                alu_enable = 1;
            end
            CALL: begin
                alu_enable = 1;
                mem_write = 1;
                branch_taken = 1;
            end
            RETURN: begin
                mem_read = 1;
                mem_pc = 1;
            end
            PRIV: begin
                // No control signals
            end
            default: begin
                // No-op
            end
        endcase

        case (opcode)
            ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: rs = rd;
            default: ; // No change
        endcase
    end
endmodule



module de_ex_register (
    input clk,
    input flush,
    input [63:0] literal_in,
    input [4:0] rd_in,
    input [4:0] rs_in,
    input [4:0] rt_in,
    input [4:0] opcode_in,
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,
    input mem_to_reg_in,
    input branch_taken_in,
    input mem_pc_in,
    output reg [63:00] literal_out,
    output reg [4:0] rd_out,
    output reg [4:0] rs_out,
    output reg [4:0] rt_out,
    output reg [4:0] opcode_out,
    output reg alu_enable_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_out,
    output reg mem_pc_out
);
    always @(posedge clk) begin
        if (flush) begin
            opcode_out <= 5'b0;
            literal_out <= 64'b0;
            rd_out <= 5'b0;
            rs_out <= 5'b0;
            rt_out <= 5'b0;
            alu_enable_out <= 1'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            branch_taken_out <= 1'b0;
            mem_pc_out <= 1'b0;
        end else begin
            opcode_out <= opcode_in;
            literal_out <= literal_in;
            rd_out <= rd_in;
            rs_out <= rs_in;
            rt_out <= rt_in;
            alu_enable_out <= alu_enable_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            branch_taken_out <= branch_taken_in;
            mem_pc_out <= mem_pc_in;
        end
    end
endmodule


module alu (
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,
    input mem_to_reg_in,
    input mem_pc_in,
    input [4:0] opcode,
    input [63:0] input1,
    input [63:0] input2,
    input [63:0] rd,
    input [63:0] inputPc,
    input [63:0] r31,
    output reg [63:0] result,
    output reg [63:0] writeData,
    output reg [63:0] rwAddress,
    output reg [63:0] pc,
    output reg hlt,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_out,
    output reg mem_pc_out
);
    // Instruction opcodes from Tinker Instruction Manual
    localparam AND     = 5'h00;  // and rd, rs, rt
    localparam OR      = 5'h01;  // or rd, rs, rt
    localparam XOR     = 5'h02;  // xor rd, rs, rt
    localparam NOT     = 5'h03;  // not rd, rs
    localparam SHFTR   = 5'h04;  // shftr rd, rs, rt
    localparam SHFTRI  = 5'h05;  // shftri rd, L
    localparam SHFTL   = 5'h06;  // shftl rd, rs, rt
    localparam SHFTLI  = 5'h07;  // shftli rd, L
    localparam BR      = 5'h08;  // br rd
    localparam BRR     = 5'h09;  // brr rd
    localparam BRRI    = 5'h0A;  // brr L
    localparam BRNZ    = 5'h0B;  // brnz rd, rs
    localparam CALL    = 5'h0C;  // call rd, rs, rt
    localparam RETURN  = 5'h0D;  // return
    localparam BRGT    = 5'h0E;  // brgt rd, rs, rt
    localparam PRIV    = 5'h0F;  // priv (halt instruction)
    localparam MOV_MEM = 5'h10;  // mov rd, (rs)(L)
    localparam MOV_REG = 5'h11;  // mov rd, rs
    localparam MOV_LIT = 5'h12;  // mov rd, L
    localparam MOV_STR = 5'h13;  // mov (rd)(L), rs
    localparam ADDF    = 5'h14;  // addf rd, rs, rt
    localparam SUBF    = 5'h15;  // subf rd, rs, rt
    localparam MULF    = 5'h16;  // mulf rd, rs, rt
    localparam DIVF    = 5'h17;  // divf rd, rs, rt
    localparam ADD     = 5'h18;  // add rd, rs, rt
    localparam ADDI    = 5'h19;  // addi rd, L
    localparam SUB     = 5'h1A;  // sub rd, rs, rt
    localparam SUBI    = 5'h1B;  // subi rd, L
    localparam MUL     = 5'h1C;  // mul rd, rs, rt
    localparam DIV     = 5'h1D;  // div rd, rs, rt

    // Memory base address
    localparam MEM_BASE_ADDR = 64'h2000;

    // Floating point operations
    real float_operand1, float_operand2, float_result;
    assign float_operand1 = $bitstoreal(input1);
    assign float_operand2 = $bitstoreal(input2);

    always @(*) begin
        // Pass through flags from DE/EX to EX/MEM
        mem_read_out = mem_read_in;
        mem_write_out = mem_write_in;
        reg_write_out = reg_write_in;
        mem_to_reg_out = mem_to_reg_in;
        mem_pc_out = mem_pc_in;
        
        // Default values
        result = 64'b0;
        pc = inputPc + 4;  // Default: increment PC by 4
        writeData = 64'b0;
        rwAddress = MEM_BASE_ADDR;
        hlt = 1'b0;
        branch_taken_out = 1'b0;

        if (alu_enable_in) begin
            case (opcode)
                // Integer Arithmetic Instructions
                ADD, ADDI: result = input1 + input2;
                SUB, SUBI: result = input1 - input2;
                MUL: result = input1 * input2;
                DIV: result = input1 / input2;
                
                // Logic Instructions
                AND: result = input1 & input2;
                OR:  result = input1 | input2;
                XOR: result = input1 ^ input2;
                NOT: result = ~input1;
                
                // Shift Instructions
                SHFTR, SHFTRI: result = input1 >> input2;
                SHFTL, SHFTLI: result = input1 << input2;
                
                // Data Movement Instructions
                MOV_MEM: rwAddress = input1 + input2;
                MOV_REG: result = input1;
                MOV_LIT: result = {input1[63:12], input2[11:0]};
                MOV_STR: begin
                    rwAddress = rd + input2;
                    writeData = input1;
                end
                
                // Floating Point Instructions
                ADDF: begin
                    float_result = float_operand1 + float_operand2;
                    result = $realtobits(float_result);
                end
                SUBF: begin
                    float_result = float_operand1 - float_operand2;
                    result = $realtobits(float_result);
                end
                MULF: begin
                    float_result = float_operand1 * float_operand2;
                    result = $realtobits(float_result);
                end
                DIVF: begin
                    float_result = float_operand1 / float_operand2;
                    result = $realtobits(float_result);
                end
                
                // Control Instructions
                BR: begin
                    pc = rd;
                    branch_taken_out = 1;
                end
                BRR: begin
                    pc = inputPc + rd;
                    branch_taken_out = 1;
                end
                BRRI: begin
                    pc = inputPc + $signed(input2);
                    branch_taken_out = 1;
                end
                BRNZ: begin
                    if (input1 != 0) begin
                        pc = rd;
                        branch_taken_out = 1;
                    end else begin
                        pc = inputPc + 4;
                        branch_taken_out = 0;
                    end
                end
                BRGT: begin
                    if (input1 > input2) begin
                        pc = rd;
                        branch_taken_out = 1;
                    end else begin
                        pc = inputPc + 4;
                        branch_taken_out = 0;
                    end
                end
                
                // Subroutine Instructions
                CALL: begin
                    pc = rd;
                    writeData = inputPc + 4;  // Save return address
                    rwAddress = r31 - 8;
                    branch_taken_out = 1;
                end
                RETURN: begin
                    rwAddress = r31 - 8;
                    branch_taken_out = 1;  // Treat as a branch for simplicity
                end
                
                // Privileged Instructions
                PRIV: hlt = 1'b1;  // Assuming PRIV is used as HALT here
                
                default: begin
                    // Default case (should not happen but added for safety)
                    result = 64'b0;
                end
            endcase
        end
    end
endmodule


module ex_mem_register(
    input clk,
    input reg [63:0] result_in,
    input reg [63:0] writeData_in,
    input reg [63:0] rwAddress_in,
    input reg [63:0] pc_in,
    input reg hlt_in,
    input reg mem_read_in,
    input reg mem_write_in,
    input reg reg_write_in,
    input reg mem_to_reg_in,
    input reg branch_taken_in,
    input reg mem_pc_in,
    output reg [63:0] result_out,
    output reg [63:0] writeData_out,
    output reg [63:00] rwAddress_out,
    output reg [63:0] pc_out,
    output reg hlt_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_out,
    output reg mem_pc_out
);
    always @(posedge clk) begin
        result_out <= result_in;
        writeData_out <= writeData_in;
        rwAddress_out <= rwAddress_in;
        pc_out <= pc_in;
        hlt_out <= hlt_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        branch_taken_out <= branch_taken_in;
        mem_pc_out <= mem_pc_in;
    end
endmodule


module memory (
    input [63:0] pc,
    input clk,
    input reset,
    input writeFlag,
    input memFlag,
    input memRead,
    input [63:0] writeData,
    input [63:0] rwAddress,
    output reg [63:0] readData,
    output reg [31:0] instruction
);
    reg [7:0] bytes [0:524287];
    integer memory_index;

    // Reset memory on positive edge of reset
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (memory_index = 0; memory_index < 524288; memory_index = memory_index + 1) begin
                bytes[memory_index] <= 8'b0;
            end
        end
    end

    // Instruction fetch (continuous assignment)
    assign instruction[7:0]   = bytes[pc];
    assign instruction[15:8]  = bytes[pc+1];
    assign instruction[23:16] = bytes[pc+2];
    assign instruction[31:24] = bytes[pc+3];

    // Memory read and write operations
    always @(memFlag) begin
        // Read operation
        if (memRead) begin
            readData[7:0]   = bytes[rwAddress];
            readData[15:8]  = bytes[rwAddress+1];
            readData[23:16] = bytes[rwAddress+2];
            readData[31:24] = bytes[rwAddress+3];
            readData[39:32] = bytes[rwAddress+4];
            readData[47:40] = bytes[rwAddress+5];
            readData[55:48] = bytes[rwAddress+6];
            readData[63:56] = bytes[rwAddress+7];
        end

        // Write operation
        if (writeFlag) begin
            bytes[rwAddress]   = writeData[7:0];
            bytes[rwAddress+1] = writeData[15:8];
            bytes[rwAddress+2] = writeData[23:16];
            bytes[rwAddress+3] = writeData[31:24];
            bytes[rwAddress+4] = writeData[39:32];
            bytes[rwAddress+5] = writeData[47:40];
            bytes[rwAddress+6] = writeData[55:48];
            bytes[rwAddress+7] = writeData[63:56];
        end
    end
endmodule



module aluMemMux (
    input mem_pc,
    input [63:0] memData,
    input [63:0] aluOut,
    output reg [63:0] newPc
);
    always @(*) begin
        if (mem_pc) begin
            newPc = memData;
        end
        else begin
            newPc = aluOut;
        end
    end
endmodule


module reglitmux (
    input [4:0] sel,
    input [63:0] reg1,
    input [63:0] lit,
    output reg [63:0] out
);
    always @(*) begin
        if (sel == 5'b11001 || sel == 5'b11011 || sel == 5'b00101 || 
            sel == 5'b00111 || sel == 5'b10010 || sel == 5'b01010 || 
            sel == 5'h10    || sel == 5'h13)
            out = lit;
        else
            out = reg1;
    end
endmodule

module memRegMux (
    input mem_to_reg,                // Add this input
    input [63:0] readData,
    input [63:0] aluResult,
    output reg [63:0] regWriteData
);
    always @(*) begin
        if (mem_to_reg)
            regWriteData = readData;
        else
            regWriteData = aluResult;
    end
endmodule