module tinker_core (
    input clk,
    input reset,
    output hlt
);
    // Instruction-related signals
    logic [31:0] instruction_word;
    logic [4:0] instruction_type;
    logic [4:0] target_register, source_register_a, source_register_b;
    
    // Address-related signals
    logic [63:0] current_address, subsequent_address, alu_calculated_address;
    
    // Data signals
    logic [63:0] constant_value;
    logic [63:0] source_a_value, source_b_value, target_reg_value;
    logic [63:0] secondary_input, computation_result;
    logic [63:0] stack_address;
    logic [63:0] memory_write_value, memory_access_address, memory_read_value;
    logic [63:0] register_input_value;
    
    // Control signals
    logic memory_write_flag, use_memory_address, memory_read_flag;
    logic computation_start, computation_complete;
    logic register_read_start, register_write_start;
    logic fetch_start, memory_operation_start;

    // Fetch unit - handles instruction fetching
    fetch instruction_fetcher(
        .clk(clk),
        .fetchFlag(fetch_start),
        .reset(reset),
        .next_pc(subsequent_address),
        .pc(current_address)
    );
    
    // Memory module - handles memory operations and instruction storage
    memory memory(
        .pc(current_address),
        .clk(clk),
        .reset(reset),
        .writeFlag(memory_write_flag),
        .fetchFlag(fetch_start),
        .memFlag(memory_operation_start),
        .memRead(memory_read_flag),
        .writeData(memory_write_value),
        .rwAddress(memory_access_address),
        .readData(memory_read_value),
        .instruction(instruction_word)
    );

    // Instruction decoder - decodes fetched instructions
    instructionDecoder instruction_parser(
        .instructionLine(instruction_word),
        .clk(clk),
        .rst(reset),
        .aluDone(computation_complete),
        .memFlag(memory_operation_start),
        .literal(constant_value),
        .rd(target_register),
        .rs(source_register_a),
        .rt(source_register_b),
        .opcode(instruction_type),
        .aluEnable(computation_start),
        .fetchFlag(fetch_start),
        .regReadFlag(register_read_start),
        .regWriteFlag(register_write_start)
    );

    // Register file - manages CPU registers
    registerFile reg_file(
        .data(register_input_value),
        .read1(source_register_a),
        .read2(source_register_b),
        .write(target_register),
        .reset(reset),
        .clk(clk),
        .regReadFlag(register_read_start),
        .regWriteFlag(register_write_start),
        .output1(source_a_value),
        .output2(source_b_value),
        .output3(target_reg_value),
        .stackPtr(stack_address)
    );

    // Input selector - selects between register or immediate value
    reglitmux input_selector(
        .sel(instruction_type),
        .reg1(source_b_value),
        .lit(constant_value),
        .out(secondary_input)
    );

    // ALU - handles computational operations
    alu calculation_unit(
        .aluEnable(computation_start),
        .control(instruction_type),
        .input1(source_a_value),
        .input2(secondary_input),
        .inputPc(current_address),
        .r31(stack_address),
        .rd(target_reg_value),
        .clk(clk),
        .result(computation_result),
        .pc(alu_calculated_address),
        .writeFlag(memory_write_flag),
        .memRead(memory_read_flag),
        .aluDone(computation_complete),
        .writeData(memory_write_value),
        .rwAddress(memory_access_address),
        .hlt(hlt),
        .mem_pc(use_memory_address)
    );

    // Address source selector - selects next program counter value
    aluMemMux address_source_selector(
        .mem_pc(use_memory_address),
        .memData(memory_read_value),
        .aluOut(alu_calculated_address),
        .newPc(subsequent_address)
    );

    // Data source selector - selects write data for registers
    memRegMux data_source_selector(
        .opcode(instruction_type),
        .readData(memory_read_value),
        .aluResult(computation_result),
        .regWriteData(register_input_value)
    );
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
    always @(*) begin
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
module memRegMux (
    input [4:0] opcode,
    input [63:0] readData,
    input [63:0] aluResult,
    output reg [63:0] regWriteData
);
    always @(*) begin
        if (opcode == 5'h10)
            regWriteData = readData;
        else
            regWriteData = aluResult;
    end
endmodule

module memory (
    input [63:0] pc,
    input clk,
    input reset,
    input writeFlag,
    input fetchFlag,
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


module instructionDecoder (
    input [31:0] instructionLine,
    input clk,
    input rst,
    input aluDone,
    output reg fetchFlag,
    output reg memFlag,
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg aluEnable,
    output reg regReadFlag,
    output reg regWriteFlag
);
    reg [2:0] decoder_state;
    reg [4:0] cached_opcode;

    always @(posedge rst) begin
        if (rst) decoder_state <= 3'b0;
    end

    always @(posedge clk) begin
        aluEnable <= 0;
        regWriteFlag <= 0;
        regReadFlag <= 0;
        memFlag <= 0;

        case (decoder_state)
            3'b0: decoder_state <= 3'b1;
            3'b1: decoder_state <= 3'b10;
            3'b10: begin
                if (!aluDone) decoder_state <= 3'b10;
                else if (cached_opcode == 5'h10 || cached_opcode == 5'h13 || cached_opcode == 5'h0C || cached_opcode == 5'h0D) begin
                    decoder_state <= 3'b11;
                end
                else if (cached_opcode == 5'h08 || cached_opcode == 5'h09 || cached_opcode == 5'h0A || cached_opcode == 5'h0B || cached_opcode == 5'h0E) begin
                    decoder_state <= 3'b0;
                end
                else decoder_state <= 3'b100;
            end
            3'b11: begin
                if (cached_opcode == 5'h10) decoder_state <= 3'b100;
                else decoder_state <= 3'b0;
            end
            3'b100: decoder_state <= 3'b0;
        endcase
    end

    always @(*) begin
        case (decoder_state)
            3'b0: begin
                fetchFlag = 1;
                memFlag = 1;
            end
            3'b1: begin
                opcode = instructionLine[31:27];
                cached_opcode = opcode;
                rd = instructionLine[26:22];
                rs = instructionLine[21:17];
                rt = instructionLine[16:12];
                literal = {52'b0, instructionLine[11:0]};

                case (opcode)
                    5'b11001: rs = rd;
                    5'b11011: rs = rd;
                    5'b00101: rs = rd;
                    5'b00111: rs = rd;
                    5'b10010: rs = rd;
                endcase

                regReadFlag = 1;
            end
            3'b10: aluEnable = 1;
            3'b11: memFlag = 1;
            3'b100: regWriteFlag = 1;
        endcase
    end
endmodule


module fetch (
    input clk,
    input fetchFlag,
    input reset,
    input [63:0] next_pc,
    output reg [63:0] pc
);
    reg [63:0] address_register;

    always @(*) begin
        if (fetchFlag) pc = address_register;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            address_register <= 64'h2000;
        end else if (fetchFlag) begin
            if (next_pc === 64'hx) begin
                address_register = 64'h2000;
            end
            else address_register <= next_pc;
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

module alu (
    input               aluEnable,
    input       [4:0]   control,
    input       [63:0]  input1,
    input       [63:0]  input2,
    input       [63:0]  rd,
    input       [63:0]  inputPc,
    input       [63:0]  r31,
    input               clk,
    output reg  [63:0]  result,
    output reg  [63:0]  writeData,
    output reg  [63:0]  rwAddress,
    output reg          aluDone, writeFlag, memRead,
    output reg  [63:0]  pc,
    output reg          hlt,
    output reg          mem_pc
);

    localparam OP_AND    = 5'h00;
    localparam OP_OR     = 5'h01;
    localparam OP_XOR    = 5'h02;
    localparam OP_NOT    = 5'h03;
    localparam OP_SHR    = 5'h04;
    localparam OP_SRA    = 5'h05;
    localparam OP_SHL    = 5'h06;
    localparam OP_SLA    = 5'h07;
    localparam OP_JMP    = 5'h08;
    localparam OP_JAL    = 5'h09;
    localparam OP_BR     = 5'h0A;
    localparam OP_BNZ    = 5'h0B;
    localparam OP_CALL   = 5'h0C;
    localparam OP_RET    = 5'h0D;
    localparam OP_BGT    = 5'h0E;
    localparam OP_HALT   = 5'h0F;
    localparam OP_LD     = 5'h10;
    localparam OP_MVHI   = 5'h11;
    localparam OP_MERGE  = 5'h12;
    localparam OP_ST     = 5'h13;
    localparam OP_FADD   = 5'h14;
    localparam OP_FSUB   = 5'h15;
    localparam OP_FMUL   = 5'h16;
    localparam OP_FDIV   = 5'h17;
    localparam OP_ADD    = 5'h18;
    localparam OP_ADDI   = 5'h19;
    localparam OP_SUB    = 5'h1A;
    localparam OP_SUBI   = 5'h1B;
    localparam OP_MUL    = 5'h1C;
    localparam OP_DIV    = 5'h1D;

    real float_operand1, float_operand2, float_output;
    assign float_operand1 = $bitstoreal(input1);
    assign float_operand2 = $bitstoreal(input2);

    always @(*) begin
        result      = 64'b0;
        writeData   = 64'b0;
        rwAddress   = 64'h2000;
        pc          = inputPc + 4;
        aluDone     = 1'b0;
        writeFlag   = 1'b0;
        memRead     = 1'b0;
        hlt         = 1'b0;
        mem_pc      = 1'b0;

        if (aluEnable) begin
            aluDone = 1'b1;

            case (control)
                OP_ADD:    result = input1 + input2;
                OP_ADDI:   result = input1 + input2;
                OP_SUB:    result = input1 - input2;
                OP_SUBI:   result = input1 - input2;
                OP_MUL:    result = input1 * input2;
                OP_DIV:    result = input1 / input2;
                OP_AND:    result = input1 & input2;
                OP_OR:     result = input1 | input2;
                OP_XOR:    result = input1 ^ input2;
                OP_NOT:    result = ~input1;
                OP_SHR:    result = input1 >> input2;
                OP_SRA:    result = input1 >> input2; // Original used logical shift here
                OP_SHL:    result = input1 << input2;
                OP_SLA:    result = input1 << input2; // Arithmetic Left Shift is same as Logical

                OP_LD: begin
                    rwAddress = input1 + input2;
                    memRead = 1'b1;
                end

                OP_MVHI:   result = input1;
                OP_MERGE:  result = {input1[63:12], input2[11:0]};

                OP_ST: begin
                    rwAddress = rd + input2;
                    writeData = input1;
                    writeFlag = 1'b1;
                end

                OP_FADD: begin
                    float_output = float_operand1 + float_operand2;
                    result = $realtobits(float_output);
                end
                OP_FSUB: begin
                    float_output = float_operand1 - float_operand2;
                    result = $realtobits(float_output);
                end
                OP_FMUL: begin
                    float_output = float_operand1 * float_operand2;
                    result = $realtobits(float_output);
                end
                OP_FDIV: begin
                    float_output = float_operand1 / float_operand2;
                    result = $realtobits(float_output);
                end

                OP_JMP: begin
                    pc = rd;
                end
                OP_JAL: begin
                    pc = inputPc + rd;
                end
                OP_BR: begin
                    pc = inputPc + $signed(input2);
                end
                OP_BNZ: begin
                    pc = (input1 != 64'b0) ? rd : inputPc + 4;
                end
                OP_CALL: begin
                    pc = rd;
                    writeData = inputPc + 4;
                    rwAddress = r31 - 8;
                    writeFlag = 1'b1;
                end
                OP_RET: begin
                    rwAddress = r31 - 8;
                    memRead = 1'b1;
                    mem_pc = 1'b1;
                end
                OP_BGT: begin
                    pc = (input1 > input2) ? rd : inputPc + 4;
                end
                OP_HALT: begin
                    hlt = 1'b1;
                end
                default: begin
                    // Keep default assignments
                end
            endcase
        end
    end

endmodule