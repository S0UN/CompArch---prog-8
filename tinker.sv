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
    // Pipeline stage state definitions
    localparam FETCH      = 3'b000;  // Fetch instruction
    localparam DECODE     = 3'b001;  // Decode instruction and read registers
    localparam EXECUTE    = 3'b010;  // Execute operation in ALU
    localparam MEMORY     = 3'b011;  // Memory access stage
    localparam WRITEBACK  = 3'b100;  // Write results back to registers
    
    // Instruction opcodes from Tinker ISA
    localparam MOV_MEM    = 5'h10;   // mov rd, (rs)(L)
    localparam MOV_STR    = 5'h13;   // mov (rd)(L), rs
    localparam CALL       = 5'h0C;   // call rd, rs, rt
    localparam RETURN     = 5'h0D;   // return
    localparam BR         = 5'h08;   // br rd
    localparam BRR        = 5'h09;   // brr rd
    localparam BRRI       = 5'h0A;   // brr L
    localparam BRNZ       = 5'h0B;   // brnz rd, rs
    localparam BRGT       = 5'h0E;   // brgt rd, rs, rt
    
    // Internal state register
    reg [2:0] pipeline_stage;
    reg [4:0] cached_opcode;

    // Reset logic
    always @(posedge rst) begin
        if (rst) pipeline_stage <= FETCH;
    end

    // State transition logic
    always @(posedge clk) begin
        // Default: disable all control signals at start of clock cycle
        aluEnable <= 0;
        regWriteFlag <= 0;
        regReadFlag <= 0;
        memFlag <= 0;

        case (pipeline_stage)
            FETCH: begin
                // After fetch, move to decode
                pipeline_stage <= DECODE;
            end
            
            DECODE: begin
                // After decode, move to execute
                pipeline_stage <= EXECUTE;
            end
            
            EXECUTE: begin
                if (!aluDone) begin
                    // Stay in execute until ALU finishes
                    pipeline_stage <= EXECUTE;
                end
                else if (cached_opcode == MOV_MEM || cached_opcode == MOV_STR || 
                         cached_opcode == CALL || cached_opcode == RETURN) begin
                    // Memory operations need memory access stage
                    pipeline_stage <= MEMORY;
                end
                else if (cached_opcode == BR || cached_opcode == BRR || 
                         cached_opcode == BRRI || cached_opcode == BRNZ || 
                         cached_opcode == BRGT) begin
                    // Branch operations skip to fetch
                    pipeline_stage <= FETCH;
                end
                else begin
                    // Other operations go to writeback
                    pipeline_stage <= WRITEBACK;
                end
            end
            
            MEMORY: begin
                if (cached_opcode == MOV_MEM) begin
                    // Load operations need writeback
                    pipeline_stage <= WRITEBACK;
                end
                else begin
                    // Store operations go back to fetch
                    pipeline_stage <= FETCH;
                end
            end
            
            WRITEBACK: begin
                // After writeback, go back to fetch
                pipeline_stage <= FETCH;
            end
            
            default: pipeline_stage <= FETCH;
        endcase
    end

    // Output and control signal generation
    always @(*) begin
        case (pipeline_stage)
            FETCH: begin
                // Fetch stage: activate fetch signals
                fetchFlag = 1;
                memFlag = 1;
            end
            
            DECODE: begin
                // Decode stage: extract instruction fields
                opcode = instructionLine[31:27];
                cached_opcode = opcode;
                rd = instructionLine[26:22];
                rs = instructionLine[21:17];
                rt = instructionLine[16:12];
                literal = {52'b0, instructionLine[11:0]};

                // Handle special register addressing modes for certain instructions
                case (opcode)
                    5'b11001: rs = rd;  // addi rd, L
                    5'b11011: rs = rd;  // subi rd, L
                    5'b00101: rs = rd;  // shftri rd, L
                    5'b00111: rs = rd;  // shftli rd, L
                    5'b10010: rs = rd;  // mov rd, L
                endcase

                // Activate register read
                regReadFlag = 1;
            end
            
            EXECUTE: begin
                // Execute stage: activate ALU
                aluEnable = 1;
            end
            
            MEMORY: begin
                // Memory stage: activate memory access
                memFlag = 1;
            end
            
            WRITEBACK: begin
                // Writeback stage: write to register file
                regWriteFlag = 1;
            end
            
            default: begin
                fetchFlag = 0;
                memFlag = 0;
                aluEnable = 0;
                regReadFlag = 0;
                regWriteFlag = 0;
            end
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
    // Constants
    localparam INITIAL_PC = 64'h2000;  // Initial program counter value
    
    // Internal registers
    reg [63:0] address_register;

    // Combinational logic for PC output
    always @(*) begin
        if (fetchFlag) 
            pc = address_register;
    end

    // Sequential logic for address register update
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset to initial PC value
            address_register <= INITIAL_PC;
        end 
        else if (fetchFlag) begin
            // Handle undefined next_pc by loading initial value
            if (next_pc === 64'hx) begin
                address_register <= INITIAL_PC;
            end
            else begin
                // Update address register with next PC value
                address_register <= next_pc;
            end
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
    input aluEnable,
    input [4:0] control,
    input [63:0] input1,
    input [63:0] input2,
    input [63:0] rd,
    input [63:0] inputPc,
    input [63:0] r31,
    input clk,
    output reg [63:0] result,
    output reg [63:0] writeData,
    output reg [63:0] rwAddress,
    output reg aluDone, writeFlag, memRead,
    output reg [63:0] pc,
    output reg hlt,
    output reg mem_pc
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
        if (aluEnable) begin
            // Initialize all outputs to default values
            result = 64'b0;
            pc = inputPc + 4;  // Default: increment PC by 4
            writeData = 64'b0;
            rwAddress = MEM_BASE_ADDR;
            writeFlag = 1'b0;
            memRead = 1'b0;
            mem_pc = 1'b0;
            hlt = 1'b0;
            aluDone = 1'b1;
            
            // Execute operation based on control signal
            case (control)
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
                MOV_MEM: begin
                    rwAddress = input1 + input2;
                    memRead = 1'b1;
                end
                MOV_REG: result = input1;
                MOV_LIT: result = {input1[63:12], input2[11:0]};
                MOV_STR: begin
                    rwAddress = rd + input2;
                    writeData = input1;
                    writeFlag = 1'b1;
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
                BR: pc = rd;
                BRR: pc = inputPc + rd;
                BRRI: pc = inputPc + $signed(input2);
                BRNZ: pc = (input1 != 0) ? rd : inputPc + 4;
                BRGT: pc = (input1 > input2) ? rd : inputPc + 4;
                
                // Subroutine Instructions
                CALL: begin
                    pc = rd;
                    writeData = inputPc + 4;  // Save return address
                    rwAddress = r31 - 8;
                    writeFlag = 1'b1;
                end
                RETURN: begin
                    rwAddress = r31 - 8;
                    memRead = 1'b1;
                    mem_pc = 1'b1;
                end
                
                // Privileged Instructions
                PRIV: hlt = 1'b1;  // Assuming PRIV is used as HALT here
                
                default: begin
                    // Default case (should not happen but added for safety)
                    result = 64'b0;
                end
            endcase
        end
        else begin
            // ALU not enabled
            hlt = 1'b0;
            aluDone = 1'b0;
            // Other outputs are undefined when ALU is not enabled
        end
    end
endmodule