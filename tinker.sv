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
    // Enumerated type for pipeline stages
    typedef enum reg [2:0] {
        FETCH     = 3'b000,
        DECODE    = 3'b001,
        EXECUTE   = 3'b010,
        STORE     = 3'b011,
        WRITEBACK = 3'b100
    } pipeline_stage_t;

    pipeline_stage_t current_stage;
    reg [4:0] stored_opcode;

    // Reset logic
    always @(posedge rst) begin
        if (rst) begin
            current_stage <= FETCH;
        end
    end

    // State transition logic
    always @(posedge clk) begin
        // Default flag resets
        aluEnable    <= 1'b0;
        regWriteFlag <= 1'b0;
        regReadFlag  <= 1'b0;
        memFlag      <= 1'b0;

        case (current_stage)
            FETCH: begin
                current_stage <= DECODE;
            end
            DECODE: begin
                current_stage <= EXECUTE;
            end
            EXECUTE: begin
                if (!aluDone) begin
                    current_stage <= EXECUTE; // Stay in EXECUTE since we are not ready to move on to next stage
                end
                else if (stored_opcode == 5'h10 || stored_opcode == 5'h13 ||
                         stored_opcode == 5'h0C || stored_opcode == 5'h0D) begin
                    current_stage <= STORE;
                end
                else if (stored_opcode == 5'h08 || stored_opcode == 5'h09 ||
                         stored_opcode == 5'h0A || stored_opcode == 5'h0B ||
                         stored_opcode == 5'h0E) begin
                    current_stage <= FETCH;
                end
                else begin
                    current_stage <= WRITEBACK;
                end
            end
            STORE: begin
                if (stored_opcode == 5'h10) begin
                    current_stage <= WRITEBACK;
                end
                else begin
                    current_stage <= FETCH;
                end
            end
            WRITEBACK: begin
                current_stage <= FETCH;
            end
            default: begin
                current_stage <= FETCH; // Fallback for safety
            end
        endcase
    end

    // Output control logic
    always @(*) begin
        // Default values
        fetchFlag = 1'b0;
        memFlag = 1'b0;
        opcode   = 5'b0;
        rd  = 5'b0;
        rs = 5'b0;
        rt = 5'b0;
        literal  = 64'b0;
        aluEnable= 1'b0;
        regReadFlag  = 1'b0;
        regWriteFlag = 1'b0;

        case (current_stage)
            FETCH: begin
                fetchFlag = 1'b1;
                memFlag   = 1'b1;
            end
            DECODE: begin
                // Instruction decoding
                opcode = instructionLine[31:27];
                stored_opcode = opcode;
                rd = instructionLine[26:22];
                rs = instructionLine[21:17];
                rt  = instructionLine[16:12];
                literal ={{52{1'b0}}, instructionLine[11:0]};

                // Adjust rs for specific opcodes
                if (opcode == 5'b11001 || opcode == 5'b11011 ||
                    opcode == 5'b00101 || opcode == 5'b00111 ||
                    opcode == 5'b10010) begin
                    rs = rd;
                end

                regReadFlag = 1'b1;
            end
            EXECUTE: begin
                aluEnable = 1'b1;
            end
            STORE: begin
                memFlag = 1'b1;
            end
            WRITEBACK: begin
                regWriteFlag = 1'b1;
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
    // Enumerated type for fetch states
    typedef enum reg [1:0] {
        IDLE       = 2'b00,
        FETCHING   = 2'b01,
        UPDATING   = 2'b10,
        RESETTING  = 2'b11
    } fetch_state_t;

    fetch_state_t current_state;
    reg [63:0] pc_storage;
    reg [63:0] shadow_register; 
    reg [7:0] fetch_counter;   
    reg fetch_active;           

    // Initialize state and registers
    initial begin
        current_state    = IDLE;
        pc_storage       = 64'h0;
        shadow_register  = 64'h0;
        fetch_counter    = 8'b0;
        fetch_active     = 1'b0;
    end

    // Reset and state transition logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state   <= RESETTING;
            pc_storage      <= 64'h2000;
            shadow_register <= 64'h2000; 
            fetch_counter   <= 8'b0;
            fetch_active    <= 1'b0;
        end
        else begin
            case (current_state)
                IDLE: begin
                    if (fetchFlag) begin
                        current_state <= FETCHING;
                        fetch_active  <= 1'b1;
                    end
                end
                FETCHING: begin
                    current_state <= UPDATING;
                    fetch_counter <= fetch_counter + 1; 
                end
                UPDATING: begin
                    if (fetchFlag) begin
                        if (next_pc === 64'hx) begin
                            pc_storage      <= 64'h2000;
                            shadow_register <= 64'h2000; // Redundant
                        end
                        else begin
                            pc_storage      <= next_pc;
                            shadow_register <= next_pc;  // Redundant
                        end
                    end
                    current_state <= IDLE;
                    fetch_active  <= 1'b0;
                end
                RESETTING: begin
                    current_state <= IDLE;
                end
                default: begin
                    current_state <= IDLE;
                end
            endcase
        end
    end

    // Output logic
    always @(*) begin
        pc = 64'h0; // Default value
        if (fetch_active && current_state == FETCHING) begin
            pc = pc_storage;
        end
        // Trivial redundant check
        if (shadow_register != pc_storage) begin
            shadow_register = pc_storage;
        end
    end

endmodule

module aluMemMux (
    input mem_pc,
    input [63:0] memData,
    input [63:0] aluOut,
    output reg [63:0] newPc
);
    assign newPc = mem_pc ? memData : aluOut;
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
    real float_operand1, float_operand2, float_output;
    assign float_operand1 = $bitstoreal(input1);
    assign float_operand2 = $bitstoreal(input2);

    always @(*) begin
        if (aluEnable) begin
            hlt = 0;
            pc = inputPc + 4;
            writeData = 0;
            rwAddress = 64'h2000;
            writeFlag = 0;
            memRead = 0;
            mem_pc = 0;
            case (control)
                5'h18: result = input1 + input2;
                5'h19: result = input1 + input2;
                5'h1A: result = input1 - input2;
                5'h1B: result = input1 - input2;
                5'h1C: result = input1 * input2;
                5'h1D: result = input1 / input2;
                5'h00: result = input1 & input2;
                5'h01: result = input1 | input2;
                5'h02: result = input1 ^ input2;
                5'h03: result = ~input1;
                5'h04: result = input1 >> input2;
                5'h05: result = input1 >> input2;
                5'h06: result = input1 << input2;
                5'h07: result = input1 << input2;
                5'h10: begin
                    result = 64'b0;
                    rwAddress = input1 + input2;
                    memRead = 1;
                end
                5'h11: result = input1;
                5'h12: result = {input1[63:12], input2[11:0]};
                5'h13: begin
                    result = 64'b0;
                    rwAddress = rd + input2;
                    writeData = input1;
                    writeFlag = 1;
                end
                5'h14: begin
                    float_output = float_operand1 + float_operand2;
                    result = $realtobits(float_output);
                end
                5'h15: begin
                    float_output = float_operand1 - float_operand2;
                    result = $realtobits(float_output);
                end
                5'h16: begin
                    float_output = float_operand1 * float_operand2;
                    result = $realtobits(float_output);
                end
                5'h17: begin
                    float_output = float_operand1 / float_operand2;
                    result = $realtobits(float_output);
                end
                5'h08: begin
                    pc = rd;
                    result = 64'b0;
                end
                5'h09: begin
                    pc = inputPc + rd;
                    result = 64'b0;
                end
                5'h0A: begin
                    pc = inputPc + $signed(input2);
                    result = 64'b0;
                end
                5'h0B: begin
                    pc = (input1 != 0) ? rd : inputPc + 4;
                    result = 64'b0;
                end
                5'h0C: begin
                    pc = rd;
                    result = 64'b0;
                    writeData = inputPc + 4;
                    rwAddress = r31 - 8;
                    writeFlag = 1;
                end
                5'h0D: begin
                    result = 64'b0;
                    rwAddress = r31 - 8;
                    memRead = 1;
                    mem_pc = 1;
                end
                5'h0E: begin
                    pc = (input1 > input2) ? rd : inputPc + 4;
                    result = 64'b0;
                end
                5'h0F: begin
                    result = 64'b0;
                    hlt = 1;
                end
            endcase
            aluDone = 1;
        end
        else begin
            hlt = 0;
            aluDone = 0;
        end
    end
endmodule