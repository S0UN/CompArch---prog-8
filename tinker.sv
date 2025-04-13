module tinker_core (
    input clk,
    input reset,
    output hlt
);
    
    logic [4:0] dest_reg, src_reg1, src_reg2, operation;
    logic [31:0] instr;
    logic [63:0] next_program_counter, program_counter, alu_program_counter;
    logic [63:0] immediate, dest_val, src1_val, src2_val, alu_in2, alu_result, stack_pointer, data_to_write, rw_addr, data_from_mem, register_write_data;
    logic write_enable;
    logic mem_program_counter;
    logic alu_activate, reg_read_activate, reg_write_activate, fetch_activate, mem_read, mem_activate, alu_done;

    instructionDecoder instr_decoder(
        .instructionLine(instr),
        .clk(clk),
        .rst(reset),
        .aluReady(alu_done),
        .memEnable(mem_activate),
        .literal(immediate),
        .rd(dest_reg),
        .rs(src_reg1),
        .rt(src_reg2),
        .opcode(operation),
        .aluEnable(alu_activate),
        .fetchEnable(fetch_activate),
        .regReadEnable(reg_read_activate),
        .regWriteEnable(reg_write_activate)
    );

    fetch fetch_unit(
        .clk(clk),
        .fetchEnable(fetch_activate),
        .reset(reset),
        .next_pc(next_program_counter),
        .pc(program_counter)
    );

    alu arithmetic_unit(
        .aluEnable(alu_activate),
        .control(operation),
        .input1(src1_val),
        .input2(alu_in2),
        .inputPc(program_counter),
        .r31(stack_pointer),
        .rd(dest_val),
        .clk(clk),
        .result(alu_result),
        .pc(alu_program_counter),
        .writeFlag(write_enable),
        .memRead(mem_read),
        .aluReady(alu_done),
        .writeData(data_to_write),
        .rwAddress(rw_addr),
        .hlt(hlt),
        .mem_pc(mem_program_counter)
    );

    reglitmux reg_lit_mux(
        .sel(operation),
        .reg1(src2_val),
        .lit(immediate),
        .out(alu_in2)
    );

    registerFile reg_file(
        .data(register_write_data),
        .read1(src_reg1),
        .read2(src_reg2),
        .write(dest_reg),
        .reset(reset),
        .clk(clk),
        .regReadEnable(reg_read_activate),
        .regWriteEnable(reg_write_activate),
        .output1(src1_val),
        .output2(src2_val),
        .output3(dest_val),
        .stackPtr(stack_pointer)
    );

    aluMemMux alu_mem_mux(
        .mem_pc(mem_program_counter),
        .memData(data_from_mem),
        .aluOut(alu_program_counter),
        .newPc(next_program_counter)
    );

    memory memory(
        .pc(program_counter),
        .clk(clk),
        .reset(reset),
        .writeFlag(write_enable),
        .fetchEnable(fetch_activate),
        .memEnable(mem_activate),
        .memRead(mem_read),
        .writeData(data_to_write),
        .rwAddress(rw_addr),
        .readData(data_from_mem),
        .instruction(instr)
    );

    memRegMux mem_reg_mux(
        .opcode(operation),
        .readData(data_from_mem),
        .aluResult(alu_result),
        .regWriteData(register_write_data)
    );

endmodule

module reglitmux (
    input [4:0] sel,
    input [63:0] reg1,
    input [63:0] lit,
    output reg [63:0] out
);
    always @(*) begin
        case (sel)
            5'b11001: out = lit;
            5'b11011: out = lit;
            5'b00101: out = lit;
            5'b00111: out = lit;
            5'b10010: out = lit;
            5'b01010: out = lit;
            5'h10: out = lit;
            5'h13: out = lit;
            default: out = reg1;
        endcase
    end
endmodule

module registerFile (
    input [63:0] data,
    input [4:0] read1,
    input [4:0] read2,
    input [4:0] write,
    input reset,
    input clk,
    input regReadEnable,
    input regWriteEnable,
    output reg [63:0] output1,
    output reg [63:0] output2,
    output reg [63:0] output3,
    output reg [63:0] stackPtr
);
    reg [63:0] registers [0:31];
    integer i;

    initial begin
        for (i = 0; i < 31; i = i + 1) begin
            registers[i] <= 64'b0;
        end
        registers[31] <= 64'd524288;
    end

    assign stackPtr = registers[31];
    always @(*) begin
        if (regReadEnable) begin
            output1 = registers[read1];
            output2 = registers[read2];
            output3 = registers[write];
        end

        if (regWriteEnable) begin
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
        case (opcode)
            5'h10: regWriteData = readData;
            default: regWriteData = aluResult;
        endcase
    end
endmodule

module memory (
    input [63:0] pc,
    input clk,
    input reset,
    input writeFlag,
    input fetchEnable,
    input memEnable,
    input memRead,
    input [63:0] writeData,
    input [63:0] rwAddress,
    output reg [63:0] readData,
    output reg [31:0] instruction
);
    reg [7:0] bytes [0:524287];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 524288; i = i + 1) begin
                bytes[i] <= 8'b0;
            end
        end
    end

    assign instruction[7:0] = bytes[pc];
    assign instruction[15:8] = bytes[pc+1];
    assign instruction[23:16] = bytes[pc+2];
    assign instruction[31:24] = bytes[pc+3];

    always @(memEnable) begin
        if (memRead) begin
            readData[7:0] = bytes[rwAddress];
            readData[15:8] = bytes[rwAddress+1];
            readData[23:16] = bytes[rwAddress+2];
            readData[31:24] = bytes[rwAddress+3];
            readData[39:32] = bytes[rwAddress+4];
            readData[47:40] = bytes[rwAddress+5];
            readData[55:48] = bytes[rwAddress+6];
            readData[63:56] = bytes[rwAddress+7];
        end
        if (writeFlag) begin
            bytes[rwAddress] = writeData[7:0];
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
    input aluReady,
    output reg fetchEnable,
    output reg memEnable,
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg aluEnable,
    output reg regReadEnable,
    output reg regWriteEnable
);
    reg [2:0] curr_state;
    reg [4:0] op_cache;

    always @(posedge rst) begin
        if (rst) curr_state <= 3'b0;
    end

    always @(posedge clk) begin
        aluEnable <= 0;
        regWriteEnable <= 0;
        regReadEnable <= 0;
        memEnable <= 0;

        case (curr_state)
            3'b0: curr_state <= 3'b1;
            3'b1: curr_state <= 3'b10;
            3'b10: begin
                if (!aluReady) curr_state <= 3'b10;
                else if (op_cache == 5'h10 || op_cache == 5'h13 || op_cache == 5'h0C || op_cache == 5'h0D) begin
                    curr_state <= 3'b11;
                end
                else if (op_cache == 5'h08 || op_cache == 5'h09 || op_cache == 5'h0A || op_cache == 5'h0B || op_cache == 5'h0E) begin
                    curr_state <= 3'b0;
                end
                else curr_state <= 3'b100;
            end
            3'b11: begin
                if (op_cache == 5'h10) curr_state <= 3'b100;
                else curr_state <= 3'b0;
            end
            3'b100: curr_state <= 3'b0;
        endcase
    end

    always @(*) begin
        case (curr_state)
            3'b0: begin
                fetchEnable = 1;
                memEnable = 1;
            end
            3'b1: begin
                opcode = instructionLine[31:27];
                op_cache = opcode;
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

                regReadEnable = 1;
            end
            3'b10: aluEnable = 1;
            3'b11: memEnable = 1;
            3'b100: regWriteEnable = 1;
        endcase
    end
endmodule

module fetch (
    input clk,
    input fetchEnable,
    input reset,
    input [63:0] next_pc,
    output reg [63:0] pc
);
    reg [63:0] pc_internal;

    always @(*) begin
        if (fetchEnable) pc = pc_internal;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_internal <= 64'h2000;
        end else if (fetchEnable) begin
            if (next_pc === 64'hx) begin
                pc_internal = 64'h2000;
            end
            else pc_internal <= next_pc;
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
    output reg aluReady, writeFlag, memRead,
    output reg [63:0] pc,
    output reg hlt,
    output reg mem_pc
);
    real float_in1, float_in2, float_result;
    assign float_in1 = $bitstoreal(input1);
    assign float_in2 = $bitstoreal(input2);

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
                    float_result = float_in1 + float_in2;
                    result = $realtobits(float_result);
                end
                5'h15: begin
                    float_result = float_in1 - float_in2;
                    result = $realtobits(float_result);
                end
                5'h16: begin
                    float_result = float_in1 * float_in2;
                    result = $realtobits(float_result);
                end
                5'h17: begin
                    float_result = float_in1 / float_in2;
                    result = $realtobits(float_result);
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
            aluReady = 1;
        end
        else begin
            hlt = 0;
            aluReady = 0;
        end
    end
endmodule