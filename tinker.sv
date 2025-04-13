module tinker_core (
    input clk,
    input reset,
    output hlt
);
    
    logic [4:0] rd, rs, rt, opcode;
    logic [31:0] instructionLine;
    logic [63:0] next_pc, pc, alu_pc;
    logic [63:0] lit, rd_val, rs_val, rt_val, input2, aluOut, stackPtr, writeData, rwAddress, readData, regWriteData;
    logic writeFlag;
    logic mem_pc;
    logic aluEnable, regReadEnable, regWriteEnable, fetchEnable, memRead, memEnable, aluReady;

    instructionDecoder instructionDecoder(
        .instructionLine(instructionLine),
        .clk(clk),
        .rst(reset),
        .aluReady(aluReady),
        .memEnable(memEnable),
        .literal(lit),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .opcode(opcode),
        .aluEnable(aluEnable),
        .fetchEnable(fetchEnable),
        .regReadEnable(regReadEnable),
        .regWriteEnable(regWriteEnable)
    );

    fetch fetch(
        .clk(clk),
        .fetchEnable(fetchEnable),
        .reset(reset),
        .next_pc(next_pc),
        .pc(pc)
    );

    alu alu(
        .aluEnable(aluEnable),
        .control(opcode),
        .input1(rs_val),
        .input2(input2),
        .inputPc(pc),
        .r31(stackPtr),
        .rd(rd_val),
        .clk(clk),
        .result(aluOut),
        .pc(alu_pc),
        .writeFlag(writeFlag),
        .memRead(memRead),
        .aluReady(aluReady),
        .writeData(writeData),
        .rwAddress(rwAddress),
        .hlt(hlt),
        .mem_pc(mem_pc)
    );

    reglitmux reglitmux(
        .sel(opcode),
        .reg1(rt_val),
        .lit(lit),
        .out(input2)
    );

    registerFile reg_file(
        .data(regWriteData),
        .read1(rs),
        .read2(rt),
        .write(rd),
        .reset(reset),
        .clk(clk),
        .regReadEnable(regReadEnable),
        .regWriteEnable(regWriteEnable),
        .output1(rs_val),
        .output2(rt_val),
        .output3(rd_val),
        .stackPtr(stackPtr)
    );

    aluMemMux aluMemMux(
        .mem_pc(mem_pc),
        .memData(readData),
        .aluOut(alu_pc),
        .newPc(next_pc)
    );

    memory memory(
        .pc(pc),
        .clk(clk),
        .reset(reset),
        .writeFlag(writeFlag),
        .fetchEnable(fetchEnable),
        .memEnable(memEnable),
        .memRead(memRead),
        .writeData(writeData),
        .rwAddress(rwAddress),
        .readData(readData),
        .instruction(instructionLine)
    );

    memRegMux memRegMux(
        .opcode(opcode),
        .readData(readData),
        .aluResult(aluOut),
        .regWriteData(regWriteData)
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
    reg [2:0] state;
    reg [4:0] opcodeReg;

    always @(posedge rst) begin
        if (rst) state <= 3'b0;
    end

    always @(posedge clk) begin
        aluEnable <= 0;
        regWriteEnable <= 0;
        regReadEnable <= 0;
        memEnable <= 0;

        case (state)
            3'b0: state <= 3'b1;
            3'b1: state <= 3'b10;
            3'b10: begin
                if (!aluReady) state <= 3'b10;
                else if (opcodeReg == 5'h10 || opcodeReg == 5'h13 || opcodeReg == 5'h0C || opcodeReg == 5'h0D) begin
                    state <= 3'b11;
                end
                else if (opcodeReg == 5'h08 || opcodeReg == 5'h09 || opcodeReg == 5'h0A || opcodeReg == 5'h0B || opcodeReg == 5'h0E) begin
                    state <= 3'b0;
                end
                else state <= 3'b100;
            end
            3'b11: begin
                if (opcodeReg == 5'h10) state <= 3'b100;
                else state <= 3'b0;
            end
            3'b100: state <= 3'b0;
        endcase
    end

    always @(*) begin
        case (state)
            3'b0: begin
                fetchEnable = 1;
                memEnable = 1;
            end
            3'b1: begin
                opcode = instructionLine[31:27];
                opcodeReg = opcode;
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
    reg [63:0] current_programCounter;

    always @(*) begin
        if (fetchEnable) pc = current_programCounter;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_programCounter <= 64'h2000;
        end else if (fetchEnable) begin
            if (next_pc === 64'hx) begin
                current_programCounter = 64'h2000;
            end
            else current_programCounter <= next_pc;
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
    real r1, r2, rres;
    assign r1 = $bitstoreal(input1);
    assign r2 = $bitstoreal(input2);

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
                    rres = r1 + r2;
                    result = $realtobits(rres);
                end
                5'h15: begin
                    rres = r1 - r2;
                    result = $realtobits(rres);
                end
                5'h16: begin
                    rres = r1 * r2;
                    result = $realtobits(rres);
                end
                5'h17: begin
                    rres = r1 / r2;
                    result = $realtobits(rres);
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