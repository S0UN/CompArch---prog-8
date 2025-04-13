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
    logic mem_pc; // choose pc from mem
    logic aluEnable, regReadEnable, regWriteEnable, fetchEnable, memRead, memEnable, aluReady;

    instructionDecoder id(
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

    fetch fetch_unit(
        .clk(clk),
        .fetchEnable(fetchEnable),
        .reset(reset),
        .next_pc(next_pc),
        .pc(pc)
    );

    alu alu_unit(
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

    regLitMux reg_lit_mux(
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

    aluMemMux alu_mem_mux(
        .mem_pc(mem_pc),
        .memData(readData),
        .aluOut(alu_pc),
        .newPc(next_pc)
    );

    memory mem_unit(
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

    memRegMux mem_reg_mux(
        .opcode(opcode),
        .readData(readData),
        .aluResult(aluOut),
        .regWriteData(regWriteData)
    );

endmodule

module instructionDecoder(
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

    typedef enum logic [2:0] {
        FETCH = 3'b000,
        DECODE = 3'b001,
        EXECUTE = 3'b010,
        MEMORY_ACCESS = 3'b011,
        WRITE_BACK = 3'b100
    } decode_state_t;

    decode_state_t state;
    reg [4:0] opcode_reg;

    // Reset and state transitions
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= FETCH;
            aluEnable <= 0;
            regWriteEnable <= 0;
            regReadEnable <= 0;
            memEnable <= 0;
            fetchEnable <= 0;
        end else begin
            // Clear control signals at the start of each cycle
            aluEnable <= 0;
            regWriteEnable <= 0;
            regReadEnable <= 0;
            memEnable <= 0;
            fetchEnable <= 0;

            case (state)
                FETCH: begin
                    state <= DECODE;
                end
                DECODE: begin
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    if (!aluReady) begin
                        state <= EXECUTE;
                    end else if (opcode_reg == 5'h10 || opcode_reg == 5'h13 || opcode_reg == 5'h0C || opcode_reg == 5'h0D) begin
                        state <= MEMORY_ACCESS;
                    end else if (opcode_reg == 5'h08 || opcode_reg == 5'h09 || opcode_reg == 5'h0A || opcode_reg == 5'h0B || opcode_reg == 5'h0E) begin
                        state <= FETCH;
                    end else begin
                        state <= WRITE_BACK;
                    end
                end
                MEMORY_ACCESS: begin
                    if (opcode_reg == 5'h10) begin
                        state <= WRITE_BACK;
                    end else begin
                        state <= FETCH;
                    end
                end
                WRITE_BACK: begin
                    state <= FETCH;
                end
                default: begin
                    state <= FETCH;
                end
            endcase
        end
    end

    // Control signals and instruction decoding
    always @(*) begin
        case (state)
            FETCH: begin
                fetchEnable = 1;
                memEnable = 1;
            end
            DECODE: begin
                opcode = instructionLine[31:27];
                opcode_reg = instructionLine[31:27];
                rd = instructionLine[26:22];
                rs = instructionLine[21:17];
                rt = instructionLine[16:12];
                literal = {52'b0, instructionLine[11:0]};
                case (instructionLine[31:27])
                    5'b11001, 5'b11011, 5'b00101, 5'b00111, 5'b10010: begin
                        rs = instructionLine[26:22];
                    end
                endcase
                regReadEnable = 1;
            end
            EXECUTE: begin
                aluEnable = 1;
            end
            MEMORY_ACCESS: begin
                memEnable = 1;
            end
            WRITE_BACK: begin
                regWriteEnable = 1;
            end
            default: begin
                fetchEnable = 0;
                memEnable = 0;
                regReadEnable = 0;
                aluEnable = 0;
                regWriteEnable = 0;
            end
        endcase
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
    output reg aluReady,
    output reg writeFlag,
    output reg memRead,
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
        end else begin
            hlt = 0;
            aluReady = 0;
        end
    end
endmodule

module regLitMux (
    input [4:0] sel,
    input [63:0] reg1,
    input [63:0] lit,
    output reg [63:0] out
);
    always @(*) begin
        if (sel == 5'b11001 || sel == 5'b11011 || sel == 5'b00101 || sel == 5'b00111 || sel == 5'b10010 || sel == 5'b01010 || sel == 5'h10 || sel == 5'h13) begin
            out = lit;
        end else begin
            out = reg1;
        end
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

    // Synchronous writes
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) begin
                registers[i] <= 64'b0;
            end
            registers[31] <= 64'd524288;
        end else if (regWriteEnable) begin
            registers[write] <= data;
        end
    end

    // Combinational reads
    always @(*) begin
        if (regReadEnable) begin
            output1 = registers[read1];
            output2 = registers[read2];
            output3 = registers[write];
        end else begin
            output1 = 64'b0;
            output2 = 64'b0;
            output3 = 64'b0;
        end
        stackPtr = registers[31];
    end

endmodule

module memRegMux(
    input [4:0] opcode,
    input [63:0] readData,
    input [63:0] aluResult,
    output reg [63:0] regWriteData
);
    always @(*) begin
        if (opcode == 5'h10) begin
            regWriteData = readData;
        end else begin
            regWriteData = aluResult;
        end
    end
endmodule

module memory(
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

module fetch(
    input clk,
    input fetchEnable,
    input reset,
    input [63:0] next_pc,
    output reg [63:0] pc
);

    reg [63:0] current_pc;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_pc <= 64'h2000;
        end else if (fetchEnable) begin
            if (next_pc === 64'hx) begin
                current_pc <= 64'h2000;
            end else begin
                current_pc <= next_pc;
            end
        end
    end

    always @(*) begin
        if (fetchEnable) begin
            pc = current_pc;
        end else begin
            pc = 64'h2000;
        end
    end

endmodule

module aluMemMux(
    input mem_pc,
    input [63:0] memData,
    input [63:0] aluOut,
    output reg [63:0] newPc
);
    assign newPc = mem_pc ? memData : aluOut;
endmodule