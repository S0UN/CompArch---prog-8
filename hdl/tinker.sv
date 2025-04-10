module tinker_core (
    input wire clock,
    input wire rst
);
    wire [63:0] programCounter, nextProgramCounter;
    wire [31:0] currentInstruction;
    wire [4:0] destReg, sourceReg1, sourceReg2, operationCode;
    wire [63:0] immediateValue, regDestVal, regSrc1Val, regSrc2Val;
    wire [63:0] aluOutput, memReadData, memWriteValue, memAccessAddr;
    wire aluWriteEnable, memWriteEnable, memToRegWrite;
    wire [63:0] secondOperand;

    // Fetch Unit (matched to friend’s fetch)
    module pcUpdater (
        input wire clk,
        input wire reset,
        input wire [63:0] r31,        // Unused but kept for compatibility
        input wire [63:0] next_pc,
        output reg [63:0] pc
    );
        reg [63:0] current_programCounter;
        assign pc = current_programCounter;
        always @(posedge clk or posedge reset) begin
            if (reset)
                current_programCounter <= 64'h2000;
            else
                current_programCounter <= next_pc;
        end
    endmodule

    // Memory Module (matched to friend’s memory)
    module storageUnit (
        input wire [63:0] pc,
        input wire clk,
        input wire reset,
        input wire flag,
        input wire [63:0] writeData,
        input wire [63:0] rwAddress,
        output reg [63:0] readData,
        output reg [31:0] instruction
    );
        reg [7:0] bytes [0:524287];
        integer i;
        assign instruction[7:0] = bytes[pc];
        assign instruction[15:8] = bytes[pc+1];
        assign instruction[23:16] = bytes[pc+2];
        assign instruction[31:24] = bytes[pc+3];
        assign readData[7:0] = bytes[rwAddress];
        assign readData[15:8] = bytes[rwAddress+1];
        assign readData[23:16] = bytes[rwAddress+2];
        assign readData[31:24]luğu = bytes[rwAddress+3];
        assign readData[39:32] = bytes[rwAddress+4];
        assign readData[47:40] = bytes[rwAddress+5];
        assign readData[55:48] = bytes[rwAddress+6];
        assign readData[63:56] = bytes[rwAddress+7];
        always @(posedge clk or posedge reset) begin
            if (reset) begin
                for (i = 0; i < 524288; i = i + 1)
                    bytes[i] <= 8'b0;
            end else if (flag) begin
                bytes[rwAddress] <= writeData[7:0];
                bytes[rwAddress+1] <= writeData[15:8];
                bytes[rwAddress+2] <= writeData[23:16];
                bytes[rwAddress+3] <= writeData[31:24];
                bytes[rwAddress+4] <= writeData[39:32];
                bytes[rwAddress+5] <= writeData[47:40];
                bytes[rwAddress+6] <= writeData[55:48];
                bytes[rwAddress+7] <= writeData[63:56];
            end
        end
    endmodule

    // Instruction Parser (unchanged)
    module insnParser (
        input wire [31:0] insn,
        output reg [4:0] opCode,
        output reg [4:0] regD,
        output reg [4:0] regS1,
        output reg [4:0] regS2,
        output reg [63:0] immVal
    );
        always @(*) begin
            opCode = insn[31:27];
            regD = insn[26:22];
            regS1 = insn[21:17];
            regS2 = insn[16:12];
            immVal = {{52{insn[11]}}, insn[11:0]};
            if (opCode == 5'h19 || opCode == 5'h1B || opCode == 5'h05 ||
                opCode == 5'h07 || opCode == 5'h12)
                regS1 = regD;
        end
    endmodule

    // Register Bank (matched to friend’s registerFile)
    module regBank (
        input wire clk,
        input wire reset,
        input wire [63:0] data,
        input wire [4:0] read1,
        input wire [4:0] read2,
        input wire [4:0] write,
        input wire memFlag,
        input wire regFlag,
        output reg [63:0] output1,
        output reg [63:0] output2,
        output reg [63:0] output3,
        output reg [63:0] stackPtr
    );
        reg [63:0] registers [0:31];
        reg writeReg;
        integer i;
        assign writeReg = memFlag | regFlag;
        assign output1 = registers[read1];
        assign output2 = registers[read2];
        assign output3 = registers[write];
        assign stackPtr = registers[31];
        always @(posedge clk or posedge reset) begin
            if (reset) begin
                for (i = 0; i < 31; i = i + 1)
                    registers[i] <= 64'b0;
                registers[31] <= 64'd524288;
            end else if (writeReg) begin
                registers[write] <= data;
            end
        end
    endmodule

    // ALU (matched to friend’s alu)
    module computeUnit (
        input wire [4:0] control,
        input wire [63:0] input1,
        input wire [63:0] input2,
        output reg flag,
        output reg [63:0] result
    );
        real r1, r2, rres;
        assign r1 = $bitstoreal(input1);
        assign r2 = $bitstoreal(input2);
        always @(*) begin
            flag = 1;
            case (control)
                5'b11000: result = input1 + input2;
                5'b11001: result = input1 + input2;
                5'b11010: result = input1 - input2;
                5'b11011: result = input1 - input2;
                5'b11100: result = input1 * input2;
                5'b11101: result = input1 / input2;
                5'b00000: result = input1 & input2;
                5'b00001: result = input1 | input2;
                5'b00010: result = input1 ^ input2;
                5'b00011: result = ~input1;
                5'b00100: result = input1 >> input2;
                5'b00101: result = input1 >> input2;
                5'b00110: result = input1 << input2;
                5'b00111: result = input1 << input2;
                5'b10001: result = input1;
                5'b10010: result = {input1[63:12], input2[11:0]};
                5'b10100: begin
                    rres = r1 + r2;
                    result = $realtobits(rres);
                end
                5'b10101: begin
                    rres = r1 - r2;
                    result = $realtobits(rres);
                end
                5'b10110: begin
                    rres = r1 * r2;
                    result = $realtobits(rres);
                end
                5'b10111: begin
                    rres = r1 / r2;
                    result = $realtobits(rres);
                end
                default: begin
                    flag = 0;
                    result = 64'b0;
                end
            endcase
        end
    endmodule

    // Branch Control Unit (matched to friend’s control)
    module branchController (
        input wire [4:0] op,
        input wire [63:0] rd,
        input wire [63:0] rs,
        input wire [63:0] rt,
        input wire [63:0] lit,
        input wire [63:0] inputPc,
        input wire [63:0] memData,
        output reg [63:0] pc
    );
        always @(*) begin
            case (op)
                5'b01000: pc = rd;                  // br
                5'b01001: pc = inputPc + rd;        // brr $r_d
                5'b01010: pc = inputPc + $signed(lit); // brr L
                5'b01011: pc = (rs != 0) ? rd : inputPc + 4; // brnz
                5'b01100: pc = rd;                  // call
                5'b01101: pc = memData;             // return
                5'b01110: pc = (rs > rt) ? rd : inputPc + 4; // brgt
                default: pc = inputPc + 4;
            endcase
        end
    endmodule

    // Memory Access Controller (matched to friend’s memoryHandler)
    module memController (
        input wire [4:0] opcode,
        input wire [63:0] rd,
        input wire [63:0] rs,
        input wire [63:0] lit,
        input wire [63:0] pc,
        input wire [63:0] r31,
        output reg [63:0] rwAddress,
        output reg [63:0] writeData,
        output reg writeFlag,
        output reg regWrite
    );
        always @(*) begin
            case (opcode)
                5'b01100: begin
                    rwAddress = r31 - 8;
                    writeData = pc + 4;
                    writeFlag = 1;
                    regWrite = 0;
                end
                5'b01101: begin
                    rwAddress = r31 - 8;
                    writeFlag = 0;
                    regWrite = 0;
                    writeData = 0;
                end
                5'b10000: begin
                    rwAddress = rs + lit;
                    writeFlag = 0;
                    regWrite = 1;
                    writeData = 0;
                end
                5'b10011: begin
                    rwAddress = rd + lit;
                    writeFlag = 1;
                    regWrite = 0;
                    writeData = rs;
                end
                default: begin
                    rwAddress = 64'h2000;
                    writeFlag = 0;
                    regWrite = 0;
                    writeData = 0;
                end
            endcase
        end
    endmodule

    // Operand Selector (matched to friend’s reglitmux)
    module operandMux (
        input wire [4:0] sel,
        input wire [63:0] reg1,
        input wire [63:0] lit,
        output reg [63:0] out
    );
        always @(*) begin
            case (sel)
                5'b11001: out = lit; // addi
                5'b11011: out = lit; // subi
                5'b00101: out = lit; // shftri
                5'b00111: out = lit; // shftli
                5'b10010: out = lit; // mov $r_d, L
                default: out = reg1;
            endcase
        end
    endmodule

    // Instantiate modules
    pcUpdater pcUnit (
        .clk(clock),
        .reset(rst),
        .r31(stackPtr),
        .next_pc(nextProgramCounter),
        .pc(programCounter)
    );

    storageUnit memUnit (
        .pc(programCounter),
        .clk(clock),
        .reset(rst),
        .flag(memWriteEnable),
        .writeData(memWriteValue),
        .rwAddress(memAccessAddr),
        .readData(memReadData),
        .instruction(currentInstruction)
    );

    insnParser decodeUnit (
        .insn(currentInstruction),
        .opCode(operationCode),
        .regD(destReg),
        .regS1(sourceReg1),
        .regS2(sourceReg2),
        .immVal(immediateValue)
    );

    regBank registers (
        .clk(clock),
        .reset(rst),
        .data(memToRegWrite ? memReadData : aluOutput),
        .read1(sourceReg1),
        .read2(sourceReg2),
        .write(destReg),
        .memFlag(memToRegWrite),
        .regFlag(aluWriteEnable),
        .output1(regSrc1Val),
        .output2(regSrc2Val),
        .output3(regDestVal),
        .stackPtr(stackPtr)
    );

    operandMux opSelect (
        .sel(operationCode),
        .reg1(regSrc2Val),
        .lit(immediateValue),
        .out(secondOperand)
    );

    computeUnit arithmeticUnit (
        .control(operationCode),
        .input1(regSrc1Val),
        .input2(secondOperand),
        .result(aluOutput),
        .flag(aluWriteEnable)
    );

    branchController branchUnit (
        .op(operationCode),
        .rd(regDestVal),
        .rs(regSrc1Val),
        .rt(regSrc2Val),
        .lit(immediateValue),
        .inputPc(programCounter),
        .memData(memReadData),
        .pc(nextProgramCounter)
    );

    memController memAccess (
        .opcode(operationCode),
        .rd(regDestVal),
        .rs(regSrc1Val),
        .lit(immediateValue),
        .pc(programCounter),
        .r31(stackPtr),
        .rwAddress(memAccessAddr),
        .writeData(memWriteValue),
        .writeFlag(memWriteEnable),
        .regWrite(memToRegWrite)
    );

endmodule