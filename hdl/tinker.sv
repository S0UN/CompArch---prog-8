// Top-level module for Tinker CPU
module tinker_core (
    input wire clock,
    input wire rst
);

    // Wires for interconnecting modules
    wire [63:0] programCounter, nextProgramCounter;
    wire [31:0] currentInstruction;
    wire [4:0] destReg, sourceReg1, sourceReg2, operationCode;
    wire [63:0] immediateValue, regDestVal, regSrc1Val, regSrc2Val;
    wire [63:0] aluOutput, memReadData, memWriteValue, memAccessAddr;
    wire aluWriteEnable, memWriteEnable, memToRegWrite;
    wire [63:0] secondOperand;

    // Program Counter Update Unit
    module pcUpdater (
        input wire clk,
        input wire reset,
        input wire [63:0] newPc,
        output reg [63:0] currentPc
    );
        always @(posedge clk or posedge reset) begin
            if (reset)
                currentPc <= 64'h2000; // Start at 0x2000
            else
                currentPc <= newPc;
        end
    endmodule

    // Memory Module (512 KB)
    module storageUnit (
        input wire [63:0] fetchAddr,
        input wire clk,
        input wire reset,
        input wire writeEn,
        input wire [63:0] dataIn,
        input wire [63:0] dataAddr,
        output reg [31:0] insnOut,
        output reg [63:0] dataOut
    );
        reg [7:0] memArray [0:524287]; // 512 KB
        integer idx;

        always @(*) begin
            insnOut = {memArray[fetchAddr+3], memArray[fetchAddr+2],
                       memArray[fetchAddr+1], memArray[fetchAddr]};
            dataOut = {memArray[dataAddr+7], memArray[dataAddr+6],
                       memArray[dataAddr+5], memArray[dataAddr+4],
                       memArray[dataAddr+3], memArray[dataAddr+2],
                       memArray[dataAddr+1], memArray[dataAddr]};
        end

        always @(posedge clk or posedge reset) begin
            if (reset) begin
                for (idx = 0; idx < 524288; idx = idx + 1)
                    memArray[idx] <= 8'b0;
            end else if (writeEn) begin
                {memArray[dataAddr+7], memArray[dataAddr+6], memArray[dataAddr+5],
                 memArray[dataAddr+4], memArray[dataAddr+3], memArray[dataAddr+2],
                 memArray[dataAddr+1], memArray[dataAddr]} <= dataIn;
            end
        end
    endmodule

    // Instruction Parser
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
            immVal = {{52{insn[11]}}, insn[11:0]}; // Sign-extend 12-bit literal

            // Adjust source register for immediate instructions
            if (opCode == 5'h19 || opCode == 5'h1B || opCode == 5'h05 ||
                opCode == 5'h07 || opCode == 5'h12)
                regS1 = regD;
        end
    endmodule

    // Register Bank
    module regBank (
        input wire clk,
        input wire reset,
        input wire writeEn,
        input wire [63:0] writeData,
        input wire [4:0] readPort1,
        input wire [4:0] readPort2,
        input wire [4:0] writePort,
        output reg [63:0] dataPort1,
        output reg [63:0] dataPort2,
        output reg [63:0] dataPort3,
        output reg [63:0] spOut
    );
        reg [63:0] regs [0:31];
        integer i;

        always @(*) begin
            dataPort1 = regs[readPort1];
            dataPort2 = regs[readPort2];
            dataPort3 = regs[writePort];
            spOut = regs[31];
        end

        always @(posedge clk or posedge reset) begin
            if (reset) begin
                for (i = 0; i < 31; i = i + 1)
                    regs[i] <= 64'b0;
                regs[31] <= 64'd524288; // Stack pointer init
            end else if (writeEn) begin
                regs[writePort] <= writeData;
            end
        end
    endmodule

    // Computation Unit (ALU)
    module computeUnit (
        input wire [4:0] ctrl,
        input wire [63:0] op1,
        input wire [63:0] op2,
        output reg [63:0] outVal,
        output reg writeFlag
    );
        real fpOp1, fpOp2, fpResult;

        always @(*) begin
            writeFlag = 1;
            fpOp1 = $bitstoreal(op1);
            fpOp2 = $bitstoreal(op2);

            case (ctrl)
                5'h18, 5'h19: outVal = op1 + op2;           // add, addi
                5'h1A, 5'h1B: outVal = op1 - op2;           // sub, subi
                5'h1C: outVal = op1 * op2;                  // mul
                5'h1D: outVal = (op2 != 0) ? op1 / op2 : 0; // div
                5'h00: outVal = op1 & op2;                  // and
                5'h01: outVal = op1 | op2;                  // or
                5'h02: outVal = op1 ^ op2;                  // xor
                5'h03: outVal = ~op1;                       // not
                5'h04, 5'h05: outVal = op1 >> op2[5:0];     // shftr, shftri
                5'h06, 5'h07: outVal = op1 << op2[5:0];     // shftl, shftli
                5'h11: outVal = op1;                        // mov $r_d, $r_s
                5'h12: outVal = {{52{op2[11]}}, op2[11:0]}; // mov $r_d, L
                5'h14: begin                                // addf
                    fpResult = fpOp1 + fpOp2;
                    outVal = $realtobits(fpResult);
                end
                5'h15: begin                                // subf
                    fpResult = fpOp1 - fpOp2;
                    outVal = $realtobits(fpResult);
                end
                5'h16: begin                                // mulf
                    fpResult = fpOp1 * fpOp2;
                    outVal = $realtobits(fpResult);
                end
                5'h17: begin                                // divf
                    fpResult = (fpOp2 != 0.0) ? fpOp1 / fpOp2 : 0.0;
                    outVal = $realtobits(fpResult);
                end
                default: begin
                    outVal = 64'b0;
                    writeFlag = 0;
                end
            endcase
        end
    endmodule

    // Branch Control Unit
    module branchController (
        input wire [4:0] op,
        input wire [63:0] targetReg,
        input wire [63:0] compReg1,
        input wire [63:0] compReg2,
        input wire [63:0] immOffset,
        input wire [63:0] currentPc,
        input wire [63:0] memVal,
        output reg [63:0] nextPc
    );
        always @(*) begin
            case (op)
                5'h08: nextPc = targetReg;                  // br
                5'h09: nextPc = currentPc + targetReg;      // brr $r_d
                5'h0A: nextPc = currentPc + $signed(immOffset); // brr L
                5'h0B: nextPc = (compReg1 != 0) ? targetReg : currentPc + 4; // brnz
                5'h0C: nextPc = targetReg;                  // call
                5'h0D: nextPc = memVal;                     // return
                5'h0E: nextPc = (compReg1 > compReg2) ? targetReg : currentPc + 4; // brgt
                default: nextPc = currentPc + 4;
            endcase
        end
    endmodule

    // Memory Access Controller
    module memController (
        input wire [4:0] opCode,
        input wire [63:0] baseReg,
        input wire [63:0] srcReg,
        input wire [63:0] offset,
        input wire [63:0] pcVal,
        input wire [63:0] stackReg,
        output reg [63:0] accessAddr,
        output reg [63:0] writeVal,
        output reg writeEn,
        output reg regWriteEn
    );
        always @(*) begin
            accessAddr = 64'h0;
            writeVal = 64'b0;
            writeEn = 0;
            regWriteEn = 0;

            if (opCode == 5'h0C) begin          // call
                accessAddr = stackReg - 8;
                writeVal = pcVal + 4;
                writeEn = 1;
            end else if (opCode == 5'h0D) begin // return
                accessAddr = stackReg - 8;
            end else if (opCode == 5'h10) begin // mov $r_d, ($r_s)(L)
                accessAddr = srcReg + offset;
                regWriteEn = 1;
            end else if (opCode == 5'h13) begin // mov ($r_d)(L), $r_s
                accessAddr = baseReg + offset;
                writeVal = srcReg;
                writeEn = 1;
            end
        end
    endmodule

    // Operand Selector
    module operandMux (
        input wire [4:0] select,
        input wire [63:0] regInput,
        input wire [63:0] immInput,
        output reg [63:0] selected
    );
        always @(*) begin
            if (select == 5'h19 || select == 5'h1B || select == 5'h05 ||
                select == 5'h07 || select == 5'h12)
                selected = immInput;
            else
                selected = regInput;
        end
    endmodule

    // Instantiate modules
    pcUpdater pcUnit (
        .clk(clock),
        .reset(rst),
        .newPc(nextProgramCounter),
        .currentPc(programCounter)
    );

    storageUnit memUnit (
        .fetchAddr(programCounter),
        .clk(clock),
        .reset(rst),
        .writeEn(memWriteEnable),
        .dataIn(memWriteValue),
        .dataAddr(memAccessAddr),
        .insnOut(currentInstruction),
        .dataOut(memReadData)
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
        .writeEn(aluWriteEnable | memToRegWrite),
        .writeData(memToRegWrite ? memReadData : aluOutput),
        .readPort1(sourceReg1),
        .readPort2(sourceReg2),
        .writePort(destReg),
        .dataPort1(regSrc1Val),
        .dataPort2(regSrc2Val),
        .dataPort3(regDestVal),
        .spOut(stackPtr)
    );

    operandMux opSelect (
        .select(operationCode),
        .regInput(regSrc2Val),
        .immInput(immediateValue),
        .selected(secondOperand)
    );

    computeUnit arithmeticUnit (
        .ctrl(operationCode),
        .op1(regSrc1Val),
        .op2(secondOperand),
        .outVal(aluOutput),
        .writeFlag(aluWriteEnable)
    );

    branchController branchUnit (
        .op(operationCode),
        .targetReg(regDestVal),
        .compReg1(regSrc1Val),
        .compReg2(regSrc2Val),
        .immOffset(immediateValue),
        .currentPc(programCounter),
        .memVal(memReadData),
        .nextPc(nextProgramCounter)
    );

    memController memAccess (
        .opCode(operationCode),
        .baseReg(regDestVal),
        .srcReg(regSrc1Val),
        .offset(immediateValue),
        .pcVal(programCounter),
        .stackReg(stackPtr),
        .accessAddr(memAccessAddr),
        .writeVal(memWriteValue),
        .writeEn(memWriteEnable),
        .regWriteEn(memToRegWrite)
    );

endmodule