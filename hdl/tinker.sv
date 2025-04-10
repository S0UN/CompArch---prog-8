module tinker_core (
    input clk,
    input reset
);
    logic [4:0] rd, rs, rt, opcode;
    logic [31:0] instruction;
    logic [63:0] next_pc, pc;
    logic [63:0] literal, rd_val, rs_val, rt_val, in2, data, stackPtr, memWriteData, rwAddress, memOutput;
    logic memWriteFlag, regWriteMem, regWriteALU;

    // Fetch Module
    module fetch (
        input wire clk,
        input wire reset,
        input wire [63:0] r31,
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

    // Memory Module
    module memory (
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
        assign readData[31:24] = bytes[rwAddress+3];
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

    // Control Module
    module control (
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

    // Memory Handler Module
    module memoryHandler (
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

    // Instruction Decoder Module
    module instructionDecoder (
        input wire [31:0] instructionLine,
        output reg [63:0] literal,
        output reg [4:0] rd,
        output reg [4:0] rs,
        output reg [4:0] rt,
        output reg [4:0] opcode
    );
        always @(*) begin
            opcode = instructionLine[31:27];
            rd = instructionLine[26:22];
            rs = instructionLine[21:17];
            rt = instructionLine[16:12];
            literal = {52'b0, instructionLine[11:0]};
            case (opcode)
                5'b11001: rs = rd; // addi
                5'b11011: rs = rd; // subi
                5'b00101: rs = rd; // shftri
                5'b00111: rs = rd; // shftli
                5'b10010: rs = rd; // mov $r_d, L
            endcase
        end
    endmodule

    // Register File Module
    module registerFile (
        input wire [63:0] data,
        input wire [4:0] read1,
        input wire [4:0] read2,
        input wire [4:0] write,
        input wire reset,
        input wire memFlag,
        input wire regFlag,
        input wire clk,
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

    // Register/Literal Mux Module
    module reglitmux (
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

    // ALU Module
    module alu (
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
                5'b11000: result = input1 + input2;         // add
                5'b11001: result = input1 + input2;         // addi
                5'b11010: result = input1 - input2;         // sub
                5'b11011: result = input1 - input2;         // subi
                5'b11100: result = input1 * input2;         // mul
                5'b11101: result = input1 / input2;         // div
                5'b00000: result = input1 & input2;         // and
                5'b00001: result = input1 | input2;         // or
                5'b00010: result = input1 ^ input2;         // xor
                5'b00011: result = ~input1;                 // not
                5'b00100: result = input1 >> input2;        // shftr
                5'b00101: result = input1 >> input2;        // shftri
                5'b00110: result = input1 << input2;        // shftl
                5'b00111: result = input1 << input2;        // shftli
                5'b10001: result = input1;                  // mov $r_d, $r_s
                5'b10010: result = {input1[63:12], input2[11:0]}; // mov $r_d, L
                5'b10100: begin                             // addf
                    rres = r1 + r2;
                    result = $realtobits(rres);
                end
                5'b10101: begin                             // subf
                    rres = r1 - r2;
                    result = $realtobits(rres);
                end
                5'b10110: begin                             // mulf
                    rres = r1 * r2;
                    result = $realtobits(rres);
                end
                5'b10111: begin                             // divf
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

    // Instantiate all modules within tinker_core
    fetch fetch_inst (
        .clk(clk),
        .reset(reset),
        .r31(stackPtr),
        .next_pc(pc),
        .pc(next_pc)
    );

    memory memory_inst (
        .pc(next_pc),
        .clk(clk),
        .reset(reset),
        .flag(memWriteFlag),
        .writeData(memWriteData),
        .rwAddress(rwAddress),
        .readData(memOutput),
        .instruction(instruction)
    );

    control control_inst (
        .op(opcode),
        .rd(rd_val),
        .rs(rs_val),
        .rt(rt_val),
        .lit(literal),
        .inputPc(next_pc),
        .memData(memOutput),
        .pc(pc)
    );

    memoryHandler memoryHandler_inst (
        .opcode(opcode),
        .rd(rd_val),
        .rs(rs_val),
        .lit(literal),
        .pc(next_pc),
        .r31(stackPtr),
        .rwAddress(rwAddress),
        .writeData(memWriteData),
        .writeFlag(memWriteFlag),
        .regWrite(regWriteMem)
    );

    instructionDecoder id (
        .instructionLine(instruction),
        .literal(literal),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .opcode(opcode)
    );

    registerFile reg_file (
        .clk(clk),
        .reset(reset),
        .memFlag(regWriteMem),
        .regFlag(regWriteALU),
        .data(data),
        .read1(rs),
        .read2(rt),
        .write(rd),
        .output1(rs_val),
        .output2(rt_val),
        .output3(rd_val),
        .stackPtr(stackPtr)
    );

    reglitmux mux (
        .sel(opcode),
        .reg1(rt_val),
        .lit(literal),
        .out(in2)
    );

    alu alu_inst (
        .control(opcode),
        .input1(rs_val),
        .input2(in2),
        .result(data),
        .flag(regWriteALU)
    );

endmodule