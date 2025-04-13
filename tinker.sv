module tinker_core (
    input clk,
    input reset,
    output hlt
);

    // Internal signals remain the same
    logic [4:0] rd, rs, rt, opcode;
    logic [31:0] instructionLine;
    logic [63:0] next_pc, pc, alu_pc;
    logic [63:0] lit, rd_val, rs_val, rt_val, input2, aluOut, stackPtr, writeData, rwAddress, readData, regWriteData;
    logic writeFlag;
    logic mem_pc;
    logic aluEnable, regReadEnable, regWriteEnable, fetchEnable, memRead, memEnable, aluReady;

    // Instantiate instructionDecoder (Revised FSM logic below)
    instructionDecoder id(
        .instructionLine(instructionLine),
        .clk(clk),
        .rst(reset),
        .aluReady(aluReady),
        // .memEnable() // Removed, output only now
        .literal(lit),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .opcode(opcode),
        .aluEnable(aluEnable),
        .fetchEnable(fetchEnable),
        .memEnable(memEnable), // Added memEnable as output
        .regReadEnable(regReadEnable),
        .regWriteEnable(regWriteEnable)
    );

    // Instantiate fetch (Using your original robust version)
    fetch fetch_unit(
        .clk(clk),
        .fetchEnable(fetchEnable),
        .reset(reset),
        .next_pc(next_pc),
        .pc(pc)
    );

    // Instantiate alu (Using your original version)
    alu alu_unit(
        .aluEnable(aluEnable),
        .control(opcode),
        .input1(rs_val),
        .input2(input2),
        .inputPc(pc),
        .r31(stackPtr),
        .rd(rd_val), // This is the value read from reg file addr 'rd'
        .clk(clk),
        .result(aluOut),
        .pc(alu_pc), // This is the PC calculated by the ALU
        .writeFlag(writeFlag),
        .memRead(memRead),
        .aluReady(aluReady),
        .writeData(writeData),
        .rwAddress(rwAddress),
        .hlt(hlt),
        .mem_pc(mem_pc)
    );

    // Instantiate regLitMux (Using your original version)
    regLitMux reg_lit_mux(
        .sel(opcode),
        .reg1(rt_val),
        .lit(lit),
        .out(input2)
    );

    // Instantiate registerFile (Using your original robust version)
    registerFile reg_file(
        .data(regWriteData),
        .read1(rs),
        .read2(rt),
        .write(rd),
        .reset(reset), // Pass reset through
        .clk(clk),
        .regReadEnable(regReadEnable),
        .regWriteEnable(regWriteEnable),
        .output1(rs_val),
        .output2(rt_val),
        .output3(rd_val), // Value of register rd (needed for store base, some branches)
        .stackPtr(stackPtr)
    );

    // Instantiate aluMemMux (Using your original version)
    aluMemMux alu_mem_mux(
        .mem_pc(mem_pc),
        .memData(readData),
        .aluOut(alu_pc),
        .newPc(next_pc)
    );

    // Instantiate memory (Using your original version)
    memory mem_unit(
        .pc(pc),
        .clk(clk),
        .reset(reset),
        .writeFlag(writeFlag),
        .fetchEnable(fetchEnable), // Pass through
        .memEnable(memEnable),     // Pass through
        .memRead(memRead),
        .writeData(writeData),
        .rwAddress(rwAddress),
        .readData(readData),
        .instruction(instructionLine)
    );

    // Instantiate memRegMux (Using your original version)
    memRegMux mem_reg_mux(
        .opcode(opcode),
        .readData(readData),
        .aluResult(aluOut),
        .regWriteData(regWriteData)
    );

endmodule

//-------------------------------------------------------------
// instructionDecoder - Revised FSM structure based on v_ok
//-------------------------------------------------------------
module instructionDecoder(
    input [31:0] instructionLine,
    input clk,
    input rst,
    input aluReady,
    output logic fetchEnable,
    output logic memEnable,
    output logic [63:0] literal,
    output logic [4:0] rd,
    output logic [4:0] rs,
    output logic [4:0] rt,
    output logic [4:0] opcode,
    output logic aluEnable,
    output logic regReadEnable,
    output logic regWriteEnable
);

    typedef enum logic [2:0] {
        FETCH         = 3'b000,
        DECODE        = 3'b001,
        EXECUTE       = 3'b010,
        MEMORY_ACCESS = 3'b011,
        WRITE_BACK    = 3'b100
    } state_t;

    state_t current_state, next_state;
    logic [4:0] opcode_latch; // Store opcode from DECODE phase

    // State Register
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= FETCH;
        end else begin
            current_state <= next_state;
        end
    end

     // Latch opcode during DECODE state for use in EXECUTE state transitions
     always @(posedge clk) begin
        if (current_state == DECODE) begin
             opcode_latch <= instructionLine[31:27];
        end
     end

    // Next State Logic
    always @(*) begin
        next_state = current_state; // Default: stay in current state
        case (current_state)
            FETCH: next_state = DECODE;
            DECODE: next_state = EXECUTE;
            EXECUTE: begin
                if (!aluReady) begin
                    next_state = EXECUTE; // Wait if ALU not ready
                end else begin
                    // Use latched opcode
                    if (opcode_latch == 5'h10 || opcode_latch == 5'h13 || opcode_latch == 5'h0C || opcode_latch == 5'h0D) begin // Mem ops
                        next_state = MEMORY_ACCESS;
                    end else if (opcode_latch == 5'h08 || opcode_latch == 5'h09 || opcode_latch == 5'h0A || opcode_latch == 5'h0B || opcode_latch == 5'h0E) begin // Branch/Jump ops
                        next_state = FETCH; // Go back to fetch
                    end else if (opcode_latch == 5'h0F) begin // Halt
                        next_state = FETCH; // Go back to fetch (core will halt via ALU)
                    end else begin // Other ops (ALU R/I, MOV reg/imm)
                        next_state = WRITE_BACK;
                    end
                end
            end
            MEMORY_ACCESS: begin
                // Only Load (opcode 5'h10) goes to writeback
                if (opcode_latch == 5'h10) begin
                    next_state = WRITE_BACK;
                end else begin
                    next_state = FETCH; // Others (Store, Call, Ret) go to Fetch
                end
            end
            WRITE_BACK: next_state = FETCH;
            default: next_state = FETCH; // Should not happen
        endcase
    end

    // Output Logic (Combinational based on current_state)
    always @(*) begin
        // Assign default values
        fetchEnable    = 1'b0;
        memEnable      = 1'b0;
        literal        = 64'b0;
        rd             = 5'b0;
        rs             = 5'b0;
        rt             = 5'b0;
        opcode         = 5'b0;
        aluEnable      = 1'b0;
        regReadEnable  = 1'b0;
        regWriteEnable = 1'b0;

        case (current_state)
            FETCH: begin
                fetchEnable = 1'b1;
                // memEnable = 1'b1; // Enable memory for instruction fetch - Done by mem_unit directly based on PC
            end
            DECODE: begin
                // Decode fields directly from instructionLine
                opcode  = instructionLine[31:27];
                rd      = instructionLine[26:22];
                rs      = instructionLine[21:17];
                rt      = instructionLine[16:12];
                literal = {52'b0, instructionLine[11:0]};

                // Apply rs=rd remapping from v_ok logic for specific immediate types
                // This remapping might be specific to the v_ok ISA interpretation
                case (opcode)
                   5'b11001, 5'b11011, 5'b00101, 5'b00111, 5'b10010: rs = rd;
                   default: ; // Use original rs
                endcase

                regReadEnable = 1'b1; // Enable reading rs and rt values
            end
            EXECUTE: begin
                aluEnable = 1'b1;
                // Pass through decoded values needed by ALU
                // Use the *latched* opcode for control
                opcode = opcode_latch;
                rd = rd; // Value needed by ALU comes from regFile output3
                rs = rs;
                rt = rt;
                literal = literal;
            end
            MEMORY_ACCESS: begin
                memEnable = 1'b1; // Enable memory for data load/store
                // Pass through values needed
                opcode = opcode_latch; // Needed for memRegMux
                rd = rd;
                rs = rs;
                rt = rt;
                literal = literal;
            end
            WRITE_BACK: begin
                regWriteEnable = 1'b1; // Enable writing result to register file
                 // Pass through values needed
                opcode = opcode_latch; // Needed for memRegMux
                rd = rd; // Destination register index
            end
        endcase
    end

endmodule

//-------------------------------------------------------------
// alu - Using your original version
//-------------------------------------------------------------
module alu (
    input aluEnable,
    input [4:0] control,
    input [63:0] input1, // rs_val
    input [63:0] input2, // rt_val or immediate
    input [63:0] rd,     // rd_val (value from reg file output3)
    input [63:0] inputPc,
    input [63:0] r31,    // stackPtr value
    input clk,
    output reg [63:0] result, // aluOut
    output reg [63:0] writeData,
    output reg [63:0] rwAddress,
    output reg aluReady,
    output reg writeFlag,
    output reg memRead,
    output reg [63:0] pc, // alu_pc
    output reg hlt,
    output reg mem_pc
);

    real r1, r2, rres;
    assign r1 = $bitstoreal(input1);
    assign r2 = $bitstoreal(input2);

    always @(*) begin
        if (aluEnable) begin
            // Default assignments for active cycle
            hlt = 0;
            pc = inputPc + 4; // Default next PC is PC+4
            writeData = 0;
            rwAddress = 'x; // Default address to 'x'
            writeFlag = 0;
            memRead = 0;
            mem_pc = 0;
            result = 0; // Default result

            case (control)
                // Integer Arithmetic
                5'h18: result = input1 + input2; // ADD
                5'h19: result = input1 + input2; // ADDI
                5'h1A: result = input1 - input2; // SUB
                5'h1B: result = input1 - input2; // SUBI
                5'h1C: result = input1 * input2; // MUL
                5'h1D: result = input1 / input2; // DIV

                // Logical Operations
                5'h00: result = input1 & input2; // AND
                5'h01: result = input1 | input2; // OR
                5'h02: result = input1 ^ input2; // XOR
                5'h03: result = ~input1;         // NOT

                // Shift Operations (Note: Verilog >> is arithmetic if input1 is signed, logical otherwise. Assuming logical shift based on opcodes)
                5'h04: result = input1 >> input2; // ASR/LSR? Assuming Arithmetic based on v_ok/common practice $signed(input1) >>> input2;
                5'h05: result = input1 >> input2; // ASRI/LSRI? Assuming Arithmetic $signed(input1) >>> input2;
                5'h06: result = input1 << input2; // SL
                5'h07: result = input1 << input2; // SLI

                // Memory & Move
                5'h10: begin // MOV (Load)
                    result = 64'b0; // Data comes from memory later
                    rwAddress = input1 + input2; // Base(rs_val) + Offset(imm/rt_val)
                    memRead = 1;
                end
                5'h11: result = input1; // MOV (Register to Register) rs_val -> rd
                5'h12: result = {input1[63:12], input2[11:0]}; // LI (Load Immediate) - Uses rs_val and immediate
                5'h13: begin // MOV (Store)
                    result = 64'b0; // No register result
                    rwAddress = rd + input2; // Base(rd_val) + Offset(imm)
                    writeData = input1;      // Data from rs_val
                    writeFlag = 1;
                end

                // Floating Point
                5'h14: begin rres = r1 + r2; result = $realtobits(rres); end // FADD
                5'h15: begin rres = r1 - r2; result = $realtobits(rres); end // FSUB
                5'h16: begin rres = r1 * r2; result = $realtobits(rres); end // FMUL
                5'h17: begin rres = r1 / r2; result = $realtobits(rres); end // FDIV

                // Control Flow
                5'h08: begin // JMPR (Jump Register)
                    pc = rd; // Target address is value read from rd index
                    result = 64'b0;
                end
                5'h09: begin // JMPA (Jump PC Absolute Offset)
                    pc = inputPc + rd; // Target is PC + value read from rd index
                    result = 64'b0;
                end
                5'h0A: begin // JMPAI (Jump PC Immediate Offset)
                    // Assuming input2 holds the sign-extended immediate from reglitmux
                    pc = inputPc + $signed(input2);
                    result = 64'b0;
                end
                5'h0B: begin // BRNZ (Branch if Not Zero)
                    // Branch target address is value read from rd index
                    pc = (input1 != 64'b0) ? rd : (inputPc + 4);
                    result = 64'b0;
                end
                5'h0C: begin // CALL
                    pc = rd; // Target address is value read from rd index
                    result = 64'b0;
                    writeData = inputPc + 4; // Return address (PC+4)
                    rwAddress = r31 - 8;     // Stack Pointer value minus 8
                    writeFlag = 1;           // Write return address to stack
                end
                5'h0D: begin // RETURN
                    result = 64'b0;
                    rwAddress = r31 - 8; // Read return address from stack (SP value - 8)
                    memRead = 1;
                    mem_pc = 1;           // Indicate next PC comes from memory data
                    // pc calculation is overridden by aluMemMux
                end
                5'h0E: begin // BRGT (Branch if Greater Than)
                    // Branch target address is value read from rd index
                    // Compare input1 (rs_val) with input2 (rt_val or imm)
                    pc = ($signed(input1) > $signed(input2)) ? rd : (inputPc + 4);
                    result = 64'b0;
                end
                5'h0F: begin // HLT
                    result = 64'b0;
                    hlt = 1;
                    pc = inputPc; // Keep PC the same
                end
                default: begin // Undefined Opcode
                    result = 64'b0;
                    pc = inputPc + 4; // Treat as NOP
                 end
            endcase
            aluReady = 1; // ALU finished processing for this cycle
        end else begin
            // Default values when ALU not enabled
            hlt = 0;
            aluReady = 0;
            result = 'x;
            writeData = 'x;
            rwAddress = 'x;
            writeFlag = 0;
            memRead = 0;
            pc = 'x;
            mem_pc = 0;
        end
    end
endmodule

//-------------------------------------------------------------
// regLitMux - Using your original version
//-------------------------------------------------------------
module regLitMux (
    input [4:0] sel,
    input [63:0] reg1, // rt_val
    input [63:0] lit,
    output reg [63:0] out // input2 for ALU
);
    // Your original if-else version
    always @(*) begin
        if (sel == 5'b11001 || sel == 5'b11011 || sel == 5'b00101 || sel == 5'b00111 || sel == 5'b10010 || sel == 5'b01010 || sel == 5'h10 || sel == 5'h13 || sel == 5'h0A) begin // Added 5'h0A (JMPAI) as it uses imm
             // Note: 5'b01010 is 5'h0A. Included only once. BRGT (5'h0E) not included here, assumes it uses reg1.
            out = lit;
        end else begin
            out = reg1;
        end
    end
endmodule

//-------------------------------------------------------------
// registerFile - Using your original robust version
//-------------------------------------------------------------
module registerFile (
    input [63:0] data,
    input [4:0] read1, // rs
    input [4:0] read2, // rt
    input [4:0] write, // rd
    input reset,
    input clk,
    input regReadEnable,
    input regWriteEnable,
    output logic [63:0] output1, // rs_val
    output logic [63:0] output2, // rt_val
    output logic [63:0] output3, // rd_val (value at write address)
    output logic [63:0] stackPtr // r31 value
);

    reg [63:0] registers [0:31];
    integer i;

    // Initialization - Keep non-blocking for initial block
    initial begin
        for (i = 0; i < 31; i = i + 1) begin
            registers[i] <= 64'b0;
        end
        registers[31] <= 64'd524288;
    end

    // Synchronous Write Logic
    always @(posedge clk) begin
        // Apply reset if needed (optional, depends on if registers should clear on reset signal)
        // if (reset) begin ... end else ...
        if (regWriteEnable) begin
            // Optional: Prevent writing to R0
            // if (write != 5'b0)
             registers[write] <= data; // Use non-blocking for sequential logic
        end
    end

    // Combinational Read Logic
    always @(*) begin
        if (regReadEnable) begin
            output1 = registers[read1];
            output2 = registers[read2];
            output3 = registers[write]; // Read current value at write address rd
        end else begin
            // Define behavior when reads are not enabled
            output1 = 'x;
            output2 = 'x;
            output3 = 'x;
        end
        // Stack pointer is always readable combinationally
        stackPtr = registers[31];
    end
endmodule

//-------------------------------------------------------------
// memRegMux - Using your original version
//-------------------------------------------------------------
module memRegMux(
    input [4:0] opcode,
    input [63:0] readData,
    input [63:0] aluResult,
    output reg [63:0] regWriteData
);
    // Your original if-else version
    always @(*) begin
        if (opcode == 5'h10) begin // Load uses data from memory
            regWriteData = readData;
        end else begin // Others use ALU result
            regWriteData = aluResult;
        end
    end
endmodule

//-------------------------------------------------------------
// memory - Using your original version
//-------------------------------------------------------------
module memory(
    input [63:0] pc,
    input clk,
    input reset,
    input writeFlag,
    input fetchEnable, // Not directly used in read/write block
    input memEnable,   // Trigger for read/write block
    input memRead,
    input [63:0] writeData,
    input [63:0] rwAddress,
    output reg [63:0] readData,
    output reg [31:0] instruction
);

    reg [7:0] bytes [0:524287];
    integer i;

    // Reset logic
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            for (i = 0; i < 524288; i = i + 1) begin
                bytes[i] <= 8'b0; // Non-blocking
            end
        end
    end

    // Instruction fetch
    assign instruction[7:0] = bytes[pc];
    assign instruction[15:8] = bytes[pc+1];
    assign instruction[23:16] = bytes[pc+2];
    assign instruction[31:24] = bytes[pc+3];

    // Data Read/Write Port (original sensitivity on memEnable)
    // Consider changing sensitivity to always @(*) if this causes issues.
    always @(memEnable) begin
         // Default readData to 'x' when block triggers but read not active
         readData = 'x;

        if (memRead) begin
            readData[7:0] = bytes[rwAddress];       // Blocking
            readData[15:8] = bytes[rwAddress+1];    // Blocking
            readData[23:16] = bytes[rwAddress+2];   // Blocking
            readData[31:24] = bytes[rwAddress+3];   // Blocking
            readData[39:32] = bytes[rwAddress+4];   // Blocking
            readData[47:40] = bytes[rwAddress+5];   // Blocking
            readData[55:48] = bytes[rwAddress+6];   // Blocking
            readData[63:56] = bytes[rwAddress+7];   // Blocking
        end

        if (writeFlag) begin
            bytes[rwAddress] = writeData[7:0];      // Blocking
            bytes[rwAddress+1] = writeData[15:8];   // Blocking
            bytes[rwAddress+2] = writeData[23:16];  // Blocking
            bytes[rwAddress+3] = writeData[31:24];  // Blocking
            bytes[rwAddress+4] = writeData[39:32];  // Blocking
            bytes[rwAddress+5] = writeData[47:40];  // Blocking
            bytes[rwAddress+6] = writeData[55:48];  // Blocking
            bytes[rwAddress+7] = writeData[63:56];  // Blocking
        end
    end
endmodule

//-------------------------------------------------------------
// fetch - Using your original robust version
//-------------------------------------------------------------
module fetch(
    input clk,
    input fetchEnable,
    input reset,
    input [63:0] next_pc,
    output reg [63:0] pc
);
    // Direct synchronous update of PC output register
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 64'h2000; // Non-blocking
        end else if (fetchEnable) begin
            // Check for 'x' condition from v_ok? Unlikely needed here.
            // if (next_pc === 64'hx) pc <= 64'h2000; else
            pc <= next_pc; // Non-blocking
        end
        // If not reset and not enabled, PC holds its value
    end
endmodule

//-------------------------------------------------------------
// aluMemMux - Using your original version
//-------------------------------------------------------------
module aluMemMux(
    input mem_pc,
    input [63:0] memData,
    input [63:0] aluOut, // PC calculated by ALU
    output reg [63:0] newPc // Selected next PC for fetch unit
);
    // Continuous assignment
    assign newPc = mem_pc ? memData : aluOut;
endmodule