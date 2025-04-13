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

    // Instantiate instructionDecoder (Modified logic will be used below)
    instructionDecoder id(
        .instructionLine(instructionLine),
        .clk(clk),
        .rst(reset),
        .aluReady(aluReady),
        .memEnable(memEnable), // This output is now directly controlled by the FSM combinational logic
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

    // Instantiate fetch (Modified logic will be used below)
    fetch fetch_unit(
        .clk(clk),
        .fetchEnable(fetchEnable),
        .reset(reset),
        .next_pc(next_pc),
        .pc(pc)
    );

    // Instantiate alu (Assumed to be correct as per v_ok)
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

    // Instantiate regLitMux (Assumed to be correct as per v_ok)
    regLitMux reg_lit_mux(
        .sel(opcode),
        .reg1(rt_val),
        .lit(lit),
        .out(input2)
    );

    // Instantiate registerFile (Modified logic will be used below)
    registerFile reg_file(
        .data(regWriteData),
        .read1(rs),
        .read2(rt),
        .write(rd),
        .reset(reset), // reset input is not used in v_ok's logic besides initial
        .clk(clk),     // clk input is not used in v_ok's read/write logic
        .regReadEnable(regReadEnable),
        .regWriteEnable(regWriteEnable),
        .output1(rs_val),
        .output2(rt_val),
        .output3(rd_val),
        .stackPtr(stackPtr)
    );

    // Instantiate aluMemMux (Assumed to be correct as per v_ok)
    aluMemMux alu_mem_mux(
        .mem_pc(mem_pc),
        .memData(readData),
        .aluOut(alu_pc),
        .newPc(next_pc)
    );

    // Instantiate memory (Assumed to be correct as per v_ok)
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

    // Instantiate memRegMux (Assumed to be correct as per v_ok)
    memRegMux mem_reg_mux(
        .opcode(opcode),
        .readData(readData),
        .aluResult(aluOut),
        .regWriteData(regWriteData)
    );

endmodule

//-------------------------------------------------------------
// instructionDecoder - Modified to match v_ok structure
//-------------------------------------------------------------
module instructionDecoder(
    input [31:0] instructionLine,
    input clk,
    input rst,
    input aluReady,
    // No memEnable input needed based on v_ok logic
    output reg fetchEnable,
    output reg memEnable, // Now directly driven by state
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg aluEnable,
    output reg regReadEnable,
    output reg regWriteEnable
);
    // Using numeric states like v_ok
    reg [2:0] state; // Current state register
    reg [4:0] opcodeReg; // Latched opcode for state transition logic

    // State register reset (matches v_ok)
    always @(posedge rst) begin
        if (rst) state <= 3'b0;
    end

    // State transition logic (matches v_ok)
    always @(posedge clk) begin
        // Default assignments for signals controlled by state (optional but good practice)
        // Note: v_ok placed these inside the clocked block, let's match that.
        aluEnable <= 0;
        regWriteEnable <= 0;
        regReadEnable <= 0;
        memEnable <= 0; // Controlled combinationally below now
        fetchEnable <= 0; // Controlled combinationally below now

        case (state)
            3'b0: begin // State 0 -> State 1 (Fetch -> Decode)
                state <= 3'b1;
            end
            3'b1: begin // State 1 -> State 2 (Decode -> Execute)
                 state <= 3'b10;
            end
            3'b10: begin // State 2 (Execute) -> Next State
                if (!aluReady) state <= 3'b10; // Wait if ALU not ready
                else if (opcodeReg == 5'h10 || opcodeReg == 5'h13 || opcodeReg == 5'h0C || opcodeReg == 5'h0D) begin // mem instructions
                    state <= 3'b11; // -> State 3 (Memory Access)
                end
                else if (opcodeReg == 5'h08 || opcodeReg == 5'h09 || opcodeReg == 5'h0A || opcodeReg == 5'h0B || opcodeReg == 5'h0E) begin // branch instructions
                    state <= 3'b0; // -> State 0 (Fetch)
                end
                 else if (opcodeReg == 5'h0F) begin // Halt instruction
                    state <= 3'b0; // -> State 0 (Fetch), but core should halt via ALU signal
                 end
                else state <= 3'b100; // reg write instructions -> State 4 (Write Back)
            end
            3'b11: begin // State 3 (Memory Access) -> Next State
                if (opcodeReg == 5'h10) state <= 3'b100; // Load needs write back -> State 4
                else state <= 3'b0; // Others go back to fetch -> State 0
            end
            3'b100: begin // State 4 (Write Back) -> Next State
                 state <= 3'b0; // -> State 0 (Fetch)
            end
            default: state <= 3'b0; // Default to fetch state
        endcase
    end

    // Combinational logic for outputs and decoding (matches v_ok)
    always @(*) begin
        // Default values for combinational outputs
        fetchEnable = 1'b0;
        memEnable = 1'b0;
        literal = 64'b0;
        rd = 5'b0;
        rs = 5'b0;
        rt = 5'b0;
        opcode = 5'b0;
        aluEnable = 1'b0;
        regReadEnable = 1'b0;
        regWriteEnable = 1'b0;
        opcodeReg = 5'b0; // Need to assign this intermediate signal too

        case (state)
            3'b0: begin // Fetch State
                fetchEnable = 1;
                memEnable = 1; // Enable memory for instruction fetch
            end
            3'b1: begin // Decode State
                // Decode instruction fields
                opcode = instructionLine[31:27];
                opcodeReg = opcode; // Latch opcode for use in state transitions
                rd = instructionLine[26:22];
                rs = instructionLine[21:17];
                rt = instructionLine[16:12];
                literal = {52'b0, instructionLine[11:0]};

                // Handle immediate instructions where Rd is destination but also used as source 1 index implicitly
                // NOTE: This logic from v_ok might need careful review depending on ISA definition.
                // It assigns rs = rd for certain opcodes.
                case (opcode)
                    5'b11001: rs = rd; // ADDI
                    5'b11011: rs = rd; // SUBI
                    5'b00101: rs = rd; // ASRI
                    5'b00111: rs = rd; // SLRI / SRLI?
                    5'b10010: rs = rd; // LI
                    // Note: v_ok had cases for 5'h10 (Load) and 5'h13 (Store) here too, which seems incorrect.
                    // Sticking to the non-mem opcodes from v_ok's list. If issues persist, this might need revisit.
                    default:; // Keep original rs for other instructions
                endcase

                regReadEnable = 1; // Enable register reads
            end
            3'b10: begin // Execute State
                 aluEnable = 1;
                 // Pass through decoded info needed by ALU/later stages
                 // These are held by the combinational logic based on the latched 'state'
                 opcode = opcodeReg; // Use the latched opcode
                 rd = rd; // These just pass through if needed downstream
                 rs = rs;
                 rt = rt;
                 literal = literal;
            end
            3'b11: begin // Memory Access State
                memEnable = 1; // Enable memory for data access
                 // Pass through decoded info
                 opcode = opcodeReg;
                 rd = rd;
                 rs = rs;
                 rt = rt;
                 literal = literal;
            end
            3'b100: begin // Write Back State
                 regWriteEnable = 1;
                 // Pass through decoded info
                 opcode = opcodeReg;
                 rd = rd;
            end
        endcase
    end

endmodule

//-------------------------------------------------------------
// alu - Assumed correct as per v_ok
//-------------------------------------------------------------
module alu (
    input aluEnable,
    input [4:0] control,
    input [63:0] input1,
    input [63:0] input2,
    input [63:0] rd, // This is rd_val (value from register file output3)
    input [63:0] inputPc,
    input [63:0] r31, // This is stackPtr value
    input clk,
    output reg [63:0] result,
    output reg [63:0] writeData,
    output reg [63:0] rwAddress,
    output reg aluReady, writeFlag, memRead,
    output reg [63:0] pc, // This is alu_pc (calculated next PC)
    output reg hlt,
    output reg mem_pc
);

    real r1, r2, rres;
    assign r1 = $bitstoreal(input1);
    assign r2 = $bitstoreal(input2);

    always @(*) begin
        // Default assignments MUST be inside the 'if(aluEnable)' block
        // or handled in the 'else' block to avoid latches when not enabled.
        if (aluEnable) begin
            // Defaults for this cycle
            hlt = 0;
            pc = inputPc + 4; // Default next PC
            writeData = 0;
            rwAddress = 64'hxxxx_xxxx_xxxx_xxxx; // Default to invalid? v_ok used 2000h. Let's use 'x'.
            writeFlag = 0;
            memRead = 0;
            mem_pc = 0;
            result = 64'b0; // Default result

            case (control)
                5'h18: result = input1 + input2; // ADD
                5'h19: result = input1 + input2; // ADDI (input2 is already immediate)
                5'h1A: result = input1 - input2; // SUB
                5'h1B: result = input1 - input2; // SUBI (input2 is already immediate)
                5'h1C: result = input1 * input2; // MUL
                5'h1D: result = input1 / input2; // DIV
                5'h00: result = input1 & input2; // AND
                5'h01: result = input1 | input2; // OR
                5'h02: result = input1 ^ input2; // XOR
                5'h03: result = ~input1;           // NOT
                5'h04: result = input1 >>> input2; // ASR (Arithmetic Shift Right)
                5'h05: result = input1 >>> input2; // ASRI (input2 is already immediate)
                5'h06: result = input1 << input2;  // SL (Shift Left)
                5'h07: result = input1 << input2;  // SLI (input2 is already immediate)
                5'h10: begin // MOV (Load)
                    result = 64'b0; // Data comes from memory later
                    rwAddress = input1 + input2; // Base(rs) + Offset(imm/rt)
                    memRead = 1;
                end
                5'h11: result = input1; // MOV (Register to Register)
                5'h12: result = {input1[63:12], input2[11:0]}; // LI (Load Immediate)
                5'h13: begin // MOV (Store)
                    result = 64'b0; // No register result for store
                    rwAddress = rd + input2; // Base(rd_val) + Offset(imm)
                    writeData = input1;      // Data to store comes from rs_val
                    writeFlag = 1;
                end
                5'h14: begin // FADD
                    rres = r1 + r2;
                    result = $realtobits(rres);
                end
                5'h15: begin // FSUB
                    rres = r1 - r2;
                    result = $realtobits(rres);
                end
                5'h16: begin // FMUL
                    rres = r1 * r2;
                    result = $realtobits(rres);
                end
                5'h17: begin // FDIV
                    rres = r1 / r2;
                    result = $realtobits(rres);
                end
                5'h08: begin // JMPR (Jump Register)
                    pc = rd; // Target is rd_val
                    result = 64'b0;
                end
                5'h09: begin // JMPA (Jump Absolute Address)
                    pc = inputPc + rd; // Target is PC + rd_val
                    result = 64'b0;
                end
                5'h0A: begin // JMPAI (Jump Absolute Address Immediate)
                    pc = inputPc + $signed(input2); // Target is PC + immediate
                    result = 64'b0;
                end
                5'h0B: begin // BRNZ (Branch if Not Zero)
                    pc = (input1 != 64'b0) ? rd : (inputPc + 4); // Target(rd_val) if rs_val != 0
                    result = 64'b0;
                end
                5'h0C: begin // CALL
                    pc = rd; // Target is rd_val
                    result = 64'b0;
                    writeData = inputPc + 4; // Return address
                    rwAddress = r31 - 8;     // Push onto stack (SP value from reg file)
                    writeFlag = 1;
                end
                5'h0D: begin // RETURN
                    result = 64'b0;
                    rwAddress = r31 - 8; // Address to read return PC from stack
                    memRead = 1;
                    mem_pc = 1; // Indicate next PC comes from memory data via aluMemMux
                    pc = inputPc; // PC value itself isn't updated here, comes from mem via mux
                end
                5'h0E: begin // BRGT (Branch if Greater Than)
                    pc = ($signed(input1) > $signed(input2)) ? rd : (inputPc + 4); // Target(rd_val) if rs_val > rt_val/imm
                    result = 64'b0;
                end
                5'h0F: begin // HLT
                    result = 64'b0;
                    hlt = 1;
                    pc = inputPc; // PC stops incrementing
                end
                default: begin
                    // Undefined opcode - Treat as NOP?
                    result = 64'b0;
                    pc = inputPc + 4;
                 end
            endcase
            aluReady = 1; // Signal ALU operation is complete for this cycle
        end else begin
            // Outputs when ALU is not enabled
            hlt = 0;
            aluReady = 0;
            // Assign 'x' or previous value? Let's assign 'x' to avoid implied latches.
            result = 'x;
            writeData = 'x;
            rwAddress = 'x;
            writeFlag = 1'b0; // Default inactive
            memRead = 1'b0;   // Default inactive
            pc = 'x;
            mem_pc = 1'b0;  // Default inactive
        end
    end
endmodule

//-------------------------------------------------------------
// regLitMux - Assumed correct as per v_ok
//-------------------------------------------------------------
module regLitMux (
    input [4:0] sel,
    input [63:0] reg1, // rt_val
    input [63:0] lit,
    output reg [63:0] out // input2 for ALU
);
    // Using case statement like v_ok
    always @(*) begin
        case (sel)
            // Opcodes that use immediate for ALU input 2
            5'h19, // ADDI
            5'h1B, // SUBI
            5'h05, // ASRI
            5'h07, // SLI / SRLI?
            5'h12, // LI (uses immediate bits)
            5'h10, // MOV Load (uses immediate offset)
            5'h13, // MOV Store (uses immediate offset)
            5'h0A, // JMPAI (uses immediate offset for calculation)
            5'h0E: // BRGT (can compare reg with immediate) - Check if rt or immediate is used
                   // v_ok reglitmux included 5'b01010 which is 5'h0A, not 5'h0E.
                   // Let's assume comparison uses Rt if not immediate.
                   // Need to check ALU logic for 5'h0E (BRGT) - it uses input1 > input2.
                   // If sel=5'h0E, should input2 be lit or reg1(rt_val)?
                   // Assuming register comparison if rt field is non-zero? ISA dependent.
                   // v_ok reglitmux included 5'b01010 (JMPAI?) not BRGT.
                   // Reverting to exact match from v_ok reglitmux provided earlier.
                   out = lit;

            // Opcodes listed in v_ok reglitmux check (binary)
            5'b11001, // ADDI (19h)
            5'b11011, // SUBI (1Bh)
            5'b00101, // ASRI (05h)
            5'b00111, // SLRI (07h)
            5'b10010, // LI   (12h)
            // 5'b01010, // This was in v_ok reglitmux, maps to 0Ah (JMPAI), already covered
            5'h10, // MOV Load
            5'h13: // MOV Store
                   out = lit;

            default: // All other opcodes use register rt_val for ALU input 2
                   out = reg1;
        endcase
    end
endmodule

//-------------------------------------------------------------
// registerFile - Modified to match v_ok logic
//-------------------------------------------------------------
module registerFile (
    input [63:0] data, // Data to write
    input [4:0] read1, // rs index
    input [4:0] read2, // rt index
    input [4:0] write, // rd index
    input reset,       // Not used in active logic of v_ok
    input clk,         // Not used in active logic of v_ok
    input regReadEnable,
    input regWriteEnable,
    output reg [63:0] output1, // rs_val
    output reg [63:0] output2, // rt_val
    output reg [63:0] output3, // rd_val (value read from destination reg index)
    output reg [63:0] stackPtr // r31 value
);

    reg [63:0] registers [0:31];
    integer i;

    // Initialization (matches v_ok)
    initial begin
        for (i = 0; i < 31; i = i + 1) begin
            registers[i] = 64'b0; // Use blocking like v_ok initial (though non-blocking often preferred)
        end
        registers[31] = 64'd524288; // Use blocking like v_ok initial
    end

    // Continuous assignment for stack pointer (matches v_ok)
    assign stackPtr = registers[31];

    // Combinational Read and Write Logic (matches v_ok)
    always @(*) begin
        // Default assignments for outputs (to avoid latches if read not enabled)
        // v_ok didn't explicitly handle this, implying outputs held value.
        // Let's add default assignments to avoid latch warnings, outputting 'x'.
        output1 = 'x;
        output2 = 'x;
        output3 = 'x;

        // Read Ports: Assign outputs based on read indices if enabled
        if (regReadEnable) begin
            output1 = registers[read1]; // Blocking assignment
            output2 = registers[read2]; // Blocking assignment
            output3 = registers[write]; // Read current value of dest reg using write index
        end

        // Write Port: Update register content if write is enabled (Combinational Latch behavior)
        if (regWriteEnable) begin
            // Optional: Prevent writing to R0 if it should be hardwired zero
             // if (write != 5'b00000)
            registers[write] = data; // Blocking assignment (Latch write)
        end
    end

endmodule

//-------------------------------------------------------------
// memRegMux - Assumed correct as per v_ok
//-------------------------------------------------------------
module memRegMux(
    input [4:0] opcode,
    input [63:0] readData,
    input [63:0] aluResult,
    output reg [63:0] regWriteData
);
    // Using case statement like v_ok
    always @(*) begin
        case (opcode)
            5'h10: begin // Load instruction gets data from memory
                regWriteData = readData;
            end
            default: begin // Others get data from ALU result
                regWriteData = aluResult;
            end
        endcase
    end
endmodule

//-------------------------------------------------------------
// memory - Assumed correct as per v_ok
//-------------------------------------------------------------
module memory(
    input [63:0] pc,
    input clk,
    input reset,
    input writeFlag,
    input fetchEnable, // Not directly used by v_ok read/write logic
    input memEnable,   // Sensitivity trigger for read/write in v_ok
    input memRead,
    input [63:0] writeData,
    input [63:0] rwAddress,
    output reg [63:0] readData,
    output reg [31:0] instruction
);

    reg [7:0] bytes [0:524287];
    integer i;

    // Reset logic (matches v_ok)
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            for (i = 0; i < 524288; i = i + 1) begin
                bytes[i] <= 8'b0; // Non-blocking
            end
        end
    end

    // Instruction fetch (matches v_ok)
    assign instruction[7:0] = bytes[pc];
    assign instruction[15:8] = bytes[pc+1];
    assign instruction[23:16] = bytes[pc+2];
    assign instruction[31:24] = bytes[pc+3];

    // Data Read/Write Port (matches v_ok unusual sensitivity)
    always @(memEnable) begin
         // Default readData to avoid latches when not reading?
         // v_ok didn't specify. Let's add 'x'.
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
        // Note: Writes happen *concurrently* with reads if both flags are high, based on this structure.
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
// fetch - Modified to match v_ok structure
//-------------------------------------------------------------
module fetch(
    input clk,
    input fetchEnable,
    input reset,
    input [63:0] next_pc,
    output reg [63:0] pc // This is the output Program Counter
);
    // Internal register (matches v_ok)
    reg [63:0] current_programCounter;

    // Combinational output assignment (matches v_ok)
    always @(*) begin
        if (fetchEnable) begin
             pc = current_programCounter;
        end else begin
             // What should pc be when fetch is not enabled?
             // Assign 'x' to avoid latch warnings.
             pc = 'x;
        end
    end

    // Clocked update of internal register (matches v_ok)
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            current_programCounter <= 64'h2000; // Non-blocking
        end else if (fetchEnable) begin
            // Check for 'x' from v_ok logic (purpose unclear, but matching it)
            if (next_pc === 64'hx) begin
                // v_ok used blocking assignment here, which is unusual in clocked block
                // Let's use non-blocking for consistency in clocked block.
                current_programCounter <= 64'h2000;
            end else begin
                current_programCounter <= next_pc; // Non-blocking
            end
        end
        // If not reset and not fetchEnable, internal register holds value
    end

endmodule

//-------------------------------------------------------------
// aluMemMux - Assumed correct as per v_ok
//-------------------------------------------------------------
module aluMemMux(
    input mem_pc, // From ALU, indicates RET instruction
    input [63:0] memData, // Return address from Memory
    input [63:0] aluOut, // Calculated PC from ALU
    output reg [63:0] newPc // Selected PC for Fetch unit
);
    // Logic matches v_ok
    assign newPc = mem_pc ? memData : aluOut;
endmodule