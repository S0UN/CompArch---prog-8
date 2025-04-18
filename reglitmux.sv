module tinker_core (
    input clk,
    input reset,
    output hlt
);
    // Instruction-related signals
    logic [31:0] instruction_word;
    logic [4:0] instruction_type;
    logic [4:0] target_register, src_reg_a, src_reg_b;
    
    // Address-related signals
    logic [63:0] current_address, next_address, alu_calculated_address;
    
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
        .next_pc(next_address),
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
        .rs(src_reg_a),
        .rt(src_reg_b),
        .opcode(instruction_type),
        .aluFlag(computation_start),
        .fetchFlag(fetch_start),
        .regReadFlag(register_read_start),
        .regWriteFlag(register_write_start)
    );

    // Register file - manages CPU registers
    registerFile reg_file(
        .data(register_input_value),
        .read1(src_reg_a),
        .read2(src_reg_b),
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
        .aluFlag(computation_start),
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
        .newPc(next_address)
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

module de_ex_register(
    input clk,
    input reg [63:0] literal_in,
    input reg [4:0] rd_in,
    input reg [4:0] rs_in,
    input reg [4:0] rt_in,
    input reg [4:0] opcode_in,
    input reg alu_enable_in,             
    input reg mem_read_in,               
    input reg mem_write_in,              
    input reg reg_write_in,               
    input reg mem_to_reg_in,             
    input reg branch_taken_in,          
    input reg mem_pc_in,
    output reg [63:0] literal_out,
    output reg [4:0] rd_out,
    output reg [4:0] rs_out,
    output reg [4:0] rt_out,
    output reg [4:0] opcode_out,
    output reg alu_enable_out,             
    output reg mem_read_out,               
    output reg mem_write_out,              
    output reg reg_write_out,               
    output reg mem_to_reg_out,             
    output reg branch_taken_out,          
    output reg mem_pc_out
);
    always @(posedge clk) begin
        opcode_out <= opcode_in;
        rd_out <= rd_in;
        rs_out <= rs_in;
        rt_out <= rt_in;
        literal_out <= literal_in;
        alu_enable_out <= alu_enable_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        branch_taken_out <= branch_taken_in;
        mem_pc_out <= mem_pc_in;
    end
endmodule

module instructionDecoder (
    input [31:0] instructionLine,
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg alu_enable,             
    output reg mem_read,               
    output reg mem_write,              
    output reg reg_write,               
    output reg mem_to_reg,             
    output reg branch_taken,          
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

    // Internal state register
    reg [4:0] op;

    opcode = instructionLine[31:27];
    rd = instructionLine[26:22];
    rs = instructionLine[21:17];
    rt = instructionLine[16:12];
    literal = {52'b0, instructionLine[11:0]};
    // Output and control signal generation

    //need to implement stalling here
    always @(*) begin
    case (opcode)
            // Integer Arithmetic Instructions
            ADD, ADDI, SUB, SUBI, MUL, DIV: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            
            // Logic Instructions
            AND, OR, XOR, NOT: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
            
            // Shift Instructions
            SHFTR, SHFTRI, SHFTL, SHFTLI: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end
           // Data Movement Instructions
            MOV_MEM: begin
                alu_enable = 1;  // For address calculation: rs + L
                mem_read = 1;
                reg_write = 1;
                mem_to_reg = 1;
            end
            MOV_REG: begin
                alu_enable = 1;  // ALU passes rs to rd
                reg_write = 1;
                mem_to_reg = 0;
            end
            MOV_LIT: begin
                alu_enable = 1;  // ALU passes literal to rd
                reg_write = 1;
                mem_to_reg = 0;
            end
            MOV_STR: begin
                alu_enable = 1;  // For address calculation: rd + L
                mem_write = 1;
            end
            // Floating-Point Instructions
            ADDF, SUBF, MULF, DIVF: begin
                alu_enable = 1;
                reg_write = 1;
                mem_to_reg = 0;
            end

            // Control (Branch) Instructions
            BR, BRR, BRRI, BRNZ, BRGT: begin
                alu_enable = 1;  // For branch target calculation
                branch_taken = 1; // Set here; actual resolution may occur in EXECUTE
            end

            // Subroutine Instructions
            CALL: begin
                alu_enable = 1;  // For branch target calculation
                mem_write = 1;   // Save return address
                branch_taken = 1;
            end
            RETURN: begin
                mem_read = 1;    // Load return address from memory
                mem_pc = 1;      // Update PC from memory
            end
            
            // Privileged Instructions
            PRIV: begin
                // All flags remain 0 (e.g., for halt)
            end            
            default: begin
                // Default case (should not happen but added for safety)
                result = 64'b0;
            end
        endcase

    case (opcode)
        ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: rs = rd;
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
     @(*) begin
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

module ex_mem_register(
    input clk,
    input reg [63:0] result_in,
    input reg [63:0] writeData_in,
    input reg [63:0] rwAddress_in,
    input reg [63:0] pc_in,
    input reg hlt_in,
    input reg mem_read_in,
    input reg mem_write_in,
    input reg reg_write_in,
    input reg mem_to_reg_in,
    input reg branch_taken_in,
    input reg mem_pc_in,
    output reg [63:0] result_out,
    output reg [63:0] writeData_out,
    output reg [63:00] rwAddress_out,
    output reg [63:0] pc_out,
    output reg hlt_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_out,
    output reg mem_pc_out
);
    always @(posedge clk) begin
        result_out <= result_in;
        writeData_out <= writeData_in;
        rwAddress_out <= rwAddress_in;
        pc_out <= pc_in;
        hlt_out <= hlt_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        branch_taken_out <= branch_taken_in;
        mem_pc_out <= mem_pc_in;
    end
endmodule

module alu (
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,
    input mem_to_reg_in,
    input mem_pc_in,
    input [4:0] opcode,
    input [63:0] input1,
    input [63:0] input2,
    input [63:0] rd,
    input [63:0] inputPc,
    input [63:0] r31,
    output reg [63:0] result,
    output reg [63:0] writeData,
    output reg [63:0] rwAddress,
    output reg [63:0] pc,
    output reg hlt,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_out,
    output reg mem_pc_out
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
        // Pass through flags from DE/EX to EX/MEM
        mem_read_out = mem_read_in;
        mem_write_out = mem_write_in;
        reg_write_out = reg_write_in;
        mem_to_reg_out = mem_to_reg_in;
        mem_pc_out = mem_pc_in;
        
        // Default values
        result = 64'b0;
        pc = inputPc + 4;  // Default: increment PC by 4
        writeData = 64'b0;
        rwAddress = MEM_BASE_ADDR;
        hlt = 1'b0;
        branch_taken_out = 1'b0;

        if (alu_enable_in) begin
            case (opcode)
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
                MOV_MEM: rwAddress = input1 + input2;
                MOV_REG: result = input1;
                MOV_LIT: result = {input1[63:12], input2[11:0]};
                MOV_STR: begin
                    rwAddress = rd + input2;
                    writeData = input1;
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
                BR: begin
                    pc = rd;
                    branch_taken_out = 1;
                end
                BRR: begin
                    pc = inputPc + rd;
                    branch_taken_out = 1;
                end
                BRRI: begin
                    pc = inputPc + $signed(input2);
                    branch_taken_out = 1;
                end
                BRNZ: begin
                    if (input1 != 0) begin
                        pc = rd;
                        branch_taken_out = 1;
                    end else begin
                        pc = inputPc + 4;
                        branch_taken_out = 0;
                    end
                end
                BRGT: begin
                    if (input1 > input2) begin
                        pc = rd;
                        branch_taken_out = 1;
                    end else begin
                        pc = inputPc + 4;
                        branch_taken_out = 0;
                    end
                end
                
                // Subroutine Instructions
                CALL: begin
                    pc = rd;
                    writeData = inputPc + 4;  // Save return address
                    rwAddress = r31 - 8;
                    branch_taken_out = 1;
                end
                RETURN: begin
                    rwAddress = r31 - 8;
                    branch_taken_out = 1;  // Treat as a branch for simplicity
                end
                
                // Privileged Instructions
                PRIV: hlt = 1'b1;  // Assuming PRIV is used as HALT here
                
                default: begin
                    // Default case (should not happen but added for safety)
                    result = 64'b0;
                end
            endcase
        end
    end
endmodule