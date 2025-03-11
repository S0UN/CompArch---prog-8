
// 1) Instruction Decoder
module instruction_decoder(
    input  [31:0] instruction,
    output reg [4:0]  opcode, 
    output reg [4:0]  rd,    
    output reg [4:0]  rs,    
    output reg [4:0]  rt,      
    output reg [11:0] literal, 
    
    // 4-bit ALU opcode
    output reg [3:0]  alu_op,
    
    // Control signals
    output reg        is_immediate,
    output reg        reg_write_enable,
    output reg        is_float
);

    // ALU operation codes (4 bits)
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_MUL  = 4'b0010;
    localparam ALU_DIV  = 4'b0011;
    localparam ALU_SHR  = 4'b0100; 
    localparam ALU_SHL  = 4'b0101; 
    localparam ALU_AND  = 4'b0110;
    localparam ALU_OR   = 4'b0111;
    localparam ALU_XOR  = 4'b1000;
    localparam ALU_NOT  = 4'b1001;

    always @(*) begin
        // Extract fields from instruction
        opcode   = instruction[31:27];
        rd       = instruction[26:22];
        rs       = instruction[21:17];
        rt       = instruction[16:12];
        literal  = instruction[11:0];
        
        // Default control signals
        is_immediate     = 0;
        reg_write_enable = 1;
        is_float         = 0;
        alu_op           = ALU_ADD; // default to ADD, can be overridden below

        case (opcode)
            // Integer Arithmetic (From Tinker Manual: 0x18..0x1D)
            5'b11000: alu_op = ALU_ADD;  // add  (0x18)
            5'b11001: begin             // addi (0x19)
                alu_op       = ALU_ADD;
                is_immediate = 1;
            end
            5'b11010: alu_op = ALU_SUB;  // sub  (0x1A)
            5'b11011: begin             // subi (0x1B)
                alu_op       = ALU_SUB;
                is_immediate = 1;
            end
            5'b11100: alu_op = ALU_MUL;  // mul  (0x1C)
            5'b11101: alu_op = ALU_DIV;  // div  (0x1D)
            
            // Logic instructions (0x0..0x3)
            5'b00000: alu_op = ALU_AND;  // and (0x0)
            5'b00001: alu_op = ALU_OR;   // or  (0x1)
            5'b00010: alu_op = ALU_XOR;  // xor (0x2)
            5'b00011: alu_op = ALU_NOT;  // not (0x3)
            
            // Shift instructions (0x4..0x7)
            5'b00100: alu_op = ALU_SHR;  // shftr (0x4)
            5'b00101: begin             // shftri (0x5)
                alu_op       = ALU_SHR;
                is_immediate = 1;
            end
            5'b00110: alu_op = ALU_SHL;  // shftl (0x6)
            5'b00111: begin             // shftli (0x7)
                alu_op       = ALU_SHL;
                is_immediate = 1;
            end
            
            // Data Movement (0x11..0x12)
            5'b10001: begin // mov rd, rs (0x11)
                alu_op = ALU_ADD;
            end
            5'b10010: begin // mov rd, L (0x12)
                alu_op       = ALU_ADD;
                is_immediate = 1;
            end
            
            // Floating-Point (0x14..0x17)
            5'b10100: begin // addf (0x14)
                alu_op   = ALU_ADD; 
                is_float = 1;
            end
            5'b10101: begin // subf (0x15)
                alu_op   = ALU_SUB;
                is_float = 1;
            end
            5'b10110: begin // mulf (0x16)
                alu_op   = ALU_MUL;
                is_float = 1;
            end
            5'b10111: begin // divf (0x17)
                alu_op   = ALU_DIV;
                is_float = 1;
            end
            
            default: begin
                // Unknown opcode => disable writes
                reg_write_enable = 0;
            end
        endcase
    end
endmodule

module register_file(
    input clk,
    input  [4:0] rs_addr,
    input  [4:0] rt_addr,
    input  [4:0] rd_addr,
    input  [63:0] write_data,
    input         write_enable,
    output [63:0] rs_data,
    output [63:0] rt_data
);
    reg [63:0] registers [0:31];
    integer i;
    
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 64'd0;
        end
    end
    
    // Combinational reads
    assign rs_data = registers[rs_addr];
    assign rt_data = registers[rt_addr];
    
    // Combinational write
    always @(posedge clk) begin
        if (write_enable && (rd_addr != 5'd0)) begin
            registers[rd_addr] = write_data;
        end
    end
endmodule


// 3) ALU / FPU Combined
// 4-bit op codes for integer arithmetic, logic, shift, etc.
module alu_fpu(
    input  [63:0] a,
    input  [63:0] b,
    input  [3:0]  op,
    input         is_float,
    output reg [63:0] result
);
    real float_a, float_b, float_res;
    
    // 4-bit ALU codes
    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001;
    localparam ALU_MUL = 4'b0010;
    localparam ALU_DIV = 4'b0011;
    localparam ALU_SHR = 4'b0100; // Shift Right
    localparam ALU_SHL = 4'b0101; // Shift Left
    localparam ALU_AND = 4'b0110;
    localparam ALU_OR  = 4'b0111;
    localparam ALU_XOR = 4'b1000;
    localparam ALU_NOT = 4'b1001;

    always @(*) begin
        if (is_float) begin
            // Floating-Point path
            float_a = $bitstoreal(a);
            float_b = $bitstoreal(b);
            
            case (op)
                ALU_ADD: float_res = float_a + float_b;
                ALU_SUB: float_res = float_a - float_b;
                ALU_MUL: float_res = float_a * float_b;
                ALU_DIV: float_res = float_a / float_b;
                default: float_res = 0.0;
            endcase
            
            result = $realtobits(float_res);
        end
        else begin
            // Integer / Logic path
            case (op)
                ALU_ADD: result = a + b;
                ALU_SUB: result = a - b;
                ALU_MUL: result = a * b;
                ALU_DIV: result = (b != 0) ? (a / b) : 64'd0; 
                ALU_SHR: result = a >> b[5:0];
                ALU_SHL: result = a << b[5:0];
                ALU_AND: result = a & b;
                ALU_OR : result = a | b;
                ALU_XOR: result = a ^ b;
                ALU_NOT: result = ~a;
                default: result = 64'd0;
            endcase
        end
    end
endmodule

module control(
    input clk,
    input reset,
    input branch,
    input [31:0] branch_address,
    output reg [31:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 32'b0;         
        else if (branch)
            pc <= branch_address; 
        else
            pc <= pc + 4;         
    end 
endmodule

module clock_generator(
    output reg clk
);
    initial begin
        clk = 0;
    end
    
    always begin
        #5 clk = ~clk;
    end
endmodule

module memory(
    input clk,
    input mem_read,
    input mem_write,
    input [31:0] address,
    input [31:0] write_data,
    output reg [31:0] read_data
);
    reg [31:0] mem_array [0:131071];


    always @(posedge clk) begin
        if (mem_write)
            mem_array[address[16:2]] <= write_data;  
        if (mem_read)
            read_data <= mem_array[address[16:2]];
    end
endmodule

// Instruction Fetch Stage: connects PC to memory
module instruction_fetch(
    input clk,
    input reset,
    input mem_read,
    output [31:0] instruction,
    input [31:0] pc   // Provided by the program counter
);
    memory inst_mem(
        .clk(clk),
        .mem_read(mem_read),
        .mem_write(1'b0),  // No writes during instruction fetch
        .address(pc),
        .write_data(32'd0),
        .read_data(instruction)
    );
endmodule


// Top-Level Module: tinker_core
module tinker_core(
    input clk,
    input reset
);
    // Program Counter 
    wire [31:0] pc;
    // For now, no branch instructions are implemented.
    wire branch = 1'b0;
    wire [31:0] branch_address = 32'd0;
    
    program_counter pc_inst(
        .clk(clk),
        .reset(reset),
        .branch(branch),
        .branch_address(branch_address),
        .pc(pc)
    );
    
    // Instruction Fetch 
    wire [31:0] instruction;
    instruction_fetch fetch_inst(
        .clk(clk),
        .reset(reset),
        .mem_read(1'b1),
        .instruction(instruction),
        pc(pc))

Instruction Decoder 
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] literal;
    wire [3:0] alu_op;
    wire is_immediate, reg_write_enable, is_float;
    
    instruction_decoder decoder_inst(
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .literal(literal),
        .alu_op(alu_op),
        .is_immediate(is_immediate),
        .reg_write_enable(reg_write_enable),
        .is_float(is_float)
    );
    
    // Register File 
    wire [63:0] rs_data, rt_data;
    register_file reg_file(
        .clk(clk),
        .rs_addr(rs),
        .rt_addr(rt),
        .rd_addr(rd),
        .write_data(result),
        .write_enable(reg_write_enable),
        .rs_data(rs_data),
        .rt_data(rt_data)
    );
    
    // ALU/FPU 
    wire [63:0] result;
    // Select operand B: either sign-extended immediate or rt_data
    wire [63:0] operand_b;
    assign operand_b = is_immediate ? {{52{literal[11]}}, literal} : rt_data;
    
    alu_fpu alu_unit(
        .a(rs_data),
        .b(operand_b),
        .op(alu_op),
        .is_float(is_float),
        .result(result)
    );
endmodule