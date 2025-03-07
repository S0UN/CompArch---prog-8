
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
    always @(*) begin
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


// 4) Top-Level: tinker_core
module tinker_core(
    input [31:0] instruction
);
    // Wires for decoder
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] literal;
    wire [3:0]  alu_op;
    wire        is_immediate, reg_write_enable, is_float;
    
    // Wires for register file
    wire [63:0] rs_data, rt_data;
    
    // Wires for ALU/FPU result
    wire [63:0] result;
    
    // Instantiate decoder
    instruction_decoder decoder(
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
    
    // Instantiate register file
    register_file reg_file(
        .rs_addr(rs),
        .rt_addr(rt),
        .rd_addr(rd),
        .write_data(result),
        .write_enable(reg_write_enable),
        .rs_data(rs_data),
        .rt_data(rt_data)
    );
    
    // Construct second operand: if immediate, sign-extend literal
    wire [63:0] imm_ext = {{52{literal[11]}}, literal};
    wire [63:0] operand_b = is_immediate ? imm_ext : rt_data;
    
    // ALU/FPU
    alu_fpu alu_unit(
        .a(rs_data),
        .b(operand_b),
        .op(alu_op),
        .is_float(is_float),
        .result(result)
    );
endmodule
