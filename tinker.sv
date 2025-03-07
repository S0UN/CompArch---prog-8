module tinker_core(
    input [31:0] instruction
);
    // Instruction decoding
    wire [4:0] opcode;
    wire [4:0] rd, rs, rt;
    wire [11:0] literal;
    wire [2:0] alu_op;
    wire is_immediate;
    wire reg_write_enable;
    wire is_float;
    
    // Connections
    wire [63:0] rs_data, rt_data, result;
    
    // Instantiate modules
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
    
    register_file reg_file(
        .rs_addr(rs),
        .rt_addr(rt),
        .rd_addr(rd),
        .write_data(result),
        .write_enable(reg_write_enable),
        .rs_data(rs_data),
        .rt_data(rt_data)
    );
    
    alu_fpu alu(
        .a(rs_data),
        .b(is_immediate ? {{52{1'b0}}, literal} : rt_data),
        .op(alu_op),
        .is_float(is_float),
        .result(result)
    );
    
endmodule

// Instruction Decoder
module instruction_decoder(
    input [31:0] instruction,
    output reg [4:0] opcode,
    output reg [4:0] rd, rs, rt,
    output reg [11:0] literal,
    output reg [2:0] alu_op,
    output reg is_immediate,
    output reg reg_write_enable,
    output reg is_float
);
    always @(*) begin
        // Correct bit positions
        opcode = instruction[31:27];
        rd = instruction[26:22];
        rs = instruction[21:17];
        rt = instruction[16:12];
        literal = instruction[11:0];
        is_immediate = 0;
        reg_write_enable = 1;
        is_float = 0;
        alu_op = 3'b000;
        
        case(opcode)
            // Integer Arithmetic Instructions
            5'b11000: begin // add rd, rs, rt (0x18)
                alu_op = 3'b000;
            end
            5'b11001: begin // addi rd, L (0x19)
                alu_op = 3'b000;
                is_immediate = 1;
            end
            5'b11010: begin // sub rd, rs, rt (0x1a)
                alu_op = 3'b001;
            end
            5'b11011: begin // subi rd, L (0x1b)
                alu_op = 3'b001;
                is_immediate = 1;
            end
            5'b11100: begin // mul rd, rs, rt (0x1c)
                alu_op = 3'b010;
            end
            5'b11101: begin // div rd, rs, rt (0x1d)
                alu_op = 3'b011;
            end
            
            // Logic instructions
            5'b00000: begin // and rd, rs, rt (0x0)
                alu_op = 3'b100;
            end
            5'b00001: begin // or rd, rs, rt (0x1)
                alu_op = 3'b101;
            end
            5'b00010: begin // xor rd, rs, rt (0x2)
                alu_op = 3'b110;
            end
            5'b00011: begin // not rd, rs (0x3)
                alu_op = 3'b111;
            end
            5'b00100: begin // shftr rd, rs, rt (0x4)
<<<<<<< HEAD
                alu_op = 3'b100;
=======
                alu_op = 3'b000;
>>>>>>> eb18a377747c76443e6ee3f2d0061d92ad11e1a5
            end
            5'b00101: begin // shftri rd, L (0x5)
                alu_op = 3'b000;
                is_immediate = 1;
            end
            5'b00110: begin // shftl rd, rs, rt (0x6)
<<<<<<< HEAD
                alu_op = 3'b101;
=======
                alu_op = 3'b001;
>>>>>>> eb18a377747c76443e6ee3f2d0061d92ad11e1a5
            end
            5'b00111: begin // shftli rd, L (0x7)
                alu_op = 3'b001;
                is_immediate = 1;
            end
            
            // Data Movement Instructions (excluding memory access)
            5'b10001: begin // mov rd, rs (0x11)
                alu_op = 3'b000; // Use ALU as pass-through
                rt = 5'b0;
            end
            5'b10010: begin // mov rd, L (0x12)
                alu_op = 3'b000;
                is_immediate = 1;
            end
            
            // Floating Point Instructions
            5'b10100: begin // addf rd, rs, rt (0x14)
                alu_op = 3'b000;
                is_float = 1;
            end
            5'b10101: begin // subf rd, rs, rt (0x15)
                alu_op = 3'b001;
                is_float = 1;
            end
            5'b10110: begin // mulf rd, rs, rt (0x16)
                alu_op = 3'b010;
                is_float = 1;
            end
            5'b10111: begin // divf rd, rs, rt (0x17)
                alu_op = 3'b011;
                is_float = 1;
            end
            
            default: begin
                reg_write_enable = 0;
            end
        endcase
    end
endmodule

// Register File
module register_file(
    input [4:0] rs_addr,
    input [4:0] rt_addr,
    input [4:0] rd_addr,
    input [63:0] write_data,
    input write_enable,
    output [63:0] rs_data,
    output [63:0] rt_data
);
    reg [63:0] registers [0:31];
    
    // Initialize registers (for simulation)
    initial begin
        for (int i = 0; i < 32; i = i + 1) begin
            registers[i] = 64'h0;
        end
    end
    
    // Read operations are combinational
    assign rs_data = registers[rs_addr];
    assign rt_data = registers[rt_addr];
    
    // Write operation is also combinational (no clock)
    always @(*) begin
        if (write_enable && rd_addr != 0) begin
            registers[rd_addr] = write_data;
        end
    end
endmodule

// ALU and FPU
module alu_fpu(
    input [63:0] a,
    input [63:0] b,
    input [2:0] op,
    input is_float,
    output reg [63:0] result
);
    // Helper for floating-point operations
    real float_a, float_b, float_result;
    
    always @(*) begin
        if (is_float) begin
            // Convert bit patterns to real values for floating-point operations
            float_a = $bitstoreal(a);
            float_b = $bitstoreal(b);
            
            case(op)
                3'b000: float_result = float_a + float_b;  // Add
                3'b001: float_result = float_a - float_b;  // Subtract
                3'b010: float_result = float_a * float_b;  // Multiply
                3'b011: float_result = float_a / float_b;  // Divide

                default: float_result = 0.0;
            endcase
            
            result = $realtobits(float_result);
        end
        else begin
            case(op)
                // Integer operations
                3'b000: result = a + b;  // Add
                3'b001: result = a - b;  // Subtract
                3'b010: result = a * b;  // Multiply
                3'b011: result = a / b;  // Divide
    
                3'b100: result = a >> b[5:0]; // SHIFT RIGHT
                3'b101: result = a << b[5:0]; // SHIFT LEFT

                // Logic operations
                3'b100: result = a & b;  // AND
                3'b101: result = a | b;  // OR
                3'b110: result = a ^ b;  // XOR
                3'b111: result = ~a;     // NOT
                
                default: result = 0;
            endcase
        end
    end
endmodule