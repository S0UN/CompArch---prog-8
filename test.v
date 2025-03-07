`include "./hdl/instructionDecoder.sv"

module testInstructionDecoder();

    reg[31:0] instructionLine;
    reg[63:0] literal;
    reg[4:0] rd;
    reg[4:0] rs;
    reg[4:0] rt;
    reg[4:0] opcode;

    reg[63:0] expected_literal;
    reg[4:0] expected_rd;
    reg[4:0] expected_rs;
    reg[4:0] expected_rt;
    reg[4:0] expected_opcode;

    instructionDecoder my_id(.instructionLine(instructionLine), .literal(literal), .rd(rd), .rs(rs), .rt(rt), .opcode(opcode));

    initial 
    begin
        // Test 1:
        instructionLine = 32'b01011110101010101010000001011111;
        expected_literal = 64'b000001011111;
        expected_opcode = 5'b01011;
        expected_rd = 5'b11010;
        expected_rs = 5'b10101;
        expected_rt = 5'b01010;
        #10;
        $display("expected_literal = %b, literal = %b, expected_opcode = %b, opcode = %b, expected_rd = %b, rd = %b, expected_rs = %b, rs = %b, expected_rt = %b, rt = %b\n", expected_literal, literal, expected_opcode, opcode, expected_rd, rd, expected_rs, rs, expected_rt, rt);

        // Test 2:
        instructionLine = 32'b10101111100011000000000101110101;
        expected_literal = 12'b000101110101;
        expected_opcode = 5'b10101;
        expected_rd = 5'b11110;
        expected_rs = 5'b00110;
        expected_rt = 5'b00000;
        #10;
        $display("expected_literal = %b, literal = %b, expected_opcode = %b, opcode = %b, expected_rd = %b, rd = %b, expected_rs = %b, rs = %b, expected_rt = %b, rt = %b\n", expected_literal, literal, expected_opcode, opcode, expected_rd, rd, expected_rs, rs, expected_rt, rt);

        // Test 3 (addi cause uses switch statement for literal)
        instructionLine = 32'b11001000100000000000000000000110;
        expected_literal = 12'b000000000110;
        expected_opcode = 5'b11001;
        expected_rd = 5'b00010;
        expected_rs = 5'b00010;
        expected_rt = 5'b00000;
        #10;
        $display("expected_literal = %b, literal = %b, expected_opcode = %b, opcode = %b, expected_rd = %b, rd = %b, expected_rs = %b, rs = %b, expected_rt = %b, rt = %b\n", expected_literal, literal, expected_opcode, opcode, expected_rd, rd, expected_rs, rs, expected_rt, rt);
        $finish;
    end

endmodule
`include "./hdl/instructionDecoder.sv"

module testInstructionDecoder();

    reg[31:0] instructionLine;
    reg[63:0] literal;
    reg[4:0] rd;
    reg[4:0] rs;
    reg[4:0] rt;
    reg[4:0] opcode;

    reg[63:0] expected_literal;
    reg[4:0] expected_rd;
    reg[4:0] expected_rs;
    reg[4:0] expected_rt;
    reg[4:0] expected_opcode;

    instructionDecoder my_id(.instructionLine(instructionLine), .literal(literal), .rd(rd), .rs(rs), .rt(rt), .opcode(opcode));

    initial 
    begin
        // Test 1:
        instructionLine = 32'b01011110101010101010000001011111;
        expected_literal = 64'b000001011111;
        expected_opcode = 5'b01011;
        expected_rd = 5'b11010;
        expected_rs = 5'b10101;
        expected_rt = 5'b01010;
        #10;
        $display("expected_literal = %b, literal = %b, expected_opcode = %b, opcode = %b, expected_rd = %b, rd = %b, expected_rs = %b, rs = %b, expected_rt = %b, rt = %b\n", expected_literal, literal, expected_opcode, opcode, expected_rd, rd, expected_rs, rs, expected_rt, rt);

        // Test 2:
        instructionLine = 32'b10101111100011000000000101110101;
        expected_literal = 12'b000101110101;
        expected_opcode = 5'b10101;
        expected_rd = 5'b11110;
        expected_rs = 5'b00110;
        expected_rt = 5'b00000;
        #10;
        $display("expected_literal = %b, literal = %b, expected_opcode = %b, opcode = %b, expected_rd = %b, rd = %b, expected_rs = %b, rs = %b, expected_rt = %b, rt = %b\n", expected_literal, literal, expected_opcode, opcode, expected_rd, rd, expected_rs, rs, expected_rt, rt);

        // Test 3 (addi cause uses switch statement for literal)
        instructionLine = 32'b11001000100000000000000000000110;
        expected_literal = 12'b000000000110;
        expected_opcode = 5'b11001;
        expected_rd = 5'b00010;
        expected_rs = 5'b00010;
        expected_rt = 5'b00000;
        #10;
        $display("expected_literal = %b, literal = %b, expected_opcode = %b, opcode = %b, expected_rd = %b, rd = %b, expected_rs = %b, rs = %b, expected_rt = %b, rt = %b\n", expected_literal, literal, expected_opcode, opcode, expected_rd, rd, expected_rs, rs, expected_rt, rt);
        $finish;
    end

endmodule
