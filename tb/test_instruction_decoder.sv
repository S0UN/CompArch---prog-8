
module test_instruction_decoder;
    reg [31:0] instruction;
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] literal;
    wire [3:0] alu_op;
    wire is_immediate, reg_write_enable, is_float;

    instruction_decoder uut (
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

    initial begin
        // Test 1: ADD instruction
        instruction = 32'b11000_00001_00010_00011_000000000000; // ADD R1, R2, R3
        #10;
        $display("ADD: opcode=%b, rd=%b, rs=%b, rt=%b, literal=%b, alu_op=%b, is_immediate=%b, reg_write_enable=%b, is_float=%b",
                 opcode, rd, rs, rt, literal, alu_op, is_immediate, reg_write_enable, is_float);

        // Test 2: ADDI instruction
        instruction = 32'b11001_00001_00010_0000000000111111; // ADDI R1, R2, 255
        #10;
        $display("ADDI: opcode=%b, rd=%b, rs=%b, rt=%b, literal=%b, alu_op=%b, is_immediate=%b, reg_write_enable=%b, is_float=%b",
                 opcode, rd, rs, rt, literal, alu_op, is_immediate, reg_write_enable, is_float);

        $finish;
    end
endmodule