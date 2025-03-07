`include "../hdl/tinker.sv"

module test_alu_fpu;
    reg [63:0] a, b;
    reg [3:0] op;
    reg is_float;
    wire [63:0] result;

    alu_fpu uut (
        .a(a),
        .b(b),
        .op(op),
        .is_float(is_float),
        .result(result)
    );

    initial begin
        $display("Testing ALU/FPU...");

        // Test 1: Integer ADD
        a = 64'd10;
        b = 64'd20;
        op = 4'b0000; // ALU_ADD
        is_float = 0;
        #10;
        $display("Integer ADD: %d + %d = %d", a, b, result);

        // Test 2: Integer SUB
        a = 64'd30;
        b = 64'd10;
        op = 4'b0001; // ALU_SUB
        is_float = 0;
        #10;
        $display("Integer SUB: %d - %d = %d", a, b, result);

        // Test 3: Floating-Point ADD
        a = $realtobits(3.14);
        b = $realtobits(2.71);
        op = 4'b0000; // ALU_ADD
        is_float = 1;
        #10;
        $display("Floating-Point ADD: %f + %f = %f", $bitstoreal(a), $bitstoreal(b), $bitstoreal(result));

        // Test 4: Floating-Point MUL
        a = $realtobits(3.0);
        b = $realtobits(2.0);
        op = 4'b0010; // ALU_MUL
        is_float = 1;
        #10;
        $display("Floating-Point MUL: %f * %f = %f", $bitstoreal(a), $bitstoreal(b), $bitstoreal(result));

        $finish;
    end
endmodule