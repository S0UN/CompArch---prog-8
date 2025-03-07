`include "../hdl/tinker.sv"

module test_tinker_core;
    reg [31:0] instruction;
    wire [63:0] result;

    tinker_core uut (
        .instruction(instruction)
    );

    initial begin
        $display("Testing Tinker Core...");

        // Test 1: ADD instruction
        instruction = 32'b11000_00001_00010_00011_000000000000; // ADD R1, R2, R3
        #10;
        $display("ADD Result: %h", result);

        // Test 2: ADDI instruction
        instruction = 32'b11001_00001_00010_0000000000111111; // ADDI R1, R2, 255
        #10;
        $display("ADDI Result: %h", result);

        // Test 3: MOV instruction
        instruction = 32'b10001_00001_00010_00000_000000000000; // MOV R1, R2
        #10;
        $display("MOV Result: %h", result);

        $finish;
    end
endmodule