// alu.v
// ---------------------------------------------------------
// ALU Module for Integer & Logic Operations
//  Operations are selected by a 4-bit control signal (alu_op):
//    0: AND, 1: OR, 2: XOR, 3: NOT,
//    4: Shift Right, 5: Shift Left,
//    6: ADD, 7: SUB, 8: MUL, 9: DIV
// ---------------------------------------------------------
module alu(
    input  [63:0] operand1,
    input  [63:0] operand2,
    input  [3:0]  alu_op,
    output reg [63:0] result
);
    always @(*) begin
        case (alu_op)
            4'd0: result = operand1 & operand2;         // AND
            4'd1: result = operand1 | operand2;         // OR
            4'd2: result = operand1 ^ operand2;         // XOR
            4'd3: result = ~operand1;                   // NOT
            4'd4: result = operand1 >> operand2[5:0];     // Shift Right (using lower 6 bits)
            4'd5: result = operand1 << operand2[5:0];     // Shift Left
            4'd6: result = operand1 + operand2;         // ADD
            4'd7: result = operand1 - operand2;         // SUB
            4'd8: result = operand1 * operand2;         // MUL
            4'd9: result = operand1 / operand2;         // DIV (note: no zero-check)
            default: result = 64'd0;
        endcase
    end
endmodule
