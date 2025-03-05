// instruction_decoder.v
// ---------------------------------------------------------
// Instruction Decoder Module
//  Format of a 32-bit instruction:
//    [4:0]   : opcode
//    [9:5]   : rd
//    [14:10] : rs
//    [19:15] : rt
//    [31:20] : immediate (12 bits)
// ---------------------------------------------------------
module instruction_decoder(
    input  [31:0] instruction,
    output [4:0]  opcode,
    output [4:0]  rd,
    output [4:0]  rs,
    output [4:0]  rt,
    output [11:0] immediate
);
    assign opcode    = instruction[4:0];    // bits [4:0]
    assign rd        = instruction[9:5];    // bits [9:5]
    assign rs        = instruction[14:10];  // bits [14:10]
    assign rt        = instruction[19:15];  // bits [19:15]
    assign immediate = instruction[31:20];  // bits [31:20]
endmodule
