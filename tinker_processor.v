// tinker_processor.v
// ---------------------------------------------------------
// Top-Level Tinker Processor Module
//  Integrates the instruction decoder, register file, ALU, and FPU.
//  Includes control logic to handle:
//    - Expanded opcode decoding for integer, logic, shift, immediate,
//      data movement (mov), and floating-point instructions.
//    - Operand selection (using immediate values where required).
//    - Routing of operations to either the ALU or FPU.
// ---------------------------------------------------------
module tinker_processor(
    input         clk,
    input         rst,
    input  [31:0] instruction,  // Instruction from program memory or testbench
    output [63:0] result_out    // Final result output (for demonstration)
);

    // --- Instruction Decoding ---
    wire [4:0]  opcode;
    wire [4:0]  rd, rs, rt;
    wire [11:0] immediate;
    
    instruction_decoder decoder (
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .immediate(immediate)
    );
    
    // Sign-extend the 12-bit immediate to 64 bits.
    wire [63:0] immediate_ext;
    assign immediate_ext = {{52{immediate[11]}}, immediate};
    
    // --- Control Signals ---
    // These signals determine operand selection and which unit to use.
    reg        use_imm;  // Use immediate for operand2 when high.
    reg        op1_sel;  // Selects operand1: 0 = value from rs, 1 = value from rd.
    reg        is_fpu;   // When high, operation is floating-point.
    reg        is_mov;   // When high, it's a MOV instruction.
    reg [3:0]  alu_op;   // ALU operation code.
    reg [3:0]  fpu_op;   // FPU operation code.
    
    // Define opcode parameters (5-bit values).
    localparam OP_AND     = 5'b00000;
    localparam OP_OR      = 5'b00001;
    localparam OP_XOR     = 5'b00010;
    localparam OP_NOT     = 5'b00011;
    localparam OP_SHFTR   = 5'b00100;
    localparam OP_SHFTRI  = 5'b00101;
    localparam OP_SHFTL   = 5'b00110;
    localparam OP_SHFTLI  = 5'b00111;
    localparam OP_MOV_REG = 5'b10001;  // mov rd, rs (opcode 0x11)
    localparam OP_MOV_IMM = 5'b10010;  // mov rd, L  (opcode 0x12)
    localparam OP_ADDF    = 5'b10100;  // addf (opcode 0x14)
    localparam OP_SUBF    = 5'b10101;  // subf (opcode 0x15)
    localparam OP_MULF    = 5'b10110;  // mulf (opcode 0x16)
    localparam OP_DIVF    = 5'b10111;  // divf (opcode 0x17)
    localparam OP_ADD     = 5'b11000;  // add  (opcode 0x18)
    localparam OP_ADDI    = 5'b11001;  // addi (opcode 0x19)
    localparam OP_SUB     = 5'b11010;  // sub  (opcode 0x1A)
    localparam OP_SUBI    = 5'b11011;  // subi (opcode 0x1B)
    localparam OP_MUL     = 5'b11100;  // mul  (opcode 0x1C)
    localparam OP_DIV     = 5'b11101;  // div  (opcode 0x1D)
    
    // Combinational control logic based on opcode.
    always @(*) begin
        // Default assignments
        use_imm = 0;
        op1_sel = 0;
        is_fpu  = 0;
        is_mov  = 0;
        alu_op  = 4'd0;
        fpu_op  = 4'd0;
        case (opcode)
            // Logic instructions
            OP_AND:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd0};
            OP_OR:     {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd1};
            OP_XOR:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd2};
            OP_NOT:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd3};
            
            // Shift instructions
            OP_SHFTR:  {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd4};
            OP_SHFTRI: {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd4};
            OP_SHFTL:  {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd5};
            OP_SHFTLI: {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd5};
            
            // Integer arithmetic
            OP_ADD:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd6};
            OP_ADDI:   {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd6};
            OP_SUB:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd7};
            OP_SUBI:   {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd7};
            OP_MUL:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd8};
            OP_DIV:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd9};
            
            // Data movement (MOV) instructions.
            // For "mov rd, rs": simply forward the value from rs.
            OP_MOV_REG: {use_imm, op1_sel, is_fpu, is_mov} = {1'b0, 1'b0, 1'b0, 1'b1};
            // For "mov rd, L": use the immediate.
            OP_MOV_IMM: {use_imm, op1_sel, is_fpu, is_mov} = {1'b1, 1'b0, 1'b0, 1'b1};
            
            // Floating-point operations.
            OP_ADDF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd0};
            OP_SUBF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd1};
            OP_MULF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd2};
            OP_DIVF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd3};
            
            default:   {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {4'b0, 4'd0};
        endcase
    end

    // --- Register File Instantiation ---
    // Three read ports: for rs, rt, and rd.
    wire [63:0] reg_data1, reg_data2, reg_data3;
    reg_file rf (
        .clk(clk),
        .rst(rst),
        .read_addr1(rs),
        .read_addr2(rt),
        .read_addr3(rd),
        .write_addr(rd),         // Write back to rd.
        .write_data(final_result),
        .write_en(1'b1),         // For simplicity, assume every instruction writes.
        .read_data1(reg_data1),
        .read_data2(reg_data2),
        .read_data3(reg_data3)
    );
    
    // --- Operand Selection ---
    // Operand1 is selected from either rs or (if op1_sel==1) from rd.
    wire [63:0] operand1;
    assign operand1 = (op1_sel) ? reg_data3 : reg_data1;
    
    // Operand2: if use_imm is asserted, use the immediate value; otherwise, use rt.
    wire [63:0] operand2;
    assign operand2 = (use_imm) ? immediate_ext : reg_data2;
    
    // --- ALU and FPU Computation ---
    wire [63:0] alu_result;
    alu alu_inst (
        .operand1(operand1),
        .operand2(operand2),
        .alu_op(alu_op),
        .result(alu_result)
    );
    
    wire [63:0] fpu_result;
    fpu fpu_inst (
        .operand1(operand1),
        .operand2(operand2),
        .fpu_op(fpu_op),
        .result(fpu_result)
    );
    
    // Select the computed result based on whether the operation is floating-point.
    wire [63:0] computed_result;
    assign computed_result = (is_fpu) ? fpu_result : alu_result;
    
    // --- Final Result Selection ---
    // For MOV instructions, override the computed result:
    //   - For "mov rd, rs", forward the value from rs.
    //   - For "mov rd, L", use the immediate.
    wire [63:0] final_result;
    assign final_result = (is_mov) ? ((opcode == OP_MOV_REG) ? reg_data1 : immediate_ext) : computed_result;
    
    // --- Output ---
    assign result_out = final_result;
    
endmodule
