// tinker_processor.v
// ---------------------------------------------------------
// Top-Level Tinker Processor Module (Combinational Version)
//  - Instantiates the instruction decoder, register file, ALU, and FPU.
//  - Contains control logic to:
//      * Decode the opcode.
//      * Select operands (including sign-extending immediates).
//      * Route the operation to the ALU or FPU.
//      * Handle data movement (MOV) instructions.
//  - This version does not use a clock, so it is fully combinational.
// ---------------------------------------------------------
module tinker_processor(
    input  [31:0] instruction,  // 32-bit instruction input.
    output [63:0] result_out    // Final result output.
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
    // The expression replicates the sign bit (bit 11) 52 times.
    wire [63:0] immediate_ext;
    assign immediate_ext = {{52{immediate[11]}}, immediate};
    
    // --- Control Signals ---
    // These signals determine how the processor will process the instruction.
    reg        use_imm;  // If high, use the immediate value as operand2.
    reg        op1_sel;  // If high, select operand1 from register rd instead of rs.
    reg        is_fpu;   // If high, the operation is floating-point.
    reg        is_mov;   // If high, it is a MOV instruction.
    reg [3:0]  alu_op;   // ALU operation code.
    reg [3:0]  fpu_op;   // FPU operation code.
    
    // Define opcode constants (5-bit values) for clarity.
    localparam OP_AND     = 5'b00000;
    localparam OP_OR      = 5'b00001;
    localparam OP_XOR     = 5'b00010;
    localparam OP_NOT     = 5'b00011;
    localparam OP_SHFTR   = 5'b00100;
    localparam OP_SHFTRI  = 5'b00101;
    localparam OP_SHFTL   = 5'b00110;
    localparam OP_SHFTLI  = 5'b00111;
    localparam OP_MOV_REG = 5'b10001;  // MOV rd, rs.
    localparam OP_MOV_IMM = 5'b10010;  // MOV rd, L.
    localparam OP_ADDF    = 5'b10100;  // Floating-point add.
    localparam OP_SUBF    = 5'b10101;  // Floating-point subtract.
    localparam OP_MULF    = 5'b10110;  // Floating-point multiply.
    localparam OP_DIVF    = 5'b10111;  // Floating-point divide.
    localparam OP_ADD     = 5'b11000;  // ADD.
    localparam OP_ADDI    = 5'b11001;  // ADDI.
    localparam OP_SUB     = 5'b11010;  // SUB.
    localparam OP_SUBI    = 5'b11011;  // SUBI.
    localparam OP_MUL     = 5'b11100;  // MUL.
    localparam OP_DIV     = 5'b11101;  // DIV.
    
    // Set control signals based on the opcode.
    always @(*) begin
        // Default assignments.
        use_imm = 0;
        op1_sel = 0;
        is_fpu  = 0;
        is_mov  = 0;
        alu_op  = 4'd0;
        fpu_op  = 4'd0;
        case (opcode)
            // Logic instructions:
            OP_AND:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd0};
            OP_OR:     {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd1};
            OP_XOR:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd2};
            OP_NOT:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd3};
            
            // Shift instructions:
            OP_SHFTR:  {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd4};
            OP_SHFTRI: {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd4};
            OP_SHFTL:  {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd5};
            OP_SHFTLI: {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd5};
            
            // Integer arithmetic:
            OP_ADD:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd6};
            OP_ADDI:   {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd6};
            OP_SUB:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd7};
            OP_SUBI:   {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b1, 1'b1, 1'b0, 1'b0, 4'd7};
            OP_MUL:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd8};
            OP_DIV:    {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {1'b0, 1'b0, 1'b0, 1'b0, 4'd9};
            
            // Data movement (MOV) instructions:
            OP_MOV_REG: {use_imm, op1_sel, is_fpu, is_mov} = {1'b0, 1'b0, 1'b0, 1'b1};  // MOV rd, rs.
            OP_MOV_IMM: {use_imm, op1_sel, is_fpu, is_mov} = {1'b1, 1'b0, 1'b0, 1'b1};  // MOV rd, L.
            
            // Floating-point operations:
            OP_ADDF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd0};
            OP_SUBF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd1};
            OP_MULF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd2};
            OP_DIVF:   {use_imm, op1_sel, is_fpu, is_mov, fpu_op} = {1'b0, 1'b0, 1'b1, 1'b0, 4'd3};
            
            default:   {use_imm, op1_sel, is_fpu, is_mov, alu_op} = {4'b0, 4'd0};
        endcase
    end

    // --- Register File Instantiation ---
    // Instantiate the register file with three read ports.
    wire [63:0] reg_data1, reg_data2, reg_data3;
    reg_file rf (
        .read_addr1(rs),
        .read_addr2(rt),
        .read_addr3(rd),
        .write_addr(rd),         // Write result to register rd.
        .write_data(final_result),
        .write_en(1'b1),         // For this example, assume every instruction writes.
        .read_data1(reg_data1),
        .read_data2(reg_data2),
        .read_data3(reg_data3)
    );
    
    // --- Operand Selection ---
    // Choose operand1: either from register rs or, if op1_sel is high, from register rd.
    wire [63:0] operand1;
    assign operand1 = (op1_sel) ? reg_data3 : reg_data1;
    
    // Choose operand2: if use_imm is asserted, use the sign-extended immediate; otherwise, use register rt.
    wire [63:0] operand2;
    assign operand2 = (use_imm) ? immediate_ext : reg_data2;
    
    // --- ALU and FPU Computation ---
    // Instantiate ALU and FPU.
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
    
    // Select the computed result: if the operation is floating-point, use the FPU result; otherwise, use the ALU result.
    wire [63:0] computed_result;
    assign computed_result = (is_fpu) ? fpu_result : alu_result;
    
    // --- Final Result Selection ---
    // For MOV instructions, override the computed result:
    //   - For MOV rd, rs, forward the value from rs.
    //   - For MOV rd, L, use the immediate.
    wire [63:0] final_result;
    assign final_result = (is_mov) ? ((opcode == OP_MOV_REG) ? reg_data1 : immediate_ext) : computed_result;
    
    // --- Output ---
    assign result_out = final_result;
    
endmodule
