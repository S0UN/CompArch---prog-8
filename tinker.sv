// Combined Multi-Cycle Processor File
 // Saturday, April 12, 2025

 ////////////////////////////////////////////////////////////////////////////////
 // Top-Level Module: tinker_core
 ////////////////////////////////////////////////////////////////////////////////
 module tinker_core (
     input clk,
     input reset,
     output logic hlt // Halt signal
 );

     // --- Internal Signals ---
     // Control Signals from FSM
     logic pcWriteEnable;
     logic regReadEnable;
     logic regWriteEnable;
     logic aluEnable;
     logic memReadEnable;
     logic memWriteEnable;
     logic selectALUOutForReg; // Mux select: 1=ALU, 0=Memory
     logic selectALUPC;        // Mux select: 1=ALU calculated PC, 0=PC+4/Mem
     logic selectMemPC;        // Mux select: 1=Memory Data (Return), 0=ALU/PC+4

     // Fetch Stage
     logic [63:0] pc_current;
     logic [63:0] pc_next_selected;

     // Decode Stage
     logic [31:0] instruction_word_fetched;
     logic [63:0] immediate_decoded;
     logic [4:0]  dest_reg_addr_decoded;
     logic [4:0]  src1_reg_addr_decoded;
     logic [4:0]  src2_reg_addr_decoded;
     logic [4:0]  opcode_decoded;
     logic [63:0] reg_data_read1;
     logic [63:0] reg_data_read2;
     logic [63:0] reg_data_dest_read; // Value of destination register (for some instructions like MOV ($rd)L, $rs)

     // Execute Stage
     logic [63:0] alu_operand_b;
     logic [63:0] alu_result_calculated;
     logic [63:0] alu_next_pc_calculated; // PC calculated by ALU (branches, jumps, PC+4)
     logic        alu_mem_access_required; // Indicates if ALU op requires memory access stage
     logic        alu_mem_is_write;        // Indicates if memory access is a write
     logic        alu_mem_is_read;         // Indicates if memory access is a read
     logic        alu_is_return;           // Indicates if ALU op is return
     logic [63:0] alu_mem_address_calculated;
     logic [63:0] alu_mem_data_to_write;

     // Memory Stage
     logic [63:0] mem_data_read;

     // Write Back Stage
     logic [63:0] data_to_write_to_reg;

     // Halt signal from ALU
     logic internal_halt;


     // --- Module Instantiations ---

     // Controller Finite State Machine (FSM)
     controller_fsm control_fsm_inst (
         .clk(clk),
         .reset(reset),
         .opcode(opcode_decoded),
         .mem_access_required_from_alu(alu_mem_access_required),
         .is_return_from_alu(alu_is_return),
         .halt_from_alu(internal_halt),
        .alu_mem_is_read(alu_mem_is_read),   // Connect the signal from tinker_core scope
        .alu_mem_is_write(alu_mem_is_write), // Connect the signal from tinker_core scope


         .pcWriteEnable(pcWriteEnable),
         .regReadEnable(regReadEnable),
         .regWriteEnable(regWriteEnable),
         .aluEnable(aluEnable),
         .memReadEnable(memReadEnable),
         .memWriteEnable(memWriteEnable),
         .selectALUOutForReg(selectALUOutForReg),
         .selectALUPC(selectALUPC),
         .selectMemPC(selectMemPC),
         .halt_out(hlt) // Connect internal halt to module output
     );

     // Fetch Unit
     fetch_unit fetch_unit_inst (
         .clk(clk),
         .reset(reset),
         .writeEnable(pcWriteEnable),
         .pc_in(pc_next_selected),
         .pc_out(pc_current)
     );

     // Memory Unit (Handles instructions and data)
     memory_unit memory (
         .clk(clk),
         .reset(reset),
         .readEnable(memReadEnable), // Controlled by FSM
         .writeEnable(memWriteEnable), // Controlled by FSM
         .instructionAddr(pc_current),
         .dataAddr(alu_mem_address_calculated), // Address comes from ALU calculation
         .dataIn(alu_mem_data_to_write),      // Data to write comes from ALU calculation
         .instructionOut(instruction_word_fetched),
         .dataOut(mem_data_read)
     );

     // Instruction Decoder
     instruction_decoder decoder_inst (
         .instruction(instruction_word_fetched), // Input is the fetched instruction
         .imm(immediate_decoded),
         .dest(dest_reg_addr_decoded),
         .src1(src1_reg_addr_decoded),
         .src2(src2_reg_addr_decoded),
         .opcode(opcode_decoded)
     );

     // Register File
     reg_file_bank reg_file (
         .clk(clk),
         .reset(reset),
         .readEnable(regReadEnable),   // Controlled by FSM
         .writeEnable(regWriteEnable), // Controlled by FSM
         .writeData(data_to_write_to_reg), // Data comes from WriteBack Mux
         .readAddr1(src1_reg_addr_decoded),
         .readAddr2(src2_reg_addr_decoded),
         .writeAddr(dest_reg_addr_decoded),
         .readData1(reg_data_read1),
         .readData2(reg_data_read2),
         .readDataDest(reg_data_dest_read), // Read destination register value
         .stackPtrOut() // Stack pointer output (unused in this simplified connection)
     );

     // Register/Literal Mux (Selects ALU Operand B)
     reg_lit_mux reg_lit_mux_inst (
         .opcode(opcode_decoded),
         .regValue(reg_data_read2),
         .litValue(immediate_decoded),
         .selectedValue(alu_operand_b)
     );

     // ALU Unit
     alu_unit alu_unit_inst (
         .enable(aluEnable), // Controlled by FSM
         .opControl(opcode_decoded),
         .operandA(reg_data_read1),
         .operandB(alu_operand_b),
         .operandDestRegValue(reg_data_dest_read), // Pass Rd value for instructions like MOV (Rd)L, Rs
         .currentPC(pc_current),
         .immediateValue(immediate_decoded), // Pass immediate for branches/memory offsets
         .stackPtrIn(64'd524288), // Assuming fixed initial stack pointer for now
         .aluResult(alu_result_calculated),
         .nextPCResult(alu_next_pc_calculated),
         .memAddressResult(alu_mem_address_calculated),
         .memWriteDataResult(alu_mem_data_to_write),
         .memAccessRequired(alu_mem_access_required),
         .memIsWrite(alu_mem_is_write),
         .memIsRead(alu_mem_is_read),
         .isReturn(alu_is_return),
         .haltSignal(internal_halt)
     );

     // Mux for Register File Write Data Selection
     always @(*) begin
         if (selectALUOutForReg) begin
             data_to_write_to_reg = alu_result_calculated;
         end else begin
             data_to_write_to_reg = mem_data_read; // From LOAD instructions
         end
     end

     // Mux for Next PC Selection
     always @(*) begin
         if (selectMemPC) begin // Highest priority for RETURN
             pc_next_selected = mem_data_read;
         end else if (selectALUPC) begin // Next priority for branches/jumps/calls calculated by ALU
             pc_next_selected = alu_next_pc_calculated;
         end else begin // Default: PC + 4 (also calculated by ALU now)
             pc_next_selected = alu_next_pc_calculated;
         end
     end

 endmodule

 ////////////////////////////////////////////////////////////////////////////////
 // Controller Finite State Machine (FSM)
 ////////////////////////////////////////////////////////////////////////////////
 module controller_fsm (
     input clk,
     input reset,
     input [4:0] opcode,
     input mem_access_required_from_alu, // Signal from ALU if MEM stage is needed
     input is_return_from_alu,         // Signal from ALU if it's a return instruction
     input halt_from_alu,              // Signal from ALU if HALT instruction
     input logic alu_mem_is_read,      // ALU determined if mem op is read
     input logic alu_mem_is_write,     // ALU determined if mem op is write

     output logic pcWriteEnable,
     output logic regReadEnable,
     output logic regWriteEnable,
     output logic aluEnable,
     output logic memReadEnable,
     output logic memWriteEnable,
     output logic selectALUOutForReg, // Mux select: 1=ALU, 0=Memory
     output logic selectALUPC,        // Mux select: 1=ALU calculated PC, 0=PC+4/Mem
     output logic selectMemPC,        // Mux select: 1=Memory Data (Return), 0=ALU/PC+4
     output logic halt_out
 );

     typedef enum logic [2:0] {
         FETCH      = 3'b000,
         DECODE     = 3'b001,
         EXECUTE    = 3'b010,
         MEM_ACCESS = 3'b011,
         WRITE_BACK = 3'b100
     } processor_state_t;

     processor_state_t current_state, next_state;

     // State Register Logic
     always @(posedge clk or posedge reset) begin
         if (reset) begin
             current_state <= FETCH;
         end else begin
             current_state <= next_state;
         end
     end

     // Next State Logic (Combinational)
     always @(*) begin
         next_state = current_state; // Default: stay in current state (should be overridden)
         case (current_state)
             FETCH:      next_state = DECODE;
             DECODE:     next_state = EXECUTE;
             EXECUTE: begin
                 if (halt_from_alu) begin
                     next_state = EXECUTE; // Stay in EXECUTE on HALT
                 end else if (mem_access_required_from_alu) begin
                     next_state = MEM_ACCESS;
                 end else if (|opcode) begin // If it's a valid non-memory, non-halt op, needs WB
                     next_state = WRITE_BACK;
                 end else begin // Should not happen with valid instructions
                     next_state = FETCH;
                 end
             end
             MEM_ACCESS: begin
                 // Instructions needing writeback after mem access: LOAD (MOV Rd, (Rs)L)
                 if (opcode == 5'b10000) begin // mov $r_d, ($r_s)(L)
                     next_state = WRITE_BACK;
                 end else begin
                     next_state = FETCH; // Store, Call, Return go back to fetch
                 end
             end
             WRITE_BACK: next_state = FETCH;
             default:    next_state = FETCH; // Default back to FETCH
         endcase
     end

     // Output Logic (Combinational based on current_state)
     always @(*) begin
         // Default values (inactive)
         pcWriteEnable = 1'b0;
         regReadEnable = 1'b0;
         regWriteEnable = 1'b0;
         aluEnable = 1'b0;
         memReadEnable = 1'b0;
         memWriteEnable = 1'b0;
         selectALUOutForReg = 1'b0; // Default select memory for safety (though WB state controls write)
         selectALUPC = 1'b0;        // Default select PC+4 / Mem
         selectMemPC = 1'b0;        // Default don't select Mem for PC
         halt_out = 1'b0;

         case (current_state)
             FETCH: begin
                 pcWriteEnable = 1'b1; // Enable PC update
                 // Memory read for instruction is implicitly enabled by address
             end
             DECODE: begin
                 regReadEnable = 1'b1; // Read source registers
             end
             EXECUTE: begin
                 aluEnable = 1'b1; // Perform ALU operation
                 if (halt_from_alu) begin
                     halt_out = 1'b1; // Assert halt signal if ALU detected HALT opcode
                 end
                 // Determine PC source based on ALU output for branches/jumps
                 case (opcode)
                   5'b01000, // br $rd
                   5'b01001, // brr $rd
                   5'b01010, // brr L
                   5'b01011, // brnz $rd, $rs
                   5'b01100, // call $rd
                   5'b01110: // brgt $rd, $rs, $rt
                       selectALUPC = 1'b1;
                   default:
                       selectALUPC = 1'b0; // Use PC+4 (also from ALU)
                 endcase
             end
             MEM_ACCESS: begin
                 // Enable memory read or write based on ALU determination
                 memReadEnable  = alu_mem_is_read;
                 memWriteEnable = alu_mem_is_write;
                 // If it's a return, select the data read from memory as the next PC
                 if (is_return_from_alu) begin
                    selectMemPC = 1'b1;
                    selectALUPC = 1'b0; // Override ALU PC select
                 end
             end
             WRITE_BACK: begin
                 regWriteEnable = 1'b1; // Write result back to register file
                 // Determine source for register write data
                 if (opcode == 5'b10000) begin // mov $r_d, ($r_s)(L) -> Load
                     selectALUOutForReg = 1'b0; // Select data from Memory
                 end else begin
                     selectALUOutForReg = 1'b1; // Select data from ALU
                 end
             end
             default: ; // Keep inactive defaults
         endcase
     end

 endmodule


 ////////////////////////////////////////////////////////////////////////////////
 // Fetch Unit
 ////////////////////////////////////////////////////////////////////////////////
 module fetch_unit (
     input clk,
     input reset,
     input writeEnable,        // Control signal from FSM
     input [63:0] pc_in,      // Next PC value selected by mux
     output logic [63:0] pc_out // Current PC value
 );
     logic [63:0] pc_register;

     assign pc_out = pc_register;

     always @(posedge clk or posedge reset) begin
         if (reset) begin
             pc_register <= 64'h2000; // Initial PC address
         end else if (writeEnable) begin
             pc_register <= pc_in;
         end
     end
 endmodule


 ////////////////////////////////////////////////////////////////////////////////
 // Memory Unit (Byte Addressable)
 ////////////////////////////////////////////////////////////////////////////////
 module memory_unit (
     input clk,
     input reset,
     input readEnable,           // From FSM for data reads
     input writeEnable,          // From FSM for data writes
     input [63:0] instructionAddr, // Address for instruction fetch (from PC)
     input [63:0] dataAddr,       // Address for data load/store (from ALU)
     input [63:0] dataIn,         // Data to be written (from ALU/RegFile)
     output logic [31:0] instructionOut, // Fetched instruction
     output logic [63:0] dataOut         // Data read from memory
 );
     localparam MEM_SIZE = 524288; // 0x80000 bytes
     logic [7:0] memory_array [0:MEM_SIZE-1];
     integer idx;

     // Instruction Fetch (Combinational Read based on instructionAddr)
     // Assuming little-endian fetch
     assign instructionOut[7:0]   = memory_array[instructionAddr];
     assign instructionOut[15:8]  = memory_array[instructionAddr + 1];
     assign instructionOut[23:16] = memory_array[instructionAddr + 2];
     assign instructionOut[31:24] = memory_array[instructionAddr + 3];

     // Data Read (Combinational Read based on dataAddr, enabled by readEnable)
     // Assuming little-endian read
     assign dataOut[7:0]   = readEnable ? memory_array[dataAddr]     : 8'hXX;
     assign dataOut[15:8]  = readEnable ? memory_array[dataAddr + 1] : 8'hXX;
     assign dataOut[23:16] = readEnable ? memory_array[dataAddr + 2] : 8'hXX;
     assign dataOut[31:24] = readEnable ? memory_array[dataAddr + 3] : 8'hXX;
     assign dataOut[39:32] = readEnable ? memory_array[dataAddr + 4] : 8'hXX;
     assign dataOut[47:40] = readEnable ? memory_array[dataAddr + 5] : 8'hXX;
     assign dataOut[55:48] = readEnable ? memory_array[dataAddr + 6] : 8'hXX;
     assign dataOut[63:56] = readEnable ? memory_array[dataAddr + 7] : 8'hXX;


     // Memory Initialization and Write Logic (Synchronous)
     always @(posedge clk or posedge reset) begin
         if (reset) begin
             for (idx = 0; idx < MEM_SIZE; idx = idx + 1) begin
                 memory_array[idx] <= 8'h0;
             end
         end else if (writeEnable) begin
             // Assuming little-endian write
             memory_array[dataAddr]     <= dataIn[7:0];
             memory_array[dataAddr + 1] <= dataIn[15:8];
             memory_array[dataAddr + 2] <= dataIn[23:16];
             memory_array[dataAddr + 3] <= dataIn[31:24];
             memory_array[dataAddr + 4] <= dataIn[39:32];
             memory_array[dataAddr + 5] <= dataIn[47:40];
             memory_array[dataAddr + 6] <= dataIn[55:48];
             memory_array[dataAddr + 7] <= dataIn[63:56];
         end
     end

 endmodule


 ////////////////////////////////////////////////////////////////////////////////
 // Instruction Decoder
 ////////////////////////////////////////////////////////////////////////////////
 module instruction_decoder (
     input  logic [31:0] instruction, // Input instruction word
     output logic [63:0] imm,         // Decoded immediate value (zero/sign extended)
     output logic [4:0]  dest,        // Decoded destination register address
     output logic [4:0]  src1,        // Decoded source 1 register address
     output logic [4:0]  src2,        // Decoded source 2 register address
     output logic [4:0]  opcode       // Decoded opcode
 );

     // Combinational decoding logic
     always @(*) begin
         opcode = instruction[31:27];
         dest   = instruction[26:22];
         src1   = instruction[21:17];
         src2   = instruction[16:12];

         // Simple zero-extension for immediate value used by most instructions
         // Sign extension might be needed for branches, handled in ALU if required
         imm = {52'h0, instruction[11:0]};

         // Handle instructions where destination is also the first source
         // Example: ADDI $rd, $rd, L  (uses rd as src1)
         // Example: MOV $rd, L       (uses rd as src1, but value comes from imm)
         case (opcode)
             // Immediate ALU ops
             5'b11001, // addi
             5'b11011, // subi
             5'b00101, // shftri
             5'b00111, // shftli
             // Immediate Move
             5'b10010: // mov $r_d, L
                 src1 = dest;
             default: ; // Keep originally decoded src1
         endcase

         // Handle instructions where destination is used differently
         case (opcode)
             // Store instruction: MOV ($rd)(L), $rs
             // Here 'dest' holds the base address register, 'src1' holds the data source
             5'b10011: begin
                 // Keep dest as is (base address register)
                 // Keep src1 as is (data source register)
             end
             // Branches/Call/Return often use 'dest' for the target address/register
             5'b01000, // br $rd (target address in Rd)
             5'b01001, // brr $rd (target offset in Rd)
             5'b01010, // brr L (target offset in Immediate)
             5'b01011, // brnz $rd, $rs (target address in Rd, check Rs)
             5'b01100, // call $rd (target address in Rd)
             5'b01101, // return (no explicit operands in these fields used traditionally)
             5'b01110: // brgt $rd, $rs, $rt (target address in Rd, compare Rs/Rt)
                 // Keep dest as is (target address register)
                 // Keep src1 and src2 as needed for comparison/check
                 ; // No changes needed here based on simple decoding
             default: ;
         endcase

     end
 endmodule


 ////////////////////////////////////////////////////////////////////////////////
 // Register File Bank
 ////////////////////////////////////////////////////////////////////////////////
 module reg_file_bank (
     input clk,
     input reset,
     input readEnable,           // Control signal from FSM
     input writeEnable,          // Control signal from FSM
     input [63:0] writeData,     // Data to write (from ALU/Mem mux)
     input [4:0] readAddr1,      // Address for first read port (Rs)
     input [4:0] readAddr2,      // Address for second read port (Rt)
     input [4:0] writeAddr,      // Address for write port (Rd)
     output logic [63:0] readData1, // Data from first read port
     output logic [63:0] readData2, // Data from second read port
     output logic [63:0] readDataDest,// Data from reading the destination addr (for MOV ($rd)L, $rs)
     output logic [63:0] stackPtrOut // Output stack pointer (R31)
 );
     localparam NUM_REGS = 32;
     localparam STACK_REG_IDX = 31;
     localparam INITIAL_STACK_PTR = 64'd524288; // 0x80000

     logic [63:0] registers [0:NUM_REGS-1];
     integer i;

     // Combinational Read Logic (Enabled by FSM)
     assign readData1 = readEnable ? registers[readAddr1] : 64'hXXXXXXXXXXXXXXXX;
     assign readData2 = readEnable ? registers[readAddr2] : 64'hXXXXXXXXXXXXXXXX;
     // Special read for destination register value needed by some instructions
     assign readDataDest = readEnable ? registers[writeAddr] : 64'hXXXXXXXXXXXXXXXX;
     assign stackPtrOut = registers[STACK_REG_IDX];

     // Synchronous Write Logic
     always @(posedge clk or posedge reset) begin
         if (reset) begin
             for (i = 0; i < STACK_REG_IDX; i = i + 1) begin
                 registers[i] <= 64'h0;
             end
             registers[STACK_REG_IDX] <= INITIAL_STACK_PTR;
         end else if (writeEnable) begin
             // Ensure writing to R0 has no effect if R0 is hardwired zero
             // if (writeAddr != 5'b00000)
             registers[writeAddr] <= writeData;
         end
     end
 endmodule


 ////////////////////////////////////////////////////////////////////////////////
 // Register/Literal Mux (Selects Operand B for ALU)
 ////////////////////////////////////////////////////////////////////////////////
 module reg_lit_mux (
     input logic [4:0] opcode,     // Instruction opcode
     input logic [63:0] regValue,   // Value from register file (Rt)
     input logic [63:0] litValue,   // Immediate value from decoder
     output logic [63:0] selectedValue // Output selected value for ALU operand B
 );

     always @(*) begin
         // Select immediate value for immediate instructions
         case (opcode)
             5'b11001, // addi
             5'b11011, // subi
             5'b00101, // shftri
             5'b00111, // shftli
             5'b10010, // mov $r_d, L
             5'b10000, // mov $r_d, ($r_s)(L) - Immediate is offset
             5'b10011: // mov ($r_d)(L), $r_s - Immediate is offset
                 selectedValue = litValue;
             default: // Otherwise, select register value
                 selectedValue = regValue;
         endcase
     end
 endmodule

 ////////////////////////////////////////////////////////////////////////////////
 // ALU Unit
 ////////////////////////////////////////////////////////////////////////////////
 module alu_unit (
     input enable,                  // From FSM
     input logic [4:0] opControl,   // Opcode from decoder
     input logic [63:0] operandA,   // From RegFile (Rs) or PC
     input logic [63:0] operandB,   // From RegFile (Rt) or Immediate (via Mux)
     input logic [63:0] operandDestRegValue, // Value of Rd register (for store base addr)
     input logic [63:0] currentPC,  // Current program counter for relative branches
     input logic [63:0] immediateValue, // Raw immediate value for branch offset sign extension
     input logic [63:0] stackPtrIn, // Current stack pointer (R31)

     output logic [63:0] aluResult,  // Result of arithmetic/logic operation
     output logic [63:0] nextPCResult, // Calculated next PC for branches/jumps/default
     output logic [63:0] memAddressResult, // Calculated address for memory access
     output logic [63:0] memWriteDataResult, // Data to write to memory (for store/call)
     output logic memAccessRequired, // Does this operation need the MEM stage?
     output logic memIsWrite,        // Is the memory access a write?
     output logic memIsRead,         // Is the memory access a read?
     output logic isReturn,          // Is this a return instruction?
     output logic haltSignal         // Is this a halt instruction?
 );

     real floatA, floatB, floatResult;
     logic [63:0] defaultNextPC;

     // Default PC is PC+4
     assign defaultNextPC = currentPC + 4;

     // Convert inputs for floating point operations
     assign floatA = $bitstoreal(operandA);
     assign floatB = $bitstoreal(operandB);

     // Combinational ALU logic, active only when enabled by FSM
     always @(*) begin
         // Default outputs
         aluResult         = 64'hXXXXXXXXXXXXXXXX;
         nextPCResult      = defaultNextPC; // Default PC update
         memAddressResult  = 64'hXXXXXXXXXXXXXXXX;
         memWriteDataResult= 64'hXXXXXXXXXXXXXXXX;
         memAccessRequired = 1'b0;
         memIsWrite        = 1'b0;
         memIsRead         = 1'b0;
         isReturn          = 1'b0;
         haltSignal        = 1'b0;

         if (enable) begin
             // Default ALU result often pass-through or zero for non-calc ops
             aluResult = 64'b0;

             case (opControl)
                 // --- Integer Arithmetic ---
                 5'b11000: aluResult = operandA + operandB; // add
                 5'b11001: aluResult = operandA + operandB; // addi (operandB is Imm)
                 5'b11010: aluResult = operandA - operandB; // sub
                 5'b11011: aluResult = operandA - operandB; // subi (operandB is Imm)
                 5'b11100: aluResult = operandA * operandB; // mul
                 5'b11101: aluResult = operandA / operandB; // div (watch for division by zero)

                 // --- Logical Operations ---
                 5'b00000: aluResult = operandA & operandB; // and
                 5'b00001: aluResult = operandA | operandB; // or
                 5'b00010: aluResult = operandA ^ operandB; // xor
                 5'b00011: aluResult = ~operandA;           // not

                 // --- Shifts ---
                 5'b00100: aluResult = operandA >> operandB; // shftr
                 5'b00101: aluResult = operandA >> operandB; // shftri (operandB is Imm)
                 5'b00110: aluResult = operandA << operandB; // shftl
                 5'b00111: aluResult = operandA << operandB; // shftli (operandB is Imm)

                 // --- Moves ---
                 5'b10000: begin // mov $r_d, ($r_s)(L) -> Load
                     memAddressResult = operandA + operandB; // Addr = Rs + Imm
                     memAccessRequired = 1'b1;
                     memIsRead = 1'b1;
                     // ALU result is not used, data comes from memory in WB stage
                 end
                 5'b10001: aluResult = operandA;           // mov $r_d, $r_s
                 5'b10010: aluResult = {operandA[63:12], operandB[11:0]}; // mov $r_d, L (operandB is Imm)
                 5'b10011: begin // mov ($r_d)(L), $r_s -> Store
                     // Address uses the *value* of the destination register Rd + Imm
                     memAddressResult = operandDestRegValue + operandB; // Addr = Rd + Imm
                     memWriteDataResult = operandA; // Data = Rs
                     memAccessRequired = 1'b1;
                     memIsWrite = 1'b1;
                     // ALU result is not used
                 end

                 // --- Floating Point ---
                 5'b10100: begin // addf
                     floatResult = floatA + floatB;
                     aluResult = $realtobits(floatResult);
                 end
                 5'b10101: begin // subf
                     floatResult = floatA - floatB;
                     aluResult = $realtobits(floatResult);
                 end
                 5'b10110: begin // mulf
                     floatResult = floatA * floatB;
                     aluResult = $realtobits(floatResult);
                 end
                 5'b10111: begin // divf
                     // Consider handling division by zero if necessary
                     floatResult = floatA / floatB;
                     aluResult = $realtobits(floatResult);
                 end

                 // --- Control Flow ---
                 5'b01000: nextPCResult = operandDestRegValue; // br $rd (Target is Rd value)
                 5'b01001: nextPCResult = currentPC + operandDestRegValue; // brr $rd (Target is PC + Rd value)
                 5'b01010: nextPCResult = currentPC + $signed(immediateValue); // brr L (Target is PC + signed Imm)
                 5'b01011: begin // brnz $rd, $rs (Target in Rd, check Rs)
                     if (operandA != 0) begin // Check Rs value
                         nextPCResult = operandDestRegValue; // Target is Rd value
                     end else begin
                         nextPCResult = defaultNextPC; // PC+4
                     end
                 end
                 5'b01100: begin // call $rd
                     nextPCResult = operandDestRegValue; // Target is Rd value
                     memAddressResult = stackPtrIn - 8; // Push return address onto stack
                     memWriteDataResult = defaultNextPC; // Return address is PC+4
                     memAccessRequired = 1'b1;
                     memIsWrite = 1'b1;
                 end
                 5'b01101: begin // return
                     memAddressResult = stackPtrIn - 8; // Pop return address from stack
                     memAccessRequired = 1'b1;
                     memIsRead = 1'b1;
                     isReturn = 1'b1; // Signal FSM this is a return
                     // nextPCResult will be overwritten by FSM using memory data
                     nextPCResult = 64'hXXXXXXXXXXXXXXXX; // Indicate PC comes from memory
                 end
                 5'b01110: begin // brgt $rd, $rs, $rt (Target in Rd, compare Rs > Rt)
                     if ($signed(operandA) > $signed(operandB)) begin // Compare Rs > Rt (signed comparison assumed)
                         nextPCResult = operandDestRegValue; // Target is Rd value
                     end else begin
                         nextPCResult = defaultNextPC; // PC+4
                     end
                 end

                 // --- Halt ---
                 5'b01111: begin // hlt
                     haltSignal = 1'b1; // Signal FSM to halt
                     // No result or PC change needed
                 end

                 default: begin
                     // Undefined or NOP opcode - treat as NOP
                     aluResult = 64'b0;
                     nextPCResult = defaultNextPC;
                 end
             endcase
         end // if (enable)
     end // always

 endmodule