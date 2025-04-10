// 1) Instruction Decoder
module instruction_decoder(
    input  [31:0] instruction,
    output reg [4:0]  opcode,
    output reg [4:0]  rd,
    output reg [4:0]  rs,
    output reg [4:0]  rt,
    output reg [11:0] literal,

    // 4-bit ALU opcode
    output reg [3:0]  alu_op,

    // Control signals
    output reg        is_immediate,
    output reg        reg_write_enable,
    output reg        is_float
);

    // ALU operation codes (4 bits)
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_MUL  = 4'b0010;
    localparam ALU_DIV  = 4'b0011;
    localparam ALU_SHR  = 4'b0100;
    localparam ALU_SHL  = 4'b0101;
    localparam ALU_AND  = 4'b0110;
    localparam ALU_OR   = 4'b0111;
    localparam ALU_XOR  = 4'b1000;
    localparam ALU_NOT  = 4'b1001;

    // Memory opcodes needed for disabling reg write
    localparam STORE = 5'b10011;
    localparam CALL  = 5'b01100;
    localparam RET   = 5'b01101;
    localparam LOAD  = 5'b10000;

    // Branch opcodes needed for disabling reg write
    localparam BR     = 5'b01000;
    localparam BRR_RD = 5'b01001;
    localparam BRR_L  = 5'b01010;
    localparam BRNZ   = 5'b01011;
    localparam BRGT   = 5'b01110;


    always @(*) begin
        // Extract fields from instruction
        opcode   = instruction[31:27];
        rd       = instruction[26:22];
        rs       = instruction[21:17];
        rt       = instruction[16:12];
        literal  = instruction[11:0];

        // Default control signals
        is_immediate     = 0;
        reg_write_enable = 1; // Enable by default
        is_float         = 0;
        alu_op           = ALU_ADD; // default to ADD, can be overridden below

        case (opcode)
            // Integer Arithmetic (0x18..0x1D)
            5'b11000: alu_op = ALU_ADD;  // add  (0x18)
            5'b11001: begin             // addi (0x19)
                alu_op       = ALU_ADD;
                is_immediate = 1;
            end
            5'b11010: alu_op = ALU_SUB;  // sub  (0x1A)
            5'b11011: begin             // subi (0x1B)
                alu_op       = ALU_SUB;
                is_immediate = 1;
            end
            5'b11100: alu_op = ALU_MUL;  // mul  (0x1C)
            5'b11101: alu_op = ALU_DIV;  // div  (0x1D)

            // Logic instructions (0x0..0x3)
            5'b00000: alu_op = ALU_AND;  // and (0x0)
            5'b00001: alu_op = ALU_OR;   // or  (0x1)
            5'b00010: alu_op = ALU_XOR;  // xor (0x2)
            5'b00011: alu_op = ALU_NOT;  // not (0x3)

            // Shift instructions (0x4..0x7)
            5'b00100: alu_op = ALU_SHR;  // shftr (0x4)
            5'b00101: begin             // shftri (0x5)
                alu_op       = ALU_SHR;
                is_immediate = 1;
            end
            5'b00110: alu_op = ALU_SHL;  // shftl (0x6)
            5'b00111: begin             // shftli (0x7)
                alu_op       = ALU_SHL;
                is_immediate = 1;
            end

            // Data Movement (0x10..0x13)
            LOAD: begin // mov rd, (rs)(L) (0x10) - Load
                reg_write_enable = 1; // Enable write for Load
                // ALU operation isn't directly used for result, but might be for address calc if needed elsewhere
            end
            5'b10001: begin // mov rd, rs (0x11)
                alu_op = ALU_ADD; // Can use ALU to pass rs to rd (a + 0)
            end
            5'b10010: begin // mov rd, L (0x12)
                alu_op       = ALU_ADD; // Use ALU to pass immediate (0 + L)
                is_immediate = 1;
            end
             STORE: begin // mov (rd)(L), rs (0x13) - Store
                reg_write_enable = 0; // Disable write for Store
            end

            // Floating-Point (0x14..0x17)
            5'b10100: begin // addf (0x14)
                alu_op   = ALU_ADD;
                is_float = 1;
            end
            5'b10101: begin // subf (0x15)
                alu_op   = ALU_SUB;
                is_float = 1;
            end
            5'b10110: begin // mulf (0x16)
                alu_op   = ALU_MUL;
                is_float = 1;
            end
            5'b10111: begin // divf (0x17)
                alu_op   = ALU_DIV;
                is_float = 1;
            end

            // Control flow instructions - typically don't write to registers (except maybe CALL internally)
            BR, BRR_RD, BRR_L, BRNZ, CALL, RET, BRGT: begin
                reg_write_enable = 0; // Disable reg writes for branches/jumps/ret
            end

            default: begin
                // Unknown or Privileged opcode => disable writes
                reg_write_enable = 0;
            end
        endcase
    end
endmodule

// 2) Register File - CORRECTED
module register_file(
    input clk,
    input  [4:0] rs_addr,      // Address for read port 1
    input  [4:0] rt_addr,      // Address for read port 2
    input  [4:0] rd_read_addr, // Address for read port 3 (for control unit) << NEW
    input  [4:0] rd_addr,      // Address for write port
    input  [63:0] write_data,   // Data to write
    input         write_enable, // Write enable signal
    output [63:0] rs_data,      // Data from read port 1
    output [63:0] rt_data,      // Data from read port 2
    output [63:0] rd_read_data  // Data from read port 3 << NEW
);
    reg [63:0] registers [0:31];
    integer i;

    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 64'd0;
        end
        registers[31] = 64'h80000; // Initialize stack pointer R31 towards top of 512KB memory
    end

    // Combinational reads
    assign rs_data = registers[rs_addr];
    assign rt_data = registers[rt_addr];
    assign rd_read_data = registers[rd_read_addr]; // << NEW READ PORT

    // Synchronous write
    always @(posedge clk) begin
        if (write_enable && (rd_addr != 5'd0)) // Ensure write is enabled and not writing to R0
            registers[rd_addr] <= write_data;
    end
endmodule


// 3) Control Unit - CORRECTED
module control(
    input [4:0] op,          // Opcode from decoder
    input [63:0] input_rd_data, // Data from register specified by rd field << NEW
    input [63:0] rs,          // Data from register rs
    input [63:0] rt,          // Data from register rt
    input [63:0] signed_lit,  // Sign-extended literal << RENAMED/CLARIFIED
    input [63:0] inputPc,     // Current PC from fetch stage
    input [63:0] memData,     // Data read from memory (for RET)
    output reg [63:0] pc        // Calculated next PC
);
     // Opcodes for control flow
     localparam BR      = 5'b01000; // Branch unconditional to Reg[rd]
     localparam BRR_RD  = 5'b01001; // Branch relative by Reg[rd] value
     localparam BRR_L   = 5'b01010; // Branch relative by Literal
     localparam BRNZ    = 5'b01011; // Branch to Reg[rd] if Reg[rs] != 0
     localparam CALL    = 5'b01100; // Call subroutine at Reg[rd]
     localparam RET     = 5'b01101; // Return from subroutine
     localparam BRGT    = 5'b01110; // Branch to Reg[rd] if Reg[rs] > Reg[rt] (signed)

    always @(*) begin
      case(op)
        BR:      pc = input_rd_data;                // Jump to address in rd_data
        BRR_RD:  pc = inputPc + input_rd_data;     // Jump relative using rd_data
        BRR_L:   pc = inputPc + signed_lit;        // Jump relative using sign-extended literal
        BRNZ:    pc = (rs != 0) ? input_rd_data : inputPc + 4; // Conditional branch
        CALL:    pc = input_rd_data;                // Jump to subroutine address in rd_data
        RET:     pc = memData;                     // Return using address loaded from stack
        BRGT:    pc = ($signed(rs) > $signed(rt)) ? input_rd_data : inputPc + 4; // Conditional branch (signed compare)
        default: pc = inputPc + 4;                 // Default: increment PC
      endcase
    end
endmodule


// 4) ALU / FPU Combined (Unchanged from your version)
module alu_fpu(
    input  [63:0] a,
    input  [63:0] b,
    input  [3:0]  op,
    input         is_float,
    output reg [63:0] result
);
    real float_a, float_b, float_res;

    // 4-bit ALU codes
    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001;
    localparam ALU_MUL = 4'b0010;
    localparam ALU_DIV = 4'b0011;
    localparam ALU_SHR = 4'b0100; // Shift Right
    localparam ALU_SHL = 4'b0101; // Shift Left
    localparam ALU_AND = 4'b0110;
    localparam ALU_OR  = 4'b0111;
    localparam ALU_XOR = 4'b1000;
    localparam ALU_NOT = 4'b1001;

    always @(*) begin
        if (is_float) begin
            // Floating-Point path
            float_a = $bitstoreal(a);
            float_b = $bitstoreal(b);

            case (op)
                ALU_ADD: float_res = float_a + float_b;
                ALU_SUB: float_res = float_a - float_b;
                ALU_MUL: float_res = float_a * float_b;
                ALU_DIV: float_res = float_a / float_b;
                default: float_res = 0.0; // Default for safety
            endcase

            result = $realtobits(float_res);
        end
        else begin
            // Integer / Logic path
            case (op)
                ALU_ADD: result = a + b;
                ALU_SUB: result = a - b;
                ALU_MUL: result = a * b;
                ALU_DIV: result = (b != 0) ? ($signed(a) / $signed(b)) : 64'd0; // Signed division, check for zero
                ALU_SHR: result = $signed(a) >>> b[5:0]; // Arithmetic shift right
                ALU_SHL: result = a << b[5:0];        // Logical shift left
                ALU_AND: result = a & b;
                ALU_OR : result = a | b;
                ALU_XOR: result = a ^ b;
                ALU_NOT: result = ~a;
                default: result = 64'd0; // Default for safety
            endcase
        end
    end
endmodule


// 5) Clock Generator (Unchanged)
module clock_generator(
    output reg clk
);
    initial begin
        clk = 0;
    end

    always begin
        #5 clk = ~clk; // 10 time units period
    end
endmodule

// 6) Fetch Unit (Unchanged)
module fetch(
    input clk,
    input reset,
  //input [63:0] sp, // SP not directly needed here
    input [63:0] new_pc, // Input from control unit
    output reg [63:0] pc_out  // Output PC for memory read and control unit input
);
    // Program Counter Register
    // reg [63:0] curr_pc; // Use pc_out directly as the register
     initial pc_out = 64'h2000; // Initial PC value if needed outside reset

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            pc_out <= 64'h2000; // Reset PC to starting address
        end else begin
            pc_out <= new_pc;   // Update PC with value from control unit
        end
    end
endmodule


// 7) Memory Handler - UPDATED for R31 access
module memoryHandler(
    input [4:0] opcode,
    // Remove direct rd/rs/lit inputs if address comes solely from ALU/Regs
    input [63:0] addr_operand_rs,  // Value from register RS (used for base address in LD/ST)
    input [63:0] addr_operand_rd,  // Value from register RD (used for base address in ST)
    input [63:0] data_to_store_rs, // Value from register RS (data for ST)
    input [63:0] sign_ext_literal, // Sign-extended literal (offset for LD/ST)
    input [63:0] pc,               // Current PC (for CALL return address)
    input [63:0] r31_value,        // Value of R31 (stack pointer) <<< NEW INPUT

    output reg [63:0] mem_addr,     // Address for memory read/write
    output reg [63:0] mem_wr_data,  // Data to write to memory
    output reg mem_wr_en,        // Memory write enable
    output reg mem_reg_write     // Does this memory operation result in a register write? (LOAD=1)
);
    // Opcodes related to memory access
    localparam CALL  = 5'b01100; // Call: Writes PC+4 to Mem[R31-8]
    localparam RET   = 5'b01101; // Return: Reads PC from Mem[R31-8] (Read handled by mem, PC updated by control)
    localparam LOAD  = 5'b10000; // Load: Reads from Mem[Reg[rs] + L] to Reg[rd]
    localparam STORE = 5'b10011; // Store: Writes Reg[rs] to Mem[Reg[rd] + L]

    always @(*) begin
        // Defaults
        mem_addr = 64'h0; // Default address (unused)
        mem_wr_data = 64'd0;
        mem_wr_en = 0;
        mem_reg_write = 0; // Most memory ops don't write back directly via this path

        case(opcode)
            CALL: begin
                mem_addr = r31_value - 8; // Stack address
                mem_wr_data = pc + 4;     // Return address
                mem_wr_en = 1;            // Enable write
                mem_reg_write = 0;        // CALL doesn't write to rd via memory stage
            end
            RET: begin
                mem_addr = r31_value - 8; // Read return address from stack
                mem_wr_en = 0;            // Disable write
                mem_reg_write = 0;        // RET destination is PC, not rd
                // Read happens in memory module, data goes to control unit via mem_data_out
            end
            LOAD: begin // mov rd, (rs)(L)
                mem_addr = addr_operand_rs + sign_ext_literal; // Calculate address
                mem_wr_en = 0;            // Disable write
                mem_reg_write = 1;        // LOAD writes back to register file (needs mux in top)
            end
            STORE: begin // mov (rd)(L), rs
                mem_addr = addr_operand_rd + sign_ext_literal; // Calculate address using rd as base
                mem_wr_data = data_to_store_rs; // Data comes from rs
                mem_wr_en = 1;            // Enable write
                mem_reg_write = 0;        // STORE doesn't write back to register file
            end
            default: begin
                // Keep defaults for non-memory instructions
            end
        endcase
    end
endmodule


// 8) Memory (Unchanged)
module memory(
    input [63:0] addr_pc,     // Address for instruction fetch
    input clk,
    input reset,
    input write_en,      // Write enable from memory handler
    input [63:0] data_in,     // Data to write from memory handler/pipeline
    input [63:0] addr_rw,     // Read/Write address from memory handler
    output reg [63:0] data_out, // Data read from addr_rw (for LOAD/RET)
    output reg [31:0] inst_out  // Instruction read from addr_pc
);
    // 512KB memory = 524288 bytes
    reg [7:0] bytes [0:524287];
    integer idx;

    // Combinational read for instruction fetch (assuming aligned)
    always @(*) begin
        inst_out[7:0]   = bytes[addr_pc];
        inst_out[15:8]  = bytes[addr_pc+1];
        inst_out[23:16] = bytes[addr_pc+2];
        inst_out[31:24] = bytes[addr_pc+3];
    end

    // Combinational read for data load (assuming aligned)
    always @(*) begin
        data_out[7:0]   = bytes[addr_rw];
        data_out[15:8]  = bytes[addr_rw+1];
        data_out[23:16] = bytes[addr_rw+2];
        data_out[31:24] = bytes[addr_rw+3];
        data_out[39:32] = bytes[addr_rw+4];
        data_out[47:40] = bytes[addr_rw+5];
        data_out[55:48] = bytes[addr_rw+6];
        data_out[63:56] = bytes[addr_rw+7];
    end

    // Synchronous write or reset
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            // Initialize memory to 0 on reset (can take time in simulation)
            for(idx = 0; idx < 524288; idx = idx + 1)
                bytes[idx] <= 8'b0;
        end else if(write_en) begin
            // Write data byte by byte (assuming aligned)
            bytes[addr_rw]   <= data_in[7:0];
            bytes[addr_rw+1] <= data_in[15:8];
            bytes[addr_rw+2] <= data_in[23:16];
            bytes[addr_rw+3] <= data_in[31:24];
            bytes[addr_rw+4] <= data_in[39:32];
            bytes[addr_rw+5] <= data_in[47:40];
            bytes[addr_rw+6] <= data_in[55:48];
            bytes[addr_rw+7] <= data_in[63:56];
        end
    end
endmodule

// 9) Reg/Lit Mux (Potentially redundant - incorporated into tinker_core logic)
/* module reglitmux(
    input [4:0] sel, // Opcode typically determines this
    input [63:0] reg_val,
    input [63:0] imm_val, // Should be sign-extended
    output reg [63:0] mux_out
);
    // This logic is often simpler directly in the pipeline stage (e.g., tinker_core)
    // based on the decoder's 'is_immediate' signal.
    always @(*) begin
        // Example logic, refine based on actual immediate opcodes
        case(sel)
           // List opcodes using immediate value for ALU operand B
           5'b11001, // addi
           5'b11011, // subi
           5'b00101, // shftri
           5'b00111, // shftli
           5'b10010: // mov rd, L
                mux_out = imm_val;
           default:
                mux_out = reg_val;
        endcase
    end
endmodule
*/


// Top-Level Module: tinker_core - CORRECTED
module tinker_core(
    input clk,
    input reset
);
    // Internal wires
    wire [63:0] pc, new_pc, mem_data_out;
    wire [31:0] instruction;
    wire [4:0] opcode, rd_specifier, rs_specifier, rt_specifier; // Renamed for clarity
    wire [11:0] literal;
    wire [3:0] alu_op;
    wire is_immediate, reg_write_enable_decoder, is_float; // From decoder
    wire [63:0] rs_data, rt_data, rd_data; // Data from register file read ports
    wire [63:0] alu_result, operand_b;
    wire [63:0] mem_addr, mem_wr_data;
    wire mem_wr_en, mem_reg_write_signal; // From memory handler
    wire [63:0] sign_extended_literal;
    wire [63:0] reg_write_data_mux_out; // Data to be written back to register file
    wire        reg_write_enable_final;  // Final write enable signal for register file


    // Instantiate Memory
    memory mem_inst(
        .addr_pc(pc),
        .clk(clk),
        .reset(reset),
        .write_en(mem_wr_en), // From memory handler
        .data_in(mem_wr_data), // From memory handler
        .addr_rw(mem_addr),    // From memory handler
        .data_out(mem_data_out), // To control (for RET) and writeback mux
        .inst_out(instruction)  // To decoder
    );

    // Fetch Unit
    fetch fetch_inst(
        .clk(clk),
        .reset(reset),
      //.sp(???), // SP potentially needed if fetch depends on it later
        .new_pc(new_pc),   // From control unit
        .pc_out(pc)      // To memory (addr_pc) and control (inputPc)
    );

    // Instruction Decoder
    instruction_decoder decoder_inst(
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd_specifier),     // Renamed output pin
        .rs(rs_specifier),     // Renamed output pin
        .rt(rt_specifier),     // Renamed output pin
        .literal(literal),
        .alu_op(alu_op),
        .is_immediate(is_immediate),
        .reg_write_enable(reg_write_enable_decoder), // Decoder's initial assessment
        .is_float(is_float)
    );

    // Register File
    register_file reg_file(
        .clk(clk),
        .rs_addr(rs_specifier),            // Read address 1 from decoder
        .rt_addr(rt_specifier),            // Read address 2 from decoder
        .rd_read_addr(rd_specifier),       // Read address 3 (for control) from decoder << NEW
        .rd_addr(rd_specifier),            // Write address from decoder
        .write_data(reg_write_data_mux_out), // Data to write (from MUX) << UPDATED
        .write_enable(reg_write_enable_final), // Final write enable (from MUX logic) << UPDATED
        .rs_data(rs_data),                 // Read data 1 out
        .rt_data(rt_data),                 // Read data 2 out
        .rd_read_data(rd_data)             // Read data 3 out << NEW
    );

    // --- Execute Stage Logic ---

    // Sign extend literal for ALU and Memory Handler
    assign sign_extended_literal = {{52{literal[11]}}, literal};

    // Mux for ALU operand B: Use literal if is_immediate is high, otherwise use rt_data
    assign operand_b = is_immediate ? sign_extended_literal : rt_data;

    // ALU/FPU
    alu_fpu alu_unit(
        .a(rs_data),   // Operand A always from rs_data
        .b(operand_b), // Operand B from mux
        .op(alu_op),     // Operation from decoder
        .is_float(is_float), // Float control from decoder
        .result(alu_result) // Result to writeback mux
    );

    // --- Memory Stage Logic ---

    // Memory Handler
    memoryHandler mem_handler(
        .opcode(opcode),
        .addr_operand_rs(rs_data),          // Base address for LOAD
        .addr_operand_rd(rd_data),          // Base address for STORE (using rd_data)
        .data_to_store_rs(rs_data),         // Data for STORE
        .sign_ext_literal(sign_extended_literal), // Offset for LD/ST
        .pc(pc),                            // PC for CALL
        .r31_value(reg_file.registers[31]), // <<< NOTE: Direct access - potentially problematic for synthesis. Better to read R31 via a read port if possible.
        .mem_addr(mem_addr),                // To memory
        .mem_wr_data(mem_wr_data),          // To memory
        .mem_wr_en(mem_wr_en),              // To memory
        .mem_reg_write(mem_reg_write_signal) // To writeback mux logic
    );

     // Control Unit - Calculates next PC
    control control_inst(
        .op(opcode),
        .input_rd_data(rd_data),         // Pass data read from Reg[rd] << NEW
        .rs(rs_data),                    // Pass data read from Reg[rs]
        .rt(rt_data),                    // Pass data read from Reg[rt]
        .signed_lit(sign_extended_literal), // Pass sign-extended literal << UPDATED
        .inputPc(pc),                    // Pass current PC
        .memData(mem_data_out),          // Pass data from memory (for RET)
        .pc(new_pc)                      // Output: Next PC value to Fetch unit
    );


    // --- Writeback Stage Logic ---

    // Mux for data written back to register file: ALU result or Memory data (for LOAD)
    assign reg_write_data_mux_out = mem_reg_write_signal ? mem_data_out : alu_result;

    // Final Register Write Enable: Combine decoder signal and memory signal
    // Write is enabled if decoder says so AND memory stage doesn't override (e.g. for LOAD)
    // OR if memory stage explicitly enables it (LOAD)
    assign reg_write_enable_final = reg_write_enable_decoder | mem_reg_write_signal;


endmodule