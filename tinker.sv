module tinker_core (
    input logic clk,
    input logic reset,
    output logic hlt // Halt signal
);
    // --- Constants ---
    localparam HALT_OPCODE = 5'b11111;

    // --- State Machine Definition ---
    typedef enum logic [2:0] {
        S_FETCH, S_DECODE, S_EXECUTE, S_MEMORY, S_WRITEBACK, S_HALTED
    } state_t;
    state_t current_state, next_state;

    // --- Pipeline Registers ---
    // IF_ID: Fetch to Decode
    typedef struct packed {
        logic [63:0] pc;         // Program counter
        logic [31:0] instruction; // Fetched instruction
    } if_id_t;
    if_id_t IF_ID_reg;

    // ID_EX: Decode to Execute
    typedef struct packed {
        logic [63:0] pc;         // PC for branch calculations
        logic [4:0]  opcode;     // Instruction opcode
        logic [4:0]  rd;         // Destination register
        logic [63:0] rs1_val;    // Source register 1 value
        logic [63:0] rs2_val;    // Source register 2 value
        logic [63:0] imm;        // Immediate value
        logic        reg_write;  // Write to register file
        logic        mem_read;   // Memory read enable
        logic        mem_write;  // Memory write enable
        logic        alu_src;    // ALU source (0: rs2_val, 1: imm)
        logic [4:0]  alu_op;     // ALU operation
        logic        is_branch;  // Branch instruction flag
        logic        mem_to_reg; // Memory to register flag
    } id_ex_t;
    id_ex_t ID_EX_reg;

    // EX_MEM: Execute to Memory
    typedef struct packed {
        logic [63:0] alu_result; // ALU output
        logic [63:0] rs2_val;    // Data for store operations
        logic [4:0]  rd;         // Destination register
        logic        reg_write;  // Pass-through control
        logic        mem_read;   // Memory read enable
        logic        mem_write;  // Memory write enable
        logic        mem_to_reg; // Memory to register flag
    } ex_mem_t;
    ex_mem_t EX_MEM_reg;

    // MEM_WB: Memory to Writeback
    typedef struct packed {
        logic [63:0] mem_data;   // Data from memory
        logic [63:0] alu_result; // ALU result (for non-memory ops)
        logic [4:0]  rd;         // Destination register
        logic        reg_write;  // Write to register file
        logic        mem_to_reg; // Memory to register flag
    } mem_wb_t;
    mem_wb_t MEM_WB_reg;

    // --- Internal Signals ---
    logic [63:0] pc_current, pc_next;
    logic pc_write_enable;
    logic [31:0] instr_word;
    logic [4:0] dest_reg, src_reg1, src_reg2, opcode;
    logic [63:0] imm_value, src_val1, src_val2;
    logic [63:0] alu_output, mem_data_out;
    logic [63:0] mem_addr, mem_data; // Outputs from mem_handler

    // --- State Machine Logic ---
    always @(posedge clk or posedge reset) begin
        if (reset) current_state <= S_FETCH;
        else if (current_state == S_HALTED) current_state <= S_HALTED;
        else current_state <= next_state;
    end
    always @(*) begin
        next_state = current_state; // Default to hold current state
        case (current_state)
            S_FETCH:    next_state = S_DECODE;
            S_DECODE:   if (opcode == HALT_OPCODE)
                            next_state = S_HALTED;
                        else
                            next_state = S_EXECUTE;
            S_EXECUTE:  if (ID_EX_reg.mem_read || ID_EX_reg.mem_write)
                            next_state = S_MEMORY;
                        else
                            next_state = S_WRITEBACK;
            S_MEMORY:   next_state = S_WRITEBACK;
            S_WRITEBACK:next_state = S_FETCH;
            S_HALTED:   next_state = S_HALTED;
        endcase
    end

    // --- Pipeline Register Latching ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
   // Reset IF_ID_reg fields explicitly
        IF_ID_reg.pc <= 64'h0;
        IF_ID_reg.instruction <= 32'h0;

        // Reset ID_EX_reg fields explicitly
        ID_EX_reg.pc <= 64'h0;
        ID_EX_reg.opcode <= 5'b0;
        ID_EX_reg.rd <= 5'b0;
        ID_EX_reg.rs1_val <= 64'h0;
        ID_EX_reg.rs2_val <= 64'h0;
        ID_EX_reg.imm <= 64'h0;
        ID_EX_reg.reg_write <= 1'b0;
        ID_EX_reg.mem_read <= 1'b0;
        ID_EX_reg.mem_write <= 1'b0;
        ID_EX_reg.alu_src <= 1'b0;
        ID_EX_reg.alu_op <= 5'b0;
        ID_EX_reg.is_branch <= 1'b0;
        ID_EX_reg.mem_to_reg <= 1'b0;

        // Reset EX_MEM_reg fields explicitly
        EX_MEM_reg.alu_result <= 64'h0;
        EX_MEM_reg.rs2_val <= 64'h0;
        EX_MEM_reg.rd <= 5'b0;
        EX_MEM_reg.reg_write <= 1'b0;
        EX_MEM_reg.mem_read <= 1'b0;
        EX_MEM_reg.mem_write <= 1'b0;
        EX_MEM_reg.mem_to_reg <= 1'b0;

        // Reset MEM_WB_reg fields explicitly
        MEM_WB_reg.mem_data <= 64'h0;
        MEM_WB_reg.alu_result <= 64'h0;
        MEM_WB_reg.rd <= 5'b0;
        MEM_WB_reg.reg_write <= 1'b0;
        MEM_WB_reg.mem_to_reg <= 1'b0;
        end else begin
            // IF_ID: Latch Fetch outputs
            if (current_state == S_FETCH) begin
                IF_ID_reg.pc <= pc_current;
                IF_ID_reg.instruction <= instr_word;
            end
            // ID_EX: Latch Decode outputs
           if (current_state == S_DECODE) begin
    ID_EX_reg.mem_to_reg <= (opcode == 5'b10000); // Set for load instructions
    ID_EX_reg.pc <= IF_ID_reg.pc;
    ID_EX_reg.opcode <= opcode;
    ID_EX_reg.rd <= dest_reg;
    ID_EX_reg.rs1_val <= src_val1;
    ID_EX_reg.rs2_val <= src_val2;
    ID_EX_reg.imm <= imm_value;
    case (opcode)
        // ALU R-type instructions
        5'h18: begin // add
            ID_EX_reg.reg_write <= 1; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h18; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h1a: begin // sub
            ID_EX_reg.reg_write <= 1; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h1a; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h1c: begin // mul
            ID_EX_reg.reg_write <= 1; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h1c; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        // ALU I-type instructions
        5'h19: begin // addi
            ID_EX_reg.reg_write <= 1; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 1; ID_EX_reg.alu_op <= 5'h19; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h1b: begin // subi
            ID_EX_reg.reg_write <= 1; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 1; ID_EX_reg.alu_op <= 5'h1b; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        // Memory Operations
        5'h10: begin // load (mov $r_d, ($r_s)(L))
            ID_EX_reg.reg_write <= 1; ID_EX_reg.mem_read <= 1; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 1; ID_EX_reg.alu_op <= 5'h18; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 1;
        end
        5'h13: begin // store (mov ($r_d)(L), $r_s)
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 1;
            ID_EX_reg.alu_src <= 1; ID_EX_reg.alu_op <= 5'h18; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        // Branches
        5'h08: begin // br $r_d
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 1;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h0a: begin // brr L
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 1;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h0b: begin // brnz $r_d, $r_s
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 1;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h0c: begin // call $r_d
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 1;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 1;
            ID_EX_reg.mem_to_reg <= 0;
        end
        5'h0d: begin // return
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 1; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 1;
            ID_EX_reg.mem_to_reg <= 0;
        end
        // Halt
        5'h1f: begin // halt
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
        default: begin
            ID_EX_reg.reg_write <= 0; ID_EX_reg.mem_read <= 0; ID_EX_reg.mem_write <= 0;
            ID_EX_reg.alu_src <= 0; ID_EX_reg.alu_op <= 5'h00; ID_EX_reg.is_branch <= 0;
            ID_EX_reg.mem_to_reg <= 0;
        end
    endcase
end
            // EX_MEM: Latch Execute outputs
         if (current_state == S_EXECUTE) begin
        EX_MEM_reg.mem_to_reg <= ID_EX_reg.mem_to_reg;
        EX_MEM_reg.alu_result <= (ID_EX_reg.mem_write || ID_EX_reg.is_branch) ? mem_addr : alu_output;
        EX_MEM_reg.rs2_val <= ID_EX_reg.rs2_val; // Pass rs2_val for stores
        EX_MEM_reg.rd <= ID_EX_reg.rd;
        EX_MEM_reg.reg_write <= ID_EX_reg.reg_write;
        EX_MEM_reg.mem_read <= ID_EX_reg.mem_read;
        EX_MEM_reg.mem_write <= ID_EX_reg.mem_write;
     end
            // MEM_WB: Latch Memory outputs
            if (current_state == S_MEMORY) begin
                MEM_WB_reg.mem_to_reg <= EX_MEM_reg.mem_to_reg;
                MEM_WB_reg.mem_data <= mem_data_out;
                MEM_WB_reg.alu_result <= EX_MEM_reg.alu_result;
                MEM_WB_reg.rd <= EX_MEM_reg.rd;
                MEM_WB_reg.reg_write <= EX_MEM_reg.reg_write;
            end
        end
    end

    // --- Submodule Instantiations ---
    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_next),
        .pc_out(pc_current),
        .pc_write(pc_write_enable)
    );

    memory_unit memory (
        .clk(clk),
        .reset(reset),
        .read_en(EX_MEM_reg.mem_read),
        .write_en(EX_MEM_reg.mem_write),
        .data_in(EX_MEM_reg.rs2_val),
        .address(EX_MEM_reg.alu_result),
        .data_out(mem_data_out),
        .instruction(instr_word),
        .program_counter(pc_current)
    );

    inst_decoder dec (
        .instruction(IF_ID_reg.instruction),
        .imm(imm_value),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    logic branch_taken;
    logic [63:0] next_pc_branch;

    mem_handler mem_h (
        .op(ID_EX_reg.opcode),
        .dest(ID_EX_reg.rs1_val),
        .src(ID_EX_reg.rs2_val),
        .imm(ID_EX_reg.imm),
        .pc(ID_EX_reg.pc),
        .r31(reg_file.stack),
        .addr_out(mem_addr),
        .data_out(mem_data)
    );

    control_unit ctrl (
        .operation(ID_EX_reg.opcode),
        .dest_in(ID_EX_reg.rs1_val),
        .src_in1(ID_EX_reg.rs1_val),
        .src_in2(ID_EX_reg.rs2_val),
        .immediate(ID_EX_reg.imm),
        .current_pc(ID_EX_reg.pc),
        .memory_data(MEM_WB_reg.mem_data),
        .next_pc(next_pc_branch),
        .is_branch(), // Unconnected
        .branch_taken(branch_taken)
    );

    assign pc_write_enable = (current_state == S_WRITEBACK) || 
                            (current_state == S_EXECUTE && branch_taken && current_state != S_HALTED);
    assign pc_next = (current_state == S_EXECUTE && branch_taken) ? next_pc_branch : (pc_current + 64'd4);

    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .write_en(MEM_WB_reg.reg_write),
        .write_data(MEM_WB_reg.mem_to_reg ? MEM_WB_reg.mem_data : MEM_WB_reg.alu_result),
        .write_addr(MEM_WB_reg.rd),
        .addr1(src_reg1),
        .addr2(src_reg2),
        .data1(src_val1),
        .data2(src_val2)
    );

    alu_unit alu (
        .ctrl(ID_EX_reg.alu_op),
        .in1(ID_EX_reg.rs1_val),
        .in2(ID_EX_reg.alu_src ? ID_EX_reg.imm : ID_EX_reg.rs2_val),
        .out(alu_output)
    );

    // --- Halt Signal ---
    assign hlt = (current_state == S_HALTED);

endmodule

//------------------------------------------------------------------------------
// Supporting Modules (MUST ALSO BE MODIFIED for always_comb/always_ff)
//------------------------------------------------------------------------------

// Modified Fetch Unit (Using always @)
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic pc_write,
    input logic [63:0] pc_in,
    output logic [63:0] pc_out
);
    logic [63:0] pc_reg;
    assign pc_out = pc_reg;

    always @(posedge clk or posedge reset) begin // Changed from always_ff
        if (reset) begin
            pc_reg <= 64'h2000;
        end else if (pc_write) begin
            pc_reg <= pc_in;
        end
    end
endmodule

// Modified Memory Unit (Using always @)
module memory_unit (
    input logic [63:0] program_counter,
    input logic clk,
    input logic reset,
    input logic read_en,
    input logic write_en,
    input logic [63:0] data_in,
    input logic [63:0] address,
    output logic [63:0] data_out,
    output logic [31:0] instruction
);
    localparam MEM_SIZE_BYTES = 524288;
    logic [7:0] bytes [0:MEM_SIZE_BYTES-1];
    integer j, k;

    assign instruction[7:0]   = (address + 0 < MEM_SIZE_BYTES) ? bytes[address + 0] : 8'h0;
    // ... other assign statements for instruction/data_out ...
    assign instruction[15:8]  = (address + 1 < MEM_SIZE_BYTES) ? bytes[address + 1] : 8'h0;
    assign instruction[23:16] = (address + 2 < MEM_SIZE_BYTES) ? bytes[address + 2] : 8'h0;
    assign instruction[31:24] = (address + 3 < MEM_SIZE_BYTES) ? bytes[address + 3] : 8'h0;
    assign data_out[7:0]   = (address + 0 < MEM_SIZE_BYTES) ? bytes[address + 0] : 8'h0;
    assign data_out[15:8]  = (address + 1 < MEM_SIZE_BYTES) ? bytes[address + 1] : 8'h0;
    assign data_out[23:16] = (address + 2 < MEM_SIZE_BYTES) ? bytes[address + 2] : 8'h0;
    assign data_out[31:24] = (address + 3 < MEM_SIZE_BYTES) ? bytes[address + 3] : 8'h0;
    assign data_out[39:32] = (address + 4 < MEM_SIZE_BYTES) ? bytes[address + 4] : 8'h0;
    assign data_out[47:40] = (address + 5 < MEM_SIZE_BYTES) ? bytes[address + 5] : 8'h0;
    assign data_out[55:48] = (address + 6 < MEM_SIZE_BYTES) ? bytes[address + 6] : 8'h0;
    assign data_out[63:56] = (address + 7 < MEM_SIZE_BYTES) ? bytes[address + 7] : 8'h0;


    always @(posedge clk or posedge reset) begin // Changed from always_ff
        if (reset) begin
            for (j = 0; j < MEM_SIZE_BYTES; j = j + 1)
                bytes[j] <= 8'h0;
        end
        else if (write_en) begin
            for (k = 0; k < 8; k = k + 1) begin
                 if ((address + k) < MEM_SIZE_BYTES) begin
                    bytes[address + k] <= data_in[8*k +: 8];
                 end
            end
        end
    end
endmodule


module control_unit (
    input logic [4:0] operation,
    input logic [63:0] dest_in,
    input logic [63:0] src_in1,
    input logic [63:0] src_in2,
    input logic [63:0] immediate,
    input logic [63:0] current_pc,
    input logic [63:0] memory_data,
    output logic [63:0] next_pc,
    output logic is_branch,
    output logic branch_taken
);
    always @(*) begin
        next_pc = current_pc + 64'd4;
        branch_taken = 1'b0;
        is_branch = (operation >= 5'h08 && operation <= 5'h0e);
        case (operation)
            5'h08: begin // br $r_d
                next_pc = dest_in;
                branch_taken = 1'b1;
            end
            5'h0a: begin // brr L
                next_pc = current_pc + immediate;
                branch_taken = 1'b1;
            end
            5'h0b: begin // brnz $r_d, $r_s
                if (src_in1 != 64'b0) begin
                    next_pc = dest_in;
                    branch_taken = 1'b1;
                end
            end
            5'h0c: begin // call $r_d
                next_pc = dest_in;
                branch_taken = 1'b1;
            end
            5'h0d: begin // return
                next_pc = memory_data;
                branch_taken = 1'b1;
            end
            5'h0e: begin // brgt $r_d, $r_s, $r_t
                if ($signed(src_in1) > $signed(src_in2)) begin
                    next_pc = dest_in;
                    branch_taken = 1'b1;
                end
            end
            default: begin
                next_pc = current_pc + 64'd4;
                branch_taken = 1'b0;
            end
        endcase
    end
endmodule


module mem_handler (
    input logic [4:0] op,
    input logic [63:0] dest,
    input logic [63:0] src,
    input logic [63:0] imm,
    input logic [63:0] pc,
    input logic [63:0] r31,
    output logic [63:0] addr_out,
    output logic [63:0] data_out
);
    always @(*) begin
        addr_out = 64'h0;
        data_out = 64'h0;
        case (op)
            5'h10: begin // load: Mem[$r_s + L]
                addr_out = src + imm;
                data_out = 64'h0;
            end
            5'h13: begin // store: Mem[$r_d + L] = $r_s
                addr_out = dest + imm;
                data_out = src;
            end
            5'h0c: begin // call
                addr_out = r31 - 8;
                data_out = pc + 64'd4;
            end
            5'h0d: begin // return
                addr_out = r31 - 8;
                data_out = 64'h0;
            end
            default: begin
                addr_out = 64'h0;
                data_out = 64'h0;
            end
        endcase
    end
endmodule

// Modified Register File (Using always @)
// Modified Register File (Using always @) - R0 NOT hardwired
module reg_file_bank (
    input logic clk,
    input logic reset,
    input logic write_en,
    input logic [63:0] write_data,
    input logic [4:0] addr1,
    input logic [4:0] addr2,
    input logic [4:0] write_addr,
    output logic [63:0] data1,
    output logic [63:0] data2,
    output logic [63:0] data_dest,
    output logic [63:0] stack
);
    logic [63:0] registers [0:31];
    integer i;
    localparam MEMSIZE = 64'd524288;

    // Modified assignments to read actual register values
    assign data1 = registers[addr1];
    assign data2 = registers[addr2];
    assign data_dest = registers[write_addr];
    assign stack = registers[31];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1) begin registers[i] <= 64'h0; end
            registers[31] <= MEMSIZE;
        end else if (write_en) begin
            registers[write_addr] <= write_data;
        end
    end
endmodule

module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic [63:0] out
);
    always @(*) begin // Changed from always_comb
        case (ctrl)
            5'b11000: out = in1 + in2;
            5'b11001: out = in1 + in2;
            5'b01010: out = in1 + in2; // ADDI: add immediate
            5'b11010: out = in1 - in2;
            5'b11011: out = in1 - in2;
            5'b01011: out = in1 - in2; // SUBI: subtract immediate
            5'b11100: out = in1 * in2;
            5'b11101: out = (in2 == 0) ? 64'h0 : in1 / in2;
            5'b00000: out = in1 & in2;
            5'b00001: out = in1 | in2;
            5'b00010: out = in1 ^ in2;
            5'b00011: out = ~in1;
            5'b00100: out = in1 >> in2;
            5'b00101: out = $signed(in1) >>> in2;
            5'b00110: out = in1 << in2;
            5'b00111: out = in1 << in2;
            5'b10001: out = in1;
            5'b10010: out = in2;
            default: out = 64'h0;
        endcase
    end
endmodule
// Instruction Decoder (Modified src1 mapping for ADDI/SUBI/SHFTRI/SHFTLI)
module inst_decoder (
    input logic [31:0] instruction,
    output logic [63:0] imm,
    output logic [4:0] dest,         // Assigned normally from dest field
    output logic [4:0] src1,         // Now conditionally assigned
    output logic [4:0] src2,         // Assigned normally from src2 field
    output logic [4:0] opcode
);
    logic [11:0] imm_raw;
    logic [4:0] opcode_internal; // Internal signal for opcode

    // Decode fields that are always the same position or mapping
    assign imm_raw = instruction[11:0];
    assign opcode_internal = instruction[31:27];
    assign opcode = opcode_internal; // Assign to output port
    assign imm = {{52{imm_raw[11]}}, imm_raw}; // Sign extend immediate
    assign dest = instruction[26:22];          // Destination register is always field [26:22]
    assign src2 = instruction[16:12];          // src2 output always comes from src2 field [16:12]

    // Determine src1 register address based on opcode
    always @(*) begin
        // Default mapping: src1 normally comes from field [21:17]
        src1 = instruction[21:17];

        // Override mapping specifically for certain I-type ALU instructions
        if (opcode_internal == 5'b11001 || // addi (Op1)
            opcode_internal == 5'b11011 || // subi (Op1)
            opcode_internal == 5'b01010 || // addi (Op2)
            opcode_internal == 5'b01011 || // subi (Op2)
            opcode_internal == 5'b00101 || // shftri <-- ADDED
            opcode_internal == 5'b00111 )  // shftli <-- ADDED
        begin
            // For these I-type ops: Use the destination register field [26:22] as src1 address
            src1 = instruction[26:22];
        end
        // For all other opcodes (R-type, branches, loads, stores, etc.), the default mapping applies.
    end

endmodule

module reg_lit_mux (
    input logic [4:0] op,
    input logic [63:0] reg_val,
    input logic [63:0] lit_val,
    output logic [63:0] out
);
    always @(*) begin
        if (op == 5'b11001 || op == 5'b11011 ||
            op == 5'b00101 || op == 5'b00111 ||
            op == 5'b10010 ||
            op == 5'b10000 || op == 5'b10011 ||
            op == 5'b01010 || op == 5'b01011)
            out = lit_val;
        else
            out = reg_val;
    end
endmodule