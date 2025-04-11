// Top-Level Multi-Cycle FSM Module (Simplified for ALU Ops)
module tinker_core (
    input logic clk,
    input logic reset
);
    // State Definition
    typedef enum logic [2:0] {  // Reduced states for ALU ops
        S_FETCH,
        S_DECODE,
        S_EXECUTE,
        S_WRITEBACK
    } StateType;

    StateType current_state, next_state;

    // Datapath Registers
    logic [63:0] pc;
    logic [31:0] ir;
    logic [63:0] a, b;
    logic [63:0] alu_out;

    // Decoded Instruction Fields
    logic [4:0] opcode;
    logic [4:0] dest_reg;
    logic [4:0] src_reg1;
    logic [4:0] src_reg2;
    logic [63:0] immediate;

    // Internal Wires
    logic [63:0] reg_read_data1, reg_read_data2, reg_write_data;
    logic [63:0] mem_read_data;
    logic [63:0] alu_in1, alu_in2;
    logic [31:0] instr_word;

    // Control Signals
    logic pc_write, ir_write, a_write, b_write, alu_out_write;
    logic reg_write;
    logic alu_src_b;  // 0: B, 1: Immediate

    // Instantiate Functional Units
    logic alu_valid;
    alu_unit alu (
        .ctrl(opcode),
        .in1(alu_in1),
        .in2(alu_in2),
        .valid(alu_valid),
        .out(alu_out_internal)
    );
    logic [63:0] alu_out_internal;

    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .mem_write_en(1'b0),
        .alu_write_en(reg_write),
        .write_data(reg_write_data),
        .addr1(src_reg1),
        .addr2(src_reg2),
        .write_addr(dest_reg),
        .data1(reg_read_data1),
        .data2(reg_read_data2),
        .data_dest(),
        .stack()
    );

    memory_unit memory (
        .program_counter(pc),
        .clk(clk),
        .reset(reset),
        .write_en(1'b0),  // No writes for ALU tests
        .data_in(64'h0),
        .address(pc),
        .data_out(mem_read_data),
        .instruction(instr_word)
    );

    inst_decoder dec (
        .instruction(ir),
        .imm(immediate),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    // Datapath MUXes
    assign alu_in1 = a;
    assign alu_in2 = alu_src_b ? immediate : b;
    assign reg_write_data = alu_out;

    // State Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= S_FETCH;
        else
            current_state <= next_state;
    end

    // Datapath Register Updates
    always_ff @(posedge clk) begin
        if (reset) begin
            pc <= 64'h2000;  // Match original reset PC
        end else if (pc_write) begin
            pc <= pc + 4;
        end
        if (ir_write) ir <= instr_word;
        if (a_write) a <= reg_read_data1;
        if (b_write) b <= reg_read_data2;
        if (alu_out_write) alu_out <= alu_out_internal;
    end

    // FSM Next State Logic
    always_comb begin
        next_state = current_state;
        case (current_state)
            S_FETCH: next_state = S_DECODE;
            S_DECODE: begin
                case (opcode)
                    5'b11000, 5'b11010, 5'b11100, 5'b11101, // add, sub, mul, div
                    5'b00000, 5'b00001, 5'b00010, 5'b00011, // and, or, xor, not
                    5'b11001, 5'b11011:                     // addi, subi
                        next_state = S_EXECUTE;
                    default: next_state = S_FETCH;  // Skip unsupported ops
                endcase
            end
            S_EXECUTE: next_state = S_WRITEBACK;
            S_WRITEBACK: next_state = S_FETCH;
            default: next_state = S_FETCH;
        endcase
    end

    // FSM Output Logic
    always_comb begin
        pc_write = 0;
        ir_write = 0;
        a_write = 0;
        b_write = 0;
        alu_out_write = 0;
        reg_write = 0;
        alu_src_b = 0;

        case (current_state)
            S_FETCH: begin
                ir_write = 1;
                pc_write = 1;
            end
            S_DECODE: begin
                a_write = 1;
                b_write = 1;
                alu_src_b = (opcode == 5'b11001 || opcode == 5'b11011); // addi, subi
            end
            S_EXECUTE: begin
                alu_out_write = 1;
            end
            S_WRITEBACK: begin
                reg_write = 1;
            end
        endcase
    end
endmodule

// ALU Unit
module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic valid,
    output logic [63:0] out
);
    always_comb begin
        valid = 1;
        case (ctrl)
            5'b11000: out = in1 + in2;  // add
            5'b11001: out = in1 + in2;  // addi
            5'b11010: out = in1 - in2;  // sub
            5'b11011: out = in1 - in2;  // subi
            5'b11100: out = in1 * in2;  // mul
            5'b11101: out = in1 / in2;  // div
            5'b00000: out = in1 & in2;  // and
            5'b00001: out = in1 | in2;  // or
            5'b00010: out = in1 ^ in2;  // xor
            5'b00011: out = ~in1;       // not
            default: begin
                valid = 0;
                out = 64'h0;
            end
        endcase
    end
endmodule

// Register File
module reg_file_bank (
    input logic clk,
    input logic reset,
    input logic mem_write_en,
    input logic alu_write_en,
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
    logic write_en;
    integer i;

    assign write_en = mem_write_en | alu_write_en;
    assign data1 = registers[addr1];
    assign data2 = registers[addr2];
    assign data_dest = registers[write_addr];
    assign stack = registers[31];

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'h0;
            registers[31] <= 64'd524288;
        end
        else if (write_en)
            registers[write_addr] <= write_data;
    end
endmodule

// Instruction Decoder
module inst_decoder (
    input logic [31:0] instruction,
    output logic [63:0] imm,
    output logic [4:0] dest,
    output logic [4:0] src1,
    output logic [4:0] src2,
    output logic [4:0] opcode
);
    always_comb begin
        opcode = instruction[31:27];
        dest = instruction[26:22];
        src1 = instruction[21:17];
        src2 = instruction[16:12];
        imm = {{52{instruction[11]}}, instruction[11:0]};
        case (opcode)
            5'b11001, 5'b11011: src1 = dest;  // addi, subi use dest as src1
            default: ;
        endcase
    end
endmodule

// Memory Unit
module memory_unit (
    input logic [63:0] program_counter,
    input logic clk,
    input logic reset,
    input logic write_en,
    input logic [63:0] data_in,
    input logic [63:0] address,
    output logic [63:0] data_out,
    output logic [31:0] instruction
);
    logic [7:0] bytes [0:524287];
    integer j, k;

    assign instruction = {bytes[program_counter+3], bytes[program_counter+2],
                          bytes[program_counter+1], bytes[program_counter]};
    assign data_out = {bytes[address+7], bytes[address+6], bytes[address+5], bytes[address+4],
                       bytes[address+3], bytes[address+2], bytes[address+1], bytes[address]};

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (j = 0; j < 524288; j = j + 1)
                bytes[j] <= 8'h0;
        end
        else if (write_en) begin
            for (k = 0; k < 8; k = k + 1)
                bytes[address + k] <= data_in[8*k +: 8];
        end
    end
endmodule