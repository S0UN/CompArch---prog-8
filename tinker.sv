// Define HALT opcode
`define HALT_OPCODE 5'b11111

// Top-Level Multi-Cycle FSM Module
module tinker_core (
    input logic clk,
    input logic reset,
    output logic hlt
);
    // State Definition
    typedef enum logic [3:0] {
        S_FETCH,
        S_DECODE,
        S_EXEC_ADDR,
        S_EXEC_RTYPE,
        S_EXEC_ITYPE,
        S_EXEC_BRANCH,
        S_EXEC_JUMP,
        S_MEM_READ,
        S_MEM_WRITE,
        S_MEM_WB,
        S_ALU_WB,
        S_HALT
    } StateType;

    StateType current_state, next_state;

    // Datapath Registers
    logic [63:0] pc;
    logic [31:0] ir;
    logic [63:0] a, b;
    logic [63:0] alu_out;
    logic [63:0] mdr;
    logic [63:0] stack_ptr_reg;

    // Decoded Instruction Fields
    logic [4:0] opcode;
    logic [4:0] dest_reg;
    logic [4:0] src_reg1;
    logic [4:0] src_reg2;
    logic [63:0] immediate;

    // Internal Wires
    logic [63:0] reg_read_data1, reg_read_data2, reg_write_data;
    logic [63:0] mem_read_data, mem_write_data;
    logic [63:0] alu_in1, alu_in2;
    logic [63:0] next_pc_select;
    logic [63:0] mem_addr_select;

    // Control Signals
    logic pc_write, ir_write, a_write, b_write, alu_out_write, mdr_write;
    logic reg_write, mem_write;
    logic [1:0] pc_source;
    logic [4:0] alu_op_select;
    logic alu_src_a;
    logic [1:0] alu_src_b;
    logic adr_source;
    logic mem_to_reg;

    // Instantiate Functional Units
    logic alu_valid;
    alu_unit alu (
        .ctrl(alu_op_select),
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
        .stack(stack_ptr_reg)
    );

    memory_unit memory (
        .program_counter(pc),
        .clk(clk),
        .reset(reset),
        .write_en(mem_write),
        .data_in(mem_write_data),
        .address(mem_addr_select),
        .data_out(mem_read_data),
        .instruction(ir_internal)
    );
    logic [31:0] ir_internal;

    inst_decoder dec (
        .instruction(ir),
        .imm(immediate),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    // Datapath MUXes
    always_comb begin
        case (pc_source)
            2'b00: next_pc_select = pc + 4;
            2'b01: next_pc_select = alu_out;
            2'b10: next_pc_select = mdr;
            2'b11: next_pc_select = a;
            default: next_pc_select = pc + 4;
        endcase
    end

    assign mem_addr_select = adr_source ? alu_out : pc;
    assign alu_in1 = alu_src_a ? a : pc;

    always_comb begin
        case (alu_src_b)
            2'b00: alu_in2 = b;
            2'b01: alu_in2 = immediate;
            2'b10: alu_in2 = 64'd4;
            2'b11: alu_in2 = 64'd8;
            default: alu_in2 = 64'b0;
        endcase
    end

    assign reg_write_data = mem_to_reg ? mdr : alu_out;
    assign mem_write_data = (opcode == 5'b01100) ? (pc + 4) : b;

    // State Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= S_FETCH;
        else
            current_state <= next_state;
    end

    // Datapath Register Updates
    always_ff @(posedge clk) begin
        if (reset) pc <= 64'h2000;  // Initialize PC on reset
        else if (pc_write) pc <= next_pc_select;
        if (ir_write) ir <= ir_internal;
        if (a_write) a <= reg_read_data1;
        if (b_write) b <= reg_read_data2;
        if (alu_out_write) alu_out <= alu_out_internal;
        if (mdr_write) mdr <= mem_read_data;
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
                    5'b00100, 5'b00110, 5'b10001:           // shftr, shftl, mov $r_d, $r_s
                        next_state = S_EXEC_RTYPE;
                    5'b11001, 5'b11011, 5'b00101, 5'b00111, // addi, subi, shftri, shftli
                    5'b10010:                               // mov $r_d, L
                        next_state = S_EXEC_ITYPE;
                    5'b10000, 5'b10011, 5'b01100, 5'b01101: // mov $r_d, ($r_s)(L), mov ($r_d)(L), $r_s, call, return
                        next_state = S_EXEC_ADDR;
                    5'b01011, 5'b01110:                     // brnz, brgt
                        next_state = S_EXEC_BRANCH;
                    5'b01000, 5'b01001, 5'b01010:           // br, brr $r_d, brr L
                        next_state = S_EXEC_JUMP;
                    `HALT_OPCODE: next_state = S_HALT;
                    default: next_state = S_FETCH;
                endcase
            end
            S_EXEC_ADDR: begin
                case (opcode)
                    5'b10000: next_state = S_MEM_READ;
                    5'b10011: next_state = S_MEM_WRITE;
                    5'b01100: next_state = S_MEM_WRITE;
                    5'b01101: next_state = S_MEM_READ;
                    default: next_state = S_FETCH;
                endcase
            end
            S_EXEC_RTYPE: next_state = S_ALU_WB;
            S_EXEC_ITYPE: next_state = S_ALU_WB;
            S_EXEC_BRANCH: next_state = S_FETCH;
            S_EXEC_JUMP: next_state = S_FETCH;
            S_MEM_READ: next_state = (opcode == 5'b01101) ? S_FETCH : S_MEM_WB;
            S_MEM_WRITE: next_state = S_FETCH;
            S_MEM_WB: next_state = S_FETCH;
            S_ALU_WB: next_state = S_FETCH;
            S_HALT: next_state = S_HALT;
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
        mdr_write = 0;
        reg_write = 0;
        mem_write = 0;
        pc_source = 2'b00;
        alu_op_select = 5'b11000;
        alu_src_a = 0;
        alu_src_b = 2'b10;
        adr_source = 0;
        mem_to_reg = 0;

        case (current_state)
            S_FETCH: begin
                adr_source = 0;
                ir_write = 1;
                alu_src_a = 0;
                alu_src_b = 2'b10;
                alu_op_select = 5'b11000;
                alu_out_write = 1;
                pc_write = 1;
                pc_source = 2'b01;
            end
            S_DECODE: begin
                a_write = 1;
                b_write = 1;
            end
            S_EXEC_ADDR: begin
                alu_src_a = 1;
                alu_src_b = (opcode == 5'b01100 || opcode == 5'b01101) ? 2'b11 : 2'b01;
                alu_op_select = (opcode == 5'b01100 || opcode == 5'b01101) ? 5'b11010 : 5'b11000;
                alu_out_write = 1;
            end
            S_EXEC_RTYPE: begin
                alu_src_a = 1;
                alu_src_b = 2'b00;
                alu_op_select = opcode;
                alu_out_write = 1;
            end
            S_EXEC_ITYPE: begin
                alu_src_a = 1;
                alu_src_b = 2'b01;
                alu_op_select = opcode;
                alu_out_write = 1;
            end
            S_EXEC_BRANCH: begin
                logic take_branch;
                take_branch = (opcode == 5'b01011) ? (a != 0) : (opcode == 5'b01110) ? ($signed(a) > $signed(b)) : 0;
                if (take_branch) begin
                    pc_write = 1;
                    pc_source = 2'b11;
                end
            end
            S_EXEC_JUMP: begin
                pc_write = 1;
                case (opcode)
                    5'b01000: pc_source = 2'b11;
                    5'b01001: begin
                        alu_src_a = 0;
                        alu_src_b = 2'b00;
                        alu_op_select = 5'b11000;
                        alu_out_write = 1;
                        pc_source = 2'b01;
                    end
                    5'b01010: begin
                        alu_src_a = 0;
                        alu_src_b = 2'b01;
                        alu_op_select = 5'b11000;
                        alu_out_write = 1;
                        pc_source = 2'b01;
                    end
                endcase
            end
            S_MEM_READ: begin
                adr_source = 1;
                mdr_write = 1;
                if (opcode == 5'b01101) begin
                    pc_write = 1;
                    pc_source = 2'b10;
                end
            end
            S_MEM_WRITE: begin
                adr_source = 1;
                mem_write = 1;
                if (opcode == 5'b01100) begin
                    pc_write = 1;
                    pc_source = 2'b11;
                end
            end
            S_MEM_WB: begin
                reg_write = 1;
                mem_to_reg = 1;
            end
            S_ALU_WB: begin
                reg_write = 1;
                mem_to_reg = 0;
            end
            S_HALT: ;
        endcase
    end

    assign hlt = (current_state == S_HALT);
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
            5'b11000: out = in1 + in2;         // add
            5'b11001: out = in1 + in2;         // addi
            5'b11010: out = in1 - in2;         // sub
            5'b11011: out = in1 - in2;         // subi
            5'b11100: out = in1 * in2;         // mul
            5'b11101: out = in1 / in2;         // div
            5'b00000: out = in1 & in2;         // and
            5'b00001: out = in1 | in2;         // or
            5'b00010: out = in1 ^ in2;         // xor
            5'b00011: out = ~in1;              // not
            5'b00100: out = in1 >> in2;        // shftr
            5'b00101: out = in1 >> in2;        // shftri
            5'b00110: out = in1 << in2;        // shftl
            5'b00111: out = in1 << in2;        // shftli
            5'b10001: out = in1;               // mov $r_d, $r_s
            5'b10010: out = {in1[63:12], in2[11:0]}; // mov $r_d, L
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
    output logic [63:64] imm,
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
            5'b11001, 5'b11011, 5'b00101, 5'b00111, 5'b10010: src1 = dest;
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