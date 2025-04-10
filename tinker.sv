// Top-Level Module
module tinker_core (
    input logic clk,
    input logic reset
);
    logic [4:0] target_reg, source_reg1, source_reg2, operation;
    logic [31:0] instruction_word;
    logic [63:0] program_counter, next_counter;
    logic [63:0] immediate, target_value, source_value1, source_value2, second_operand, alu_result;
    logic [63:0] stack_pointer, memory_write_value, memory_address, memory_read_data;
    logic memory_write_enable, reg_write_from_mem, reg_write_from_alu;

    fetch_unit fetcher (
        .clk(clk),
        .reset(reset),
        .stack_reg(stack_pointer),
        .next_pc(next_counter),
        .current_pc(program_counter)
    );

    memory_unit mem (
        .pc(program_counter),
        .clk(clk),
        .reset(reset),
        .write_enable(memory_write_enable),
        .data_in(memory_write_value),
        .addr(memory_address),
        .data_out(memory_read_data),
        .instr(instruction_word)
    );

    control_unit controller (
        .opcode(operation),
        .dest(target_value),
        .src1(source_value1),
        .src2(source_value2),
        .imm(immediate),
        .pc_in(program_counter),
        .mem_in(memory_read_data),
        .pc_out(next_counter)
    );

    mem_handler mem_control (
        .opcode(operation),
        .dest(target_value),
        .src(source_value1),
        .imm(immediate),
        .pc(program_counter),
        .r31(stack_pointer),
        .rw_addr(memory_address),
        .write_data(memory_write_value),
        .write_flag(memory_write_enable),
        .reg_write(reg_write_from_mem)
    );

    inst_decoder decoder (
        .instr(instruction_word),
        .imm_out(immediate),
        .rd(target_reg),
        .rs1(source_reg1),
        .rs2(source_reg2),
        .op(operation)
    );

    reg_file_bank registers (
        .clk(clk),
        .reset(reset),
        .write_mem(reg_write_from_mem),
        .write_alu(reg_write_from_alu),
        .data_in(reg_write_from_mem ? memory_read_data : alu_result),
        .read_addr1(source_reg1),
        .read_addr2(source_reg2),
        .write_addr(target_reg),
        .read_data1(source_value1),
        .read_data2(source_value2),
        .write_data(target_value),
        .sp(stack_pointer)
    );

    reg_lit_mux mux (
        .select(operation),
        .register_in(source_value2),
        .literal(immediate),
        .mux_out(second_operand)
    );

    alu_unit alu (
        .op(operation),
        .operand1(source_value1),
        .operand2(second_operand),
        .write_enable(reg_write_from_alu),
        .result(alu_result)
    );
endmodule

// ALU Unit
module alu_unit (
    input logic [4:0] op,
    input logic [63:0] operand1,
    input logic [63:0] operand2,
    output logic write_enable,
    output logic [63:0] result
);
    real float1, float2, float_res;
    assign float1 = $bitstoreal(operand1);
    assign float2 = $bitstoreal(operand2);

    always_comb begin
        write_enable = 1'b1;
        if (op == 5'b11000) result = operand1 + operand2;  // add
        else if (op == 5'b11001) result = operand1 + operand2;  // addi
        else if (op == 5'b11010) result = operand1 - operand2;  // sub
        else if (op == 5'b11011) result = operand1 - operand2;  // subi
        else if (op == 5'b11100) result = operand1 * operand2;  // mul
        else if (op == 5'b11101) result = operand1 / operand2;  // div
        else if (op == 5'b00000) result = operand1 & operand2;  // and
        else if (op == 5'b00001) result = operand1 | operand2;  // or
        else if (op == 5'b00010) result = operand1 ^ operand2;  // xor
        else if (op == 5'b00011) result = ~operand1;            // not
        else if (op == 5'b00100) result = operand1 >> operand2; // shftr
        else if (op == 5'b00101) result = operand1 >> operand2; // shftri
        else if (op == 5'b00110) result = operand1 << operand2; // shftl
        else if (op == 5'b00111) result = operand1 << operand2; // shftli
        else if (op == 5'b10001) result = operand1;            // mov $r_d, $r_s
        else if (op == 5'b10010) result = {operand1[63:12], operand2[11:0]}; // mov $r_d, L
        else if (op == 5'b10100) begin                         // addf
            float_res = float1 + float2;
            result = $realtobits(float_res);
        end
        else if (op == 5'b10101) begin                         // subf
            float_res = float1 - float2;
            result = $realtobits(float_res);
        end
        else if (op == 5'b10110) begin                         // mulf
            float_res = float1 * float2;
            result = $realtobits(float_res);
        end
        else if (op == 5'b10111) begin                         // divf
            float_res = float1 / float2;
            result = $realtobits(float_res);
        end
        else begin
            write_enable = 1'b0;
            result = 64'b0;
        end
    end
endmodule

// Register/Literal Mux
module reg_lit_mux (
    input logic [4:0] select,
    input logic [63:0] register_in,
    input logic [63:0] literal,
    output logic [63:0] mux_out
);
    always_comb begin
        casez (select)
            5'b11001: mux_out = literal; // addi
            5'b11011: mux_out = literal; // subi
            5'b00101: mux_out = literal; // shftri
            5'b00111: mux_out = literal; // shftli
            5'b10010: mux_out = literal; // mov $r_d, L
            default:  mux_out = register_in;
        endcase
    end
endmodule

// Register File
module reg_file_bank (
    input logic clk,
    input logic reset,
    input logic write_mem,
    input logic write_alu,
    input logic [63:0] data_in,
    input logic [4:0] read_addr1,
    input logic [4:0] read_addr2,
    input logic [4:0] write_addr,
    output logic [63:0] read_data1,
    output logic [63:0] read_data2,
    output logic [63:0] write_data,
    output logic [63:0] sp
);
    logic [63:0] registers [0:31];
    logic write_control;
    integer idx;

    assign write_control = write_mem | write_alu;
    assign read_data1 = registers[read_addr1];
    assign read_data2 = registers[read_addr2];
    assign write_data = registers[write_addr];
    assign sp = registers[31];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (idx = 0; idx < 31; idx = idx + 1)
                registers[idx] <= 64'h0;
            registers[31] <= 64'd524288;
        end
        else if (write_control)
            registers[write_addr] <= data_in;
    end
endmodule

// Instruction Decoder
module inst_decoder (
    input logic [31:0] instr,
    output logic [63:0] imm_out,
    output logic [4:0] rd,
    output logic [4:0] rs1,
    output logic [4:0] rs2,
    output logic [4:0] op
);
    always_comb begin
        op = instr[31:27];
        rd = instr[26:22];
        rs1 = instr[21:17];
        rs2 = instr[16:12];
        imm_out = {52'h0, instr[11:0]};

        if (op == 5'b11001 || op == 5'b11011 || op == 5'b00101 || 
            op == 5'b00111 || op == 5'b10010)
            rs1 = rd;
    end
endmodule

// Memory Handler
module mem_handler (
    input logic [4:0] opcode,
    input logic [63:0] dest,
    input logic [63:0] src,
    input logic [63:0] imm,
    input logic [63:0] pc,
    input logic [63:0] r31,
    output logic [63:0] rw_addr,
    output logic [63:0] write_data,
    output logic write_flag,
    output logic reg_write
);
    always_comb begin
        if (opcode == 5'b01100) begin       // call
            rw_addr = r31 - 8;
            write_data = pc + 4;
            write_flag = 1;
            reg_write = 0;
        end
        else if (opcode == 5'b01101) begin  // return
            rw_addr = r31 - 8;
            write_data = 64'h0;
            write_flag = 0;
            reg_write = 0;
        end
        else if (opcode == 5'b10000) begin  // mov $r_d, ($r_s)(L)
            rw_addr = src + imm;
            write_data = 64'h0;
            write_flag = 0;
            reg_write = 1;
        end
        else if (opcode == 5'b10011) begin  // mov ($r_d)(L), $r_s
            rw_addr = dest + imm;
            write_data = src;
            write_flag = 1;
            reg_write = 0;
        end
        else begin
            rw_addr = 64'h2000;
            write_data = 64'h0;
            write_flag = 0;
            reg_write = 0;
        end
    end
endmodule

// Control Unit
module control_unit (
    input logic [4:0] opcode,
    input logic [63:0] dest,
    input logic [63:0] src1,
    input logic [63:0] src2,
    input logic [63:0] imm,
    input logic [63:0] pc_in,
    input logic [63:0] mem_in,
    output logic [63:0] pc_out
);
    always_comb begin
        case (opcode)
            5'b01000: pc_out = dest;                   // br
            5'b01001: pc_out = pc_in + dest;           // brr $r_d
            5'b01010: pc_out = pc_in + $signed(imm);   // brr L
            5'b01011: pc_out = (src1 != 0) ? dest : pc_in + 4; // brnz
            5'b01100: pc_out = dest;                   // call
            5'b01101: pc_out = mem_in;                 // return
            5'b01110: pc_out = (src1 > src2) ? dest : pc_in + 4; // brgt
            default:  pc_out = pc_in + 4;
        endcase
    end
endmodule

// Memory Unit
module memory_unit (
    input logic [63:0] pc,
    input logic clk,
    input logic reset,
    input logic write_enable,
    input logic [63:0] data_in,
    input logic [63:0] addr,
    output logic [63:0] data_out,
    output logic [31:0] instr
);
    logic [7:0] bytes [0:524287];
    integer i;

    assign instr = {bytes[pc+3], bytes[pc+2], bytes[pc+1], bytes[pc]};
    assign data_out = {bytes[addr+7], bytes[addr+6], bytes[addr+5], bytes[addr+4],
                       bytes[addr+3], bytes[addr+2], bytes[addr+1], bytes[addr]};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 524288; i = i + 1)
                bytes[i] <= 8'h0;
        end
        else if (write_enable) begin
            {bytes[addr+7], bytes[addr+6], bytes[addr+5], bytes[addr+4],
             bytes[addr+3], bytes[addr+2], bytes[addr+1], bytes[addr]} <= data_in;
        end
    end
endmodule

// Fetch Unit
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic [63:0] stack_reg,
    input logic [63:0] next_pc,
    output logic [63:0] current_pc
);
    logic [63:0] pc_reg;
    assign current_pc = pc_reg;

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_reg <= 64'h2000;
        else
            pc_reg <= next_pc;
    end
endmodule