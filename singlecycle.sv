// Top-Level Module
module tinker_core (
    input logic clk,
    input logic reset
);
    logic [4:0] dest_reg, src_reg1, src_reg2, opcode;
    logic [31:0] instr_word;
    logic [63:0] pc_current, pc_next;
    logic [63:0] imm_value, dest_val, src_val1, src_val2, alu_operand2, alu_output;
    logic [63:0] stack_ptr, mem_data_in, mem_addr, mem_data_out;
    logic mem_write, write_from_mem, write_from_alu;

    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .stack(stack_ptr),
        .pc_in(pc_next),
        .pc_out(pc_current)
    );

    memory_unit memory (  
        .program_counter(pc_current),
        .clk(clk),
        .reset(reset),
        .write_en(mem_write),
        .data_in(mem_data_in),
        .address(mem_addr),
        .data_out(mem_data_out),
        .instruction(instr_word)
    );

    inst_decoder dec (
        .instruction(instr_word),
        .imm(imm_value),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    reg_file_bank reg_file (  
        .clk(clk),
        .reset(reset),
        .mem_write_en(write_from_mem),
        .alu_write_en(write_from_alu),
        .write_data(write_from_mem ? mem_data_out : alu_output),
        .addr1(src_reg1),
        .addr2(src_reg2),
        .write_addr(dest_reg),
        .data1(src_val1),
        .data2(src_val2),
        .data_dest(dest_val),
        .stack(stack_ptr)
    );

    reg_lit_mux mux (
        .op(opcode),
        .reg_val(src_val2),
        .lit_val(imm_value),
        .out(alu_operand2)
    );

    alu_unit alu (
        .ctrl(opcode),
        .in1(src_val1),
        .in2(alu_operand2),
        .valid(write_from_alu),
        .out(alu_output)
    );
endmodule



// ALU Unit
module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic valid,
    output logic [63:0] out
);
    real f1, f2, fres;
    assign f1 = $bitstoreal(in1);
    assign f2 = $bitstoreal(in2);

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
            5'b10100: begin                    // addf
                fres = f1 + f2;
                out = $realtobits(fres);
            end
            5'b10101: begin                    // subf
                fres = f1 - f2;
                out = $realtobits(fres);
            end
            5'b10110: begin                    // mulf
                fres = f1 * f2;
                out = $realtobits(fres);
            end
            5'b10111: begin                    // divf
                fres = f1 / f2;
                out = $realtobits(fres);
            end
            default: begin
                valid = 0;
                out = 64'h0;
            end
        endcase
    end
endmodule

// Register/Literal Mux
module reg_lit_mux (
    input logic [4:0] op,
    input logic [63:0] reg_val,
    input logic [63:0] lit_val,
    output logic [63:0] out
);
    always_comb begin
        if (op == 5'b11001 || op == 5'b11011 || op == 5'b00101 || 
            op == 5'b00111 || op == 5'b10010)
            out = lit_val;
        else
            out = reg_val;
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
    input logic IF_DE_pc,
    input logic [31:0] IF_DE_instruction,
    output logic [63:0] DE_EX_imm,
    output logic [4:0] DE_EX_dest,
    output logic [4:0] DE_EX_src1,
    output logic [4:0] DE_EX_src2,
    output logic [4:0] DE_EX_opcode
    output logic DE_EX_pc
);
    always_comb begin
        DE_EX_opcode = IF_DE_instruction[31:27];
        DE_EX_dest = IF_DE_instruction[26:22];
        DE_EX_src1 = IF_DE_instruction[21:17];
        DE_EX_src2 = IF_DE_instruction[16:12];
        DE_EX_imm = {52'h0, IF_DE_instruction[11:0]};
        case (opcode)
            5'b11001, 5'b11011, 5'b00101, 5'b00111, 5'b10010: DE_EX_src1 = DE_EX_dest;
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

    assign instruction[31:24] = bytes[program_counter+3];
    assign instruction[23:16] = bytes[program_counter+2];
    assign instruction[15:8]  = bytes[program_counter+1];
    assign instruction[7:0]   = bytes[program_counter];
    assign data_out[63:56] = bytes[address+7];
    assign data_out[55:48] = bytes[address+6];
    assign data_out[47:40] = bytes[address+5];
    assign data_out[39:32] = bytes[address+4];
    assign data_out[31:24] = bytes[address+3];
    assign data_out[23:16] = bytes[address+2];
    assign data_out[15:8]  = bytes[address+1];
    assign data_out[7:0]   = bytes[address];

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

// Fetch Unit
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic [63:0] stack,
    input logic [63:0] pc_in,
    output logic [63:0] pc_out
);
    logic [63:0] pc;
    assign pc_out = pc;

    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc <= 64'h2000;
        else
            pc <= pc_in;
    end
endmodule