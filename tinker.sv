// Fetch Unit
module fetch_unit (
    input logic clk,
    input logic rst,
    input logic [63:0] r31,
    input logic [63:0] next_pc_in,
    output logic [63:0] curr_pc
);
    logic [63:0] prog_counter;
    assign curr_pc = prog_counter;
    always @(posedge clk or posedge rst) begin
        if (rst)
            prog_counter <= 64'h2000;
        else
            prog_counter <= next_pc_in;
    end
endmodule

// Memory Unit
module memory_unit (
    input logic [63:0] curr_pc,
    input logic clk,
    input logic rst,
    input logic flag,
    input logic [63:0] wr_data,
    input logic [63:0] mem_addr,
    output logic [63:0] rd_data,
    output logic [31:0] inst_word,
    inout logic [7:0] bytes [0:524287] // Exposed for testbench access
);
    logic [7:0] mem_bytes [0:524287];
    integer j;
    assign inst_word[7:0] = mem_bytes[curr_pc];
    assign inst_word[15:8] = mem_bytes[curr_pc+1];
    assign inst_word[23:16] = mem_bytes[curr_pc+2];
    assign inst_word[31:24] = mem_bytes[curr_pc+3];
    assign rd_data[7:0] = mem_bytes[mem_addr];
    assign rd_data[15:8] = mem_bytes[mem_addr+1];
    assign rd_data[23:16] = mem_bytes[mem_addr+2];
    assign rd_data[31:24] = mem_bytes[mem_addr+3];
    assign rd_data[39:32] = mem_bytes[mem_addr+4];
    assign rd_data[47:40] = mem_bytes[mem_addr+5];
    assign rd_data[55:48] = mem_bytes[mem_addr+6];
    assign rd_data[63:56] = mem_bytes[mem_addr+7];
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (j = 0; j < 524288; j = j + 1)
                mem_bytes[j] <= 8'b0;
        end else if (flag) begin
            mem_bytes[mem_addr] <= wr_data[7:0];
            mem_bytes[mem_addr+1] <= wr_data[15:8];
            mem_bytes[mem_addr+2] <= wr_data[23:16];
            mem_bytes[mem_addr+3] <= wr_data[31:24];
            mem_bytes[mem_addr+4] <= wr_data[39:32];
            mem_bytes[mem_addr+5] <= wr_data[47:40];
            mem_bytes[mem_addr+6] <= wr_data[55:48];
            mem_bytes[mem_addr+7] <= wr_data[63:56];
        end
    end
    // Connect internal array to external port
    generate
        genvar i;
        for (i = 0; i < 524288; i = i + 1) begin : mem_connect
            assign bytes[i] = mem_bytes[i];
        end
    endgenerate
endmodule

// Control Unit
module control_unit (
    input logic [4:0] op,
    input logic [63:0] dest_val,
    input logic [63:0] src_val,
    input logic [63:0] src_val2,
    input logic [63:0] imm_val,
    input logic [63:0] curr_pc,
    input logic [63:0] memData,
    output logic [63:0] ctrl_pc
);
    always @(*) begin
        case (op)
            5'b01000: ctrl_pc = dest_val;                     // br
            5'b01001: ctrl_pc = curr_pc + dest_val;           // brr $r_d
            5'b01010: ctrl_pc = curr_pc + $signed(imm_val);   // brr L
            5'b01011: ctrl_pc = (src_val != 0) ? dest_val : curr_pc + 4; // brnz
            5'b01100: ctrl_pc = dest_val;                     // call
            5'b01101: ctrl_pc = memData;                      // return
            5'b01110: ctrl_pc = (src_val > src_val2) ? dest_val : curr_pc + 4; // brgt
            default:  ctrl_pc = curr_pc + 4;
        endcase
    end
endmodule

// Memory Handler
module mem_handler (
    input logic [4:0] op,
    input logic [63:0] dest_val,
    input logic [63:0] src_val,
    input logic [63:0] imm_val,
    input logic [63:0] curr_pc,
    input logic [63:0] r31,
    output logic [63:0] mem_rw_addr,
    output logic [63:0] mem_wr_data,
    output logic wr_flag,
    output logic reg_write
);
    always @(*) begin
        case (op)
            5'b01100: begin
                mem_rw_addr = r31 - 8;
                mem_wr_data = curr_pc + 4;
                wr_flag = 1;
                reg_write = 0;
            end
            5'b01101: begin
                mem_rw_addr = r31 - 8;
                wr_flag = 0;
                reg_write = 0;
                mem_wr_data = 0;
            end
            5'b10000: begin
                mem_rw_addr = src_val + imm_val;
                wr_flag = 0;
                reg_write = 1;
                mem_wr_data = 0;
            end
            5'b10011: begin
                mem_rw_addr = dest_val + imm_val;
                wr_flag = 1;
                reg_write = 0;
                mem_wr_data = src_val;
            end
            default: begin
                mem_rw_addr = 64'h2000;
                wr_flag = 0;
                reg_write = 0;
                mem_wr_data = 0;
            end
        endcase
    end
endmodule

// Instruction Decoder
module inst_decoder (
    input logic [31:0] inst_line,
    output logic [63:0] imm_val,
    output logic [4:0] dest,
    output logic [4:0] src1,
    output logic [4:0] src2,
    output logic [4:0] op
);
    always @(*) begin
        op = inst_line[31:27];
        dest = inst_line[26:22];
        src1 = inst_line[21:17];
        src2 = inst_line[16:12];
        imm_val = {52'b0, inst_line[11:0]};
        case (op)
            5'b11001: src1 = dest; // addi
            5'b11011: src1 = dest; // subi
            5'b00101: src1 = dest; // shftri
            5'b00111: src1 = dest; // shftli
            5'b10010: src1 = dest; // mov $r_d, L
        endcase
    end
endmodule

// Register File
module reg_file_bank (
    input logic [63:0] wr_data,
    input logic [4:0] raddr1,
    input logic [4:0] raddr2,
    input logic [4:0] wr_addr,
    input logic rst,
    input logic mem_wr,
    input logic reg_wr,
    input logic clk,
    output logic [63:0] out1,
    output logic [63:0] out2,
    output logic [63:0] out3,
    output logic [63:0] stack_ptr,
    inout logic [63:0] registers [0:31] // Exposed for testbench access
);
    logic [63:0] reg_array [0:31];
    logic write_en;
    integer k;
    assign write_en = mem_wr | reg_wr;
    assign out1 = reg_array[raddr1];
    assign out2 = reg_array[raddr2];
    assign out3 = reg_array[wr_addr];
    assign stack_ptr = reg_array[31];
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (k = 0; k < 31; k = k + 1)
                reg_array[k] <= 64'b0;
            reg_array[31] <= 64'd524288;
        end else if (write_en) begin
            reg_array[wr_addr] <= wr_data;
        end
    end
    // Connect internal array to external port
    generate
        genvar i;
        for (i = 0; i < 32; i = i + 1) begin : reg_connect
            assign registers[i] = reg_array[i];
        end
    endgenerate
endmodule

// Register/Literal Mux
module reg_lit_mux (
    input logic [4:0] sel,
    input logic [63:0] reg_val,
    input logic [63:0] imm_val,
    output logic [63:0] out
);
    always @(*) begin
        case (sel)
            5'b11001: out = imm_val; // addi
            5'b11011: out = imm_val; // subi
            5'b00101: out = imm_val; // shftri
            5'b00111: out = imm_val; // shftli
            5'b10010: out = imm_val; // mov $r_d, L
            default: out = reg_val;
        endcase
    end
endmodule

// ALU Unit
module alu_unit (
    input logic [4:0] control,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic op_flag,
    output logic [63:0] out
);
    real r1, r2, rres;
    assign r1 = $bitstoreal(in1);
    assign r2 = $bitstoreal(in2);
    always @(*) begin
        op_flag = 1;
        case (control)
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
                rres = r1 + r2;
                out = $realtobits(rres);
            end
            5'b10101: begin                    // subf
                rres = r1 - r2;
                out = $realtobits(rres);
            end
            5'b10110: begin                    // mulf
                rres = r1 * r2;
                out = $realtobits(rres);
            end
            5'b10111: begin                    // divf
                rres = r1 / r2;
                out = $realtobits(rres);
            end
            default: begin
                op_flag = 0;
                out = 64'b0;
            end
        endcase
    end
endmodule

// Top-Level Module
module tinker_core (
    input logic clk,
    input logic rst,
    inout logic [7:0] memory_bytes [0:524287],  // Exposed for testbench
    inout logic [63:0] reg_file_registers [0:31] // Exposed for testbench
);
    logic [4:0] dest, src1, src2, opCode;
    logic [31:0] inst_word;
    logic [63:0] nextPC, currPC;
    logic [63:0] imm_val, dest_val, src_val1, src_val2, operand2, alu_out;
    logic [63:0] stack_ptr, mem_wr_data, mem_rw_addr, mem_out;
    logic mem_wr_flag, reg_wr_mem, reg_wr_alu;

    fetch_unit fetch_inst (
        .clk(clk),
        .rst(rst),
        .r31(stack_ptr),
        .next_pc_in(nextPC),
        .curr_pc(currPC)
    );

    memory_unit memory (
        .curr_pc(currPC),
        .clk(clk),
        .rst(rst),
        .flag(mem_wr_flag),
        .wr_data(mem_wr_data),
        .mem_addr(mem_rw_addr),
        .rd_data(mem_out),
        .inst_word(inst_word),
        .bytes(memory_bytes)
    );

    control_unit control_inst (
        .op(opCode),
        .dest_val(dest_val),
        .src_val(src_val1),
        .src_val2(src_val2),
        .imm_val(imm_val),
        .curr_pc(currPC),
        .memData(mem_out),
        .ctrl_pc(nextPC)
    );

    mem_handler mem_handler_inst (
        .op(opCode),
        .dest_val(dest_val),
        .src_val(src_val1),
        .imm_val(imm_val),
        .curr_pc(currPC),
        .r31(stack_ptr),
        .mem_rw_addr(mem_rw_addr),
        .mem_wr_data(mem_wr_data),
        .wr_flag(mem_wr_flag),
        .reg_write(reg_wr_mem)
    );

    inst_decoder inst_dec (
        .inst_line(inst_word),
        .imm_val(imm_val),
        .dest(dest),
        .src1(src1),
        .src2(src2),
        .op(opCode)
    );

    reg_file_bank reg_file (
        .clk(clk),
        .rst(rst),
        .mem_wr(reg_wr_mem),
        .reg_wr(reg_wr_alu),
        .wr_data(reg_wr_mem ? mem_out : alu_out),
        .raddr1(src1),
        .raddr2(src2),
        .wr_addr(dest),
        .out1(src_val1),
        .out2(src_val2),
        .out3(dest_val),
        .stack_ptr(stack_ptr),
        .registers(reg_file_registers)
    );

    reg_lit_mux mux_inst (
        .sel(opCode),
        .reg_val(src_val2),
        .imm_val(imm_val),
        .out(operand2)
    );

    alu_unit alu_inst (
        .control(opCode),
        .in1(src_val1),
        .in2(operand2),
        .op_flag(reg_wr_alu),
        .out(alu_out)
    );
endmodule