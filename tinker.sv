module tinker_core (
    input clk,
    input reset,
    output hlt
);
    
    logic [4:0] dest_reg, src_reg1, src_reg2, operation_code;
    logic [31:0] instruction_data;
    logic [63:0] next_program_counter, program_counter, alu_program_counter;
    logic [63:0] literal_value, dest_reg_value, src_reg1_value, src_reg2_value, alu_input2, alu_result, stack_pointer, mem_write_data, mem_rw_address, mem_read_data, reg_write_data;
    logic mem_write_enable;
    logic mem_pc_select;
    logic alu_enable, reg_read_enable, reg_write_enable, fetch_enable, mem_read_enable, mem_access_enable, alu_ready;

    instructionDecoder instr_decoder (
        .instructionLine(instruction_data),
        .clk(clk),
        .rst(reset),
        .aluReady(alu_ready),
        .memEnable(mem_access_enable),
        .literal(literal_value),
        .rd(dest_reg),
        .rs(src_reg1),
        .rt(src_reg2),
        .opcode(operation_code),
        .aluEnable(alu_enable),
        .fetchEnable(fetch_enable),
        .regReadEnable(reg_read_enable),
        .regWriteEnable(reg_write_enable)
    );

    fetch fetch_unit (
        .clk(clk),
        .fetchEnable(fetch_enable),
        .reset(reset),
        .next_pc(next_program_counter),
        .pc(program_counter)
    );

    alu alu_unit (
        .aluEnable(alu_enable),
        .control(operation_code),
        .input1(src_reg1_value),
        .input2(alu_input2),
        .inputPc(program_counter),
        .r31(stack_pointer),
        .rd(dest_reg_value),
        .clk(clk),
        .result(alu_result),
        .pc(alu_program_counter),
        .writeFlag(mem_write_enable),
        .memRead(mem_read_enable),
        .aluReady(alu_ready),
        .writeData(mem_write_data),
        .rwAddress(mem_rw_address),
        .hlt(hlt),
        .mem_pc(mem_pc_select)
    );

    reglitmux reg_lit_mux (
        .sel(operation_code),
        .reg1(src_reg2_value),
        .lit(literal_value),
        .out(alu_input2)
    );

    registerFile reg_file (
        .data(reg_write_data),
        .read1(src_reg1),
        .read2(src_reg2),
        .write(dest_reg),
        .reset(reset),
        .clk(clk),
        .regReadEnable(reg_read_enable),
        .regWriteEnable(reg_write_enable),
        .output1(src_reg1_value),
        .output2(src_reg2_value),
        .output3(dest_reg_value),
        .stackPtr(stack_pointer)
    );

    aluMemMux alu_mem_mux (
        .mem_pc(mem_pc_select),
        .memData(mem_read_data),
        .aluOut(alu_program_counter),
        .newPc(next_program_counter)
    );

    memory memory (
        .pc(program_counter),
        .clk(clk),
        .reset(reset),
        .writeFlag(mem_write_enable),
        .fetchEnable(fetch_enable),
        .memEnable(mem_access_enable),
        .memRead(mem_read_enable),
        .writeData(mem_write_data),
        .rwAddress(mem_rw_address),
        .readData(mem_read_data),
        .instruction(instruction_data)
    );

    memRegMux mem_reg_mux (
        .opcode(operation_code),
        .readData(mem_read_data),
        .aluResult(alu_result),
        .regWriteData(reg_write_data)
    );

endmodule

module reglitmux (
    input [4:0] select_code,
    input [63:0] reg_input,
    input [63:0] literal_input,
    output reg [63:0] mux_output
);
    always @(*) begin
        case (select_code)
            5'b11001: mux_output = literal_input;
            5'b11011: mux_output = literal_input;
            5'b00101: mux_output = literal_input;
            5'b00111: mux_output = literal_input;
            5'b10010: mux_output = literal_input;
            5'b01010: mux_output = literal_input;
            5'h10: mux_output = literal_input;
            5'h13: mux_output = literal_input;
            default: mux_output = reg_input;
        endcase
    end
endmodule

module registerFile (
    input [63:0] write_data,
    input [4:0] read_addr1,
    input [4:0] read_addr2,
    input [4:0] write_addr,
    input reset,
    input clk,
    input reg_read_enable,
    input reg_write_enable,
    output reg [63:0] read_data1,
    output reg [63:0] read_data2,
    output reg [63:0] read_data3,
    output reg [63:0] stack_pointer
);
    reg [63:0] registers [0:31];
    integer idx;

    initial begin
        for (idx = 0; idx < 31; idx = idx + 1) begin
            registers[idx] <= 64'b0;
        end
        registers[31] <= 64'd524288;
    end

    assign stack_pointer = registers[31];
    always @(*) begin
        if (reg_read_enable) begin
            read_data1 = registers[read_addr1];
            read_data2 = registers[read_addr2];
            read_data3 = registers[write_addr];
        end

        if (reg_write_enable) begin
            registers[write_addr] = write_data;
        end
    end
endmodule

module memRegMux (
    input [4:0] operation_code,
    input [63:0] mem_read_data,
    input [63:0] alu_result,
    output reg [63:0] reg_write_data
);
    always @(*) begin
        case (operation_code)
            5'h10: reg_write_data = mem_read_data;
            default: reg_write_data = alu_result;
        endcase
    end
endmodule

module memory (
    input [63:0] program_counter,
    input clk,
    input reset,
    input write_flag,
    input fetch_enable,
    input mem_enable,
    input mem_read,
    input [63:0] write_data,
    input [63:0] rw_address,
    output reg [63:0] read_data,
    output reg [31:0] instruction
);
    reg [7:0] bytes [0:524287];
    integer idx;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (idx = 0; idx < 524288; idx = idx + 1) begin
                bytes[idx] <= 8'b0;
            end
        end
    end

    assign instruction[7:0] = bytes[program_counter];
    assign instruction[15:8] = bytes[program_counter+1];
    assign instruction[23:16] = bytes[program_counter+2];
    assign instruction[31:24] = bytes[program_counter+3];

    always @(mem_enable) begin
        if (mem_read) begin
            read_data[7:0] = bytes[rw_address];
            read_data[15:8] = bytes[rw_address+1];
            read_data[23:16] = bytes[rw_address+2];
            read_data[31:24] = bytes[rw_address+3];
            read_data[39:32] = bytes[rw_address+4];
            read_data[47:40] = bytes[rw_address+5];
            read_data[55:48] = bytes[rw_address+6];
            read_data[63:56] = bytes[rw_address+7];
        end
        if (write_flag) begin
            bytes[rw_address] = write_data[7:0];
            bytes[rw_address+1] = write_data[15:8];
            bytes[rw_address+2] = write_data[23:16];
            bytes[rw_address+3] = write_data[31:24];
            bytes[rw_address+4] = write_data[39:32];
            bytes[rw_address+5] = write_data[47:40];
            bytes[rw_address+6] = write_data[55:48];
            bytes[rw_address+7] = write_data[63:56];
        end
    end
endmodule

module instructionDecoder (
    input [31:0] instruction_data,
    input clk,
    input rst,
    input alu_ready,
    output reg fetch_enable,
    output reg mem_access_enable,
    output reg [63:0] literal_value,
    output reg [4:0] dest_reg,
    output reg [4:0] src_reg1,
    output reg [4:0] src_reg2,
    output reg [4:0] operation_code,
    output reg alu_enable,
    output reg reg_read_enable,
    output reg reg_write_enable
);
    reg [2:0] decode_state;
    reg [4:0] opcode_reg;

    always @(posedge rst) begin
        if (rst) decode_state <= 3'b0;
    end

    always @(posedge clk) begin
        alu_enable <= 0;
        reg_write_enable <= 0;
        reg_read_enable <= 0;
        mem_access_enable <= 0;

        case (decode_state)
            3'b0: decode_state <= 3'b1;
            3'b1: decode_state <= 3'b10;
            3'b10: begin
                if (!alu_ready) decode_state <= 3'b10;
                else if (opcode_reg == 5'h10 || opcode_reg == 5'h13 || opcode_reg == 5'h0C || opcode_reg == 5'h0D) begin
                    decode_state <= 3'b11;
                end
                else if (opcode_reg == 5'h08 || opcode_reg == 5'h09 || opcode_reg == 5'h0A || opcode_reg == 5'h0B || opcode_reg == 5'h0E) begin
                    decode_state <= 3'b0;
                end
                else decode_state <= 3'b100;
            end
            3'b11: begin
                if (opcode_reg == 5'h10) decode_state <= 3'b100;
                else decode_state <= 3'b0;
            end
            3'b100: decode_state <= 3'b0;
        endcase
    end

    always @(*) begin
        case (decode_state)
            3'b0: begin
                fetch_enable = 1;
                mem_access_enable = 1;
            end
            3'b1: begin
                operation_code = instruction_data[31:27];
                opcode_reg = operation_code;
                dest_reg = instruction_data[26:22];
                src_reg1 = instruction_data[21:17];
                src_reg2 = instruction_data[16:12];
                literal_value = {52'b0 INSTR_data[11:0]};

                case (operation_code)
                    5'b11001: src_reg1 = dest_reg;
                    5'b11011: src_reg1 = dest_reg;
                    5'b00101: src_reg1 = dest_reg;
                    5'b00111: src_reg1 = dest_reg;
                    5'b10010: src_reg1 = dest_reg;
                endcase

                reg_read_enable = 1;
            end
            3'b10: alu_enable = 1;
            3'b11: mem_access_enable = 1;
            3'b100: reg_write_enable = 1;
        endcase
    end
endmodule

module fetch (
    input clk,
    input fetch_enable,
    input reset,
    input [63:0] next_program_counter,
    output reg [63:0] program_counter
);
    reg [63:0] current_program_counter;

    always @(*) begin
        if (fetch_enable) program_counter = current_program_counter;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_program_counter <= 64'h2000;
        end else if (fetch_enable) begin
            if (next_program_counter === 64'hx) begin
                current_program_counter = 64'h2000;
            end
            else current_program_counter <= next_program_counter;
        end
    end
endmodule

module aluMemMux (
    input mem_pc_select,
    input [63:0] mem_data,
    input [63:0] alu_output,
    output reg [63:0] new_program_counter
);
    assign new_program_counter = mem_pc_select ? mem_data : alu_output;
endmodule

module alu (
    input alu_enable,
    input [4:0] control_code,
    input [63:0] input1,
    input [63:0] input2,
    input [63:0] dest_input,
    input [63:0] input_program_counter,
    input [63:0] stack_reg,
    input clk,
    output reg [63:0] alu_result,
    output reg [63:0] mem_write_data,
    output reg [63:0] mem_rw_address,
    output reg alu_ready,
    output reg mem_write_flag,
    output reg mem_read_flag,
    output reg [63:0] program_counter,
    output reg halt_signal,
    output reg mem_pc_select
);
    real real_input1, real_input2, real_result;
    assign real_input1 = $bitstoreal(input1);
    assign real_input2 = $bitstoreal(input2);

    always @(*) begin
        if (alu_enable) begin
            halt_signal = 0;
            program_counter = input_program_counter + 4;
            mem_write_data = 0;
            mem_rw_address = 64'h2000;
            mem_write_flag = 0;
            mem_read_flag = 0;
            mem_pc_select = 0;
            case (control_code)
                5'h18: alu_result = input1 + input2;
                5'h19: alu_result = input1 + input2;
                5'h1A: alu_result = input1 - input2;
                5'h1B: alu_result = input1 - input2;
                5'h1C: alu_result = input1 * input2;
                5'h1D: alu_result = input1 / input2;
                5'h00: alu_result = input1 & input2;
                5'h01: alu_result = input1 | input2;
                5'h02: alu_result = input1 ^ input2;
                5'h03: alu_result = ~input1;
                5'h04: alu_result = input1 >> input2;
                5'h05: alu_result = input1 >> input2;
                5'h06: alu_result = input1 << input2;
                5'h07: alu_result = input1 << input2;
                5'h10: begin
                    alu_result = 64'b0;
                    mem_rw_address = input1 + input2;
                    mem_read_flag = 1;
                end
                5'h11: alu_result = input1;
                5'h12: alu_result = {input1[63:12], input2[11:0]};
                5'h13: begin
                    alu_result = 64'b0;
                    mem_rw_address = dest_input + input2;
                    mem_write_data = input1;
                    mem_write_flag = 1;
                end
                5'h14: begin
                    real_result = real_input1 + real_input2;
                    alu_result = $realtobits(real_result);
                end
                5'h15: begin
                    real_result = real_input1 - real_input2;
                    alu_result = $realtobits(real_result);
                end
                5'h16: begin
                    real_result = real_input1 * real_input2;
                    alu_result = $realtobits(real_result);
                end
                5'h17: begin
                    real_result = real_input1 / real_input2;
                    alu_result = $realtobits(real_result);
                end
                5'h08: begin
                    program_counter = dest_input;
                    alu_result = 64'b0;
                end
                5'h09: begin
                    program_counter = input_program_counter + dest_input;
                    alu_result = 64'b0;
                end
                5'h0A: begin
                    program_counter = input_program_counter + $signed(input2);
                    alu_result = 64'b0;
                end
                5'h0B: begin
                    program_counter = (input1 != 0) ? dest_input : input_program_counter + 4;
                    alu_result = 64'b0;
                end
                5'h0C: begin
                    program_counter = dest_input;
                    alu_result = 64'b0;
                    mem_write_data = input_program_counter + 4;
                    mem_rw_address = stack_reg - 8;
                    mem_write_flag = 1;
                end
                5'h0D: begin
                    alu_result = 64'b0;
                    mem_rw_address = stack_reg - 8;
                    mem_read_flag = 1;
                    mem_pc_select = 1;
                end
                5'h0E: begin
                    program_counter = (input1 > input2) ? dest_input : input_program_counter + 4;
                    alu_result = 64'b0;
                end
                5'h0F: begin
                    alu_result = 64'b0;
                    halt_signal = 1;
                end
            endcase
            alu_ready = 1;
        end
        else begin
            halt_signal = 0;
            alu_ready = 0;
        end
    end
endmodule