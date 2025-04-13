// Multicycle Tinker Processor

// Top-Level Module
module tinker_processor (
    input wire clock,
    input wire rst,
    output wire halt_signal
);

    // Internal signals
    wire [4:0] target_reg, source_reg_a, source_reg_b, operation_code;
    wire [31:0] instruction_data;
    wire [63:0] program_counter, next_program_counter, alu_result, alu_new_pc;
    wire [63:0] immediate_val, target_val, source_val_a, source_val_b, alu_input_b;
    wire [63:0] mem_address, mem_write_data, mem_read_data, stack_pointer;
    wire memory_write_enable, select_mem_pc, mem_read_enable;
    wire alu_active, reg_read_active, reg_write_active, fetch_active, mem_access_active, alu_done;
    wire [63:0] reg_write_data;

    // Instruction Fetch Unit
    instruction_fetch fetch_inst (
        .clk(clock),
        .reset(rst),
        .enable_fetch(fetch_active),
        .pc_input(next_program_counter),
        .pc_output(program_counter)
    );

    // Memory Unit
    data_memory mem_inst (
        .clk(clock),
        .reset(rst),
        .write_enable(memory_write_enable),
        .enable_access(mem_access_active),
        .read_enable(mem_read_enable),
        .pc(program_counter),
        .addr(mem_address),
        .data_in(mem_write_data),
        .instr_out(instruction_data),
        .data_out(mem_read_data)
    );

    // Instruction Decoder
    decode_unit decode_inst (
        .clk(clock),
        .reset(rst),
        .alu_complete(alu_done),
        .instr(instruction_data),
        .op_code(operation_code),
        .rd_addr(target_reg),
        .rs_addr(source_reg_a),
        .rt_addr(source_reg_b),
        .imm_out(immediate_val),
        .fetch_en(fetch_active),
        .mem_en(mem_access_active),
        .alu_en(alu_active),
        .reg_read_en(reg_read_active),
        .reg_write_en(reg_write_active)
    );

    // Register File
    register_bank reg_inst (
        .clk(clock),
        .reset(rst),
        .read_en(reg_read_active),
        .write_en(reg_write_active),
        .data_in(reg_write_data),
        .read_addr1(source_reg_a),
        .read_addr2(source_reg_b),
        .write_addr(target_reg),
        .data_out1(source_val_a),
        .data_out2(source_val_b),
        .data_out3(target_val),
        .stack_reg(stack_pointer)
    );

    // ALU Unit
    arithmetic_logic_unit alu_inst (
        .enable(alu_active),
        .ctrl(operation_code),
        .op1(source_val_a),
        .op2(alu_input_b),
        .rd_val(target_val),
        .curr_pc(program_counter),
        .stack_val(stack_pointer),
        .done(alu_done),
        .halt(halt_signal),
        .mem_read(mem_read_enable),
        .mem_write(memory_write_enable),
        .select_mem_pc(select_mem_pc),
        .result(alu_result),
        .mem_addr(mem_address),
        .mem_data(mem_write_data),
        .new_pc(alu_new_pc)
    );

    // Register/Literal Multiplexer
    operand_mux mux_inst (
        .op_code(operation_code),
        .reg_input(source_val_b),
        .lit_input(immediate_val),
        .selected_output(alu_input_b)
    );

    // Memory/Register Writeback Mux
    writeback_mux wb_mux_inst (
        .op_code(operation_code),
        .alu_data(alu_result),
        .mem_data(mem_read_data),
        .write_data(reg_write_data)
    );

    // PC Source Mux
    pc_source_mux pc_mux_inst (
        .select(select_mem_pc),
        .alu_pc(alu_new_pc),
        .mem_pc(mem_read_data),
        .pc_out(next_program_counter)
    );

endmodule

// Instruction Fetch Unit
module instruction_fetch (
    input wire clk,
    input wire reset,
    input wire enable_fetch,
    input wire [63:0] pc_input,
    output reg [63:0] pc_output
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_output <= 64'h2000;
        else if (enable_fetch)
            pc_output <= pc_input;
    end
endmodule

// Memory Unit
module data_memory (
    input wire clk,
    input wire reset,
    input wire write_enable,
    input wire enable_access,
    input wire read_enable,
    input wire [63:0] pc,
    input wire [63:0] addr,
    input wire [63:0] data_in,
    output reg [31:0] instr_out,
    output reg [63:0] data_out
);
    reg [7:0] memory_array [0:524287];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 524288; i = i + 1)
                memory_array[i] <= 8'h0;
        end
        else if (write_enable && enable_access) begin
            memory_array[addr] <= data_in[7:0];
            memory_array[addr+1] <= data_in[15:8];
            memory_array[addr+2] <= data_in[23:16];
            memory_array[addr+3] <= data_in[31:24];
            memory_array[addr+4] <= data_in[39:32];
            memory_array[addr+5] <= data_in[47:40];
            memory_array[addr+6] <= data_in[55:48];
            memory_array[addr+7] <= data_in[63:56];
        end
    end

    always @(*) begin
        if (enable_access) begin
            if (read_enable) begin
                data_out = {memory_array[addr+7], memory_array[addr+6], memory_array[addr+5], memory_array[addr+4],
                            memory_array[addr+3], memory_array[addr+2], memory_array[addr+1], memory_array[addr]};
            end
            instr_out = {memory_array[pc+3], memory_array[pc+2], memory_array[pc+1], memory_array[pc]};
        end else begin
            data_out = 64'h0;
            instr_out = 32'h0;
        end
    end
endmodule

// Instruction Decoder
module decode_unit (
    input wire clk,
    input wire reset,
    input wire alu_complete,
    input wire [31:0] instr,
    output reg [4:0] op_code,
    output reg [4:0] rd_addr,
    output reg [4:0] rs_addr,
    output reg [4:0] rt_addr,
    output reg [63:0] imm_out,
    output reg fetch_en,
    output reg mem_en,
    output reg alu_en,
    output reg reg_read_en,
    output reg reg_write_en
);
    typedef enum logic [2:0] {
        FETCH = 3'b000,
        DECODE = 3'b001,
        EXECUTE = 3'b010,
        LOAD_STORE = 3'b011,
        WRITE_BACK = 3'b100
    } processor_state;

    processor_state state;
    reg [4:0] stored_op;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= FETCH;
            fetch_en <= 1;
            mem_en <= 1;
            alu_en <= 0;
            reg_read_en <= 0;
            reg_write_en <= 0;
        end else begin
            case (state)
                FETCH: begin
                    fetch_en <= 0;
                    mem_en <= 0;
                    state <= DECODE;
                end
                DECODE: begin
                    reg_read_en <= 1;
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    reg_read_en <= 0;
                    if (!alu_complete) begin
                        alu_en <= 1;
                    end else begin
                        alu_en <= 0;
                        if (stored_op == 5'h10 || stored_op == 5'h13 || stored_op == 5'h0C || stored_op == 5'h0D)
                            state <= LOAD_STORE;
                        else if (stored_op == 5'h08 || stored_op == 5'h09 || stored_op == 5'h0A || stored_op == 5'h0B || stored_op == 5'h0E)
                            state <= FETCH;
                        else
                            state <= WRITE_BACK;
                    end
                end
                LOAD_STORE: begin
                    mem_en <= 1;
                    if (stored_op == 5'h10)
                        state <= WRITE_BACK;
                    else
                        state <= FETCH;
                end
                WRITE_BACK: begin
                    reg_write_en <= 1;
                    state <= FETCH;
                    fetch_en <= 1;
                    mem_en <= 1;
                end
                default: state <= FETCH;
            endcase
        end
    end

    always @(*) begin
        op_code = instr[31:27];
        rd_addr = instr[26:22];
        rs_addr = instr[21:17];
        rt_addr = instr[16:12];
        imm_out = {{52{instr[11]}}, instr[11:0]};
        stored_op = op_code;

        if (state == DECODE) begin
            case (op_code)
                5'h19, 5'h1B, 5'h05, 5'h07, 5'h12: rs_addr = rd_addr;
                default: ;
            endcase
        end
    end
endmodule

// Register File
module register_bank (
    input wire clk,
    input wire reset,
    input wire read_en,
    input wire write_en,
    input wire [63:0] data_in,
    input wire [4:0] read_addr1,
    input wire [4:0] read_addr2,
    input wire [4:0] write_addr,
    output reg [63:0] data_out1,
    output reg [63:0] data_out2,
    output reg [63:0] data_out3,
    output reg [63:0] stack_reg
);
    reg [63:0] regs [0:31];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                regs[i] <= 64'h0;
            regs[31] <= 64'd524288;
        end
        else if (write_en)
            regs[write_addr] <= data_in;
    end

    always @(*) begin
        stack_reg = regs[31];
        if (read_en) begin
            data_out1 = regs[read_addr1];
            data_out2 = regs[read_addr2];
            data_out3 = regs[write_addr];
        end else begin
            data_out1 = 64'h0;
            data_out2 = 64'h0;
            data_out3 = 64'h0;
        end
    end
endmodule

// ALU Unit
module arithmetic_logic_unit (
    input wire enable,
    input wire [4:0] ctrl,
    input wire [63:0] op1,
    input wire [63:0] op2,
    input wire [63:0] rd_val,
    input wire [63:0] curr_pc,
    input wire [63:0] stack_val,
    output reg done,
    output reg halt,
    output reg mem_read,
    output reg mem_write,
    output reg select_mem_pc,
    output reg [63:0] result,
    output reg [63:0] mem_addr,
    output reg [63:0] mem_data,
    output reg [63:0] new_pc
);
    real fp1, fp2, fp_res;

    always @(*) begin
        fp1 = $bitstoreal(op1);
        fp2 = $bitstoreal(op2);
        done = 0;
        halt = 0;
        mem_read = 0;
        mem_write = 0;
        select_mem_pc = 0;
        result = 64'h0;
        mem_addr = 64'h2000;
        mem_data = 64'h0;
        new_pc = curr_pc + 4;

        if (enable) begin
            done = 1;
            case (ctrl)
                5'h18: result = op1 + op2; // add
                5'h19: result = op1 + op2; // addi
                5'h1A: result = op1 - op2; // sub
                5'h1B: result = op1 - op2; // subi
                5'h1C: result = op1 * op2; // mul
                5'h1D: result = op1 / op2; // div
                5'h00: result = op1 & op2; // and
                5'h01: result = op1 | op2; // or
                5'h02: result = op1 ^ op2; // xor
                5'h03: result = ~op1;      // not
                5'h04: result = op1 >> op2; // shftr
                5'h05: result = op1 >> op2; // shftri
                5'h06: result = op1 << op2; // shftl
                5'h07: result = op1 << op2; // shftli
                5'h08: begin               // br
                    new_pc = rd_val;
                    result = 64'h0;
                end
                5'h09: begin               // brr $r_d
                    new_pc = curr_pc + rd_val;
                    result = 64'h0;
                end
                5'h0A: begin               // brr L
                    new_pc = curr_pc + $signed(op2);
                    result = 64'h0;
                end
                5'h0B: begin               // brnz
                    new_pc = (op1 != 0) ? rd_val : curr_pc + 4;
                    result = 64'h0;
                end
                5'h0C: begin               // call
                    new_pc = rd_val;
                    mem_addr = stack_val - 8;
                    mem_data = curr_pc + 4;
                    mem_write = 1;
                    result = 64'h0;
                end
                5'h0D: begin               // return
                    mem_addr = stack_val - 8;
                    mem_read = 1;
                    select_mem_pc = 1;
                    result = 64'h0;
                end
                5'h0E: begin               // brgt
                    new_pc = (op1 > op2) ? rd_val : curr_pc + 4;
                    result = 64'h0;
                end
                5'h0F: begin               // priv (halt)
                    halt = 1;
                    result = 64'h0;
                end
                5'h10: begin               // mov $r_d, ($r_s)(L)
                    mem_addr = op1 + op2;
                    mem_read = 1;
                    result = 64'h0;
                end
                5'h11: result = op1;       // mov $r_d, $r_s
                5'h12: result = {op1[63:12], op2[11:0]}; // mov $r_d, L
                5'h13: begin               // mov ($r_d)(L), $r_s
                    mem_addr = rd_val + op2;
                    mem_data = op1;
                    mem_write = 1;
                    result = 64'h0;
                end
                5'h14: begin               // addf
                    fp_res = fp1 + fp2;
                    result = $realtobits(fp_res);
                end
                5'h15: begin               // subf
                    fp_res = fp1 - fp2;
                    result = $realtobits(fp_res);
                end
                5'h16: begin               // mulf
                    fp_res = fp1 * fp2;
                    result = $realtobits(fp_res);
                end
                5'h17: begin               // divf
                    fp_res = fp1 / fp2;
                    result = $realtobits(fp_res);
                end
                default: begin
                    done = 0;
                    result = 64'h0;
                end
            endcase
        end
    end
endmodule

// Register/Literal Multiplexer
module operand_mux (
    input wire [4:0] op_code,
    input wire [63:0] reg_input,
    input wire [63:0] lit_input,
    output reg [63:0] selected_output
);
    always @(*) begin
        case (op_code)
            5'h19, 5'h1B, 5'h05, 5'h07, 5'h12, 5'h0A, 5'h10, 5'h13:
                selected_output = lit_input;
            default:
                selected_output = reg_input;
        endcase
    end
endmodule

// Memory/Register Writeback Mux
module writeback_mux (
    input wire [4:0] op_code,
    input wire [63:0] alu_data,
    input wire [63:0] mem_data,
    output reg [63:0] write_data
);
    always @(*) begin
        write_data = (op_code == 5'h10) ? mem_data : alu_data;
    end
endmodule

// PC Source Mux
module pc_source_mux (
    input wire select,
    input wire [63:0] alu_pc,
    input wire [63:0] mem_pc,
    output reg [63:0] pc_out
);
    always @(*) begin
        pc_out = select ? mem_pc : alu_pc;
    end
endmodule