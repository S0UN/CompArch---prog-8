// Top-Level Module
module tinker_core (
    input clk,
    input reset,
    output logic hlt
);

    // Signal declarations
    logic [4:0] target_reg, source_reg1, source_reg2, op_code;
    logic [31:0] instr_data;
    logic [63:0] pc_now, pc_future;
    logic [63:0] immediate, target_val, source_val1, source_val2, alu_input2, alu_result;
    logic [63:0] stack_top, mem_input_data, mem_location, mem_output_data;
    logic mem_store, mem_load, alu_done, select_pc_from_mem;
    logic fetch_active, mem_active, alu_active, reg_fetch_active, reg_store_active;
    logic [63:0] reg_write_value, alu_pc_next;

    // Instantiate modules
    fetch_unit fetch (
        .clock(clk),
        .rst(reset),
        .fetch_trigger(fetch_active),
        .pc_input(pc_future),
        .pc_output(pc_now)
    );

    memory_unit memory (
        .pc_address(pc_now),
        .clock(clk),
        .rst(reset),
        .mem_trigger(mem_active),
        .store_en(mem_store),
        .load_en(mem_load),
        .data_input(mem_input_data),
        .mem_address(mem_location),
        .data_output(mem_output_data),
        .instr_output(instr_data)
    );

    inst_decoder dec (
        .clock(clk),
        .rst(reset),
        .instr_input(instr_data),
        .alu_finished(alu_done),
        .imm_out(immediate),
        .dest_out(target_reg),
        .src1_out(source_reg1),
        .src2_out(source_reg2),
        .op_out(op_code),
        .fetch_on(fetch_active),
        .mem_on(mem_active),
        .alu_on(alu_active),
        .reg_read_on(reg_fetch_active),
        .reg_write_on(reg_store_active)
    );

    reg_file_bank reg_file (
        .clock(clk),
        .rst(reset),
        .read_enable(reg_fetch_active),
        .write_enable(reg_store_active),
        .data_in(reg_write_value),
        .read_addr1(source_reg1),
        .read_addr2(source_reg2),
        .write_addr(target_reg),
        .data_out1(source_val1),
        .data_out2(source_val2),
        .data_out_dest(target_val),
        .stack_out(stack_top)
    );

    alu_unit alu (
        .alu_trigger(alu_active),
        .operation(op_code),
        .operand1(source_val1),
        .operand2(alu_input2),
        .pc_present(pc_now),
        .stack_val(stack_top),
        .dest_val(target_val),
        .alu_complete(alu_done),
        .alu_out(alu_result),
        .pc_next(alu_pc_next),
        .mem_pc_sel(select_pc_from_mem),
        .halt(hlt),
        .mem_fetch(mem_load),
        .mem_store_en(mem_store),
        .mem_store_data(mem_input_data),
        .mem_store_addr(mem_location)
    );

    reg_lit_mux mux (
        .op_select(op_code),
        .reg_input(source_val2),
        .lit_input(immediate),
        .mux_output(alu_input2)
    );

    write_data_selector wds (
        .op_code(op_code),
        .mem_data(mem_output_data),
        .alu_data(alu_result),
        .selected_data(reg_write_value)
    );

    // Next PC Selection Logic
    always @(*) begin
        if (select_pc_from_mem) begin
            pc_future = mem_output_data;
        end else begin
            pc_future = alu_pc_next;
        end
    end

endmodule

// Fetch Unit
module fetch_unit (
    input logic clock,
    input logic rst,
    input logic fetch_trigger,
    input logic [63:0] pc_input,
    output logic [63:0] pc_output
);
    logic [63:0] program_counter;
    assign pc_output = program_counter;

    always @(posedge clock or posedge rst) begin
        if (rst) begin
            program_counter <= 64'h2000;
        end else if (fetch_trigger) begin
            program_counter <= pc_input;
        end
    end
endmodule

// Memory Unit
module memory_unit (
    input logic [63:0] pc_address,
    input logic clock,
    input logic rst,
    input logic mem_trigger,
    input logic store_en,
    input logic load_en,
    input logic [63:0] data_input,
    input logic [63:0] mem_address,
    output logic [63:0] data_output,
    output logic [31:0] instr_output
);
    logic [7:0] bytes [0:524287];
    integer idx, byte_idx;

    assign instr_output[31:24] = bytes[pc_address + 3];
    assign instr_output[23:16] = bytes[pc_address + 2];
    assign instr_output[15:8]  = bytes[pc_address + 1];
    assign instr_output[7:0]   = bytes[pc_address];

    always @(*) begin
        if (mem_trigger && load_en) begin
            data_output[63:56] = bytes[mem_address + 7];
            data_output[55:48] = bytes[mem_address + 6];
            data_output[47:40] = bytes[mem_address + 5];
            data_output[39:32] = bytes[mem_address + 4];
            data_output[31:24] = bytes[mem_address + 3];
            data_output[23:16] = bytes[mem_address + 2];
            data_output[15:8]  = bytes[mem_address + 1];
            data_output[7:0]   = bytes[mem_address];
        end else begin
            data_output = 64'h0;
        end
    end

    always @(posedge clock or posedge rst) begin
        if (rst) begin
            for (idx = 0; idx < 524288; idx = idx + 1)
                bytes[idx] <= 8'h0;
        end else if (mem_trigger && store_en) begin
            for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1)
                bytes[mem_address + byte_idx] <= data_input[8 * byte_idx +: 8];
        end
    end
endmodule

// Instruction Decoder with State Machine
module inst_decoder (
    input logic clock,
    input logic rst,
    input logic [31:0] instr_input,
    input logic alu_finished,
    output logic [63:0] imm_out,
    output logic [4:0] dest_out,
    output logic [4:0] src1_out,
    output logic [4:0] src2_out,
    output logic [4:0] op_out,
    output logic fetch_on,
    output logic mem_on,
    output logic alu_on,
    output logic reg_read_on,
    output logic reg_write_on
);
    typedef enum logic [2:0] {
        FETCH = 3'b000,
        DECODE = 3'b001,
        EXECUTE = 3'b010,
        LOAD_STORE = 3'b011,
        WRITE_BACK = 3'b100
    } states;

    logic [2:0] current_phase;
    logic [4:0] stored_op;

    always @(posedge clock or posedge rst) begin
        if (rst) begin
            current_phase <= FETCH;
        end else begin
            case (current_phase)
                FETCH: current_phase <= DECODE;
                DECODE: current_phase <= EXECUTE;
                EXECUTE: begin
                    if (!alu_finished) current_phase <= EXECUTE;
                    else if (stored_op == 5'b10000 || stored_op == 5'b10011 || stored_op == 5'b01100 || stored_op == 5'b01101) begin
                        current_phase <= LOAD_STORE;
                    end else if (stored_op == 5'b01000 || stored_op == 5'b01001 || stored_op == 5'b01010 || stored_op == 5'b01011 || stored_op == 5'b01110) begin
                        current_phase <= FETCH;
                    end else begin
                        current_phase <= WRITE_BACK;
                    end
                end
                LOAD_STORE: begin
                    if (stored_op == 5'b10000) current_phase <= WRITE_BACK;
                    else current_phase <= FETCH;
                end
                WRITE_BACK: current_phase <= FETCH;
                default: current_phase <= FETCH;
            endcase
        end
    end

    always @(*) begin
        fetch_on = 0;
        mem_on = 0;
        alu_on = 0;
        reg_read_on = 0;
        reg_write_on = 0;
        imm_out = 64'h0;
        dest_out = 5'h0;
        src1_out = 5'h0;
        src2_out = 5'h0;
        op_out = 5'h0;

        case (current_phase)
            FETCH: begin
                fetch_on = 1;
                mem_on = 1;
            end
            DECODE: begin
                op_out = instr_input[31:27];
                stored_op = op_out;
                dest_out = instr_input[26:22];
                src1_out = instr_input[21:17];
                src2_out = instr_input[16:12];
                imm_out = {52'h0, instr_input[11:0]};
                if (op_out == 5'b11001 || op_out == 5'b11011 || op_out == 5'b00101 || op_out == 5'b00111 || op_out == 5'b10010) begin
                    src1_out = dest_out;
                end
                reg_read_on = 1;
            end
            EXECUTE: begin
                alu_on = 1;
            end
            LOAD_STORE: begin
                mem_on = 1;
            end
            WRITE_BACK: begin
                reg_write_on = 1;
            end
        endcase
    end
endmodule

// Register File
module reg_file_bank (
    input logic clock,
    input logic rst,
    input logic read_enable,
    input logic write_enable,
    input logic [63:0] data_in,
    input logic [4:0] read_addr1,
    input logic [4:0] read_addr2,
    input logic [4:0] write_addr,
    output logic [63:0] data_out1,
    output logic [63:0] data_out2,
    output logic [63:0] data_out_dest,
    output logic [63:0] stack_out
);
    logic [63:0] reg_bank [0:31];
    integer reg_idx;

    always @(*) begin
        if (read_enable) begin
            data_out1 = reg_bank[read_addr1];
            data_out2 = reg_bank[read_addr2];
            data_out_dest = reg_bank[write_addr];
            stack_out = reg_bank[31];
        end else begin
            data_out1 = 64'h0;
            data_out2 = 64'h0;
            data_out_dest = 64'h0;
            stack_out = 64'h0;
        end
    end

    always @(posedge clock or posedge rst) begin
        if (rst) begin
            for (reg_idx = 0; reg_idx < 31; reg_idx = reg_idx + 1)
                reg_bank[reg_idx] <= 64'h0;
            reg_bank[31] <= 64'd524288;
        end else if (write_enable) begin
            reg_bank[write_addr] <= data_in;
        end
    end
endmodule

// ALU Unit
module alu_unit (
    input logic alu_trigger,
    input logic [4:0] operation,
    input logic [63:0] operand1,
    input logic [63:0] operand2,
    input logic [63:0] pc_present,
    input logic [63:0] stack_val,
    input logic [63:0] dest_val,
    output logic alu_complete,
    output logic [63:0] alu_out,
    output logic [63:0] pc_next,
    output logic mem_pc_sel,
    output logic halt,
    output logic mem_fetch,
    output logic mem_store_en,
    output logic [63:0] mem_store_data,
    output logic [63:0] mem_store_addr
);
    real fp_op1, fp_op2, fp_res;
    assign fp_op1 = $bitstoreal(operand1);
    assign fp_op2 = $bitstoreal(operand2);

    always @(*) begin
        if (alu_trigger) begin
            alu_complete = 1;
            halt = 0;
            mem_pc_sel = 0;
            mem_fetch = 0;
            mem_store_en = 0;
            mem_store_data = 64'h0;
            mem_store_addr = 64'h2000;
            pc_next = pc_present + 4;
            case (operation)
                5'b11000: alu_out = operand1 + operand2;  // add
                5'b11001: alu_out = operand1 + operand2;  // addi
                5'b11010: alu_out = operand1 - operand2;  // sub
                5'b11011: alu_out = operand1 - operand2;  // subi
                5'b11100: alu_out = operand1 * operand2;  // mul
                5'b11101: alu_out = operand1 / operand2;  // div
                5'b00000: alu_out = operand1 & operand2;  // and
                5'b00001: alu_out = operand1 | operand2;  // or
                5'b00010: alu_out = operand1 ^ operand2;  // xor
                5'b00011: alu_out = ~operand1;            // not
                5'b00100: alu_out = operand1 >> operand2; // shftr
                5'b00101: alu_out = operand1 >> operand2; // shftri
                5'b00110: alu_out = operand1 << operand2; // shftl
                5'b00111: alu_out = operand1 << operand2; // shftli
                5'b10001: alu_out = operand1;             // mov $r_d, $r_s
                5'b10010: alu_out = {operand1[63:12], operand2[11:0]}; // mov $r_d, L
                5'b10100: begin                           // addf
                    fp_res = fp_op1 + fp_op2;
                    alu_out = $realtobits(fp_res);
                end
                5'b10101: begin                           // subf
                    fp_res = fp_op1 - fp_op2;
                    alu_out = $realtobits(fp_res);
                end
                5'b10110: begin                           // mulf
                    fp_res = fp_op1 * fp_op2;
                    alu_out = $realtobits(fp_res);
                end
                5'b10111: begin                           // divf
                    fp_res = fp_op1 / fp_op2;
                    alu_out = $realtobits(fp_res);
                end
                5'b10000: begin  // mov $r_d, ($r_s)(L)
                    mem_store_addr = operand1 + operand2;
                    mem_fetch = 1;
                    alu_out = 64'h0;
                end
                5'b10011: begin  // mov ($r_d)(L), $r_s
                    mem_store_addr = dest_val + operand2;
                    mem_store_data = operand1;
                    mem_store_en = 1;
                    alu_out = 64'h0;
                end
                5'b01100: begin  // call
                    pc_next = dest_val;
                    mem_store_addr = stack_val - 8;
                    mem_store_data = pc_present + 4;
                    mem_store_en = 1;
                    alu_out = 64'h0;
                end
                5'b01101: begin  // return
                    mem_store_addr = stack_val - 8;
                    mem_fetch = 1;
                    mem_pc_sel = 1;
                    alu_out = 64'h0;
                end
                5'b01000: begin  // br
                    pc_next = dest_val;
                    alu_out = 64'h0;
                end
                5'b01001: begin  // brr $r_d
                    pc_next = pc_present + dest_val;
                    alu_out = 64'h0;
                end
                5'b01010: begin  // brr L
                    pc_next = pc_present + $signed(operand2);
                    alu_out = 64'h0;
                end
                5'b01011: begin  // brnz
                    pc_next = (operand1 != 0) ? dest_val : pc_present + 4;
                    alu_out = 64'h0;
                end
                5'b01110: begin  // brgt
                    pc_next = (operand1 > operand2) ? dest_val : pc_present + 4;
                    alu_out = 64'h0;
                end
                5'b01111: begin  // hlt (assumed opcode)
                    halt = 1;
                    alu_out = 64'h0;
                end
                default: alu_out = 64'h0;
            endcase
        end else begin
            alu_complete = 0;
            alu_out = 64'h0;
            pc_next = 64'h0;
            mem_pc_sel = 0;
            halt = 0;
            mem_fetch = 0;
            mem_store_en = 0;
            mem_store_data = 64'h0;
            mem_store_addr = 64'h0;
        end
    end
endmodule

// Register/Literal Multiplexer
module reg_lit_mux (
    input logic [4:0] op_select,
    input logic [63:0] reg_input,
    input logic [63:0] lit_input,
    output logic [63:0] mux_output
);
    always @(*) begin
        if (op_select == 5'b11001 || op_select == 5'b11011 || op_select == 5'b00101 || 
            op_select == 5'b00111 || op_select == 5'b10010) begin
            mux_output = lit_input;
        end else begin
            mux_output = reg_input;
        end
    end
endmodule

// Write Data Selector
module write_data_selector (
    input logic [4:0] op_code,
    input logic [63:0] mem_data,
    input logic [63:0] alu_data,
    output logic [63:0] selected_data
);
    always @(*) begin
        if (op_code == 5'b10000) begin  // load instruction
            selected_data = mem_data;
        end else begin
            selected_data = alu_data;
        end
    end
endmodule