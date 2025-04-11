module tinker_core (
    input logic clk,
    input logic reset,
    output logic hlt
);
    localparam HALT_OPCODE = 5'b11111;

    typedef enum logic [2:0] {
        S_FETCH, S_DECODE, S_EXECUTE, S_MEMORY, S_WRITEBACK, S_HALTED
    } state_t;

    state_t current_state, next_state;

    logic [63:0] pc_current;
    logic [63:0] pc_next;
    logic        pc_write_enable;
    logic [31:0] instr_word;
    logic [31:0] instr_reg;
    logic [4:0]  dest_reg;
    logic [4:0]  src_reg1;
    logic [4:0]  src_reg2;
    logic [4:0]  opcode;
    logic [63:0] imm_value;
    logic [63:0] dest_val;
    logic [63:0] src_val1;
    logic [63:0] src_val2;
    logic [63:0] stack_ptr;
    logic        reg_write;
    logic        mem_to_reg;
    logic [63:0] alu_operand2;
    logic [63:0] alu_output;
    logic [63:0] alu_out_reg;
    logic [63:0] mem_addr;
    logic [63:0] mem_data_in;
    logic [63:0] mem_data_out;
    logic [63:0] mem_data_reg;
    logic        mem_read;
    logic        mem_write;
    logic        ir_write;
    logic [63:0] memory_unit_address;
    logic        is_branch_instr;
    logic        branch_taken;

    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= S_FETCH;
        else if (current_state == S_HALTED)
             current_state <= S_HALTED;
        else
            current_state <= next_state;
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            S_FETCH:   next_state = S_DECODE;
            S_DECODE: begin
                if (opcode == HALT_OPCODE) begin
                    next_state = S_HALTED;
                end else begin
                    next_state = S_EXECUTE;
                end
            end
            S_EXECUTE: begin
                 if (is_memory_operation(opcode)) begin
                     next_state = S_MEMORY;
                 end else if (is_branch_instr) begin
                     next_state = S_FETCH;
                 end else begin
                     next_state = S_WRITEBACK;
                 end
            end
            S_MEMORY:    next_state = S_WRITEBACK;
            S_WRITEBACK: next_state = S_FETCH;
            S_HALTED:    next_state = S_HALTED;
            default:     next_state = S_FETCH;
        endcase
    end

    always @(*) begin
        ir_write = 1'b0;
        pc_write_enable = 1'b0;
        reg_write = 1'b0;
        mem_read = 1'b0;
        mem_write = 1'b0;
        mem_to_reg = 1'b0;
        hlt = 1'b0;
        memory_unit_address = mem_addr;

        case (current_state)
            S_FETCH: begin
                ir_write = 1'b1;
                mem_read = 1'b1;
                memory_unit_address = pc_current;
            end
            S_DECODE: begin
            end
            S_EXECUTE: begin
                if (is_branch_instr && branch_taken) begin
                    pc_write_enable = 1'b1;
                end
            end
            S_MEMORY: begin
                if (is_load_operation(opcode)) begin
                    mem_read = 1'b1;
                end else if (is_store_operation(opcode)) begin
                    mem_write = 1'b1;
                end
            end
            S_WRITEBACK: begin
                if (!is_store_operation(opcode) && !is_branch_no_writeback(opcode) && opcode != HALT_OPCODE) begin
                    reg_write = 1'b1;
                end
                if (is_load_operation(opcode)) begin
                    mem_to_reg = 1'b1;
                end else begin
                    mem_to_reg = 1'b0;
                end
                if (!branch_taken) begin
                    pc_write_enable = 1'b1;
                end
            end
            S_HALTED: begin
                hlt = 1'b1;
            end
            default: begin
            end
        endcase
    end

    function logic is_memory_operation(logic [4:0] op);
         return (op == 5'b10000 || op == 5'b10011 || op == 5'b01100 || op == 5'b01101);
    endfunction
    function logic is_load_operation(logic [4:0] op);
         return (op == 5'b10000 || op == 5'b01101);
    endfunction
    function logic is_store_operation(logic [4:0] op);
        return (op == 5'b10011 || op == 5'b01100);
    endfunction
    function logic is_branch_no_writeback(logic [4:0] op);
         return (op >= 5'b01000 && op <= 5'b01110) || op == HALT_OPCODE;
    endfunction

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instr_reg <= 32'b0;
        end else if (ir_write) begin
            instr_reg <= instr_word;
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_data_reg <= 64'b0;
        end else if (current_state == S_MEMORY && mem_read) begin
             mem_data_reg <= mem_data_out;
        end
    end

    always @(posedge clk or posedge reset) begin
       if (reset) begin
           alu_out_reg <= 64'b0;
       end else if (current_state == S_EXECUTE) begin
            alu_out_reg <= alu_output;
       end
    end

    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .pc_in(pc_next),
        .pc_out(pc_current),
        .pc_write(pc_write_enable)
    );

    memory_unit memory (
        .program_counter(pc_current),
        .clk(clk),
        .reset(reset),
        .read_en(mem_read),
        .write_en(mem_write),
        .data_in(mem_data_in),
        .address(memory_unit_address),
        .data_out(mem_data_out),
        .instruction(instr_word)
    );

    control_unit ctrl (
        .operation(opcode),
        .dest_in(dest_val),
        .src_in1(src_val1),
        .src_in2(src_val2),
        .immediate(imm_value),
        .current_pc(pc_current),
        .memory_data(mem_data_reg),
        .next_pc(pc_next),
        .is_branch(is_branch_instr),
        .branch_taken(branch_taken)
    );

    mem_handler mem_mgr (
        .op(opcode),
        .dest(dest_val),
        .src(src_val1),
        .imm(imm_value),
        .pc(pc_current),
        .r31(stack_ptr),
        .addr_out(mem_addr),
        .data_out(mem_data_in)
    );

    inst_decoder dec (
        .instruction(instr_reg),
        .imm(imm_value),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    reg_file_bank reg_file (
        .clk(clk),
        .reset(reset),
        .write_en(reg_write),
        .write_data(mem_to_reg ? mem_data_reg : alu_out_reg),
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
        .out(alu_output)
    );

endmodule

// Fetch Unit
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic pc_write,
    input logic [63:0] pc_in,
    output logic [63:0] pc_out
);
    logic [63:0] pc_reg;
    assign pc_out = pc_reg;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_reg <= 64'h2000;
        end else if (pc_write) begin
            pc_reg <= pc_in;
        end
    end
endmodule

// Memory Unit
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


    always @(posedge clk or posedge reset) begin
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

// Control Unit
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
    logic branch_condition_met;

    assign is_branch = (operation >= 5'b01000 && operation <= 5'b01110);

    always @(*) begin
        case(operation)
            5'b01011: branch_condition_met = (src_in1 != 0);
            5'b01110: branch_condition_met = ($signed(src_in1) > $signed(src_in2));
            default: branch_condition_met = 1'b1;
        endcase

        branch_taken = 1'b0;
        next_pc = current_pc + 4;

        case (operation)
            5'b01000: begin next_pc = dest_in; branch_taken = 1'b1; end
            5'b01100: begin next_pc = dest_in; branch_taken = 1'b1; end
            5'b01101: begin next_pc = memory_data; branch_taken = 1'b1; end
            5'b01001: begin next_pc = current_pc + dest_in; branch_taken = 1'b1; end
            5'b01010: begin next_pc = current_pc + $signed(immediate); branch_taken = 1'b1; end
            5'b01011: begin
                if (branch_condition_met) begin next_pc = dest_in; branch_taken = 1'b1; end
            end
            5'b01110: begin
                if (branch_condition_met) begin next_pc = dest_in; branch_taken = 1'b1; end
            end
            default: begin end
        endcase
    end
endmodule

// Memory Handler
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
            5'b01100: begin addr_out = r31 - 8; data_out = pc + 4; end
            5'b01101: begin addr_out = r31 - 8; data_out = 64'h0; end
            5'b10000: begin addr_out = src + imm; data_out = 64'h0; end
            5'b10011: begin addr_out = dest + imm; data_out = src; end
            default: begin end
        endcase
    end
endmodule

// Register File (R0 Hardwired)
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

    assign data1 = (addr1 == 5'b0) ? 64'b0 : registers[addr1];
    assign data2 = (addr2 == 5'b0) ? 64'b0 : registers[addr2];
    assign data_dest = (write_addr == 5'b0) ? 64'b0 : registers[write_addr];
    assign stack = registers[31];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 1; i < 31; i = i + 1) begin registers[i] <= 64'h0; end
            registers[31] <= MEMSIZE;
        end else if (write_en && write_addr != 5'b0) begin
            registers[write_addr] <= write_data;
        end
    end
endmodule

// ALU Unit
module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic [63:0] out
);
    always @(*) begin
        case (ctrl)
            5'b11000: out = in1 + in2;         // add
            5'b11001: out = in1 + in2;         // addi (Op1)
            5'b01010: out = in1 + in2;         // ADDI (Op2)
            5'b11010: out = in1 - in2;         // sub
            5'b11011: out = in1 - in2;         // subi (Op1)
            5'b01011: out = in1 - in2;         // SUBI (Op2)
            5'b11100: out = in1 * in2;         // mul
            5'b11101: out = (in2 == 0) ? 64'h0 : in1 / in2; // div
            5'b00000: out = in1 & in2;         // and
            5'b00001: out = in1 | in2;         // or
            5'b00010: out = in1 ^ in2;         // xor
            5'b00011: out = ~in1;              // not
            5'b00100: out = in1 >> in2;        // shftr
            5'b00101: out = $signed(in1) >>> in2; // shftri
            5'b00110: out = in1 << in2;        // shftl
            5'b00111: out = in1 << in2;        // shftli
            5'b10001: out = in1;               // mov $r_d, $r_s
            5'b10010: out = in2;               // mov $r_d, L
            default: out = 64'h0;
        endcase
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
    logic [11:0] imm_raw;
    logic [4:0] opcode_internal;

    assign imm_raw = instruction[11:0];
    assign opcode_internal = instruction[31:27];
    assign opcode = opcode_internal;
    assign imm = {{52{imm_raw[11]}}, imm_raw};
    assign dest = instruction[26:22];
    assign src2 = instruction[16:12];

    always @(*) begin
        src1 = instruction[21:17];
        if (opcode_internal == 5'b11001 || // addi (Op1)
            opcode_internal == 5'b11011 || // subi (Op1)
            opcode_internal == 5'b01010 || // addi (Op2)
            opcode_internal == 5'b01011 || // subi (Op2)
            opcode_internal == 5'b00101 || // shftri
            opcode_internal == 5'b00111 )  // shftli
        begin
            src1 = instruction[26:22];
        end
    end

endmodule

// Register/Literal Mux
module reg_lit_mux (
    input logic [4:0] op,
    input logic [63:0] reg_val,
    input logic [63:0] lit_val,
    output logic [63:0] out
);
    always @(*) begin
        if (op == 5'b11001 || op == 5'b11011 || // addi, subi (Original)
            op == 5'b01010 || op == 5'b01011 || // addi, subi (New?)
            op == 5'b00101 || op == 5'b00111 || // shftri, shftli
            op == 5'b10010 ||                   // mov $rd, L
            op == 5'b10000 || op == 5'b10011 )  // Load/Store base+offset
            out = lit_val;
        else
            out = reg_val;
    end
endmodule