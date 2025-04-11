// Top-Level Module
module tinker_core (
    input logic clk,
    input logic reset
);
    // State machine states
    typedef enum logic [2:0] {
        FETCH,
        DECODE,
        EXECUTE,
        MEMORY,
        WRITEBACK
    } state_t;
    
    state_t current_state, next_state;
    
    // Instruction components
    logic [4:0] dest_reg, src_reg1, src_reg2, opcode;
    logic [31:0] instr_word, instr_reg;
    
    // Program counter signals
    logic [63:0] pc_current, pc_next, pc_temp;
    logic pc_write_enable;
    
    // Register values
    logic [63:0] imm_value, dest_val, src_val1, src_val2, alu_operand2;
    logic [63:0] alu_output, alu_out_reg;
    
    // Memory signals
    logic [63:0] stack_ptr, mem_data_in, mem_addr, mem_data_out, mem_data_reg;
    logic mem_read, mem_write;
    
    // Control signals
    logic ir_write, reg_write;
    logic reg_dst, alu_src;
    logic mem_to_reg;
    logic is_branch_instr, branch_taken;
    
    // State register
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= FETCH;
        else
            current_state <= next_state;
    end
    
    // Next state logic
    always_comb begin
        case (current_state)
            FETCH:    next_state = DECODE;
            DECODE:   next_state = EXECUTE;
            EXECUTE:  next_state = is_memory_operation(opcode) ? MEMORY : 
                                  is_branch_instr ? FETCH : WRITEBACK;
            MEMORY:   next_state = WRITEBACK;
            WRITEBACK: next_state = FETCH;
            default:  next_state = FETCH;
        endcase
    end
    
    // Control signals based on state
    always_comb begin
        // Default values
        ir_write = 0;
        pc_write_enable = 0;
        reg_write = 0;
        mem_read = 0;
        mem_write = 0;
        
        case (current_state)
            FETCH: begin
                ir_write = 1;
                mem_read = 1;
                pc_write_enable = 0;  // Will update PC at end of cycle
            end
            
            DECODE: begin
                // Just decoding - no control signals needed
            end
            
            EXECUTE: begin
                if (is_branch_instr && branch_taken)
                    pc_write_enable = 1;
            end
            
            MEMORY: begin
                if (is_load_operation(opcode))
                    mem_read = 1;
                else if (is_store_operation(opcode))
                    mem_write = 1;
            end
            
            WRITEBACK: begin
                if (!is_store_operation(opcode) && !is_branch_no_writeback(opcode))
                    reg_write = 1;
                
                // If FETCH stage, update PC for next instruction
                if (current_state == WRITEBACK && next_state == FETCH && !branch_taken)
                    pc_write_enable = 1;
            end
        endcase
    end
    
    // Helper functions
    function logic is_memory_operation(logic [4:0] op);
        return (op == 5'b10000 || op == 5'b10011);  // mov with memory access
    endfunction
    
    function logic is_load_operation(logic [4:0] op);
        return (op == 5'b10000);  // mov r_d, (r_s)(L)
    endfunction
    
    function logic is_store_operation(logic [4:0] op);
        return (op == 5'b10011);  // mov (r_d)(L), r_s
    endfunction
    
    function logic is_branch_no_writeback(logic [4:0] op);
        return (op >= 5'b01000 && op <= 5'b01110);  // All branch instructions
    endfunction
    
    // Instruction Register
    always_ff @(posedge clk) begin
        if (ir_write)
            instr_reg <= instr_word;
    end
    
    // Program Counter update logic
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc_current <= 64'h2000;  // Initial PC value
        else if (pc_write_enable)
            pc_current <= pc_next;
    end
    
    // Memory Data Register
    always_ff @(posedge clk) begin
        if (current_state == MEMORY && mem_read)
            mem_data_reg <= mem_data_out;
    end
    
    // ALU Output Register
    always_ff @(posedge clk) begin
        if (current_state == EXECUTE)
            alu_out_reg <= alu_output;
    end
    
    // Fetch unit (simplified since we handle PC updates in main module)
    fetch_unit fetch (
        .clk(clk),
        .reset(reset),
        .stack(stack_ptr),
        .pc_in(pc_next),
        .pc_out(pc_current),
        .pc_write(pc_write_enable)
    );

    // Memory unit with read/write control
    memory_unit memory (  
        .program_counter(pc_current),
        .clk(clk),
        .reset(reset),
        .read_en(mem_read),
        .write_en(mem_write),
        .data_in(mem_data_in),
        .address(current_state == FETCH ? pc_current : mem_addr),
        .data_out(mem_data_out),
        .instruction(instr_word)
    );

    // Control unit for managing branching and next PC value
    control_unit ctrl (
        .operation(opcode),
        .dest_in(dest_val),
        .src_in1(src_val1),
        .src_in2(src_val2),
        .immediate(imm_value),
        .current_pc(pc_current),
        .memory_data(mem_data_out),
        .next_pc(pc_next),
        .is_branch(is_branch_instr),
        .branch_taken(branch_taken)
    );

    // Memory handler now works with state signals
    mem_handler mem_mgr (
        .state(current_state),
        .op(opcode),
        .dest(dest_val),
        .src(src_val1),
        .imm(imm_value),
        .pc(pc_current),
        .r31(stack_ptr),
        .addr_out(mem_addr),
        .data_out(mem_data_in)
    );

    // Instruction decoder uses the instruction register
    inst_decoder dec (
        .instruction(current_state == FETCH ? instr_word : instr_reg),
        .imm(imm_value),
        .dest(dest_reg),
        .src1(src_reg1),
        .src2(src_reg2),
        .opcode(opcode)
    );

    // Register file with control signals for write enable
    reg_file_bank reg_file (  
        .clk(clk),
        .reset(reset),
        .write_en(reg_write && current_state == WRITEBACK),
        .write_data(mem_to_reg ? mem_data_reg : alu_out_reg),
        .addr1(src_reg1),
        .addr2(src_reg2),
        .write_addr(dest_reg),
        .data1(src_val1),
        .data2(src_val2),
        .data_dest(dest_val),
        .stack(stack_ptr)
    );

    // Multiplexer for ALU second operand
    reg_lit_mux mux (
        .op(opcode),
        .reg_val(src_val2),
        .lit_val(imm_value),
        .out(alu_operand2)
    );

    // ALU unit - only active during EXECUTE
    alu_unit alu (
        .ctrl(opcode),
        .in1(src_val1),
        .in2(alu_operand2),
        .out(alu_output)
    );
    
    // Determine if result comes from memory or ALU
    always_comb begin
        mem_to_reg = (opcode == 5'b10000);  // mov r_d, (r_s)(L)
    end
endmodule

// Modified Fetch Unit with PC write enable
module fetch_unit (
    input logic clk,
    input logic reset,
    input logic pc_write,
    input logic [63:0] stack,
    input logic [63:0] pc_in,
    output logic [63:0] pc_out
);
    assign pc_out = pc_out_reg;
    logic [63:0] pc_out_reg;

    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            pc_out_reg <= 64'h2000;
        else if (pc_write)
            pc_out_reg <= pc_in;
    end
endmodule

// Modified Memory Unit with read enable
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
    logic [7:0] bytes [0:524287]; 
    integer j, k;

    // Instruction and data outputs
    always_comb begin
        instruction[31:24] = bytes[program_counter+3];
        instruction[23:16] = bytes[program_counter+2];
        instruction[15:8]  = bytes[program_counter+1];
        instruction[7:0]   = bytes[program_counter];
        
        if (read_en) begin
            data_out[63:56] = bytes[address+7];
            data_out[55:48] = bytes[address+6];
            data_out[47:40] = bytes[address+5];
            data_out[39:32] = bytes[address+4];
            data_out[31:24] = bytes[address+3];
            data_out[23:16] = bytes[address+2];
            data_out[15:8]  = bytes[address+1];
            data_out[7:0]   = bytes[address];
        end else begin
            data_out = 64'h0;
        end
    end

    // Memory write logic
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

// Modified Control Unit with branch detection
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
    always_comb begin
        is_branch = (operation >= 5'b01000 && operation <= 5'b01110);
        branch_taken = 0;
        
        case (operation)
            5'b01000: begin  // br
                next_pc = dest_in;
                branch_taken = 1;
            end
            
            5'b01001: begin  // brr $r_d
                next_pc = current_pc + dest_in;
                branch_taken = 1;
            end
            
            5'b01010: begin  // brr L
                next_pc = current_pc + $signed(immediate);
                branch_taken = 1;
            end
            
            5'b01011: begin  // brnz
                next_pc = (src_in1 != 0) ? dest_in : current_pc + 4;
                branch_taken = (src_in1 != 0);
            end
            
            5'b01100: begin  // call
                next_pc = dest_in;
                branch_taken = 1;
            end
            
            5'b01101: begin  // return
                next_pc = memory_data;
                branch_taken = 1;
            end
            
            5'b01110: begin  // brgt
                next_pc = (src_in1 > src_in2) ? dest_in : current_pc + 4;
                branch_taken = (src_in1 > src_in2);
            end
            
            default: begin
                next_pc = current_pc + 4;
                branch_taken = 0;
            end
        endcase
    end
endmodule

// Modified Memory Handler with state input

module mem_handler (
    input logic [2:0] state,  // Change from state_t to logic [2:0]
    input logic [4:0] op,
    input logic [63:0] dest,
    input logic [63:0] src,
    input logic [63:0] imm,
    input logic [63:0] pc,
    input logic [63:0] r31,
    output logic [63:0] addr_out,
    output logic [63:0] data_out
);
    // Rest of the module remains the same
    always_comb begin
        case (op)
            5'b01100: begin  // call
                addr_out = r31 - 8;
                data_out = pc + 4;
            end
            
            5'b01101: begin  // return
                addr_out = r31 - 8;
                data_out = 64'h0;
            end
            
            5'b10000: begin  // mov $r_d, ($r_s)(L)
                addr_out = src + imm;
                data_out = 64'h0;
            end
            
            5'b10011: begin  // mov ($r_d)(L), $r_s
                addr_out = dest + imm;
                data_out = src;
            end
            
            default: begin
                addr_out = 64'h0;
                data_out = 64'h0;
            end
        endcase
    end
endmodule

// Simplified Register File
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

    // Read register values
    assign data1 = registers[addr1];
    assign data2 = registers[addr2];
    assign data_dest = registers[write_addr];
    assign stack = registers[31];

    // Write to register file
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'h0;
            registers[31] <= 64'd524288;  // Stack pointer initialization
        end
        else if (write_en)
            registers[write_addr] <= write_data;
    end
endmodule

// ALU Unit (simplified - removed valid signal since that's handled by state machine)
module alu_unit (
    input logic [4:0] ctrl,
    input logic [63:0] in1,
    input logic [63:0] in2,
    output logic [63:0] out
);
    real f1, f2, fres;
    assign f1 = $bitstoreal(in1);
    assign f2 = $bitstoreal(in2);

    always_comb begin
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
            default: out = 64'h0;
        endcase
    end
endmodule

// Unchanged Instruction Decoder
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
        imm = {52'h0, instruction[11:0]};
        case (opcode)
            5'b11001, 5'b11011, 5'b00101, 5'b00111, 5'b10010: src1 = dest;
            default: ; 
        endcase
    end
endmodule

// Unchanged Mux
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