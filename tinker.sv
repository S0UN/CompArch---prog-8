//############################################################################
//## tinker_core (Top Level Module)
//############################################################################
module tinker_core (
    input clk,
    input reset,
    output hlt
);
    // --- Pipeline Stage Signals ---
    // IF Stage
    logic [63:0] pc_if;
    logic [31:0] instruction_if;

    // IF/DE Register Outputs
    logic [63:0] pc_de;
    logic [31:0] instruction_de;

    // DE Stage Outputs (Inputs to DE/EX Register)
    logic [63:0] operand_a_de; // rs_data
    logic [63:0] regfile_operand_b_de; // rt_data (before mux)
    logic [63:0] operand_b_de; // rt_data or literal (after mux)
    logic [63:0] operand_c_de; // rd_data (read via Port 3)
    logic [63:0] literal_de;
    logic [4:0]  rd_addr_de;
    logic [4:0]  rs_addr_de;
    logic [4:0]  rt_addr_de;
    logic [4:0]  opcode_de;
    logic [63:0] stack_ptr_de;
    logic        alu_enable_de, mem_read_de, mem_write_de, reg_write_de, mem_to_reg_de, branch_taken_ctrl_de, mem_pc_de;

    // DE/EX Register Outputs -> Inputs to EX Stage
    logic [63:0] pc_ex;
    logic [63:0] operand_a_ex; // Pipelined rs_data (or rd_data for I-type)
    logic [63:0] operand_b_ex; // Pipelined rt_data or literal
    logic [63:0] operand_c_ex; // Pipelined rd_data
    logic [63:0] literal_ex;
    logic [4:0]  rd_addr_ex;
    logic [4:0]  rs_addr_ex;
    logic [4:0]  rt_addr_ex;
    logic [4:0]  opcode_ex;
    logic [63:0] stack_ptr_ex;
    logic        alu_enable_ex, mem_read_ex, mem_write_ex, reg_write_ex, mem_to_reg_ex, mem_pc_ex;

    // EX Stage Outputs -> Inputs to EX/MEM Register
    logic [63:0] alu_result_ex;
    logic [63:0] alu_mem_addr_ex;
    logic [63:0] alu_mem_data_ex;
    logic [63:0] alu_branch_pc_ex;
    logic        branch_taken_ex;
    logic        hlt_ex;

    // EX/MEM Register Outputs -> Inputs to MEM Stage
    logic [63:0] alu_result_mem;
    logic [63:0] mem_addr_mem;
    logic [63:0] mem_wdata_mem;
    logic [63:0] branch_pc_mem;
    logic [4:0]  rd_addr_mem;
    logic        mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, branch_taken_mem, mem_pc_mem;
    logic        hlt_mem;

    // MEM Stage Outputs
    logic [63:0] mem_rdata_mem;
    logic [63:0] return_pc_mem;

    // MEM/WB Stage
    logic [63:0] reg_wdata_wb;

    // --- Control Signals ---
    logic flush_de, flush_ex;
    logic take_return_pc_fetch;

    // --- Forwarding Signals ---
    logic [63:0] forwarded_rs_data, forwarded_rt_data;
    logic forward_rs_ex, forward_rt_ex;

    // --- Module Instantiations ---

    fetch instruction_fetcher (
        .clk(clk),
        .reset(reset),
        .branch_taken(branch_taken_mem),
        .branch_pc(branch_pc_mem),
        .take_return_pc(take_return_pc_fetch),
        .return_pc(return_pc_mem),
        .pc_out(pc_if)
    );

    memory memory (
        .clk(clk),
        .reset(reset),
        .inst_addr(pc_if),
        .instruction_out(instruction_if),
        .data_addr(mem_addr_mem),
        .data_wdata(mem_wdata_mem),
        .mem_read(mem_read_mem),
        .mem_write(mem_write_mem),
        .data_rdata(mem_rdata_mem)
    );

    if_de_register if_de_reg (
        .clk(clk),
        .flush(flush_de),
        .pc_in(pc_if),
        .instruction_in(instruction_if),
        .pc_out(pc_de),
        .instruction_out(instruction_de)
    );

    instructionDecoder instruction_parser (
        .instructionLine(instruction_de),
        .literal(literal_de),
        .rd(rd_addr_de),
        .rs(rs_addr_de),
        .rt(rt_addr_de),
        .opcode(opcode_de),
        .alu_enable(alu_enable_de),
        .mem_read(mem_read_de),
        .mem_write(mem_write_de),
        .reg_write(reg_write_de),
        .mem_to_reg(mem_to_reg_de),
        .branch_taken(branch_taken_ctrl_de),
        .mem_pc(mem_pc_de)
    );

    registerFile reg_file (
        .clk(clk),
        .reset(reset),
        .write_addr(rd_addr_mem),
        .write_data(reg_wdata_wb),
        .write_enable(reg_write_mem),
        .read_addr1(rs_addr_de),
        .read_data1(operand_a_de),
        .read_addr2(rt_addr_de),
        .read_data2(regfile_operand_b_de),
        .read_addr3(rd_addr_de),
        .read_data3(operand_c_de),
        .stack_ptr_out(stack_ptr_de)
    );

    // Forwarding Logic
    assign forward_rs_ex = (rs_addr_de == rd_addr_ex) && reg_write_ex && (rd_addr_ex != 0);
    assign forward_rt_ex = (rt_addr_de == rd_addr_ex) && reg_write_ex && (rd_addr_ex != 0);
    assign forwarded_rs_data = forward_rs_ex ? alu_result_ex : operand_a_de;
    assign forwarded_rt_data = forward_rt_ex ? alu_result_ex : regfile_operand_b_de;

    reglitmux input_selector (
        .sel(opcode_de),
        .reg_in(forwarded_rt_data), // Use forwarded rt_data
        .lit_in(literal_de),
        .out(operand_b_de)
    );

    de_ex_register de_ex_reg (
        .clk(clk),
        .flush(flush_ex),
        .pc_in(pc_de),
        .operand_a_in(forwarded_rs_data), // Use forwarded rs_data
        .operand_b_in(operand_b_de),
        .operand_c_in(operand_c_de),
        .literal_in(literal_de),
        .rd_addr_in(rd_addr_de),
        .rs_addr_in(rs_addr_de),
        .rt_addr_in(rt_addr_de),
        .opcode_in(opcode_de),
        .stack_ptr_in(stack_ptr_de),
        .alu_enable_in(alu_enable_de),
        .mem_read_in(mem_read_de),
        .mem_write_in(mem_write_de),
        .reg_write_in(reg_write_de),
        .mem_to_reg_in(mem_to_reg_de),
        .branch_taken_ctrl_in(branch_taken_ctrl_de),
        .mem_pc_in(mem_pc_de),
        .pc_out(pc_ex),
        .operand_a_out(operand_a_ex),
        .operand_b_out(operand_b_ex),
        .operand_c_out(operand_c_ex),
        .literal_out(literal_ex),
        .rd_addr_out(rd_addr_ex),
        .rs_addr_out(rs_addr_ex),
        .rt_addr_out(rt_addr_ex),
        .opcode_out(opcode_ex),
        .stack_ptr_out(stack_ptr_ex),
        .alu_enable_out(alu_enable_ex),
        .mem_read_out(mem_read_ex),
        .mem_write_out(mem_write_ex),
        .reg_write_out(reg_write_ex),
        .mem_to_reg_out(mem_to_reg_ex),
        .mem_pc_out(mem_pc_ex)
    );

    alu calculation_unit (
        .alu_enable(alu_enable_ex),
        .opcode(opcode_ex),
        .input1(operand_a_ex),
        .input2(operand_b_ex),
        .input3(operand_c_ex),
        .rd_addr(rd_addr_ex),
        .literal(literal_ex),
        .pc_in(pc_ex),
        .stack_ptr(stack_ptr_ex),
        .result(alu_result_ex),
        .mem_addr(alu_mem_addr_ex),
        .mem_wdata(alu_mem_data_ex),
        .branch_pc(alu_branch_pc_ex),
        .branch_taken(branch_taken_ex),
        .hlt_out(hlt_ex),
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .mem_pc_in(mem_pc_ex)
    );

    ex_mem_register ex_mem_reg (
        .clk(clk),
        .result_in(alu_result_ex),
        .mem_addr_in(alu_mem_addr_ex),
        .mem_wdata_in(alu_mem_data_ex),
        .branch_pc_in(alu_branch_pc_ex),
        .rd_addr_in(rd_addr_ex),
        .hlt_in(hlt_ex),
        .mem_read_in(mem_read_ex),
        .mem_write_in(mem_write_ex),
        .reg_write_in(reg_write_ex),
        .mem_to_reg_in(mem_to_reg_ex),
        .branch_taken_in(branch_taken_ex),
        .mem_pc_in(mem_pc_ex),
        .result_out(alu_result_mem),
        .mem_addr_out(mem_addr_mem),
        .mem_wdata_out(mem_wdata_mem),
        .branch_pc_out(branch_pc_mem),
        .rd_addr_out(rd_addr_mem),
        .hlt_out(hlt_mem),
        .mem_read_out(mem_read_mem),
        .mem_write_out(mem_write_mem),
        .reg_write_out(reg_write_mem),
        .mem_to_reg_out(mem_to_reg_mem),
        .branch_taken_out(branch_taken_mem),
        .mem_pc_out(mem_pc_mem)
    );

    aluMemMux return_pc_selector (
        .mem_pc(mem_pc_mem),
        .memData(mem_rdata_mem),
        .aluOut(branch_pc_mem),
        .newPc(return_pc_mem)
    );

    memRegMux data_source_selector (
        .mem_to_reg(mem_to_reg_mem),
        .readData(mem_rdata_mem),
        .aluResult(alu_result_mem),
        .regWriteData(reg_wdata_wb)
    );

    assign flush_de = branch_taken_mem || mem_pc_mem;
    assign flush_ex = branch_taken_mem || mem_pc_mem;
    assign take_return_pc_fetch = mem_pc_mem;
    assign hlt = hlt_mem;

endmodule

//############################################################################
//## alu (Updated for Correct Branch and Return Handling)
//############################################################################
module alu (
    input logic alu_enable,
    input logic [4:0] opcode,
    input logic [63:0] input1,    // Pipelined rs_data (or rd_data for I-types)
    input logic [63:0] input2,    // Pipelined rt_data or literal
    input logic [63:0] input3,    // Pipelined rd_data
    input logic [4:0] rd_addr,
    input logic [63:0] literal,
    input logic [63:0] pc_in,
    input logic [63:0] stack_ptr,
    output logic [63:0] result,
    output logic [63:0] mem_addr,
    output logic [63:0] mem_wdata,
    output logic [63:0] branch_pc,
    output logic branch_taken,
    output logic hlt_out,
    input logic mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in, mem_pc_in
);
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;

    always @(*) begin
        result = 64'b0; mem_addr = 64'b0; mem_wdata = 64'b0;
        branch_pc = pc_in + 4; branch_taken = 1'b0; hlt_out = 1'b0;

        if (alu_enable) begin
            case (opcode)
                ADD, ADDI: result = $signed(input1) + $signed(input2);
                SUB, SUBI: result = $signed(input1) - $signed(input2);
                MUL: result = $signed(input1) * $signed(input2);
                DIV: if (input2 != 0) result = $signed(input1) / $signed(input2); else result = 64'b0;
                AND: result = input1 & input2;
                OR:  result = input1 | input2;
                XOR: result = input1 ^ input2;
                NOT: result = ~input1;
                SHFTR, SHFTRI: result = input1 >> input2[5:0];
                SHFTL, SHFTLI: result = input1 << input2[5:0];
                MOV_MEM: mem_addr = input1 + $signed(input2);
                MOV_REG: result = input1;
                MOV_LIT: result = {input1[63:12], input2[11:0]};
                MOV_STR: begin mem_addr = input3 + $signed(literal); mem_wdata = input1; end
                ADDF, SUBF, MULF, DIVF: result = 64'b0; // FP ops skipped
                BR: begin branch_pc = input3; branch_taken = 1'b1; end // Unconditional branch to rd_data
                BRR: begin branch_pc = pc_in + $signed(input3); branch_taken = 1'b1; end // PC + rd_data
                BRRI: begin branch_pc = pc_in + $signed(literal); branchгранич

_taken = 1'b1; end // PC + literal
                BRNZ: begin 
                    branch_pc = input3; // Target is rd_data
                    branch_taken = ($signed(input1) != 0); 
                end
                BRGT: begin 
                    branch_pc = input3; // Target is rd_data
                    branch_taken = ($signed(input1) > $signed(input2)); 
                end
                CALL: begin 
                    branch_pc = input3; 
                    mem_addr = stack_ptr - 8; 
                    mem_wdata = pc_in + 4; 
                    branch_taken = 1'b1; 
                end
                RETURN: begin 
                    mem_addr = stack_ptr - 8; 
                    branch_taken = 1'b0; // Handled by fetch
                end
                PRIV: if (literal[11:0] == 12'h0) hlt_out = 1'b1;
                default: result = 64'b0;
            endcase
        end
    end
endmodule

// Include unchanged modules here for completeness (not modified but required for the design)
module registerFile (
    input clk,
    input reset,
    input [4:0] write_addr,
    input [63:0] write_data,
    input write_enable,
    input [4:0] read_addr1,
    output logic [63:0] read_data1,
    input [4:0] read_addr2,
    output logic [63:0] read_data2,
    input [4:0] read_addr3,
    output logic [63:0] read_data3,
    output logic [63:0] stack_ptr_out
);
    reg [63:0] registers [0:31];
    integer idx;

    initial begin
        for (idx = 0; idx < 31; idx = idx + 1) begin
            registers[idx] = 64'b0;
        end
        registers[31] = 64'h0008_0000;
    end

    assign read_data1 = (read_addr1 == 5'd31) ? registers[31] : registers[read_addr1];
    assign read_data2 = (read_addr2 == 5'd31) ? registers[31] : registers[read_addr2];
    assign read_data3 = (read_addr3 == 5'd31) ? registers[31] : registers[read_addr3];
    assign stack_ptr_out = registers[31];

    always @(posedge clk) begin
        if (!reset && write_enable) begin
            registers[write_addr] <= write_data;
        end
    end
endmodule

module fetch (
    input clk,
    input reset,
    input branch_taken,
    input [63:0] branch_pc,
    input take_return_pc,
    input [63:0] return_pc,
    output logic [63:0] pc_out
);
    localparam INITIAL_PC = 64'h2000;
    reg [63:0] current_pc;
    assign pc_out = current_pc;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_pc <= INITIAL_PC;
        end else begin
            if (take_return_pc) current_pc <= return_pc;
            else if (branch_taken) current_pc <= branch_pc;
            else current_pc <= current_pc + 64'd4;
        end
    end
endmodule

module if_de_register (
    input clk,
    input flush,
    input [63:0] pc_in,
    input [31:0] instruction_in,
    output reg [63:0] pc_out,
    output reg [31:0] instruction_out
);
    always @(posedge clk) begin
        if (flush) begin
            pc_out <= 64'b0;
            instruction_out <= 32'b0;
        end else begin
            pc_out <= pc_in;
            instruction_out <= instruction_in;
        end
    end
endmodule

module instructionDecoder (
    input [31:0] instructionLine,
    output reg [63:0] literal,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [4:0] opcode,
    output reg alu_enable,
    output reg mem_read,
    output reg mem_write,
    output reg reg_write,
    output reg mem_to_reg,
    output reg branch_taken,
    output reg mem_pc
);
    localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
               SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
               CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
               MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
               ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;

    always @(*) begin
        opcode = instructionLine[31:27];
        rd = instructionLine[26:22];
        rs = instructionLine[21:17];
        rt = instructionLine[16:12];
        literal = {{52{instructionLine[11]}}, instructionLine[11:0]}; // Sign-extend literal

        alu_enable = 1'b0; mem_read = 1'b0; mem_write = 1'b0; reg_write = 1'b0;
        mem_to_reg = 1'b0; branch_taken = 1'b0; mem_pc = 1'b0;

        case (opcode)
            ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end
            ADDI, SUBI, SHFTRI, SHFTLI: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end
            MOV_MEM: begin alu_enable=1'b1; mem_read=1'b1; reg_write=1'b1; mem_to_reg=1'b1; end
            MOV_STR: begin alu_enable=1'b1; mem_write=1'b1; reg_write=1'b0; end
            MOV_REG: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end
            MOV_LIT: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end
            ADDF, SUBF, MULF, DIVF: begin alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; end
            BR, BRR, BRRI, BRNZ, BRGT: begin alu_enable=1'b1; reg_write=1'b0; branch_taken=1'b1; end
            CALL: begin alu_enable=1'b1; mem_write=1'b1; reg_write=1'b0; branch_taken=1'b1; end
            RETURN: begin alu_enable=1'b1; mem_read=1'b1; reg_write=1'b0; mem_pc=1'b1; branch_taken=1'b1; end
            PRIV: if(literal[11:0]==12'h0) begin alu_enable=1'b1; reg_write=1'b0; end else alu_enable=1'b0;
            default: ;
        endcase

        case (opcode)
            ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: rs = rd;
            default: ;
        endcase
    end
endmodule

module de_ex_register (
    input clk,
    input flush,
    input [63:0] pc_in,
    input [63:0] operand_a_in,
    input [63:0] operand_b_in,
    input [63:0] operand_c_in,
    input [63:0] literal_in,
    input [4:0] rd_addr_in,
    input [4:0] rs_addr_in,
    input [4:0] rt_addr_in,
    input [4:0] opcode_in,
    input [63:0] stack_ptr_in,
    input alu_enable_in,
    input mem_read_in,
    input mem_write_in,
    input reg_write_in,
    input mem_to_reg_in,
    input branch_taken_ctrl_in,
    input mem_pc_in,
    output reg [63:0] pc_out,
    output reg [63:0] operand_a_out,
    output reg [63:0] operand_b_out,
    output reg [63:0] operand_c_out,
    output reg [63:0] literal_out,
    output reg [4:0] rd_addr_out,
    output reg [4:0] rs_addr_out,
    output reg [4:0] rt_addr_out,
    output reg [4:0] opcode_out,
    output reg [63:0] stack_ptr_out,
    output reg alu_enable_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_ctrl_out,
    output reg mem_pc_out
);
    always @(posedge clk) begin
        if (flush) begin
            pc_out <= 64'b0; operand_a_out <= 64'b0; operand_b_out <= 64'b0; operand_c_out <= 64'b0;
            literal_out <= 64'b0; rd_addr_out <= 5'b0; rs_addr_out <= 5'b0; rt_addr_out <= 5'b0;
            opcode_out <= 5'b0; stack_ptr_out <= 64'b0; alu_enable_out <= 1'b0; mem_read_out <= 1'b0;
            mem_write_out <= 1'b0; reg_write_out <= 1'b0; mem_to_reg_out <= 1'b0;
            branch_taken_ctrl_out <= 1'b0; mem_pc_out <= 1'b0;
        end else begin
            pc_out <= pc_in; operand_a_out <= operand_a_in; operand_b_out <= operand_b_in; operand_c_out <= operand_c_in;
            literal_out <= literal_in; rd_addr_out <= rd_addr_in; rs_addr_out <= rs_addr_in; rt_addr_out <= rt_addr_in;
            opcode_out <= opcode_in; stack_ptr_out <= stack_ptr_in; alu_enable_out <= alu_enable_in;
            mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in; branch_taken_ctrl_out <= branch_taken_ctrl_in; mem_pc_out <= mem_pc_in;
        end
    end
endmodule

module ex_mem_register (
    input clk,
    input logic [63:0] result_in,
    input logic [63:0] mem_addr_in,
    input logic [63:0] mem_wdata_in,
    input logic [63:0] branch_pc_in,
    input logic [4:0] rd_addr_in,
    input logic hlt_in,
    input logic mem_read_in,
    input logic mem_write_in,
    input logic reg_write_in,
    input logic mem_to_reg_in,
    input logic branch_taken_in,
    input logic mem_pc_in,
    output logic [63:0] result_out,
    output logic [63:0] mem_addr_out,
    output logic [63:0] mem_wdata_out,
    output logic [63:0] branch_pc_out,
    output logic [4:0] rd_addr_out,
    output logic hlt_out,
    output logic mem_read_out,
    output logic mem_write_out,
    output logic reg_write_out,
    output logic mem_to_reg_out,
    output logic branch_taken_out,
    output logic mem_pc_out
);
    always @(posedge clk) begin
        result_out <= result_in; mem_addr_out <= mem_addr_in; mem_wdata_out <= mem_wdata_in;
        branch_pc_out <= branch_pc_in; rd_addr_out <= rd_addr_in; hlt_out <= hlt_in;
        mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in; branch_taken_out <= branch_taken_in; mem_pc_out <= mem_pc_in;
    end
endmodule

module memory (
    input clk,
    input reset,
    input [63:0] inst_addr,
    output logic [31:0] instruction_out,
    input [63:0] data_addr,
    input [63:0] data_wdata,
    input mem_read,
    input mem_write,
    output logic [63:0] data_rdata
);
    localparam MEM_SIZE_BYTES = 524288;
    reg [7:0] bytes [0:MEM_SIZE_BYTES-1];
    integer i;
    initial begin for(i=0;i<MEM_SIZE_BYTES;i=i+1) bytes[i]=8'b0; end
    assign instruction_out[7:0]=bytes[inst_addr+0]; assign instruction_out[15:8]=bytes[inst_addr+1];
    assign instruction_out[23:16]=bytes[inst_addr+2]; assign instruction_out[31:24]=bytes[inst_addr+3];
    assign data_rdata[7:0]=bytes[data_addr+0]; assign data_rdata[15:8]=bytes[data_addr+1];
    assign data_rdata[23:16]=bytes[data_addr+2]; assign data_rdata[31:24]=bytes[data_addr+3];
    assign data_rdata[39:32]=bytes[data_addr+4]; assign data_rdata[47:40]=bytes[data_addr+5];
    assign data_rdata[55:48]=bytes[data_addr+6]; assign data_rdata[63:56]=bytes[data_addr+7];
    always @(posedge clk) begin if(mem_write) begin
        bytes[data_addr+0]<=data_wdata[7:0]; bytes[data_addr+1]<=data_wdata[15:8];
        bytes[data_addr+2]<=data_wdata[23:16]; bytes[data_addr+3]<=data_wdata[31:24];
        bytes[data_addr+4]<=data_wdata[39:32]; bytes[data_addr+5]<=data_wdata[47:40];
        bytes[data_addr+6]<=data_wdata[55:48]; bytes[data_addr+7]<=data_wdata[63:56]; end end
endmodule

module aluMemMux (
    input mem_pc,
    input [63:0] memData,
    input [63:0] aluOut,
    output reg [63:0] newPc
);
    always @(*) begin
        if (mem_pc) newPc = memData;
        else newPc = aluOut;
    end
endmodule

module reglitmux (
    input [4:0] sel,
    input [63:0] reg_in,
    input [63:0] lit_in,
    output reg [63:0] out
);
    localparam ADDI=5'h19, SUBI=5'h1B, SHFTRI=5'h05, SHFTLI=5'h07, BRRI=5'h0A,
               MOV_MEM=5'h10, MOV_LIT=5'h12, MOV_STR=5'h13;
    always @(*) begin
        case (sel)
            ADDI, SUBI, SHFTRI, SHFTLI, BRRI, MOV_MEM, MOV_LIT, MOV_STR: out = lit_in;
            default: out = reg_in;
        endcase
    end
endmodule

module memRegMux (
    input mem_to_reg,
    input [63:0] readData,
    input [63:0] aluResult,
    output reg [63:0] regWriteData
);
    always @(*) begin
        if (mem_to_reg) regWriteData = readData;
        else regWriteData = aluResult;
    end
endmodule