
// 1) Instruction Decoder
module instruction_decoder(
    input  [31:0] instruction,
    output reg [4:0]  opcode, 
    output reg [4:0]  rd,    
    output reg [4:0]  rs,    
    output reg [4:0]  rt,      
    output reg [63:0] literal,
    
    // 4-bit ALU opcode
    output reg [3:0]  alu_op,
    
    // Control signals
    output reg        is_immediate,
    output reg        reg_write_enable,
    output reg        is_float
);

    // ALU operation codes (4 bits)
    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b0001;
    localparam ALU_MUL  = 4'b0010;
    localparam ALU_DIV  = 4'b0011;
    localparam ALU_SHR  = 4'b0100; 
    localparam ALU_SHL  = 4'b0101; 
    localparam ALU_AND  = 4'b0110;
    localparam ALU_OR   = 4'b0111;
    localparam ALU_XOR  = 4'b1000;
    localparam ALU_NOT  = 4'b1001;

    always @(*) begin
        // Extract fields from instruction
        opcode   = instruction[31:27];
        rd       = instruction[26:22];
        rs       = instruction[21:17];
        rt       = instruction[16:12];
        literal = {{52{instruction[11]}}, instruction[11:0]}; // Sign-extend to 64 bits

        // Default control signals
        is_immediate     = 0;
        reg_write_enable = 1;
        is_float         = 0;
        alu_op           = ALU_ADD; // default to ADD, can be overridden below

        case (opcode)
            // Integer Arithmetic (From Tinker Manual: 0x18..0x1D)
            5'b11000: alu_op = ALU_ADD;  // add  (0x18)
            5'b11001: begin             // addi (0x19)
                alu_op       = ALU_ADD;
                is_immediate = 1;
            end
            5'b11010: alu_op = ALU_SUB;  // sub  (0x1A)
            5'b11011: begin             // subi (0x1B)
                alu_op       = ALU_SUB;
                is_immediate = 1;
            end
            5'b11100: alu_op = ALU_MUL;  // mul  (0x1C)
            5'b11101: alu_op = ALU_DIV;  // div  (0x1D)
            
            // Logic instructions (0x0..0x3)
            5'b00000: alu_op = ALU_AND;  // and (0x0)
            5'b00001: alu_op = ALU_OR;   // or  (0x1)
            5'b00010: alu_op = ALU_XOR;  // xor (0x2)
            5'b00011: alu_op = ALU_NOT;  // not (0x3)
            
            // Shift instructions (0x4..0x7)
            5'b00100: alu_op = ALU_SHR;  // shftr (0x4)
            5'b00101: begin             // shftri (0x5)
                alu_op       = ALU_SHR;
                is_immediate = 1;
            end
            5'b00110: alu_op = ALU_SHL;  // shftl (0x6)
            5'b00111: begin             // shftli (0x7)
                alu_op       = ALU_SHL;
                is_immediate = 1;
            end
            
            // Data Movement (0x11..0x12)
            5'b10001: begin // mov rd, rs (0x11)
                alu_op = ALU_ADD;
            end
            5'b10010: begin // mov rd, L (0x12)
                alu_op       = ALU_ADD;
                is_immediate = 1;
            end
            
            // Floating-Point (0x14..0x17)
            5'b10100: begin // addf (0x14)
                alu_op   = ALU_ADD; 
                is_float = 1;
            end
            5'b10101: begin // subf (0x15)
                alu_op   = ALU_SUB;
                is_float = 1;
            end
            5'b10110: begin // mulf (0x16)
                alu_op   = ALU_MUL;
                is_float = 1;
            end
            5'b10111: begin // divf (0x17)
                alu_op   = ALU_DIV;
                is_float = 1;
            end
            
            default: begin
                // Unknown opcode => disable writes
                reg_write_enable = 0;
            end
        endcase
    end
endmodule

module register_file(
    input clk,
    input reset, 
    input  [4:0] rs_addr,
    input  [4:0] rt_addr,
    input  [4:0] rd_addr,
    input  [63:0] write_data,
    input         write_enable,
    output [63:0] rs_data,
    output [63:0] rt_data,
    output [63:0] stack_ptr 
);
    reg [63:0] registers [0:31];
    
    assign stack_ptr = registers[31];
    assign rs_data = registers[rs_addr];
    assign rt_data = registers[rt_addr];
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize all registers to 0
            for (int i = 0; i < 32; i++) begin
                registers[i] <= 64'd0;
            end
            registers[31] <= 64'd524288; // Initialize stack pointer
        end else if (write_enable && (rd_addr != 5'd0)) begin
            registers[rd_addr] <= write_data;
        end
    end
endmodule

module control(
    input [4:0] op,
	input [63:0] rd,
	input [63:0] rs,
	input [63:0] rt,
	input [63:0] lit,
	input [63:0] inputPc,
	input [63:0] memData,
	output reg [63:0] pc
);
	 localparam BR = 5'b01000;
	 localparam BRR_RD = 5'b01001;
	 localparam BRR_L = 5'b01010;
	 localparam BRNZ = 5'b01011;
	 localparam CALL = 5'b01100;
	 localparam RET = 5'b01101;
	 localparam BRGT = 5'b01110;
	 always @(*) begin
	  case(op)
	   BR: pc = rd;
	   BRR_RD: pc = inputPc + rd;
	   BRR_L: pc = inputPc + $signed(lit);
	   BRNZ: pc = (rs != 0) ? rd : inputPc + 4;
	   CALL: pc = rd;
	   RET: pc = memData;
	   BRGT: pc = (rs > rt) ? rd : inputPc + 4;
	   default: pc = inputPc + 4;
	  endcase
	 end
endmodule


// 3) ALU / FPU Combined
// 4-bit op codes for integer arithmetic, logic, shift, etc.
module alu_fpu(
    input  [63:0] a,
    input  [63:0] b,
    input  [3:0]  op,
    input         is_float,
    output reg [63:0] result
);
    real float_a, float_b, float_res;
    
    // 4-bit ALU codes
    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001;
    localparam ALU_MUL = 4'b0010;
    localparam ALU_DIV = 4'b0011;
    localparam ALU_SHR = 4'b0100; // Shift Right
    localparam ALU_SHL = 4'b0101; // Shift Left
    localparam ALU_AND = 4'b0110;
    localparam ALU_OR  = 4'b0111;
    localparam ALU_XOR = 4'b1000;
    localparam ALU_NOT = 4'b1001;

    always @(*) begin
        if (is_float) begin
            // Floating-Point path
            float_a = $bitstoreal(a);
            float_b = $bitstoreal(b);
            
            case (op)
                ALU_ADD: float_res = float_a + float_b;
                ALU_SUB: float_res = float_a - float_b;
                ALU_MUL: float_res = float_a * float_b;
                ALU_DIV: float_res = float_a / float_b;
                default: float_res = 0.0;
            endcase
            
            result = $realtobits(float_res);
        end
        else begin
            // Integer / Logic path
            case (op)
                ALU_ADD: result = a + b;
                ALU_SUB: result = a - b;
                ALU_MUL: result = a * b;
                ALU_DIV: result = (b != 0) ? (a / b) : 64'd0; 
                ALU_SHR: result = a >> b[5:0];
                ALU_SHL: result = a << b[5:0];
                ALU_AND: result = a & b;
                ALU_OR : result = a | b;
                ALU_XOR: result = a ^ b;
                ALU_NOT: result = ~a;
                default: result = 64'd0;
            endcase
        end
    end
endmodule



module clock_generator(
    output reg clk
);
    initial begin
        clk = 0;
    end
    
    always begin
        #5 clk = ~clk;
    end
endmodule

module fetch(
	input clk,
	input reset,
	input [63:0] sp,
	input [63:0] new_pc,
	output reg [63:0] pc_out
);
	reg [63:0] curr_pc;
	assign pc_out = curr_pc;
	always @(posedge clk or posedge reset) begin
		if(reset) begin
			curr_pc <= 64'h2000;
		end else begin
			curr_pc <= new_pc;
		end
	end
endmodule


module memoryHandler(
	input [4:0] opcode,
	input [63:0] rd, rs, lit, pc, r31,
	output reg [63:0] mem_addr,
	output reg [63:0] mem_wr_data,
	output reg mem_wr_en,
	output reg mem_reg_write
);
	always @(*) begin
		case(opcode)
			5'b01100: begin
				mem_addr = r31 - 8;
				mem_wr_data = pc + 4;
				mem_wr_en = 1;
				mem_reg_write = 0;
			end
			5'b01101: begin
				mem_addr = r31 - 8;
				mem_wr_data = 0;
				mem_wr_en = 0;
				mem_reg_write = 0;
			end
			5'b10000: begin
				mem_addr = rs + lit;
				mem_wr_data = 0;
				mem_wr_en = 0;
				mem_reg_write = 1;
			end
			5'b10011: begin
				mem_addr = rd + lit;
				mem_wr_data = rs;
				mem_wr_en = 1;
				mem_reg_write = 0;
			end
			default: begin
				mem_addr = 64'h2000;
				mem_wr_data = 0;
				mem_wr_en = 0;
				mem_reg_write = 0;
			end
		endcase
	end
endmodule
module memory(
    input [63:0] address_pc,      
    input clock,                  
    input reset_signal,            
    input write_enable,           
    input [63:0] data_in,          
    input [63:0] address_rw,       
    output reg [63:0] data_out,    
    output reg [31:0] inst_out     
);

    reg [7:0] memory_bytes [0:524287];  
    integer idx;                        

    assign inst_out[7:0] = memory_bytes[address_pc];
    assign inst_out[15:8] = memory_bytes[address_pc+1];
    assign inst_out[23:16] = memory_bytes[address_pc+2];
    assign inst_out[31:24] = memory_bytes[address_pc+3];

    assign data_out[7:0] = memory_bytes[address_rw];
    assign data_out[15:8] = memory_bytes[address_rw+1];
    assign data_out[23:16] = memory_bytes[address_rw+2];
    assign data_out[31:24] = memory_bytes[address_rw+3];
    assign data_out[39:32] = memory_bytes[address_rw+4];
    assign data_out[47:40] = memory_bytes[address_rw+5];
    assign data_out[55:48] = memory_bytes[address_rw+6];
    assign data_out[63:56] = memory_bytes[address_rw+7];

    always @(posedge clock or posedge reset_signal) begin
        if(reset_signal) begin
            for (idx = 0; idx < 524288; idx = idx + 1) begin
                memory_bytes[idx] <= 8'b0;
            end
        end else begin
            if (write_enable) begin
                memory_bytes[address_rw] <= data_in[7:0];
                memory_bytes[address_rw+1] <= data_in[15:8];
                memory_bytes[address_rw+2] <= data_in[23:16];
                memory_bytes[address_rw+3] <= data_in[31:24];
                memory_bytes[address_rw+4] <= data_in[39:32];
                memory_bytes[address_rw+5] <= data_in[47:40];
                memory_bytes[address_rw+6] <= data_in[55:48];
                memory_bytes[address_rw+7] <= data_in[63:56];
            end
        end
    end

endmodule


module reglitmux(
    input [4:0] sel,
    input [63:0] reg_val,
    input [63:0] imm_val,
    output reg [63:0] mux_out
);
    always @(*) begin
        case(sel)
        5'b11001, 5'b11011, 5'b00101, 5'b00111, 5'b10010: mux_out = imm_val;
   default: mux_out = reg_val;
     endcase
    end
endmodule


// Top-Level Module: tinker_core
module tinker_core(
    input clk,
    input reset
);
    // Internal wires
    wire [63:0] pc, new_pc, mem_data_out, stack_ptr; 
    wire [31:0] instruction;
    wire [4:0] opcode, rd, rs, rt;
    wire [63:0] literal; 
    wire [3:0] alu_op;
    wire is_immediate, reg_write_enable, is_float;
    wire [63:0] rs_data, rt_data, alu_result, operand_b;
    wire [63:0] mem_addr, mem_wr_data;
    wire mem_wr_en, mem_reg_write;
    wire [63:0] write_data = mem_reg_write ? mem_data_out : alu_result;
    wire write_enable = reg_write_enable || mem_reg_write;
    wire [63:0] rd_ext = {59'd0, rd};

    // Instantiate Memory
    memory memory_inst(
        .address_pc(pc),
        .clock(clk),
        .reset_signal(reset),
        .write_enable(mem_wr_en),
        .data_in(mem_wr_data),
        .address_rw(mem_addr),
        .data_out(mem_data_out),
        .inst_out(instruction)
    );


    // Fetch Unit
    fetch fetch_inst(
        .clk(clk),
        .reset(reset),
        .sp(64'd0), // Assuming SP not used in basic tests
        .new_pc(new_pc),
        .pc_out(pc)
    );

control control_inst(
    .op(opcode),
    .rd(rd_ext),  // Use the extended signal
    .rs(rs_data),
    .rt(rt_data),
    .lit(literal),
    .inputPc(pc),
    .memData(mem_data_out),
    .pc(new_pc)
);


    // Instruction Decoder
    instruction_decoder decoder_inst(
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .literal(literal),
        .alu_op(alu_op),
        .is_immediate(is_immediate),
        .reg_write_enable(reg_write_enable),
        .is_float(is_float)
    );

    // Register File
register_file reg_file(
    .clk(clk),
    .reset(reset),
    .rs_addr(rs),
    .rt_addr(rt),
    .rd_addr(rd),
    .write_data(write_data),
    .write_enable(write_enable),
    .stack_ptr(stack_ptr), // Added comma
    .rs_data(rs_data),
    .rt_data(rt_data)
);

    // ALU Operand Mux
    assign operand_b = is_immediate ? {{52{literal[11]}}, literal} : rt_data;

    // ALU/FPU
    alu_fpu alu_unit(
        .a(rs_data),
        .b(operand_b),
        .op(alu_op),
        .is_float(is_float),
        .result(alu_result)
    );

    // Memory Handler
    memoryHandler mem_handler(
        .opcode(opcode),
        .rd(rd_ext),   // (or use rd_ext if you choose Option A above)
        .rs(rs_data),
        .lit(literal),
        .pc(pc),
        .r31(stack_ptr),
        .mem_addr(mem_addr),
        .mem_wr_data(mem_wr_data),
        .mem_wr_en(mem_wr_en),
        .mem_reg_write(mem_reg_write)
    );


endmodule