
// 1) Instruction Decoder
module instruction_decoder(
    input  [31:0] instruction,
    output reg [4:0]  opcode, 
    output reg [4:0]  rd,    
    output reg [4:0]  rs,    
    output reg [4:0]  rt,      
    output reg [63:0] literal,
    output reg [4:0]  alu_op,  // Now 5-bit to match ALU
    output reg        is_immediate,
    output reg        reg_write_enable,
    output reg        is_float
);
    always @(*) begin
        // Extract fields from instruction
        opcode = instruction[31:27];
        rd     = instruction[26:22];
        rs     = instruction[21:17];
        rt     = instruction[16:12];
        literal = {{52{instruction[11]}}, instruction[11:0]};

        // Default control signals
        is_immediate     = 0;
        reg_write_enable = 1;
        is_float         = 0;
        alu_op           = opcode;  // Direct opcode mapping

        // Handle special cases
        case (opcode)
            // Immediate instructions
            5'b11001, 5'b11011,  // addi, subi
            5'b00101, 5'b00111,  // shftri, shftli
            5'b10010: begin      // mov rd, L
                is_immediate = 1;
                rs = rd;  // For instructions using rd as source
                rt = 5'b0;
            end
            
            // Floating-point instructions
            5'b10100, 5'b10101, 5'b10110, 5'b10111: 
                is_float = 1;

            // Control flow instructions
            5'b01000, 5'b01001, 5'b01010, 5'b01011, 
            5'b01100, 5'b01101, 5'b01110: 
                reg_write_enable = 0;

            default: reg_write_enable = 1;
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
    always @(*) begin
        case(op)
            5'b01000: pc = rd;                 // br
            5'b01001: pc = inputPc + rd;       // brr rd
            5'b01010: pc = inputPc + $signed(lit); // brr L
            5'b01011: pc = (rs != 0) ? rd : inputPc + 4; // brnz
            5'b01100: pc = rd;                 // call
            5'b01101: pc = memData;            // ret
            5'b01110: pc = (rs > rt) ? rd : inputPc + 4; // brgt
            default:   pc = inputPc + 4;       // normal flow
        endcase
    end
endmodule

module alu_fpu(
    input  [63:0] a,
    input  [63:0] b,
    input  [4:0]  op,
    input         is_float,
    output reg [63:0] result
);
    real float_a, float_b, float_res;

    always @(*) begin
        if (is_float) begin
            float_a = $bitstoreal(a);
            float_b = $bitstoreal(b);
            case(op)
                5'b10100: float_res = float_a + float_b;
                5'b10101: float_res = float_a - float_b;
                5'b10110: float_res = float_a * float_b;
                5'b10111: float_res = float_a / float_b;
                default: float_res = 0.0;
            endcase
            result = $realtobits(float_res);
        end
        else begin
            case(op)
                // Arithmetic
                5'b11000: result = a + b;  // add
                5'b11001: result = a + b;  // addi
                5'b11010: result = a - b;  // sub
                5'b11011: result = a - b;  // subi
                5'b11100: result = a * b;  // mul
                5'b11101: result = b != 0 ? a / b : 0;  // div

                // Logic
                5'b00000: result = a & b;  // and
                5'b00001: result = a | b;  // or
                5'b00010: result = a ^ b;  // xor
                5'b00011: result = ~a;     // not

                // Shifts
                5'b00100: result = a >> b[5:0];  // shftr
                5'b00101: result = a >> b[5:0];  // shftri
                5'b00110: result = a << b[5:0];  // shftl
                5'b00111: result = a << b[5:0];  // shftli

                // Data movement
                5'b10001: result = a;       // mov rd, rs
                5'b10010: result = {52'b0, b[11:0]};  // mov rd, L

                default: result = 64'b0;
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
    reg [7:0] bytes [0:524287];  
    integer idx;

    assign inst_out[7:0] = bytes[address_pc];
    assign inst_out[15:8] = bytes[address_pc+1];
    assign inst_out[23:16] = bytes[address_pc+2];
    assign inst_out[31:24] = bytes[address_pc+3];

    assign data_out[7:0] = bytes[address_rw];
    assign data_out[15:8] = bytes[address_rw+1];
    assign data_out[23:16] = bytes[address_rw+2];
    assign data_out[31:24] = bytes[address_rw+3];
    assign data_out[39:32] = bytes[address_rw+4];
    assign data_out[47:40] = bytes[address_rw+5];
    assign data_out[55:48] = bytes[address_rw+6];
    assign data_out[63:56] = bytes[address_rw+7];

    always @(posedge clock or posedge reset_signal) begin
        if(reset_signal) begin
            for (idx = 0; idx < 524288; idx = idx + 1) begin
                bytes[idx] <= 8'b0;
            end
        end else if (write_enable) begin
            bytes[address_rw] <= data_in[7:0];
            bytes[address_rw+1] <= data_in[15:8];
            bytes[address_rw+2] <= data_in[23:16];
            bytes[address_rw+3] <= data_in[31:24];
            bytes[address_rw+4] <= data_in[39:32];
            bytes[address_rw+5] <= data_in[47:40];
            bytes[address_rw+6] <= data_in[55:48];
            bytes[address_rw+7] <= data_in[63:56];
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
    wire [4:0] alu_op;
    wire is_immediate, reg_write_enable, is_float;
    wire [63:0] rs_data, rt_data, alu_result, operand_b;
    wire [63:0] mem_addr, mem_wr_data;
    wire mem_wr_en, mem_reg_write;
    wire [63:0] write_data = mem_reg_write ? mem_data_out : alu_result;
    wire write_enable = reg_write_enable || mem_reg_write;
    wire [63:0] rd_ext = {59'd0, rd};

    // Instantiate Memory
    memory memory(
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
    .rd(rs_data),         
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