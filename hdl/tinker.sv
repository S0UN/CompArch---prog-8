
module instruction_decoder(
    input [31:0] instruction,
    output reg [4:0] opcode,
    output reg [4:0] rd,
    output reg [4:0] rs,
    output reg [4:0] rt,
    output reg [11:0] literal,
    output reg [3:0] alu_op,
    output reg is_immediate,
    output reg reg_write_enable,
    output reg is_float,
    output reg is_load
);

// ALU operation codes (4 bits)
localparam ALU_ADD = 4'b0000;
localparam ALU_SUB = 4'b0001;
localparam ALU_MUL = 4'b0010;
localparam ALU_DIV = 4'b0011;
localparam ALU_SHR = 4'b0100;
localparam ALU_SHL = 4'b0101;
localparam ALU_AND = 4'b0110;
localparam ALU_OR  = 4'b0111;
localparam ALU_XOR = 4'b1000;
localparam ALU_NOT = 4'b1001;

always @(*) begin
    // Extract fields from instruction
    opcode  = instruction[31:27];
    rd      = instruction[26:22];
    rs      = instruction[21:17];
    rt      = instruction[16:12];
    literal = instruction[11:0];

    // Default control signals
    is_immediate     = 0;
    reg_write_enable = 1;  // Default to 1, overridden for control/store instructions
    is_float         = 0;
    is_load          = 0;  // Default to 0, set to 1 for loads
    alu_op           = ALU_ADD; // Default to ADD, overridden as needed

    case (opcode)
        // Logic Instructions (0x0 to 0x3)
        5'h00: alu_op = ALU_AND; // and $r_d, $r_s, $r_t
        5'h01: alu_op = ALU_OR;  // or $r_d, $r_s, $r_t
        5'h02: alu_op = ALU_XOR; // xor $r_d, $r_s, $r_t
        5'h03: alu_op = ALU_NOT; // not $r_d, $r_s

        // Shift Instructions (0x4 to 0x7)
        5'h04: alu_op = ALU_SHR; // shftr $r_d, $r_s, $r_t
        5'h05: begin             // shftri $r_d, L
            alu_op = ALU_SHR;
            is_immediate = 1;
        end
        5'h06: alu_op = ALU_SHL; // shftl $r_d, $r_s, $r_t
        5'h07: begin             // shftli $r_d, L
            alu_op = ALU_SHL;
            is_immediate = 1;
        end

        // Control Instructions (0x8 to 0xe)
        5'h08: reg_write_enable = 0; // br $r_d
        5'h09: reg_write_enable = 0; // brr $r_d
        5'h0A: reg_write_enable = 0; // brr L
        5'h0B: reg_write_enable = 0; // brnz $r_d, $r_s
        5'h0C: reg_write_enable = 0; // call $r_d, $r_s, $r_t
        5'h0D: reg_write_enable = 0; // return
        5'h0E: reg_write_enable = 0; // brgt $r_d, $r_s, $r_t

        // Privileged Instructions (0xf) - Not implemented this week
        5'h0F: reg_write_enable = 0; // priv $r_d, $r_s, $r_t, L

        // Data Movement Instructions (0x10 to 0x13)
        5'h10: begin                 // mov $r_d, ($r_s)(L) - Load
            is_load = 1;
            reg_write_enable = 1;
        end
        5'h11: alu_op = ALU_ADD;     // mov $r_d, $r_s
        5'h12: begin                 // mov $r_d, L
            alu_op = ALU_ADD;
            is_immediate = 1;
        end
        5'h13: reg_write_enable = 0; // mov ($r_d)(L), $r_s - Store

        // Floating-Point Instructions (0x14 to 0x17)
        5'h14: begin                 // addf $r_d, $r_s, $r_t
            alu_op = ALU_ADD;
            is_float = 1;
        end
        5'h15: begin                 // subf $r_d, $r_s, $r_t
            alu_op = ALU_SUB;
            is_float = 1;
        end
        5'h16: begin                 // mulf $r_d, $r_s, $r_t
            alu_op = ALU_MUL;
            is_float = 1;
        end
        5'h17: begin                 // divf $r_d, $r_s, $r_t
            alu_op = ALU_DIV;
            is_float = 1;
        end

        // Integer Arithmetic Instructions (0x18 to 0x1D)
        5'h18: alu_op = ALU_ADD;     // add $r_d, $r_s, $r_t
        5'h19: begin                 // addi $r_d, L
            alu_op = ALU_ADD;
            is_immediate = 1;
        end
        5'h1A: alu_op = ALU_SUB;     // sub $r_d, $r_s, $r_t
        5'h1B: begin                 // subi $r_d, L
            alu_op = ALU_SUB;
            is_immediate = 1;
        end
        5'h1C: alu_op = ALU_MUL;     // mul $r_d, $r_s, $r_t
        5'h1D: alu_op = ALU_DIV;     // div $r_d, $r_s, $r_t

        // Unknown opcodes
        default: begin
            reg_write_enable = 0;
            is_load = 0;
        end
    endcase
end

endmodule

module register_file(
    input clk,
    input [4:0] rs_addr,
    input [4:0] rt_addr,
    input [4:0] rd_addr,
    input [63:0] write_data,
    input write_enable,
    output [63:0] rs_data,
    output [63:0] rt_data,
    output [63:0] rd_data  // Added third read port
);

    reg [63:0] registers [0:31];
    integer i;

    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 64'd0;
        end
    end

    // Combinational reads
    assign rs_data = registers[rs_addr];
    assign rt_data = registers[rt_addr];
    assign rd_data = registers[rd_addr];  // Provide register $r_d$'s value

    always @(posedge clk) begin
        if (write_enable && (rd_addr != 5'd0))
            registers[rd_addr] <= write_data;
    end
endmodule

module control(
    input [4:0] op,
    input [63:0] rd,    // Now receives registers[$r_d]
    input [63:0] rs,
    input [63:0] rt,
    input [63:0] lit,
    input [63:0] inputPc,
    input [63:0] memData,
    output reg [63:0] pc
);

    localparam BR     = 5'b01000;
    localparam BRR_RD = 5'b01001;
    localparam BRR_L  = 5'b01010;
    localparam BRNZ   = 5'b01011;
    localparam CALL   = 5'b01100;
    localparam RET    = 5'b01101;
    localparam BRGT   = 5'b01110;

    always @(*) begin
        case(op)
            BR:     pc = rd;                    // pc ← $r_d
            BRR_RD: pc = inputPc + rd;          // pc ← pc + $r_d
            BRR_L:  pc = inputPc + $signed(lit); // pc ← pc + L (signed)
            BRNZ:   pc = (rs != 0) ? rd : inputPc + 4;  // if $r_s != 0, pc ← $r_d
            CALL:   pc = rd;                    // pc ← $r_d
            RET:    pc = memData;               // pc ← Mem[$r_{31} - 8]
            BRGT:   pc = (rs > rt) ? rd : inputPc + 4;  // if $r_s > $r_t, pc ← $r_d
            default: pc = inputPc + 4;          // Default: next instruction
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
    input [63:0] addr_pc,
    input clk,
    input reset,
    input write_en,
    input [63:0] data_in,
    input [63:0] addr_rw,
    output reg [63:0] data_out,
    output reg [31:0] inst_out
);
    reg [7:0] bytes [0:524287];
    integer idx;
    always @(*) begin
        inst_out[7:0] = bytes[addr_pc];
        inst_out[15:8] = bytes[addr_pc+1];
        inst_out[23:16] = bytes[addr_pc+2];
        inst_out[31:24] = bytes[addr_pc+3];
    end
    always @(*) begin
        data_out[7:0] = bytes[addr_rw];
        data_out[15:8] = bytes[addr_rw+1];
        data_out[23:16] = bytes[addr_rw+2];
        data_out[31:24] = bytes[addr_rw+3];
        data_out[39:32] = bytes[addr_rw+4];
        data_out[47:40] = bytes[addr_rw+5];
        data_out[55:48] = bytes[addr_rw+6];
        data_out[63:56] = bytes[addr_rw+7];
    end
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            for(idx = 0; idx < 524288; idx = idx + 1)
                bytes[idx] <= 8'b0;
        end else if(write_en) begin
            bytes[addr_rw] <= data_in[7:0];
            bytes[addr_rw+1] <= data_in[15:8];
            bytes[addr_rw+2] <= data_in[23:16];
            bytes[addr_rw+3] <= data_in[31:24];
            bytes[addr_rw+4] <= data_in[39:32];
            bytes[addr_rw+5] <= data_in[47:40];
            bytes[addr_rw+6] <= data_in[55:48];
            bytes[addr_rw+7] <= data_in[63:56];
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

module tinker_core(
    input clk,
    input reset
);

    // Internal wires (add rd_data and is_load)
    wire [63:0] pc, new_pc, mem_data_out;
    wire [31:0] instruction;
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] literal;
    wire [3:0] alu_op;
    wire is_immediate, reg_write_enable, is_float, is_load;  // Added is_load
    wire [63:0] rs_data, rt_data, rd_data, alu_result, operand_b;  // Added rd_data
    wire [63:0] mem_addr, mem_wr_data, write_data;  // Added write_data
    wire mem_wr_en, mem_reg_write;

    // Instantiate Memory (unchanged)
    memory mem_inst(
        .addr_pc(pc),
        .clk(clk),
        .reset(reset),
        .write_en(mem_wr_en),
        .data_in(mem_wr_data),
        .addr_rw(mem_addr),
        .data_out(mem_data_out),
        .inst_out(instruction)
    );

    // Fetch Unit (unchanged)
    fetch fetch_inst(
        .clk(clk),
        .reset(reset),
        .sp(64'd0),
        .new_pc(new_pc),
        .pc_out(pc)
    );

    // Control Unit (use rd_data)
    control control_inst(
        .op(opcode),
        .rd(rd_data),  // Now 64-bit register value
        .rs(rs_data),
        .rt(rt_data),
        .lit({{52{literal[11]}}, literal}),  // Sign-extend literal
        .inputPc(pc),
        .memData(mem_data_out),
        .pc(new_pc)
    );

    // Instruction Decoder (with is_load)
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
        .is_float(is_float),
        .is_load(is_load)  // Added
    );

    // Register File (with rd_data)
    register_file reg_file(
        .clk(clk),
        .rs_addr(rs),
        .rt_addr(rt),
        .rd_addr(rd),
        .write_data(write_data),  // Use muxed write data
        .write_enable(reg_write_enable),
        .rs_data(rs_data),
        .rt_data(rt_data),
        .rd_data(rd_data)  // Added
    );

    // ALU Operand Mux (unchanged)
    assign operand_b = is_immediate ? {{52{literal[11]}}, literal} : rt_data;

    // ALU/FPU (unchanged)
    alu_fpu alu_unit(
        .a(rs_data),
        .b(operand_b),
        .op(alu_op),
        .is_float(is_float),
        .result(alu_result)
    );

    // Memory Handler (sign-extend literal)
    memoryHandler mem_handler(
        .opcode(opcode),
        .rd(rd_data),  // Use rd_data for consistency (though may not be needed)
        .rs(rs_data),
        .rt(rt_data),
        .lit({{52{literal[11]}}, literal}),  // Sign-extend literal
        .pc(pc),
        .r31(reg_file.registers[31]),
        .mem_addr(mem_addr),
        .mem_wr_data(mem_wr_data),
        .mem_wr_en(mem_wr_en),
        .mem_reg_write(mem_reg_write)
    );

    // Register Write Data Mux
    assign write_data = is_load ? mem_data_out : alu_result;

endmodule