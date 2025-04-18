//############################################################################
  //## tinker_core (Top Level Module)
  //## ADDED: Forwarding Logic
  //## CHANGED: always_comb replaced with always @(*)
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
      logic [63:0] regfile_operand_a_de; // rs_data from RegFile
      logic [63:0] regfile_operand_b_de; // rt_data from RegFile (before lit mux)
      logic [63:0] regfile_operand_c_de; // rd_data from RegFile
      logic [63:0] operand_a_de;         // rs_data after forwarding mux
      logic [63:0] operand_b_de;         // rt_data or literal (after lit mux)
      logic [63:0] operand_b_reg_de;     // rt_data after forwarding mux (before lit mux)
      logic [63:0] operand_c_de;         // rd_data after forwarding mux
      logic [63:0] literal_de;
      logic [4:0]  rd_addr_de;
      logic [4:0]  rs_addr_de;
      logic [4:0]  rt_addr_de;
      logic [4:0]  opcode_de;
      logic [63:0] stack_ptr_de;
      logic        alu_enable_de, mem_read_de, mem_write_de, reg_write_de, mem_to_reg_de, branch_taken_ctrl_de, mem_pc_de; // Control signals

      // DE/EX Register Outputs -> Inputs to EX Stage
      logic [63:0] pc_ex;
      logic [63:0] operand_a_ex; // Pipelined rs_data (or rd_data for I-type) after forwarding
      logic [63:0] operand_b_ex; // Pipelined rt_data or literal after forwarding
      logic [63:0] operand_c_ex; // Pipelined rd_data after forwarding
      logic [63:0] literal_ex;
      logic [4:0]  rd_addr_ex; // Pipelined write address (destination)
      logic [4:0]  rs_addr_ex; // Pipelined rs address
      logic [4:0]  rt_addr_ex; // Pipelined rt address
      logic [4:0]  opcode_ex;
      logic [63:0] stack_ptr_ex;
      logic        alu_enable_ex, mem_read_ex, mem_write_ex, reg_write_ex, mem_to_reg_ex, mem_pc_ex; // Control signals

      // EX Stage Outputs -> Inputs to EX/MEM Register
      logic [63:0] alu_result_ex;
      logic [63:0] alu_mem_addr_ex;
      logic [63:0] alu_mem_data_ex;
      logic [63:0] alu_branch_pc_ex;
      logic        branch_taken_ex;
      logic        hlt_ex;

      // EX/MEM Register Outputs -> Inputs to MEM Stage
      logic [63:0] alu_result_mem;
      logic [63:0] mem_addr_mem;
      logic [63:0] mem_wdata_mem;
      logic [63:0] branch_pc_mem;
      logic [4:0]  rd_addr_mem; // Pipelined write address (destination)
      logic        mem_read_mem, mem_write_mem, reg_write_mem, mem_to_reg_mem, branch_taken_mem, mem_pc_mem; // Control
      logic        hlt_mem;

      // MEM Stage Outputs
      logic [63:0] mem_rdata_mem;
      logic [63:0] return_pc_mem;

      // MEM/WB Stage (Data to be written back)
      logic [63:0] reg_wdata_wb;

      // --- Control Signals ---
      logic flush_de, flush_ex;
      logic take_return_pc_fetch;

      // --- Forwarding Signals ---
      logic [1:0] forward_a_sel_de;
      logic [1:0] forward_b_sel_de;
      logic [1:0] forward_c_sel_de;

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

      memory memory ( // Renamed instance for clarity
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
          .flush(flush_de), // Flush signal comes from branch resolution in MEM
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
          .branch_taken(branch_taken_ctrl_de), // Branch type signal
          .mem_pc(mem_pc_de)
      );

      registerFile reg_file (
          .clk(clk),
          .reset(reset),
          // Write Port (from WB stage)
          .write_addr(rd_addr_mem),       // Write address comes from MEM stage register
          .write_data(reg_wdata_wb),      // Write data comes from WB mux
          .write_enable(reg_write_mem),   // Write enable comes from MEM stage register
          // Read Port 1 (rs) -> Goes to Forwarding Mux A
          .read_addr1(rs_addr_de),
          .read_data1(regfile_operand_a_de),
          // Read Port 2 (rt) -> Goes to Forwarding Mux B
          .read_addr2(rt_addr_de),
          .read_data2(regfile_operand_b_de),
          // Read Port 3 (rd) -> Goes to Forwarding Mux C
          .read_addr3(rd_addr_de),
          .read_data3(regfile_operand_c_de),
          // Stack Pointer Output (Needed in DE)
          .stack_ptr_out(stack_ptr_de)
      );

      // --- ADDED: Forwarding Unit ---
      forwarding_unit fwd_unit (
          .rs_addr_de(rs_addr_de),
          .rt_addr_de(rt_addr_de),
          .rd_addr_de(rd_addr_de), // Need rd read address for forwarding
          .rd_addr_ex(rd_addr_ex),
          .reg_write_ex(reg_write_ex),
          .rd_addr_mem(rd_addr_mem),
          .reg_write_mem(reg_write_mem),
          .forward_a_sel_de(forward_a_sel_de),
          .forward_b_sel_de(forward_b_sel_de),
          .forward_c_sel_de(forward_c_sel_de)
       );

      // --- ADDED: Forwarding Muxes for Operands A, B, C ---
      forwarding_mux forward_mux_a (
          .regfile_data(regfile_operand_a_de),
          .ex_fwd_data(alu_result_ex),       // Data forwarded from EX stage result
          .mem_fwd_data(reg_wdata_wb),      // Data forwarded from MEM stage (result or memory data)
          .sel(forward_a_sel_de),
          .forwarded_data(operand_a_de)     // Output to DE/EX register input A
      );

      forwarding_mux forward_mux_b (
          .regfile_data(regfile_operand_b_de), // rt data from register file
          .ex_fwd_data(alu_result_ex),
          .mem_fwd_data(reg_wdata_wb),
          .sel(forward_b_sel_de),
          .forwarded_data(operand_b_reg_de) // Output is rt data *before* literal mux
      );

       forwarding_mux forward_mux_c (
          .regfile_data(regfile_operand_c_de), // rd data from register file
          .ex_fwd_data(alu_result_ex),
          .mem_fwd_data(reg_wdata_wb),
          .sel(forward_c_sel_de),
          .forwarded_data(operand_c_de)      // Output to DE/EX register input C
      );

      // Original reg/lit mux for operand B
      reglitmux input_selector (
          .sel(opcode_de),
          .reg_in(operand_b_reg_de), // <<<<<< CHANGED: Input now comes from forwarding mux B output
          .lit_in(literal_de),
          .out(operand_b_de)        // Output to DE/EX register input B
      );

      de_ex_register de_ex_reg (
          .clk(clk),
          .flush(flush_ex), // Flush signal comes from branch resolution in MEM
          // Inputs from Decode Stage (Now potentially forwarded)
          .pc_in(pc_de),
          .operand_a_in(operand_a_de), // <<<<<< CHANGED: Now comes from Forwarding Mux A
          .operand_b_in(operand_b_de), // <<<<<< UNCHANGED: Still comes from Reg/Lit Mux
          .operand_c_in(operand_c_de), // <<<<<< CHANGED: Now comes from Forwarding Mux C
          .literal_in(literal_de),
          .rd_addr_in(rd_addr_de),     // Destination address
          .rs_addr_in(rs_addr_de),     // Source address 1
          .rt_addr_in(rt_addr_de),     // Source address 2
          .opcode_in(opcode_de),
          .stack_ptr_in(stack_ptr_de), // Stack pointer read in DE
          // Control Signals
          .alu_enable_in(alu_enable_de),
          .mem_read_in(mem_read_de),
          .mem_write_in(mem_write_de),
          .reg_write_in(reg_write_de),   // Need reg_write signal for forwarding unit
          .mem_to_reg_in(mem_to_reg_de),
          .branch_taken_ctrl_in(branch_taken_ctrl_de), // Branch type signal
          .mem_pc_in(mem_pc_de),
          // Outputs to Execute Stage
          .pc_out(pc_ex),
          .operand_a_out(operand_a_ex),
          .operand_b_out(operand_b_ex),
          .operand_c_out(operand_c_ex), // Pipelined rd_data
          .literal_out(literal_ex),
          .rd_addr_out(rd_addr_ex),     // Pipelined destination address
          .rs_addr_out(rs_addr_ex),
          .rt_addr_out(rt_addr_ex),
          .opcode_out(opcode_ex),
          .stack_ptr_out(stack_ptr_ex),
          // Pipelined Control Signals
          .alu_enable_out(alu_enable_ex),
          .mem_read_out(mem_read_ex),
          .mem_write_out(mem_write_ex),
          .reg_write_out(reg_write_ex), // Pipelined reg_write for forwarding
          .mem_to_reg_out(mem_to_reg_ex),
          .branch_taken_ctrl_out(),     // This output seems unused now
          .mem_pc_out(mem_pc_ex)
      );

      alu calculation_unit (
          .alu_enable(alu_enable_ex),
          .opcode(opcode_ex),
          // Data Inputs from DE/EX (Potentially forwarded values latched)
          .input1(operand_a_ex), // Pipelined rs_data (or rd_data for I-types)
          .input2(operand_b_ex), // Pipelined rt_data or literal
          .input3(operand_c_ex), // Pipelined rd_data
          .rd_addr(rd_addr_ex),  // Pipelined destination register address
          .literal(literal_ex),
          .pc_in(pc_ex),
          .stack_ptr(stack_ptr_ex), // Pipelined stack pointer
          // Outputs to EX/MEM Register
          .result(alu_result_ex),   // Result needed for EX->DE forwarding
          .mem_addr(alu_mem_addr_ex),
          .mem_wdata(alu_mem_data_ex),
          .branch_pc(alu_branch_pc_ex),
          .branch_taken(branch_taken_ex), // Branch decision made here
          .hlt_out(hlt_ex),
          // Control Inputs Pass-through
          .mem_read_in(mem_read_ex),
          .mem_write_in(mem_write_ex),
          .reg_write_in(reg_write_ex),
          .mem_to_reg_in(mem_to_reg_ex),
          .mem_pc_in(mem_pc_ex)
      );

      ex_mem_register ex_mem_reg (
          .clk(clk),
          .flush_mem(flush_ex), // Flush signal from branch resolution in MEM
          // Inputs from Execute Stage
          .result_in(alu_result_ex),
          .mem_addr_in(alu_mem_addr_ex),
          .mem_wdata_in(alu_mem_data_ex),
          .branch_pc_in(alu_branch_pc_ex),
          .rd_addr_in(rd_addr_ex), // Pass destination addr
          .hlt_in(hlt_ex),
          // Control Inputs (from DE/EX outputs)
          .mem_read_in(mem_read_ex),
          .mem_write_in(mem_write_ex),
          .reg_write_in(reg_write_ex), // Need reg_write signal for forwarding unit
          .mem_to_reg_in(mem_to_reg_ex),
          .branch_taken_in(branch_taken_ex), // Decision from ALU
          .mem_pc_in(mem_pc_ex),
          // Outputs to Memory/Writeback Stage
          .result_out(alu_result_mem), // ALU result possibly needed for WB
          .mem_addr_out(mem_addr_mem),
          .mem_wdata_out(mem_wdata_mem),
          .branch_pc_out(branch_pc_mem), // Branch PC needed for Fetch
          .rd_addr_out(rd_addr_mem), // Write address needed for RegFile and Forwarding
          .hlt_out(hlt_mem),
          .mem_read_out(mem_read_mem),
          .mem_write_out(mem_write_mem),
          .reg_write_out(reg_write_mem), // Write enable needed for RegFile and Forwarding
          .mem_to_reg_out(mem_to_reg_mem), // Mux select needed for WB
          .branch_taken_out(branch_taken_mem), // Branch decision needed for Fetch and Flush
          .mem_pc_out(mem_pc_mem) // Return control needed for Fetch
      );

      // Mux for return address (PC + 4 or Memory Data)
      aluMemMux return_pc_selector (
          .mem_pc(mem_pc_mem),
          .memData(mem_rdata_mem),
          .aluOut(branch_pc_mem),  // Note: For RETURN, this isn't used, but branch_taken_mem is set
          .newPc(return_pc_mem)
      );

      // Mux for Writeback data (ALU Result or Memory Data)
      memRegMux data_source_selector (
          .mem_to_reg(mem_to_reg_mem),
          .readData(mem_rdata_mem),
          .aluResult(alu_result_mem),
          .regWriteData(reg_wdata_wb) // Output to RegFile Write Port & MEM->DE Forwarding Path
      );

      // --- Control Logic ---
      // Flush IF/DE and DE/EX stages if a branch is taken in MEM stage
      assign flush_de = branch_taken_mem;
      assign flush_ex = branch_taken_mem;
      // Control signal for Fetch stage PC selection
      assign take_return_pc_fetch = mem_pc_mem; // High for RETURN instruction
      // Halt signal output
      assign hlt = hlt_mem;

  endmodule

  //############################################################################
  //## registerFile (No changes)
  //############################################################################
  module registerFile (
      input clk,
      input reset,
      // Write Port
      input [4:0] write_addr,
      input [63:0] write_data,
      input write_enable,
      // Read Port 1 (rs)
      input [4:0] read_addr1,
      output logic [63:0] read_data1,
      // Read Port 2 (rt)
      input [4:0] read_addr2,
      output logic [63:0] read_data2,
      // Read Port 3 (rd)
      input [4:0] read_addr3,
      output logic [63:0] read_data3,
      // Stack Pointer Output
      output logic [63:0] stack_ptr_out
  );
      reg [63:0] registers [0:31];
      integer idx;

      initial begin
          for (idx = 0; idx < 31; idx = idx + 1) begin
              registers[idx] = 64'b0;
          end
          registers[31] = 64'h0008_0000; // Example Stack Pointer Init
      end

      // Combinational Read Port 1 (rs)
      assign read_data1 = (read_addr1 == 5'd31) ? registers[31] : registers[read_addr1];

      // Combinational Read Port 2 (rt)
      assign read_data2 = (read_addr2 == 5'd31) ? registers[31] : registers[read_addr2];

      // Combinational Read Port 3 (rd)
      assign read_data3 = (read_addr3 == 5'd31) ? registers[31] : registers[read_addr3];

      // Stack Pointer Output
      assign stack_ptr_out = registers[31];

      // Synchronous Write Port
      always @(posedge clk) begin
          if (!reset && write_enable) begin // R0 is writable
              registers[write_addr] <= write_data;
          end
      end

  endmodule

  //############################################################################
  //## fetch (No changes)
  //############################################################################
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

  //############################################################################
  //## if_de_register (No changes)
  //############################################################################
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
              pc_out <= 64'b0; // Flush PC
              instruction_out <= 32'b0; // Flush instruction to NOP (opcode 0)
          end else begin
              pc_out <= pc_in;
              instruction_out <= instruction_in;
          end
      end
  endmodule

  //############################################################################
  //## instructionDecoder (Uses always @(*), no change needed here)
  //############################################################################
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
      output reg branch_taken, // Indicates branch *type*
      output reg mem_pc
  );
      // Opcodes
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
        // Sign-extend the 12-bit literal
          literal = {{52{instructionLine[11]}}, instructionLine[11:0]}; // Sign extension for literals

          // Default control signals (inactive)
          alu_enable = 1'b0; mem_read = 1'b0; mem_write = 1'b0; reg_write = 1'b0;
          mem_to_reg = 1'b0; branch_taken = 1'b0; mem_pc = 1'b0;

          case (opcode)
              // R-Type Arithmetic/Logic
              ADD, SUB, MUL, DIV, AND, OR, XOR, NOT, SHFTR, SHFTL, ADDF, SUBF, MULF, DIVF: begin
                  alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0;
              end
              // I-Type Arithmetic/Logic
              ADDI, SUBI, SHFTRI, SHFTLI: begin
                  alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0;
              end
              // Load Word (I-Type format, but uses rs)
              MOV_MEM: begin // rd <- Mem[rs + L]
                  alu_enable=1'b1; // ALU calculates address
                  mem_read=1'b1; reg_write=1'b1; mem_to_reg=1'b1; // Data comes from memory
             end
              // Store Word (Uses rd as base, rs as data, like S-type)
              MOV_STR: begin // Mem[rd + L] <- rs
                  alu_enable=1'b1; // ALU calculates address
                  mem_write=1'b1; reg_write=1'b0; // No register write
              end
              // Register Move (R-Type format)
              MOV_REG: begin // rd <- rs
                  alu_enable=1'b1; // ALU just passes rs
                  reg_write=1'b1; mem_to_reg=1'b0; // Data comes from ALU (pass-through)
              end
              // Load Immediate (I-Type format, uses rd)
              MOV_LIT: begin // rd[11:0] <- L (actually modified in ALU)
                  alu_enable=1'b1; reg_write=1'b1; mem_to_reg=1'b0; // Data comes from ALU
              end
              // Unconditional Branches (rd holds target/offset)
              BR, BRR: begin // pc <- rd OR pc <- pc + rd
                  alu_enable=1'b1; // ALU calculates target PC
                  reg_write=1'b0; branch_taken=1'b1; // It's a branch type
              end
              // Branch Relative Immediate (I-Type format)
              BRRI: begin // pc <- pc + L
                  alu_enable=1'b1; // ALU calculates target PC
                  reg_write=1'b0; branch_taken=1'b1; // It's a branch type
              end
              // Conditional Branches (rd holds target, use rs/rt for condition)
              BRNZ, BRGT: begin // pc <- rd if cond(rs, rt)
                  alu_enable=1'b1; // ALU calculates target PC and checks condition
                  reg_write=1'b0; branch_taken=1'b1; // It's a branch type (ALU decides if *actually* taken)
              end
              // Call (rd holds target, pushes PC+4 to stack)
              CALL: begin // Mem[sp-8] <- pc+4; pc <- rd
                  alu_enable=1'b1; // ALU calculates target PC and stack address
                  mem_write=1'b1; // Writes return address to stack
                  reg_write=1'b0; branch_taken=1'b1; // It's a branch type
              end
              // Return (reads PC from stack)
              RETURN: begin // pc <- Mem[sp-8]
                  alu_enable=1'b1; // ALU calculates stack address
                  mem_read=1'b1; // Reads return address from stack
                  reg_write=1'b0; mem_pc=1'b1; // PC comes from Memory result
                  branch_taken=1'b1; // Treat as taken for flushing previous stages
              end
              // Privileged (Halt)
              PRIV: begin
                  if(literal[11:0]==12'h0) begin // Only check HALT (L=0)
                      alu_enable=1'b1; // ALU needs to signal halt
                      reg_write=1'b0;
                  end else begin
                      // Treat other PRIV codes as NOPs for now
                      alu_enable=1'b0; reg_write=1'b0;
                  end
              end
              default: begin // Treat undefined opcodes as NOPs
                  alu_enable=1'b0; reg_write=1'b0;
              end
          endcase

          // Remap rs source for I-type instructions using rd field for source/dest
        // This simplifies ALU input muxing, ALU uses input1 as first operand always.
        // Note: This assumes I-types read/write the same register specified in 'rd'.
          case (opcode)
             ADDI, SUBI, SHFTRI, SHFTLI, MOV_LIT: rs = rd; // Use rd field as the source register
             default: ; // Keep original rs for R-types, MOV_MEM, MOV_STR, Branches etc.
          endcase
      end
  endmodule

  //############################################################################
  //## de_ex_register (No functional changes, just wiring)
  //############################################################################
  module de_ex_register (
      input clk,
      input flush,
      // Inputs from Decode Stage / Forwarding Muxes
      input [63:0] pc_in,
      input [63:0] operand_a_in,  // Data from Fwd Mux A
      input [63:0] operand_b_in,  // Data from Reg/Lit Mux
      input [63:0] operand_c_in,  // Data from Fwd Mux C
      input [63:0] literal_in,    // Original literal from decode
      input [4:0] rd_addr_in,     // Destination register address
      input [4:0] rs_addr_in,     // Source register 1 address
      input [4:0] rt_addr_in,     // Source register 2 address
      input [4:0] opcode_in,
      input [63:0] stack_ptr_in,  // Stack pointer value read in Decode
      // Control Signals from Decode
      input alu_enable_in,
      input mem_read_in,
      input mem_write_in,
      input reg_write_in,        // Passed through for forwarding unit
      input mem_to_reg_in,
      input branch_taken_ctrl_in,// Passed through (unused?)
      input mem_pc_in,
      // Outputs to Execute Stage
      output reg [63:0] pc_out,
      output reg [63:0] operand_a_out,
      output reg [63:0] operand_b_out,
      output reg [63:0] operand_c_out, // Pipelined rd_data
      output reg [63:0] literal_out,
      output reg [4:0] rd_addr_out,   // Pipelined destination address (for forwarding)
      output reg [4:0] rs_addr_out,
      output reg [4:0] rt_addr_out,
      output reg [4:0] opcode_out,
      output reg [63:0] stack_ptr_out, // Pipelined stack pointer
      // Pipelined Control Signals
      output reg alu_enable_out,
      output reg mem_read_out,
      output reg mem_write_out,
      output reg reg_write_out,      // Pipelined write enable (for forwarding)
      output reg mem_to_reg_out,
      output reg branch_taken_ctrl_out, // Unused output now
      output reg mem_pc_out
  );
      always @(posedge clk) begin
          if (flush) begin
            // Flush: Insert NOP (represented by all zeros)
              pc_out <= 64'b0; operand_a_out <= 64'b0; operand_b_out <= 64'b0; operand_c_out <= 64'b0;
              literal_out <= 64'b0; rd_addr_out <= 5'b0; rs_addr_out <= 5'b0; rt_addr_out <= 5'b0;
              opcode_out <= 5'b0; // NOP Opcode
            stack_ptr_out <= 64'b0; // Flush stack pointer value too
            // Deassert all control signals
              alu_enable_out <= 1'b0; mem_read_out <= 1'b0;
              mem_write_out <= 1'b0; reg_write_out <= 1'b0; mem_to_reg_out <= 1'b0;
              branch_taken_ctrl_out <= 1'b0; mem_pc_out <= 1'b0;
          end else begin
            // Normal operation: Latch all inputs to outputs
              pc_out <= pc_in; operand_a_out <= operand_a_in; operand_b_out <= operand_b_in; operand_c_out <= operand_c_in;
              literal_out <= literal_in; rd_addr_out <= rd_addr_in; rs_addr_out <= rs_addr_in; rt_addr_out <= rt_addr_in;
              opcode_out <= opcode_in; stack_ptr_out <= stack_ptr_in; alu_enable_out <= alu_enable_in;
              mem_read_out <= mem_read_in; mem_write_out <= mem_write_in; reg_write_out <= reg_write_in;
              mem_to_reg_out <= mem_to_reg_in; branch_taken_ctrl_out <= branch_taken_ctrl_in; mem_pc_out <= mem_pc_in;
          end
      end
  endmodule

  //############################################################################
  //## alu (No changes needed for forwarding itself)
  //## CHANGED: always_comb replaced with always @(*)
  //############################################################################
  module alu (
      // Control Inputs
      input logic alu_enable,
      input logic [4:0] opcode,
      // Data Inputs from DE/EX Register
      input logic [63:0] input1,    // Pipelined rs_data (or rd_data for I-types, post-forwarding)
      input logic [63:0] input2,    // Pipelined rt_data or literal (post-forwarding/muxing)
      input logic [63:0] input3,    // Pipelined rd_data (post-forwarding)
      input logic [4:0] rd_addr,    // Pipelined destination register address
      input logic [63:0] literal,   // Pipelined literal value
      input logic [63:0] pc_in,     // Pipelined PC value
      input logic [63:0] stack_ptr, // Pipelined R31 value
      // Outputs to EX/MEM Register
      output logic [63:0] result,     // Main ALU result (forwarded from EX)
      output logic [63:0] mem_addr,   // Address for memory stage
      output logic [63:0] mem_wdata,  // Data to write to memory
      output logic [63:0] branch_pc,  // Calculated branch target
      output logic branch_taken,      // Branch condition outcome
      output logic hlt_out,
      // Control Signal Inputs (Passed through DE/EX)
      input logic mem_read_in,
      input logic mem_write_in,
      input logic reg_write_in,
      input logic mem_to_reg_in,
      input logic mem_pc_in
  );

      // Opcodes (localparams for readability)
      localparam AND = 5'h00, OR = 5'h01, XOR = 5'h02, NOT = 5'h03, SHFTR = 5'h04, SHFTRI = 5'h05,
                 SHFTL = 5'h06, SHFTLI = 5'h07, BR = 5'h08, BRR = 5'h09, BRRI = 5'h0A, BRNZ = 5'h0B,
                 CALL = 5'h0C, RETURN = 5'h0D, BRGT = 5'h0E, PRIV = 5'h0F, MOV_MEM = 5'h10, MOV_REG = 5'h11,
                 MOV_LIT = 5'h12, MOV_STR = 5'h13, ADDF = 5'h14, SUBF = 5'h15, MULF = 5'h16, DIVF = 5'h17,
                 ADD = 5'h18, ADDI = 5'h19, SUB = 5'h1A, SUBI = 5'h1B, MUL = 5'h1C, DIV = 5'h1D;

      // NOTE: FP Operations are placeholders
      logic [63:0] fp_result; // For simulation only - synthesis tools will likely optimize away

      always @(*) begin // Combinational ALU logic - Replaced always_comb
          // Default Output Values
          result = 64'b0;
          mem_addr = 64'b0;
          mem_wdata = 64'b0;
          branch_pc = pc_in + 4; // Default next PC (only relevant for branches)
          branch_taken = 1'b0; // Default: branch not taken
          hlt_out = 1'b0;

          if (alu_enable) begin // Only perform calculation if enabled by decoder
              case (opcode)
                  // Integer Arithmetic (Signed)
                // Assumes input1 = rs/rd, input2 = rt/literal
                  ADD, ADDI: result = $signed(input1) + $signed(input2);
                  SUB, SUBI: result = $signed(input1) - $signed(input2);
                  MUL: result = $signed(input1) * $signed(input2);
                  DIV: if (input2 != 0) result = $signed(input1) / $signed(input2); else result = 64'b0; // Basic div by zero check

                  // Logical
                  AND: result = input1 & input2;
                  OR:  result = input1 | input2;
                  XOR: result = input1 ^ input2;
                  NOT: result = ~input1; // Uses only input1

                  // Shift (using lower 6 bits of input2 as shift amount)
                  SHFTR, SHFTRI: result = input1 >> input2[5:0];
                  SHFTL, SHFTLI: result = input1 << input2[5:0];

                  // Data Movement - Address Calculation / Value Passing
                // MOV_MEM: rd <- Mem[rs + L]. ALU calculates address rs + L
                  MOV_MEM: mem_addr = input1 + $signed(input2); // input1=rs, input2=literal(sign-extended)
                // MOV_REG: rd <- rs. ALU passes rs through
                  MOV_REG: result = input1; // input1=rs
                // MOV_LIT: rd[11:0] <- L. ALU calculates the value {rd[63:12], L[11:0]}
                  MOV_LIT: result = {input1[63:12], input2[11:0]}; // input1=rd, input2=literal
                // MOV_STR: Mem[rd + L] <- rs. ALU calculates address rd + L, passes rs data
                  MOV_STR: begin
                    mem_addr = input3 + $signed(literal); // Use input3 (original rd) for address base, use pipelined literal
                    mem_wdata = input1;                 // Use input1 (original rs) for data
                 end

                  // Floating Point (Simulation Placeholders)
                  ADDF: fp_result = $realtobits($bitstoreal(input1) + $bitstoreal(input2)); result = fp_result;
                SUBF: fp_result = $realtobits($bitstoreal(input1) - $bitstoreal(input2)); result = fp_result;
                MULF: fp_result = $realtobits($bitstoreal(input1) * $bitstoreal(input2)); result = fp_result;
                DIVF: if ($bitstoreal(input2) != 0.0) fp_result = $realtobits($bitstoreal(input1) / $bitstoreal(input2)); else fp_result = 64'b0; result = fp_result;

                  // Control Flow - Branches
                // BR: pc <- rd. Target address is in rd (input3)
                  BR: begin branch_pc = input3; branch_taken = 1'b1; end
                // BRR: pc <- pc + rd. Target is relative to current PC, offset in rd (input3)
                // Note: pc_in is the PC of the BRR instruction itself.
                  BRR: begin branch_pc = pc_in + $signed(input3); branch_taken = 1'b1; end
                // BRRI: pc <- pc + L. Target relative to current PC, offset is literal (input2)
                  BRRI: begin branch_pc = pc_in + $signed(input2); branch_taken = 1'b1; end
                // BRNZ: if (rs != 0) pc <- rd. Condition on rs (input1), target in rd (input3)
                // Need signed comparison for condition check based on typical ISA conventions
                  BRNZ: begin if ($signed(input1) != 0) begin branch_pc = input3; branch_taken = 1'b1; end else branch_taken = 1'b0; end
                // BRGT: if (rs > rt) pc <- rd. Signed comparison rs (input1) > rt (input2), target in rd (input3)
                  BRGT: begin if ($signed(input1) > $signed(input2)) begin branch_pc = input3; branch_taken = 1'b1; end else branch_taken = 1'b0; end

                  // Control Flow - Subroutines
                // CALL: Mem[sp-8] <- pc+4; pc <- rd. Target in rd (input3). Stack ptr is from EX stage register.
                // Needs PC of *next* instruction for return address.
                  CALL: begin
                    branch_pc = input3;           // Target address from rd (input3)
                    mem_addr = stack_ptr - 8;     // Address to write return PC
                    mem_wdata = pc_in + 4;        // Return address is PC of next instruction
                    branch_taken = 1'b1;          // Always taken
                 end
                // RETURN: pc <- Mem[sp-8]. Calculate stack address. PC value comes from memory read.
                  RETURN: begin
                    mem_addr = stack_ptr - 8;     // Address to read return PC
                    // branch_pc is irrelevant (comes from memory), but set taken flag
                    branch_taken = 1'b1;         // Set taken to trigger flush and PC update mechanism
                end

                  // Privileged - Halt (Only L=0 implemented)
                // PRIV (HALT): L=0. Set halt signal.
                  PRIV: if(literal[11:0] == 12'h0) hlt_out = 1'b1; // Check the original literal passed through pipeline

                  default: ; // Undefined opcodes - default outputs (effectively NOP)
              endcase
          end // if (alu_enable)
      end // always @(*)
  endmodule

  //############################################################################
  //## ex_mem_register (No functional changes, just wiring/comments)
  //############################################################################
  module ex_mem_register (
      input clk,
      // Flush signal to cancel instruction in this stage
      input logic flush_mem,
      // Inputs from Execute Stage
      input logic [63:0] result_in,     // ALU result
      input logic [63:0] mem_addr_in,   // Memory address (from ALU)
      input logic [63:0] mem_wdata_in,  // Memory write data (from ALU, e.g., for CALL, MOV_STR)
      input logic [63:0] branch_pc_in,   // Calculated branch target PC
      input logic [4:0] rd_addr_in,     // Destination register address (passed through)
      input logic hlt_in,               // Halt signal (from ALU)
      // Control Signals (passed through from DE/EX)
      input logic mem_read_in,
      input logic mem_write_in,
      input logic reg_write_in,        // Passed through for forwarding
      input logic mem_to_reg_in,
      input logic branch_taken_in,     // Branch decision from ALU
      input logic mem_pc_in,           // Control for RETURN PC source
      // Outputs to Memory/Writeback Stage
      output logic [63:0] result_out,    // Pipelined ALU result (for WB mux)
      output logic [63:0] mem_addr_out,  // Pipelined memory address
      output logic [63:0] mem_wdata_out, // Pipelined memory write data
      output logic [63:0] branch_pc_out, // Pipelined branch target PC (for fetch mux)
      output logic [4:0] rd_addr_out,   // Pipelined destination address (for WB and forwarding)
      output logic hlt_out,             // Pipelined halt signal
      // Pipelined Control Signals
      output logic mem_read_out,
      output logic mem_write_out,
      output logic reg_write_out,      // Pipelined write enable (for WB and forwarding)
      output logic mem_to_reg_out,     // Pipelined WB mux select
      output logic branch_taken_out,   // Pipelined branch decision (for flush/fetch)
      output logic mem_pc_out          // Pipelined RETURN PC source control
  );
      always @(posedge clk) begin
          // If flushing due to taken branch ahead, cancel control signals
          if (flush_mem) begin
            // Flush: Insert NOP-like state (control signals off)
              result_out <= 64'b0; // Clear data path for cleanliness maybe?
              mem_addr_out <= 64'b0;
              mem_wdata_out <= 64'b0;
              branch_pc_out <= 64'b0;
              rd_addr_out <= 5'b0; // Prevent matching for forwarding
              hlt_out <= 1'b0;
              // *** Crucially, force control signals low ***
              mem_read_out <= 1'b0;
              mem_write_out <= 1'b0;
              reg_write_out <= 1'b0; // Prevent register write and forwarding match
              mem_to_reg_out <= 1'b0; // Prevent incorrect WB mux selection
              branch_taken_out <= 1'b0; // This instruction is flushed, no longer affects fetch/flush
              mem_pc_out <= 1'b0;       // This instruction is flushed
          end else begin
              // Normal operation: Latch inputs to outputs
              result_out <= result_in;
              mem_addr_out <= mem_addr_in;
              mem_wdata_out <= mem_wdata_in;
              branch_pc_out <= branch_pc_in;
              rd_addr_out <= rd_addr_in;
              hlt_out <= hlt_in;
              mem_read_out <= mem_read_in;
              mem_write_out <= mem_write_in;
              reg_write_out <= reg_write_in;
              mem_to_reg_out <= mem_to_reg_in;
              branch_taken_out <= branch_taken_in; // Pass branch decision through
              mem_pc_out <= mem_pc_in;           // Pass mem_pc control through
          end
      end
  endmodule

  //############################################################################
  //## memory (No changes)
  //############################################################################
  module memory (
      input clk,
      input reset,
      // Instruction Fetch Port (Read Only)
      input [63:0] inst_addr,
      output logic [31:0] instruction_out,
      // Data Access Port (Read/Write)
      input [63:0] data_addr,
      input [63:0] data_wdata,
      input mem_read,   // Read enable signal from MEM stage
      input mem_write,  // Write enable signal from MEM stage
      output logic [63:0] data_rdata // Data read output to WB mux
  );
      // Increased size for stack potentially growing down from 0x80000
      // Adjusted to 1MB = 1048576 bytes for more space
      localparam MEM_SIZE_BYTES = 1048576;
      // Calculate address bits needed
      localparam ADDR_BITS = $clog2(MEM_SIZE_BYTES);

      // Memory array (byte addressable)
      reg [7:0] bytes [0:MEM_SIZE_BYTES-1];
      integer i;

      // Initialize memory to zero (optional, good for simulation)
      initial begin
          //$display("Initializing %0d bytes of memory...", MEM_SIZE_BYTES);
          for(i=0; i<MEM_SIZE_BYTES; i=i+1) begin
              bytes[i] = 8'b0;
          end
          //$display("Memory Initialization Complete.");
      end

      // Instruction Fetch (Combinational Read) - Assuming word aligned
      // Use only necessary address bits to index the memory array
      logic [ADDR_BITS-1:0] effective_inst_addr = inst_addr[ADDR_BITS-1:0];
      assign instruction_out[7:0]   = bytes[effective_inst_addr + 0];
      assign instruction_out[15:8]  = bytes[effective_inst_addr + 1];
      assign instruction_out[23:16] = bytes[effective_inst_addr + 2];
      assign instruction_out[31:24] = bytes[effective_inst_addr + 3];

      // Data Read (Combinational Read, enabled by mem_read)
      // Use only necessary address bits to index the memory array
      logic [ADDR_BITS-1:0] effective_data_addr = data_addr[ADDR_BITS-1:0];
      // Read logic simplified: reads happen combinationally, result used in next stage if mem_read was high
      assign data_rdata[7:0]    = bytes[effective_data_addr + 0];
      assign data_rdata[15:8]   = bytes[effective_data_addr + 1];
      assign data_rdata[23:16]  = bytes[effective_data_addr + 2];
      assign data_rdata[31:24]  = bytes[effective_data_addr + 3];
      assign data_rdata[39:32]  = bytes[effective_data_addr + 4];
      assign data_rdata[47:40]  = bytes[effective_data_addr + 5];
      assign data_rdata[55:48]  = bytes[effective_data_addr + 6];
      assign data_rdata[63:56]  = bytes[effective_data_addr + 7];

      // Data Write (Synchronous Write)
      always @(posedge clk) begin
          if (mem_write) begin // Only write if write enable is high
          // Use only necessary address bits to index the memory array
              bytes[effective_data_addr + 0] <= data_wdata[7:0];
              bytes[effective_data_addr + 1] <= data_wdata[15:8];
              bytes[effective_data_addr + 2] <= data_wdata[23:16];
              bytes[effective_data_addr + 3] <= data_wdata[31:24];
              bytes[effective_data_addr + 4] <= data_wdata[39:32];
              bytes[effective_data_addr + 5] <= data_wdata[47:40];
              bytes[effective_data_addr + 6] <= data_wdata[55:48];
              bytes[effective_data_addr + 7] <= data_wdata[63:56];
          end
      end
  endmodule

  //############################################################################
  //## aluMemMux (No changes)
  //## CHANGED: always_comb replaced with always @(*)
  //############################################################################
  module aluMemMux (
      input mem_pc,         // Control signal (high for RETURN)
      input [63:0] memData, // PC value read from memory (for RETURN)
      input [63:0] aluOut,  // Calculated branch target PC (for other branches)
      output reg [63:0] newPc // Selected PC for fetch stage
  );
      // Combinational Mux
      always @(*) begin // Replaced always_comb
          if (mem_pc) begin
            newPc = memData; // Select PC from memory for RETURN
          end else begin
            newPc = aluOut; // Select calculated PC for other branches
          end
      end
  endmodule

  //############################################################################
  //## reglitmux (No changes)
  //## CHANGED: always_comb replaced with always @(*)
  //############################################################################
  module reglitmux (
      input [4:0] sel,      // Opcode from decode stage
      input [63:0] reg_in,  // Register operand (rt, potentially forwarded)
      input [63:0] lit_in,  // Literal operand (sign-extended)
      output reg [63:0] out // Selected second operand for ALU (input2)
  );
      // Opcodes that use Literal as the second ALU operand
      localparam ADDI=5'h19, SUBI=5'h1B, SHFTRI=5'h05, SHFTLI=5'h07,
                 BRRI=5'h0A, MOV_MEM=5'h10, MOV_LIT=5'h12, MOV_STR=5'h13;

      // Combinational Mux
      always @(*) begin // Replaced always_comb
          case (sel)
              ADDI, SUBI, SHFTRI, SHFTLI, BRRI, MOV_MEM, MOV_LIT, MOV_STR:
                out = lit_in; // Select literal for these opcodes
              default:
                out = reg_in; // Select register operand (rt) for others
          endcase
      end
  endmodule

  //############################################################################
  //## memRegMux (No changes)
  //## CHANGED: always_comb replaced with always @(*)
  //############################################################################
  module memRegMux (
      input mem_to_reg,       // Control signal from MEM stage
      input [63:0] readData,  // Data read from memory in MEM stage
      input [63:0] aluResult, // Result from ALU in MEM stage
      output reg [63:0] regWriteData // Data selected for register file write port / forwarding
  );
      // Combinational Mux
      always @(*) begin // Replaced always_comb
          if (mem_to_reg) begin
            regWriteData = readData; // Select memory data (e.g., for MOV_MEM, RETURN)
          end else begin
            regWriteData = aluResult; // Select ALU result (for most other instructions)
          end
      end
  endmodule

  //############################################################################
  //## forwarding_mux (NEW MODULE)
  //## Selects between RegFile data, EX forwarded data, or MEM forwarded data
  //## CHANGED: always_comb replaced with always @(*)
  //############################################################################
  module forwarding_mux (
      input logic [63:0] regfile_data, // Data read directly from register file
      input logic [63:0] ex_fwd_data,  // Data forwarded from EX stage (ALU result)
      input logic [63:0] mem_fwd_data, // Data forwarded from MEM stage (WB data)
      input logic [1:0]  sel,          // Select signal from forwarding unit
      output logic [63:0] forwarded_data // Output data to DE/EX register input
  );

      // Combinational 3-to-1 Mux
      always @(*) begin // Replaced always_comb
          case (sel)
              2'b00: forwarded_data = regfile_data; // No forwarding
              2'b10: forwarded_data = ex_fwd_data;  // Forward from EX
              2'b01: forwarded_data = mem_fwd_data; // Forward from MEM/WB
              default: forwarded_data = regfile_data; // Default to no forwarding (should not happen)
          endcase
      end
  endmodule


  //############################################################################
  //## forwarding_unit (NEW MODULE)
  //## Determines necessary forwarding paths
  //## CHANGED: always_comb replaced with always @(*)
  //############################################################################
  module forwarding_unit (
      // Inputs: Register read addresses from DE stage
      input logic [4:0] rs_addr_de,
      input logic [4:0] rt_addr_de,
      input logic [4:0] rd_addr_de, // Need rd read address too (e.g., for MOV_STR base, BR target)

      // Inputs: Write address and enable from EX stage
      input logic [4:0] rd_addr_ex,
      input logic reg_write_ex,

      // Inputs: Write address and enable from MEM stage
      input logic [4:0] rd_addr_mem,
      input logic reg_write_mem,

      // Outputs: Select signals for the forwarding muxes in DE stage
      output logic [1:0] forward_a_sel_de, // For operand A (rs)
      output logic [1:0] forward_b_sel_de, // For operand B (rt)
      output logic [1:0] forward_c_sel_de  // For operand C (rd)
  );

      // Combinational logic to determine forwarding paths
      // Priority: Forward from EX if available, otherwise check MEM
      // Don't forward if write address is R0 (although writes to R0 might be allowed based on spec, forwarding R0 is pointless)

      always @(*) begin // Replaced always_comb
          // --- Forwarding for Operand A (rs) ---
          if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rs_addr_de)) begin
              forward_a_sel_de = 2'b10; // Forward from EX stage (ALU result)
          end else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rs_addr_de)) begin
               // Only forward from MEM if not already forwarding from EX for the same register
               // (This check is implicitly handled by the 'if/else if' structure)
              forward_a_sel_de = 2'b01; // Forward from MEM stage (Writeback data)
          end else begin
              forward_a_sel_de = 2'b00; // No forwarding needed for rs
          end

          // --- Forwarding for Operand B (rt) ---
          if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rt_addr_de)) begin
              forward_b_sel_de = 2'b10; // Forward from EX stage
          end else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rt_addr_de)) begin
              forward_b_sel_de = 2'b01; // Forward from MEM stage
          end else begin
              forward_b_sel_de = 2'b00; // No forwarding needed for rt
          end

          // --- Forwarding for Operand C (rd read port) ---
          // This is needed for instructions reading rd, like MOV_STR base, BR target, etc.
          if (reg_write_ex && (rd_addr_ex != 5'b0) && (rd_addr_ex == rd_addr_de)) begin
              forward_c_sel_de = 2'b10; // Forward from EX stage
          end else if (reg_write_mem && (rd_addr_mem != 5'b0) && (rd_addr_mem == rd_addr_de)) begin
              forward_c_sel_de = 2'b01; // Forward from MEM stage
          end else begin
              forward_c_sel_de = 2'b00; // No forwarding needed for rd read
          end
      end

      // NOTE: This forwarding logic does NOT handle the load-use hazard.
      // If an instruction immediately follows a MOV_MEM (load) and tries to use
      // the loaded value, this forwarding logic alone is insufficient.
      // A stall (pipeline bubble) would typically be required in the DE stage
      // when a load is detected in EX and its destination register matches
      // a source register in DE. This hazard detection logic is NOT included here
      // as per the request to only add forwarding.

  endmodule