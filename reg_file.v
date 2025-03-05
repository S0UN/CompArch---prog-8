// reg_file.v
// ---------------------------------------------------------
// Combinational Register File Module
//  - Contains 32 registers (64 bits wide).
//  - Three asynchronous read ports (for rs, rt, and rd).
//  - A combinational “write” is done whenever write_en is high.
//    (This style is for simulation only; real registers need a clock.)
// ---------------------------------------------------------
module reg_file(
    input  [4:0]  read_addr1,  // Address for reading (rs).
    input  [4:0]  read_addr2,  // Address for reading (rt).
    input  [4:0]  read_addr3,  // Address for reading (rd) for immediate operations.
    input  [4:0]  write_addr,  // Address to write to.
    input  [63:0] write_data,  // Data to write.
    input         write_en,    // Write enable signal.
    output [63:0] read_data1,  // Data read from register at read_addr1.
    output [63:0] read_data2,  // Data read from register at read_addr2.
    output [63:0] read_data3   // Data read from register at read_addr3.
);
    reg [63:0] regs[31:0];     // 32 registers, each 64 bits wide.
    integer i;
    
    // Initialize registers.
    initial begin
        for (i = 0; i < 32; i = i + 1)
            regs[i] = 64'd0;
    end

    // Asynchronous reads: Immediately drive outputs based on the register array.
    assign read_data1 = regs[read_addr1];
    assign read_data2 = regs[read_addr2];
    assign read_data3 = regs[read_addr3];
    
    // Combinational write: When write_en is high, update the register.
    // (Be aware that this is a simplification; a real design would use a clock.)
    always @(*) begin
        if (write_en)
            regs[write_addr] = write_data;
    end
endmodule
