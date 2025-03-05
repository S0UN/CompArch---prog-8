// reg_file.v
// ---------------------------------------------------------
// Register File Module
//  - Contains 32 registers (64 bits wide).
//  - Three asynchronous read ports (for rs, rt, and rd)
//  - One synchronous write port (with reset).
// ---------------------------------------------------------
module reg_file(
    input         clk,
    input         rst,
    input  [4:0]  read_addr1,  // For rs
    input  [4:0]  read_addr2,  // For rt
    input  [4:0]  read_addr3,  // For rd (used in some immediate instructions)
    input  [4:0]  write_addr,
    input  [63:0] write_data,
    input         write_en,
    output [63:0] read_data1,
    output [63:0] read_data2,
    output [63:0] read_data3
);
    reg [63:0] regs[31:0];
    integer i;
    
    // Asynchronous reads
    assign read_data1 = regs[read_addr1];
    assign read_data2 = regs[read_addr2];
    assign read_data3 = regs[read_addr3];
    
    // Synchronous write and reset
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                regs[i] <= 64'd0;
        end else if (write_en) begin
            regs[write_addr] <= write_data;
        end
    end
endmodule
