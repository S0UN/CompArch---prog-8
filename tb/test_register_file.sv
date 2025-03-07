`include "../hdl/tinker.sv"

module test_register_file;
    reg [4:0] rs_addr, rt_addr, rd_addr;
    reg [63:0] write_data;
    reg write_enable;
    wire [63:0] rs_data, rt_data;

    register_file uut (
        .rs_addr(rs_addr),
        .rt_addr(rt_addr),
        .rd_addr(rd_addr),
        .write_data(write_data),
        .write_enable(write_enable),
        .rs_data(rs_data),
        .rt_data(rt_data)
    );

    initial begin
        // Write to register R1
        rd_addr = 5'd1;
        write_data = 64'h123456789ABCDEF;
        write_enable = 1;
        #10;

        // Read from R1 and R2
        rs_addr = 5'd1;
        rt_addr = 5'd2;
        write_enable = 0;
        #10;
        $display("R1=%h, R2=%h", rs_data, rt_data);

        $finish;
    end
endmodule