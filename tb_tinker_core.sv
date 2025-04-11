module tb_tinker_core;
    logic clk, reset;
    tinker_core uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;
        #10 reset = 0;
        
        // Run for some time
        #2000 $finish;
    end
endmodule
