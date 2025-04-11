module tb_tinker_core;
    // Declare signals
    logic clk, reset, hlt;

    // Instantiate the unit under test (UUT)
    tinker_core uut (
        .clk(clk),
        .reset(reset),
        .hlt(hlt)
    );

    // Clock generation: Period of 10 time units
    always #5 clk = ~clk;

    // Initial setup: Clock and reset
    initial begin
        clk = 0;       // Start clock low
        reset = 1;     // Assert reset
        #10 reset = 0; // Deassert reset after 10 time units
    end

    // Memory initialization: Load program from a hex file
    initial begin
        $readmemh("program.hex", uut.memory.bytes);
    end

    // Monitor key signals for debugging
    initial begin
        $monitor("Time: %0t | PC: %h | Instruction: %h | HLT: %b", 
                 $time, uut.pc_current, uut.instr_word, hlt);
    end

    // Stop simulation when halt is asserted
    always @(posedge clk) begin
        if (hlt) begin
            $display("Halt instruction encountered at time %0t", $time);
            $finish;
        end
    end

    // Timeout to prevent infinite simulation
    initial begin
        #10000; // Maximum simulation time of 10000 time units
        $display("Simulation timeout");
        $finish;
    end
endmodule