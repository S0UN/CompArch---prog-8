// Corrected Testbench for Tinker Core
module tb_tinker_core;
    // Declare signals
    logic clk, reset, hlt;

    // Instantiate the unit under test (UUT)
    // Assuming 'tinker_core' module is defined in tinker.sv
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
        // CORRECTED: Use 'mem', the instance name of the memory_unit inside tinker_core
        // The path is uut (tinker_core instance) -> mem (memory_unit instance) -> bytes (memory array)
        $readmemh("program.hex", uut.mem.bytes);
    end

    // Monitor key signals for debugging
    initial begin
        // CORRECTED: Use the correct internal signal names 'PC' and 'IR' from tinker_core
        $monitor("Time: %0t | PC: %h | Instruction: %h | HLT: %b",
                 $time, uut.PC, uut.IR, hlt);
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