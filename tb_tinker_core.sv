`timescale 1ns/1ps

module tb_tinker_core;
  // Testbench signals
  reg        clk;
  reg        reset;
  wire       hlt;

  // Instantiate Device Under Test
  tinker_core uut (
    .clk   (clk),
    .reset (reset),
    .hlt   (hlt)
  );

  // 10 ns clock
  initial clk = 0;
  always #5 clk = ~clk;

  // Reset pulse
  initial begin
    reset = 1;
    #20;          // hold reset high for 20 ns
    reset = 0;    // release
  end

  // Finish after some time
  initial begin
    #1000;        // run 1 µs
    $finish;
  end

  // VCD dump for waveform inspection (if your simulator supports it)
  initial begin
    $dumpfile("tb_tinker_core.vcd");
    $dumpvars(0, tb_tinker_core);
  end
endmodule