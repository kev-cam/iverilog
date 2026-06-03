// Regression for an sv2ghdl/nvc translation+runtime bug: a blocking-assigned
// clock (clk = ~clk in a forever loop) lowers to a VHDL deposit (:=), which
// must register an event so @(posedge clk) receivers wake. The bug left the
// clk deposit eventless, so rising_edge never fired and the sim quiesced at
// time 0 (no output at all). PASSED iff the 8 clock edges were counted.
module top;
  reg clk;
  reg [7:0] n;
  initial begin clk = 1'b1; forever #5 clk = ~clk; end
  initial begin
    n = 0;
    repeat (8) @(posedge clk) n = n + 8'd1;
    if (n == 8'd8) $display("PASSED");
    else           $display("FAILED n=%0d", n);
    $finish;
  end
endmodule
