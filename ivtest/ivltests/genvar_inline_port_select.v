// Regression: an INLINE generate genvar (`for (genvar i=...)`) referenced in an
// instance INPUT port expression must not be shadowed by an auto-created
// implicit net. If it is, genvar arithmetic in a sibling port's bit-select
// (here .out(arr[8*i+j])) fails to constant-fold with
//   "A reference to a net or variable (`i') is not allowed in a constant
//    expression" / "Bit select expressions must be a constant integral value".
// Separately-declared genvars (genvar i;) never hit this; only the inline form.
module passthru(input in, output out);
   assign out = in;
endmodule
module main;
   wire [15:0] arr;
   reg  [6:0]  addr;
   for (genvar i=0; i<2; i++) begin : R
      for (genvar j=0; j<8; j++) begin : C
         passthru u (.in(addr == (8*i+j)), .out(arr[8*i+j]));
      end
   end
   initial begin
      addr = 7'd5;
      #1;
      if (arr === 16'h0020) $display("PASSED");
      else $display("FAILED arr=%h (expected 0020)", arr);
   end
endmodule
