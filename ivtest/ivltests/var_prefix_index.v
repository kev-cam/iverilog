// Regression: a chained packed select with a VARIABLE (non-constant) PREFIX
// index. iverilog historically required every index but the last to be a
// constant ("Array index expressions must be constant here"); the lvalue path
// aborted. A variable prefix is now lowered to a computed indexed part select
// [sum(idx_k * stride_k) +: width], matching Verilator. Covers read + write,
// constant/variable final index, and a struct-member prefix.
typedef struct packed { logic [1:0] tid; } pkt_t;
module main;
   logic [1:0][31:0] arr;
   logic [3:0][7:0]  b;
   logic [0:0]       r;
   logic [1:0]       bi;
   pkt_t             pk;
   logic [15:0] o_part;  logic o_bit;  logic [7:0] o_cbit;  logic o_sbit;

   assign o_part = arr[r][15:0];   // var prefix + constant range (rvalue)
   assign o_bit  = arr[r][bi];     // var prefix + variable bit  (rvalue)
   assign o_cbit = b[pk.tid];      // struct-member prefix, whole element
   assign o_sbit = b[pk.tid][bi];  // struct-member prefix + variable bit

   logic [1:0][31:0] w;            // lvalue target
   logic [3:0][7:0]  wb;

   initial begin
      arr[0]=32'hDEAD_BEEF; arr[1]=32'h1234_5678;
      b[0]=8'h11; b[1]=8'h22; b[2]=8'hA5; b[3]=8'h44;
      r=1; bi=2; pk.tid=2;
      // lvalue writes with a variable prefix
      w='0; wb='0;
      w[r][15:0]  = 16'hABCD;      // var prefix + const range (lvalue)
      w[r][31:16] = 16'h1234;
      wb[r][3]    = 1'b1;          // var prefix + bit (lvalue): wb[1] bit3
      wb[pk.tid]  = 8'h5A;         // struct-member prefix (lvalue): wb[2]
      #1;
      if (o_part===16'h5678 && o_bit===1'b0 && o_cbit===8'hA5 && o_sbit===1'b1
          && w[1]===32'h1234ABCD && w[0]===32'h0
          && wb[1]===8'h08 && wb[2]===8'h5A)
            $display("PASSED");
      else  $display("FAILED o_part=%h o_bit=%b o_cbit=%h o_sbit=%b w1=%h w0=%h wb1=%h wb2=%h",
                     o_part,o_bit,o_cbit,o_sbit,w[1],w[0],wb[1],wb[2]);
   end
endmodule
