// Behavioural stand-in for the Quartus 'sectorbuf' dual-port RAM used by sdspi.v
// 256 x 16, two independent clocks, registered address.
// OUTREG=1 additionally registers q (2-clock read latency) - the controller
// must work with either setting.
`timescale 1ns/1ps
module sectorbuf #(parameter OUTREG = 0) (
   input  [7:0]  address_a, input [7:0] address_b,
   input         clock_a,   input       clock_b,
   input  [15:0] data_a,    input [15:0] data_b,
   input         wren_a,    input       wren_b,
   output [15:0] q_a,       output [15:0] q_b
);
   reg [15:0] mem [0:255];
   reg [7:0]  ra, rb;
   reg [15:0] qa_r, qb_r;
   always @(posedge clock_a) begin
      if (wren_a) mem[address_a] <= data_a;
      ra <= address_a; qa_r <= mem[ra];
   end
   always @(posedge clock_b) begin
      if (wren_b) mem[address_b] <= data_b;
      rb <= address_b; qb_r <= mem[rb];
   end
   assign q_a = OUTREG ? qa_r : mem[ra];
   assign q_b = OUTREG ? qb_r : mem[rb];
endmodule
