// SPI-mode SDHC card model, just enough for sdspi.v:
//   CMD0, CMD8 (R7), CMD55, ACMD41, CMD17 (single block read), CMD24 (single
//   block write); other commands answer R1 = 0.
// Block addressing (SDHC).  For each unit u the storage covers blocks
// BASE+u*STRIDE + 0..4095 and the RD54 RCT blocks + 311200..311206.
// MOSI is sampled and MISO changed on the falling edge of SCLK; sdspi changes
// MOSI and samples MISO on the rising edge.
// fail_next: number of following CMD17/CMD24 to reject with R1 = 0x04.
`timescale 1ns/1ps
module sdcard_model #(parameter integer BASE = 0, parameter integer UNITS = 4,
                      parameter integer STRIDE = 327680) (
   input      sclk, input cs, input mosi, output reg miso
);
   localparam NBLK = 4096 + 7;               // per unit: 4096 data blocks + 7 RCT
   reg [7:0] img [0:UNITS*NBLK*512-1];
   integer   fail_next = 0;
   integer   nrd = 0, nwr = 0, nbad = 0;

   // output bit queue
   reg       q [0:16383];
   integer   qh = 0, qt = 0;
   task put_bit(input b); begin q[qt % 16384] = b; qt = qt + 1; end endtask
   task put_byte(input [7:0] v); integer j; begin for (j = 7; j >= 0; j = j - 1) put_bit(v[j]); end endtask

   function integer blk_idx(input [31:0] blk);
      reg [31:0] r, u;
      begin
         r = blk - BASE; u = r / STRIDE; r = r % STRIDE;
         if (u >= UNITS) blk_idx = -1;
         else if (r < 4096) blk_idx = u*NBLK + r;
         else if (r >= 311200 && r < 311207) blk_idx = u*NBLK + 4096 + r - 311200;
         else blk_idx = -1;
      end
   endfunction

   localparam HUNT = 0, CMD = 1, WTOK = 2, WDATA = 3;
   integer   state = HUNT, n = 0, widx = -1, j;
   reg [47:0] sr;
   reg [4111:0] wsr;                     // 4096 data + 16 crc bits

   initial miso = 1'b1;

   always @(negedge sclk) begin
      if (cs) begin
         state = HUNT; qh = 0; qt = 0; miso <= 1'b1;
      end else begin
         case (state)
         HUNT: if (mosi == 1'b0) begin sr = 48'd0; sr[0] = 1'b0; n = 1; state = CMD; end
         CMD: begin
            sr = {sr[46:0], mosi}; n = n + 1;
            if (n == 48) begin
               state = HUNT;
               put_byte(8'hFF);                              // Ncr
               case (sr[45:40])
               6'd0:  put_byte(8'h01);
               6'd8:  begin put_byte(8'h01); put_byte(8'h00); put_byte(8'h00);
                            put_byte(8'h01); put_byte(8'hAA); end
               6'd55: put_byte(8'h01);
               6'd41: put_byte(8'h00);
               6'd17: begin
                  j = blk_idx(sr[39:8]);
                  if (fail_next > 0 || j < 0) begin
                     put_byte(8'h04); if (fail_next > 0) fail_next = fail_next - 1;
                     if (j < 0) nbad = nbad + 1;
                  end else begin
                     nrd = nrd + 1;
                     put_byte(8'h00); put_byte(8'hFF); put_byte(8'hFE);
                     for (n = 0; n < 512; n = n + 1) put_byte(img[j*512 + n]);
                     put_byte(8'h12); put_byte(8'h34);          // CRC (ignored)
                  end
               end
               6'd24: begin
                  j = blk_idx(sr[39:8]);
                  if (fail_next > 0 || j < 0) begin
                     put_byte(8'h04); if (fail_next > 0) fail_next = fail_next - 1;
                     if (j < 0) nbad = nbad + 1;
                  end else begin
                     put_byte(8'h00); widx = j; state = WTOK;
                  end
               end
               default: put_byte(8'h00);
               endcase
            end
         end
         WTOK: if (mosi == 1'b0) begin n = 0; state = WDATA; end   // last bit of token 0xFE
         WDATA: begin
            wsr = {wsr[4110:0], mosi}; n = n + 1;
            if (n == 4112) begin
               for (n = 0; n < 512; n = n + 1) img[widx*512 + n] = wsr[4111 - 8*n -: 8];
               nwr = nwr + 1;
               // data response r0..r8 = 1 | 1110 0101 (xxx0 0101), then busy, then ready
               put_bit(1); put_byte(8'b1110_0101);
               for (n = 0; n < 16; n = n + 1) put_bit(0);
               state = HUNT;
            end
         end
         endcase
         if (qh != qt) begin miso <= q[qh % 16384]; qh = qh + 1; end
         else miso <= 1'b1;
      end
   end
endmodule
