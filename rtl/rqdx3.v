//=================================================================
//  RQDX3 MSCP disk controller (DU:), 4 RD54 units, on SD card
//
//  Interfaces follow the project conventions of forth32 dvk-fpga project
//
//  Behavioural reference: open-simh PDP11/pdp11_rq.c, pdp11_mscp.h,
//  pdp11_uqssp.h.  The MSCP / UQSSP logic is unchanged from
//  rqdx3_lite.v, which was cross-checked bit-for-bit against simh.
//
//  Registers (wb_adr_i[1]):  0 - IP (17772150), 1 - SA (17772152)
//  SD layout: LBN n of unit u is card block start_offset + u*STRIDE + n;
//             each slot needs 311207 blocks (311200 data + 7 RCT)
//=================================================================
`default_nettype none

module rqdx3 #(
   parameter integer UNITS     = 4,      // number of RD54 drives, 1..4 (DU0..DU3)
   parameter [26:0]  STRIDE    = 27'h50000, // card blocks per drive slot (>= 311207)
   parameter integer DMA_TMO   = 63,     // DMA reply timeout, clocks
   parameter integer RSP_RETRY = 255     // clocks between re-polls of a full response ring
)(
// Wishbone bus
   input                  wb_clk_i,   // bus clock
   input                  wb_rst_i,   // reset
   input    [1:0]         wb_adr_i,   // address: bit 1 selects IP/SA
   input    [15:0]        wb_dat_i,   // data in
   output reg [15:0]      wb_dat_o,   // data out
   input                  wb_cyc_i,   // bus cycle
   input                  wb_we_i,    // write enable (0 - read)
   input                  wb_stb_i,   // strobe
   input    [1:0]         wb_sel_i,   // byte select (IP/SA are word registers, ignored)
   output                 wb_ack_o,   // acknowledge

// interrupt
   output reg             irq,        // request
   input                  iack,       // acknowledge
   output     [8:0]       irq_vec,    // vector programmed by the host (S1 data <6:0> * 4)

// DMA
   output reg             dma_req,    // DMA request
   input                  dma_gnt,    // DMA grant
   output reg [21:0]      dma_adr_o,  // 22-bit physical address
   input      [15:0]      dma_dat_i,  // data from memory
   output reg [15:0]      dma_dat_o,  // data to memory
   output reg             dma_stb_o,  // bus strobe
   output reg             dma_we_o,   // 1 - write to memory, 0 - read from memory
   input                  dma_ack_i,  // reply

// SD card
   output                 sdcard_cs,
   output                 sdcard_mosi,
   output                 sdcard_sclk,
   input                  sdcard_miso,
   output reg             sdreq,      // card access request
   input                  sdack,      // card access granted
   input                  sdmode,     // sdspi mode: 0 - slave, 1 - master
   input                  sdclock,    // SD clock
   input [26:0]           start_offset, // first card block of DU0 (bank base)
   output [3:0]           sdcard_debug
);
   `include "rqdx3_defs.vh"

   wire reset = wb_rst_i;

   // fixed unit properties: the image is always present, no write lock
   wire blk_present_i = 1'b1;
   wire blk_wp_i      = 1'b0;

   //***********************************************
   //*  sdspi
   //***********************************************
   reg  [26:0] sdcard_addr;
   wire        sdcard_error;
   wire [15:0] sdbuf_dataout;
   wire        sdcard_idle;
   reg  [7:0]  sdbuf_addr;
   reg         sdbuf_we;
   reg  [15:0] sdbuf_datain;
   reg         sdspi_start;
   reg         sdspi_write_mode;
   wire        sdspi_io_done;

   sdspi sd1 (
      .sdcard_cs(sdcard_cs),
      .sdcard_mosi(sdcard_mosi),
      .sdcard_miso(sdcard_miso),
      .sdcard_sclk(sdcard_sclk),
      .sdcard_debug(sdcard_debug),
      .sdcard_addr(sdcard_addr),
      .sdcard_idle(sdcard_idle),
      .sdcard_error(sdcard_error),
      .sdspi_start(sdspi_start),
      .sdspi_io_done(sdspi_io_done),
      .sdspi_write_mode(sdspi_write_mode),
      .sdbuf_addr(sdbuf_addr),
      .sdbuf_dataout(sdbuf_dataout),
      .sdbuf_datain(sdbuf_datain),
      .sdbuf_we(sdbuf_we),
      .mode(sdmode),
      .controller_clk(wb_clk_i),
      .reset(reset),
      .sdclk(sdclock)
   );

   //***********************************************
   //*  Wishbone reply
   //***********************************************
   wire bus_strobe    = wb_cyc_i & wb_stb_i & ~wb_ack_o;
   wire bus_read_req  = bus_strobe & ~wb_we_i;
   wire bus_write_req = bus_strobe & wb_we_i;

   reg reply;
   always @(posedge wb_clk_i or posedge wb_rst_i)
      if (wb_rst_i == 1) reply <= 1'b0;
      else if (wb_stb_i) reply <= 1'b1;
      else reply <= 1'b0;
   assign wb_ack_o = reply & wb_stb_i;

   //***********************************************
   //*  SD sector engine: one sector, card <-> sdspi buffer
   //*  Runs independently of the port FSM so that an IP-write reset
   //*  never cuts an sdspi transaction or the card grant in half.
   //***********************************************
   localparam [1:0] SE_IDLE = 2'd0, SE_REQ = 2'd1, SE_RUN = 2'd2, SE_END = 2'd3;
   reg  [1:0]  se_st;
   reg         eng_start, eng_wr, eng_busy, eng_done, eng_err;
   reg  [31:0] eng_lba;
   reg  [26:0] eng_base;               // start_offset + unit * STRIDE
   wire        eng_active = eng_busy | eng_start;

   always @(posedge wb_clk_i) begin
      eng_done <= 1'b0;
      if (reset) begin
         se_st <= SE_IDLE; eng_busy <= 1'b0; eng_err <= 1'b0;
         sdreq <= 1'b0; sdspi_start <= 1'b0; sdspi_write_mode <= 1'b0;
         sdcard_addr <= 27'd0;
      end else case (se_st)
         SE_IDLE:
            if (eng_start) begin
               eng_busy <= 1'b1;
               sdreq <= 1'b1;                          // ask for the card
               sdspi_write_mode <= eng_wr;             // set before start
               sdcard_addr <= eng_base + eng_lba[26:0];
               se_st <= SE_REQ;
            end
         SE_REQ:                                       // card granted and sdspi free
            if (sdack && sdcard_idle && !sdspi_io_done) begin
               sdspi_start <= 1'b1;
               se_st <= SE_RUN;
            end
         SE_RUN:
            if (sdspi_io_done) begin
               eng_err <= sdcard_error;                // valid together with io_done
               sdspi_start <= 1'b0;
               se_st <= SE_END;
            end
         SE_END:                                       // sdspi back to idle
            if (!sdspi_io_done) begin
               sdspi_write_mode <= 1'b0;
               sdreq <= 1'b0;
               eng_busy <= 1'b0; eng_done <= 1'b1;
               se_st <= SE_IDLE;
            end
      endcase
   end

   //***********************************************
   //*  Interrupt handshake
   //***********************************************
   localparam [1:0] i_idle = 2'd0, i_req = 2'd1, i_wait = 2'd2;
   reg  [1:0]  interrupt_state;
   reg         int_pend;                // rq_setint(): request pending
   reg         int_set, int_clr;        // pulses from the port FSM

   always @(posedge wb_clk_i) begin
      if (reset || int_clr) begin
         interrupt_state <= i_idle; irq <= 1'b0; int_pend <= 1'b0;
      end else begin
         // a new request while one is already pending/asserted is the same request
         if (int_set && interrupt_state != i_req) int_pend <= 1'b1;
         case (interrupt_state)
            i_idle: if (int_pend) begin
                       irq <= 1'b1; int_pend <= 1'b0; interrupt_state <= i_req;
                    end
            i_req:  if (iack) begin
                       irq <= 1'b0; interrupt_state <= i_wait;
                    end
            i_wait: if (!iack) interrupt_state <= i_idle;
            default: interrupt_state <= i_idle;
         endcase
      end
   end

   //***********************************************
   //*  Port / MSCP state
   //***********************************************
   reg  [3:0]  csta;
   reg  [15:0] sa, saw, s1dat, cflgs;
   reg  [15:0] ufa [0:3];               // per-unit settable flags (uptr->uf)
   reg  [3:0]  online;                  // per-unit UNIT_ONL
   reg  [21:0] comm;
   reg         prgi, pip, saw_pend, rst_pend, st4_pend;
   reg  [3:0]  credits;
   reg  [8:0]  rq_lnt, cq_lnt;
   reg  [8:0]  rq_idx, cq_idx;
   wire [21:0] rq_ba = comm;
   wire [21:0] cq_ba = comm + {13'd0, rq_lnt};

   assign irq_vec = {s1dat[6:0], 2'b00};

   reg  [15:0] pkt [0:31];
   integer     i;

   localparam [5:0]
      S_IDLE   = 6'd0,  S_ST4Z   = 6'd1,
      S_FD0    = 6'd2,  S_FD1    = 6'd3,  S_FPKT  = 6'd4,
      S_PDW0   = 6'd5,  S_PDW1   = 6'd6,  S_PDRP  = 6'd7,  S_PDINT = 6'd8,
      S_DISP   = 6'd9,
      S_RWVAL  = 6'd10, S_RWSECT = 6'd11, S_RWEW  = 6'd12,
      S_B2HA   = 6'd13, S_B2HW   = 6'd14, S_H2B   = 6'd15, S_PAD   = 6'd16,
      S_CMPA   = 6'd17, S_CMPR   = 6'd18, S_RWADV = 6'd19, S_RWEND = 6'd20,
      S_RD0    = 6'd21, S_RD1    = 6'd22, S_RWAIT = 6'd23, S_RPKT  = 6'd24,
      S_ENGGO  = 6'd25, S_DONE   = 6'd26, S_B2HL1 = 6'd27, S_B2HL2 = 6'd28,
      S_CMPL1  = 6'd29, S_CMPL2  = 6'd30;
   reg  [5:0]  st, pd_ret;
   reg         pd_ring;
   reg  [31:0] desc;
   reg  [21:0] paddr;
   reg  [9:0]  k;
   reg  [9:0]  zlen;
   reg  [15:0] tmo;
   reg  [1:0]  gap;
   reg  [15:0] retry;

   reg  [21:0] wba;
   reg  [31:0] wbc, wbl, xdone;
   reg  [15:0] end_sts;
   reg  [7:0]  end_flg;
   reg         nxm_pend;

   wire [7:0]  opc   = pkt[CMD_OPC][7:0];
   wire [15:0] mdf   = pkt[CMD_MOD];
   wire [15:0] lu    = pkt[CMD_UN];
   // rq_getucb(): units 0..UNITS-1 exist (plug = unit number)
   wire        unit_ok = (lu < UNITS);
   wire [1:0]  un      = lu[1:0];
   wire [15:0] uf      = ufa[un];       // flags of the addressed unit
   wire        onl     = online[un];
   // GUS with MD_NXU: beyond the last plug wraps to unit 0
   wire        gus_wrap = ((mdf & MD_NXU) != 16'd0) && (lu > UNITS - 1);
   wire [15:0] lu_g     = gus_wrap ? 16'd0 : lu;
   wire        gus_ok   = (lu_g < UNITS);
   wire [15:0] uf_g     = ufa[lu_g[1:0]];
   wire        onl_g    = online[lu_g[1:0]];
   // card block of LBN 0 of the addressed unit
   wire [26:0] unit_base = start_offset + STRIDE * {25'd0, un};
   wire [31:0] bc    = {pkt[RW_BCH], pkt[RW_BCL]};
   wire [31:0] lbn   = {pkt[RW_LBNH], pkt[RW_LBNL]};
   wire [9:0]  chunk = (wbc > 32'd512) ? 10'd512 : wbc[9:0];
   wire [8:0]  cwords = chunk[9:1];
   wire        is_wrop = (opc == OP_WR) || (opc == OP_ERS);
   wire [15:0] wph   = blk_wp_i ? UF_WPH : 16'd0;

   // rq_rw_valid()
   reg  [15:0] vsts;
   always @* begin
      vsts = 16'd0;
      if (!blk_present_i)                          vsts = ST_OFL | SB_OFL_NV;
      else if (!onl)                               vsts = ST_AVL;
      else if (opc != OP_ACC && opc != OP_ERS && pkt[RW_BAL][0])
                                                   vsts = ST_HST | SB_HST_OA;
      else if (bc[0])                              vsts = ST_HST | SB_HST_OC;
      else if (bc[31:28] != 4'd0)                  vsts = ST_CMD | I_BCNT;
      else if (lbn >= RD54_LBN) begin
         if (lbn >= RD54_LBN + RD54_RCTS)          vsts = ST_CMD | I_LBN;
         else if (bc != 32'd512)                   vsts = ST_CMD | I_BCNT;
      end else if ({1'b0,lbn} + {1'b0,(bc + 32'd511) >> 9} > {1'b0,RD54_LBN})
                                                   vsts = ST_CMD | I_BCNT;
      if (vsts == 16'd0 && is_wrop) begin
         if (lbn >= RD54_LBN)                      vsts = ST_CMD | I_LBN;
         else if ((uf & UF_WPS) != 16'd0)          vsts = ST_WPR | SB_WPR_SW;
         else if (blk_wp_i)                        vsts = ST_WPR | SB_WPR_HW;
      end
   end

   // ---- DMA cycle helpers ----
   // a cycle starts only with the grant and after >= 2 idle clocks of
   // dma_stb_o
   wire bus_can  = ~dma_stb_o & dma_gnt & (gap == 2'd2);
   wire bus_tmo  = (tmo == DMA_TMO[15:0]);
   wire bus_done = dma_stb_o & (dma_ack_i | bus_tmo);
   wire bus_err  = bus_tmo & ~dma_ack_i;

   // states in which the controller is bus master: dma_req is held
   wire want_dma = (st == S_ST4Z) || (st == S_FD0)  || (st == S_FD1)  || (st == S_FPKT) ||
                   (st == S_PDW0) || (st == S_PDW1) || (st == S_PDRP) || (st == S_PDINT) ||
                   (st == S_B2HA) || (st == S_B2HL1)|| (st == S_B2HL2)|| (st == S_B2HW) ||
                   (st == S_H2B)  || (st == S_CMPA) || (st == S_CMPL1)|| (st == S_CMPL2) ||
                   (st == S_CMPR) || (st == S_RD0)  || (st == S_RD1)  || (st == S_RPKT);

   task bus_rd(input [21:0] a);
      begin dma_adr_o <= {a[21:1],1'b0}; dma_we_o <= 1'b0; dma_stb_o <= 1'b1; end
   endtask
   task bus_wr(input [21:0] a, input [15:0] d);
      begin dma_adr_o <= {a[21:1],1'b0}; dma_dat_o <= d; dma_we_o <= 1'b1; dma_stb_o <= 1'b1; end
   endtask

   task putr(input [7:0] cmd, input [7:0] flg, input [15:0] sts, input [15:0] lnt);
      begin
         pkt[RSP_OPF] <= {flg, cmd};
         pkt[RSP_STS] <= sts;
         pkt[UQ_HLNT] <= lnt;
         pkt[UQ_HCTC] <= {UQ_CID_MSCP[7:0], UQ_TYP_SEQ[3:0], 4'd0};
      end
   endtask

   task putr_unit(input all, input [15:0] ufl);
      begin
         pkt[ONL_MLUN] <= lu;
         pkt[ONL_UFL]  <= ufl | UF_RPL | wph;
         pkt[ONL_RSVL] <= 16'd0;  pkt[ONL_RSVH] <= 16'd0;
         pkt[ONL_UIDA] <= lu;     pkt[ONL_UIDB] <= 16'd0;  pkt[ONL_UIDC] <= 16'd0;
         pkt[ONL_UIDD] <= {UID_DISK[7:0], RD54_MOD[7:0]};
         pkt[ONL_MEDL] <= RD54_MED[15:0];  pkt[ONL_MEDH] <= RD54_MED[31:16];
         if (all) begin
            pkt[ONL_SIZL] <= RD54_LBN[15:0]; pkt[ONL_SIZH] <= RD54_LBN[31:16];
            pkt[ONL_VSNL] <= 16'o1234 + lu;  pkt[ONL_VSNH] <= 16'd0;
         end
      end
   endtask

   function [15:0] setf(input [15:0] ufl_in, input [15:0] mod_in);
      setf = (ufl_in & UF_MSK) |
             (((mod_in & MD_SWP) != 0 && (ufl_in & UF_WPS) != 0) ? UF_WPS : 16'd0);
   endfunction

   task port_reset;
      begin
         csta <= CST_S1; s1dat <= 16'd0; comm <= 22'd0;
         sa   <= SA_S1 | SA_S1C_Q22 | SA_S1C_DI | SA_S1C_MP;
         cflgs <= CF_RPL; credits <= 4'd15;
         pip <= 1'b0; int_clr <= 1'b1; online <= 4'd0;
         ufa[0] <= 16'd0; ufa[1] <= 16'd0; ufa[2] <= 16'd0; ufa[3] <= 16'd0;
         prgi <= 1'b0; st4_pend <= 1'b0; saw_pend <= 1'b0; rst_pend <= 1'b0;
         rq_lnt <= 9'd4; cq_lnt <= 9'd4; rq_idx <= 9'd0; cq_idx <= 9'd0;
         nxm_pend <= 1'b0;
         st <= S_IDLE;
      end
   endtask

   task fatal(input [15:0] err);
      begin port_reset; sa <= SA_ER | err; csta <= CST_DEAD; end
   endtask

   task init_int(input [15:0] s1);
      begin if (s1[7] && s1[6:0] != 7'd0) int_set <= 1'b1; end
   endtask

   task putdesc(input ring, input [5:0] ret);
      begin pd_ring <= ring; pd_ret <= ret; st <= S_PDW0; end
   endtask

   wire [21:0] pd_ba  = pd_ring ? rq_ba  : cq_ba;
   wire [8:0]  pd_idx = pd_ring ? rq_idx : cq_idx;
   wire [8:0]  pd_lnt = pd_ring ? rq_lnt : cq_lnt;
   wire [31:0] newd   = (desc & ~UQ_DESC_OWN) | UQ_DESC_F;
   wire [8:0]  pd_prv = (pd_idx - 9'd4) & (pd_lnt - 9'd1);

   wire [8:0]  s4_rlnt = 9'd4 << s1dat[10:8];
   wire [8:0]  s4_clnt = 9'd4 << s1dat[13:11];

   always @(posedge wb_clk_i) begin
      eng_start <= 1'b0;
      sdbuf_we  <= 1'b0;
      int_set   <= 1'b0;
      int_clr   <= 1'b0;
      if (dma_stb_o && !bus_done) tmo <= tmo + 16'd1; else tmo <= 16'd0;
      if (dma_stb_o) gap <= 2'd0; else if (gap != 2'd2) gap <= gap + 2'd1;
      dma_req <= (want_dma & ~rst_pend) | dma_stb_o;

      if (reset) begin
         port_reset;
         dma_stb_o <= 1'b0; dma_we_o <= 1'b0; dma_req <= 1'b0;
         dma_adr_o <= 22'd0; dma_dat_o <= 16'd0;
         wb_dat_o <= 16'd0; tmo <= 16'd0; gap <= 2'd0; retry <= 16'd0; k <= 10'd0;
         sdbuf_addr <= 8'd0; sdbuf_datain <= 16'd0;
         for (i = 0; i < 32; i = i + 1) pkt[i] <= 16'd0;
      end else begin
         if (bus_done) dma_stb_o <= 1'b0;

         if (rst_pend && !dma_stb_o) begin           // IP write -> rq_reset
            port_reset;
         end else case (st)

         // ------------------------------------------------- idle / init steps
         S_IDLE: begin
            if (saw_pend) begin
               saw_pend <= 1'b0;
               case (csta)
               CST_S1: if ((saw & SA_S1H_VL) != 16'd0) begin
                          if ((saw & SA_S1H_WR) != 16'd0) begin
                             sa <= saw; csta <= CST_S1_WR;
                          end else begin
                             s1dat <= saw;
                             sa <= SA_S2 | SA_S2C_PT | {8'd0, saw[15:8]};
                             csta <= CST_S2;
                             init_int(saw);
                          end
                       end
               CST_S1_WR: sa <= saw;
               CST_S2: begin
                          comm[15:0] <= saw & SA_S2H_CLO;
                          prgi <= saw[0];
                          sa <= SA_S3 | {8'd0, s1dat[7:0]};
                          csta <= CST_S3;
                          init_int(s1dat);
                       end
               CST_S3: begin
                          comm[21:16] <= saw[5:0];
                          if ((saw & SA_S3H_PP) != 16'd0) begin
                             sa <= 16'd0; csta <= CST_S3_PPA;
                          end else st4_pend <= 1'b1;
                       end
               CST_S3_PPA: if (saw != 16'd0) fatal(PE_PPF);
                           else csta <= CST_S3_PPB;
               CST_S4: if ((saw & SA_S4H_GO) != 16'd0) begin
                          csta <= CST_UP; sa <= 16'd0;
                       end
               default: ;
               endcase
            end else if (st4_pend) begin
               st4_pend <= 1'b0;
               rq_lnt <= s4_rlnt; cq_lnt <= s4_clnt;
               rq_idx <= 9'd0;    cq_idx <= 9'd0;
               paddr  <= comm - (prgi ? 22'd8 : 22'd4);
               zlen   <= ({1'b0,(prgi ? 9'd8 : 9'd4)} + {1'b0,s4_rlnt} + {1'b0,s4_clnt}) >> 1;
               k <= 10'd0;
               st <= S_ST4Z;
            end else if (csta == CST_UP && pip) begin
               st <= S_FD0;
            end
         end

         S_ST4Z: begin
            if (bus_can) bus_wr(paddr + {11'd0, k, 1'b0}, 16'd0);
            else if (bus_done) begin
               if (bus_err) fatal(PE_QWE);
               else if (k + 10'd1 == zlen) begin
                  sa <= SA_S4 | (RQDX3_UQPM << SA_S4C_V_MOD) | (RQ_SVER << SA_S4C_V_VER);
                  csta <= CST_S4;
                  init_int(s1dat);
                  st <= S_IDLE;
               end else k <= k + 10'd1;
            end
         end

         // ------------------------------------------------- fetch command
         // descriptor: high word (OWN) first, then the address word.  The host
         // writes the address before setting OWN, so this order can never pair
         // a fresh OWN bit with a stale address (simh reads both atomically).
         S_FD0: begin
            if (bus_can) bus_rd(cq_ba + {13'd0, cq_idx} + 22'd2);
            else if (bus_done) begin
               if (bus_err) fatal(PE_QRE);
               else begin
                  desc[31:16] <= dma_dat_i;
                  if (!dma_dat_i[15]) begin pip <= 1'b0; st <= S_IDLE; end   // ring empty
                  else st <= S_FD1;
               end
            end
         end
         S_FD1: begin
            if (bus_can) bus_rd(cq_ba + {13'd0, cq_idx});
            else if (bus_done) begin
               if (bus_err) fatal(PE_QRE);
               else begin
                  desc[15:0] <= dma_dat_i;
                  paddr <= {desc[21:16], dma_dat_i[15:1], 1'b0} - 22'd4;
                  k <= 10'd0; st <= S_FPKT;
               end
            end
         end
         S_FPKT: begin
            if (bus_can) bus_rd(paddr + {11'd0, k, 1'b0});
            else if (bus_done) begin
               if (bus_err) fatal(PE_PRE);
               else begin
                  pkt[k[4:0]] <= dma_dat_i;
                  if (k == 10'd31) putdesc(1'b0, S_DISP);
                  else k <= k + 10'd1;
               end
            end
         end

         // ------------------------------------------------- rq_putdesc
         S_PDW0: begin
            if (bus_can) bus_wr(pd_ba + {13'd0, pd_idx}, newd[15:0]);
            else if (bus_done) begin
               if (bus_err) fatal(PE_QWE); else st <= S_PDW1;
            end
         end
         S_PDW1: begin
            if (bus_can) bus_wr(pd_ba + {13'd0, pd_idx} + 22'd2, newd[31:16]);
            else if (bus_done) begin
               if (bus_err) fatal(PE_QWE);
               else if (desc[30]) begin
                  if (pd_lnt <= 9'd4) st <= S_PDINT;
                  else st <= S_PDRP;
               end else st <= S_DONE;
            end
         end
         S_PDRP: begin
            if (bus_can) bus_rd(pd_ba + {13'd0, pd_prv} + 22'd2);
            else if (bus_done) begin
               if (bus_err) fatal(PE_QRE);
               else st <= dma_dat_i[15] ? S_PDINT : S_DONE;
            end
         end
         S_PDINT: begin
            if (bus_can) bus_wr(comm - (pd_ring ? 22'd2 : 22'd4), 16'd1);
            else if (bus_done) begin
               if (s1dat[6:0] != 7'd0) int_set <= 1'b1;
               st <= S_DONE;
            end
         end
         S_DONE: begin
            if (pd_ring) rq_idx <= (rq_idx + 9'd4) & (rq_lnt - 9'd1);
            else         cq_idx <= (cq_idx + 9'd4) & (cq_lnt - 9'd1);
            st <= pd_ret;
         end

         // ------------------------------------------------- dispatch
         S_DISP: begin
            st <= S_RD0;
            if (pkt[UQ_HCTC][7:4] != UQ_TYP_SEQ[3:0]) begin
               fatal(PE_PIE);
            end else if (pkt[UQ_HCTC][15:8] == UQ_CID_DUP[7:0]) begin
               putr(OP_END, 8'd0, ST_CMD | I_OPCD, RSP_LNT);
            end else if (pkt[UQ_HCTC][15:8] != UQ_CID_MSCP[7:0]) begin
               fatal(PE_ICI);
            end else case (opc)
            OP_ABO:
               putr(opc | OP_END, 8'd0, ST_SUC, ABO_LNT);
            OP_GCS: begin
               pkt[GCS_STSL] <= 16'd0; pkt[GCS_STSH] <= 16'd0;
               putr(opc | OP_END, 8'd0, ST_SUC, GCS_LNT);
            end
            OP_AVL: if (unit_ok) begin
                  online[un] <= 1'b0; ufa[un] <= 16'd0;
                  putr(opc | OP_END, 8'd0, ST_SUC, AVL_LNT);
               end else putr(opc | OP_END, 8'd0, ST_OFL, AVL_LNT);
            OP_GUS: begin
               if (gus_wrap) pkt[RSP_UN] <= 16'd0;
               if (gus_ok) begin
                  pkt[ONL_MLUN] <= lu_g;
                  pkt[ONL_UFL]  <= uf_g | UF_RPL | wph;
                  pkt[ONL_RSVL] <= 16'd0;  pkt[ONL_RSVH] <= 16'd0;
                  pkt[ONL_UIDA] <= lu_g;   pkt[ONL_UIDB] <= 16'd0;  pkt[ONL_UIDC] <= 16'd0;
                  pkt[ONL_UIDD] <= {UID_DISK[7:0], RD54_MOD[7:0]};
                  pkt[ONL_MEDL] <= RD54_MED[15:0];  pkt[ONL_MEDH] <= RD54_MED[31:16];
                  pkt[GUS_TRK]  <= RD54_SECT;  pkt[GUS_GRP] <= RD54_TPG;
                  pkt[GUS_CYL]  <= RD54_GPC;   pkt[GUS_UVER] <= 16'd0;
                  pkt[GUS_RCTS] <= RD54_RCTS[15:0];
                  pkt[GUS_RBSC] <= 16'h0101;
                  putr(opc | OP_END, 8'd0,
                       !blk_present_i ? (ST_OFL | SB_OFL_NV) :
                       onl_g ? ST_SUC : ST_AVL, GUS_LNT_D);
               end else
                  putr(opc | OP_END, 8'd0, ST_OFL, GUS_LNT_D);
               pkt[GUS_SHUN] <= lu_g; pkt[GUS_SHST] <= 16'd0;
            end
            OP_ONL: begin
               pkt[ONL_SHUN] <= lu; pkt[ONL_SHST] <= 16'd0;
               if (!unit_ok) putr(opc | OP_END, 8'd0, ST_OFL, ONL_LNT);
               else if (!blk_present_i) begin
                  putr_unit(1'b1, uf);
                  putr(opc | OP_END, 8'd0, ST_OFL | SB_OFL_NV, ONL_LNT);
               end else if (onl) begin
                  putr_unit(1'b1, uf);
                  putr(opc | OP_END, 8'd0, ST_SUC | SB_SUC_ON, ONL_LNT);
               end else begin
                  online[un] <= 1'b1;
                  ufa[un] <= setf(pkt[ONL_UFL], mdf);
                  putr_unit(1'b1, setf(pkt[ONL_UFL], mdf));
                  putr(opc | OP_END, 8'd0, ST_SUC, ONL_LNT);
               end
            end
            OP_SCC: begin
               if (pkt[SCC_MSV] != 16'd0)
                  putr(OP_END, 8'd0, ST_CMD | I_VRSN, SCC_LNT);
               else begin
                  cflgs <= (cflgs & CF_RPL) | pkt[SCC_CFL];
                  pkt[SCC_CFL]  <= (cflgs & CF_RPL) | pkt[SCC_CFL];
                  pkt[SCC_TMO]  <= RQ_DCTMO;
                  pkt[SCC_VER]  <= (RQ_HVER << SCC_VER_V_HVER) | (RQ_SVER << SCC_VER_V_SVER);
                  pkt[SCC_CIDA] <= 16'd0; pkt[SCC_CIDB] <= 16'd0; pkt[SCC_CIDC] <= 16'd0;
                  pkt[SCC_CIDD] <= (RQ_CLASS << SCC_CIDD_V_CLS) | (RQDX3_MODEL << SCC_CIDD_V_MOD);
                  pkt[SCC_MBCL] <= 16'd0; pkt[SCC_MBCH] <= 16'd0;
                  putr(opc | OP_END, 8'd0, ST_SUC, SCC_LNT);
               end
            end
            OP_SUC: begin
               pkt[ONL_SHUN] <= lu; pkt[ONL_SHST] <= 16'd0;
               if (!unit_ok) putr(opc | OP_END, 8'd0, ST_OFL, SUC_LNT);
               else if (!blk_present_i) begin
                  putr_unit(1'b1, uf);
                  putr(opc | OP_END, 8'd0, ST_OFL | SB_OFL_NV, SUC_LNT);
               end else begin
                  ufa[un] <= setf(pkt[ONL_UFL], mdf);
                  putr_unit(1'b1, setf(pkt[ONL_UFL], mdf));
                  putr(opc | OP_END, 8'd0, ST_SUC, SUC_LNT);
               end
            end
            OP_FMT:
               putr(opc | OP_END, 8'd0, unit_ok ? (ST_CMD | I_OPCD) : ST_OFL, FMT_LNT);
            OP_ACC, OP_CMP, OP_ERS, OP_RD, OP_WR:
               st <= S_RWVAL;
            OP_CCD, OP_DAP, OP_FLU:
               putr(opc | OP_END, 8'd0, ST_SUC, RSP_LNT);
            default:
               putr(OP_END, 8'd0, ST_CMD | I_OPCD, RSP_LNT);
            endcase
         end

         // ------------------------------------------------- data transfers
         S_RWVAL: begin
            if (!unit_ok || vsts != 16'd0) begin
               pkt[RW_BCL] <= 16'd0; pkt[RW_BCH] <= 16'd0;
               putr(opc | OP_END, 8'd0, unit_ok ? vsts : ST_OFL, RW_LNT_D);
               st <= S_RD0;
            end else begin
               wba <= {pkt[RW_BAH][5:0], pkt[RW_BAL]};
               wbc <= bc; wbl <= lbn; xdone <= 32'd0; nxm_pend <= 1'b0;
               st <= S_RWSECT;
            end
         end
         S_RWSECT: if (!eng_active) begin        // sdspi buffer is free
            k <= 10'd0;
            if (!blk_present_i) begin
               end_flg <= 8'd0; end_sts <= ST_OFL | SB_OFL_NV; st <= S_RWEND;
            end else if (wbc == 32'd0) begin
               end_flg <= 8'd0; end_sts <= ST_SUC; st <= S_RWEND;
            end else if (is_wrop && blk_wp_i) begin
               end_flg <= 8'd0; end_sts <= ST_WPR | SB_WPR_HW; st <= S_RWEND;
            end else if (is_wrop && ((uf & UF_WPS) != 16'd0)) begin
               end_flg <= 8'd0; end_sts <= ST_WPR | SB_WPR_SW; st <= S_RWEND;
            end else if (opc == OP_WR)  st <= S_H2B;
            else if (opc == OP_ERS)     st <= S_PAD;
            else begin
               eng_wr <= 1'b0; eng_lba <= wbl; eng_base <= unit_base; eng_start <= 1'b1;
               st <= S_RWEW;
            end
         end
         S_RWEW: if (eng_done) begin
            if (nxm_pend) begin
               nxm_pend <= 1'b0; st <= S_RWEND;
            end else if (eng_err) begin
               end_flg <= EF_LOG; end_sts <= ST_DRV; st <= S_RWEND;
            end else begin
               k <= 10'd0;
               case (opc)
               OP_RD:   st <= S_B2HA;
               OP_CMP:  st <= S_CMPA;
               default: st <= S_RWADV;
               endcase
            end
         end
         // ---- sdspi buffer -> host (READ); 2 clocks after address change
         S_B2HA:  begin sdbuf_addr <= k[7:0]; st <= S_B2HL1; end
         S_B2HL1: st <= S_B2HL2;
         S_B2HL2: st <= S_B2HW;
         S_B2HW: begin
            if (bus_can) bus_wr(wba + {11'd0, k, 1'b0}, sdbuf_dataout);
            else if (bus_done) begin
               if (bus_err) begin
                  xdone <= xdone + {21'd0, k, 1'b0};
                  end_flg <= EF_LOG; end_sts <= ST_HST | SB_HST_NXM; st <= S_RWEND;
               end else if (k + 10'd1 == {1'b0,cwords}) st <= S_RWADV;
               else begin k <= k + 10'd1; st <= S_B2HA; end
            end
         end
         // ---- host -> sdspi buffer (WRITE)
         S_H2B: begin
            if (bus_can) bus_rd(wba + {11'd0, k, 1'b0});
            else if (bus_done) begin
               if (bus_err) begin
                  xdone <= xdone + {21'd0, k, 1'b0};
                  end_flg <= EF_LOG; end_sts <= ST_HST | SB_HST_NXM;
                  if (k != 10'd0) begin
                     nxm_pend <= 1'b1;
                     st <= S_PAD;
                  end else st <= S_RWEND;
               end else begin
                  sdbuf_we <= 1'b1; sdbuf_addr <= k[7:0]; sdbuf_datain <= dma_dat_i;
                  k <= k + 10'd1;
                  if (k + 10'd1 == {1'b0,cwords}) st <= S_PAD;
               end
            end
         end
         S_PAD: begin                            // zero-fill k..255
            if (k[8]) st <= S_ENGGO;
            else begin
               sdbuf_we <= 1'b1; sdbuf_addr <= k[7:0]; sdbuf_datain <= 16'd0;
               k <= k + 10'd1;
            end
         end
         S_ENGGO: begin
            eng_wr <= 1'b1; eng_lba <= wbl; eng_base <= unit_base; eng_start <= 1'b1;
            st <= S_RWEW;
         end
         // ---- compare (CMP)
         S_CMPA:  begin sdbuf_addr <= k[7:0]; st <= S_CMPL1; end
         S_CMPL1: st <= S_CMPL2;
         S_CMPL2: st <= S_CMPR;
         S_CMPR: begin
            if (bus_can) bus_rd(wba + {11'd0, k, 1'b0});
            else if (bus_done) begin
               if (bus_err) begin
                  xdone <= xdone + {21'd0, k, 1'b0};
                  end_flg <= EF_LOG; end_sts <= ST_HST | SB_HST_NXM; st <= S_RWEND;
               end else if (dma_dat_i[7:0] != sdbuf_dataout[7:0]) begin
                  xdone <= xdone + {21'd0, k, 1'b0};
                  end_flg <= 8'd0; end_sts <= ST_CMP; st <= S_RWEND;
               end else if (dma_dat_i[15:8] != sdbuf_dataout[15:8]) begin
                  xdone <= xdone + {21'd0, k, 1'b1};
                  end_flg <= 8'd0; end_sts <= ST_CMP; st <= S_RWEND;
               end else if (k + 10'd1 == {1'b0,cwords}) st <= S_RWADV;
               else begin k <= k + 10'd1; st <= S_CMPA; end
            end
         end
         S_RWADV: begin
            wba   <= wba + {12'd0, chunk};
            wbc   <= wbc - {22'd0, chunk};
            xdone <= xdone + {22'd0, chunk};
            wbl   <= wbl + 32'd1;
            st    <= S_RWSECT;
         end
         S_RWEND: begin
            pkt[RW_BCL] <= xdone[15:0]; pkt[RW_BCH] <= xdone[31:16];
            putr(opc | OP_END, end_flg, end_sts, RW_LNT_D);
            st <= S_RD0;
         end

         // ------------------------------------------------- send response
         S_RD0: begin                             // high word (OWN) first
            if (bus_can) bus_rd(rq_ba + {13'd0, rq_idx} + 22'd2);
            else if (bus_done) begin
               if (bus_err) fatal(PE_QRE);
               else begin
                  desc[31:16] <= dma_dat_i;
                  if (!dma_dat_i[15]) begin            // no buffer: release bus, retry later
                     retry <= 16'd0; st <= S_RWAIT;
                  end else st <= S_RD1;
               end
            end
         end
         S_RD1: begin
            if (bus_can) bus_rd(rq_ba + {13'd0, rq_idx});
            else if (bus_done) begin
               if (bus_err) fatal(PE_QRE);
               else begin
                  desc[15:0] <= dma_dat_i;
                  paddr <= {desc[21:16], dma_dat_i[15:1], 1'b0} - 22'd4;
                  pkt[UQ_HCTC][3:0] <= ((credits >= 4'd14) ? 4'd14 : credits) + 4'd1;
                  credits <= credits - ((credits >= 4'd14) ? 4'd14 : credits);
                  k <= 10'd0; st <= S_RPKT;
               end
            end
         end
         S_RWAIT: begin
            retry <= retry + 16'd1;
            if (retry == RSP_RETRY[15:0]) st <= S_RD0;
         end
         S_RPKT: begin
            if (bus_can) bus_wr(paddr + {11'd0, k, 1'b0}, pkt[k[4:0]]);
            else if (bus_done) begin
               if (bus_err) fatal(PE_PWE);
               else if (k + 10'd1 == ({1'b0,pkt[UQ_HLNT][9:1]} + 10'd2)) putdesc(1'b1, S_IDLE);
               else k <= k + 10'd1;
            end
         end

         default: st <= S_IDLE;
         endcase

         // ------------------------------------------------- register access (rq_rd / rq_wr)
         if (bus_read_req) begin
            if (!wb_adr_i[1]) begin
               wb_dat_o <= 16'd0;                          // IP reads zero
               if (csta == CST_S3_PPB) st4_pend <= 1'b1;
               else if (csta == CST_UP) pip <= 1'b1;      // poll
            end else wb_dat_o <= sa;
         end
         if (bus_write_req) begin
            if (!wb_adr_i[1]) rst_pend <= 1'b1;            // IP write: initialize
            else begin
               saw <= wb_dat_i;
               if (csta <= CST_S4) saw_pend <= 1'b1;
            end
         end
      end
   end

endmodule
`default_nettype wire
