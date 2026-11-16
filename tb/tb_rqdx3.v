// tb_rqdx3.v -- testbench for the project-integrated rqdx3.v
// DUT uses the REAL sdspi.v (with a sectorbuf model) talking SPI to an SD card
// model; DMA through dma_req/dma_gnt; interrupt through irq/iack.
`timescale 1ns/1ps
`default_nettype none
module tb;
    reg clk = 0;   always #5  clk = ~clk;        // bus clock 100 MHz
    reg sdclk = 0; always #10 sdclk = ~sdclk;    // SD clock 50 MHz
    integer errors = 0;
    reg rst = 1;
    localparam OFFSET = 27'h0000000;             // bank 0: DU0 starts at block 0
    localparam STRIDE = 27'h50000;               // DU slot size
    localparam NB = 4103;                        // model blocks per unit

    // ---------------- DUT ----------------
    reg  [1:0]  s_adr; reg [15:0] s_dat_w; wire [15:0] s_dat_r;
    reg         s_we, s_cyc, s_stb; wire s_ack;
    wire        irq; wire [8:0] vec; reg iack = 0;
    wire        dma_req; reg dma_gnt = 0;
    wire [21:0] dma_adr; wire [15:0] dma_dat_o; reg [15:0] dma_dat_i;
    wire        dma_stb, dma_we; reg dma_ack = 0;
    wire        sd_cs, sd_mosi, sd_sclk, sd_miso; wire sdreq; reg sdack = 0;
    wire [3:0]  sd_dbg;

    rqdx3 #(.UNITS(4), .STRIDE(STRIDE)) dut (
        .wb_clk_i(clk), .wb_rst_i(rst), .wb_adr_i(s_adr), .wb_dat_i(s_dat_w), .wb_dat_o(s_dat_r),
        .wb_cyc_i(s_cyc), .wb_we_i(s_we), .wb_stb_i(s_stb), .wb_sel_i(2'b11), .wb_ack_o(s_ack),
        .irq(irq), .iack(iack), .irq_vec(vec),
        .dma_req(dma_req), .dma_gnt(dma_gnt), .dma_adr_o(dma_adr), .dma_dat_i(dma_dat_i),
        .dma_dat_o(dma_dat_o), .dma_stb_o(dma_stb), .dma_we_o(dma_we), .dma_ack_i(dma_ack),
        .sdcard_cs(sd_cs), .sdcard_mosi(sd_mosi), .sdcard_sclk(sd_sclk), .sdcard_miso(sd_miso),
        .sdreq(sdreq), .sdack(sdack), .sdmode(1'b0), .sdclock(sdclk),
        .start_offset(OFFSET), .sdcard_debug(sd_dbg));

    sdcard_model #(.BASE(OFFSET), .UNITS(4), .STRIDE(STRIDE)) card (.sclk(sd_sclk), .cs(sd_cs), .mosi(sd_mosi), .miso(sd_miso));

    // ---------------- card arbiter: grant after a few clocks ----------------
    always @(posedge clk) sdack <= sdreq & ($random & 1 ? 1'b1 : sdack);

    // ---------------- DMA arbiter + host memory 256 KB (above: no reply = NXM) ----
    localparam MEMW = 131072;
    reg [15:0] mem [0:MEMW-1];
    integer lat = 0;
    always @(posedge clk) begin
        // grant follows request with a random delay, dropped with the request
        if (!dma_req) dma_gnt <= 0; else if ($random & 1) dma_gnt <= 1;
        dma_ack <= 0;
        if (dma_stb && !dma_ack) begin
            if (!dma_gnt) begin $display("FAIL: dma_stb without grant"); errors = errors + 1; end
            if (lat > 0) lat <= lat - 1;
            else begin
                lat <= ($random & 3);
                if (dma_adr < 22'h040000) begin
                    dma_ack <= 1;
                    if (dma_we) mem[dma_adr[21:1]] <= dma_dat_o;
                    else dma_dat_i <= mem[dma_adr[21:1]];
                end
            end
        end
    end

    // ---------------- interrupt acknowledge (CPU) ----------------
    integer nirq = 0;
    always @(posedge clk) begin
        if (irq && !iack) begin
            if (vec !== 9'o524) begin $display("FAIL: vector %o", vec); errors = errors + 1; end
            nirq = nirq + 1; iack <= 1;
            if ($test$plusargs("irqdbg")) $display("  [irq t=%0t st=%0d csta=%0d]", $time, dut.st, dut.csta);
        end else if (iack && !irq) iack <= 0;
    end

    // memory views used below
    function [7:0] dsk(input integer a); dsk = card.img[a]; endfunction
    // ---------------- host register access ----------------
    task wb_wr(input integer a, input [15:0] d);
        begin
            @(posedge clk); s_adr <= a[1:0]; s_dat_w <= d; s_we <= 1; s_cyc <= 1; s_stb <= 1;
            @(posedge clk); while (!s_ack) @(posedge clk);
            s_cyc <= 0; s_stb <= 0; s_we <= 0;
        end
    endtask
    task wb_rd(input integer a, output [15:0] d);
        begin
            @(posedge clk); s_adr <= a[1:0]; s_we <= 0; s_cyc <= 1; s_stb <= 1;
            @(posedge clk); while (!s_ack) @(posedge clk);
            d = s_dat_r; s_cyc <= 0; s_stb <= 0;
        end
    endtask
    task check(input [255:0] what, input [31:0] got, input [31:0] exp);
        begin
            if (got !== exp) begin
                $display("FAIL: %0s  got %h  expected %h", what, got, exp); errors = errors + 1;
            end
        end
    endtask
    task wait_sa(input [15:0] mask, output [15:0] v);
        integer n;
        begin
            n = 0; v = 0;
            while (((v & mask) == 0) && n < 2000) begin wb_rd(2, v); n = n + 1; end
        end
    endtask

    // ---------------- ring layout ----------------
    localparam COMM = 32'h001000;   // rsp ring 4 x 4 bytes, then cmd ring 4 x 4
    localparam CMDP = 32'h002000;   // command packet (header at -4)
    localparam RSPP = 32'h002100;   // response packet (header at -4)
    integer ci, ri, j;
    reg [15:0] r;

    task mw(input [31:0] a, input [15:0] d); mem[a[21:1]] = d; endtask
    function [15:0] mr(input [31:0] a); mr = mem[a[21:1]]; endfunction

    // command packet word i (pkt d[] index, d[0] at CMDP-4)
    task cw(input integer i, input [15:0] d); mw(CMDP - 4 + 2*i, d); endtask
    function [15:0] rw(input integer i); rw = mr(RSPP - 4 + 2*i); endfunction

    task new_cmd(input [7:0] opc, input [15:0] unit, input [15:0] mod, input [15:0] ref_no);
        begin
            for (j = 0; j < 32; j = j + 1) cw(j, 16'd0);
            cw(0, 16'd36);               // message length (bytes)
            cw(1, 16'h0000);             // seq, MSCP connection
            cw(2, ref_no); cw(3, 16'd0);
            cw(4, unit); cw(6, {8'd0, opc}); cw(7, mod);
            for (j = 0; j < 32; j = j + 1) mw(RSPP - 4 + 2*j, 16'hDEAD);
        end
    endtask

    // issue: give one response buffer and one command to the controller
    task issue;
        integer n;
        begin
            mw(COMM + ri*4, RSPP[15:0]); mw(COMM + ri*4 + 2, 16'hC000 | RSPP[21:16]);
            mw(COMM + 16 + ci*4, CMDP[15:0]); mw(COMM + 16 + ci*4 + 2, 16'hC000 | CMDP[21:16]);
            wb_rd(0, r);                 // IP read: start polling
            n = 0;
            while ((mr(COMM + ri*4 + 2) & 16'h8000) && n < 400000) begin @(posedge clk); n = n + 1; end
            if (n >= 400000) begin $display("FAIL: response timeout st=%0d csta=%0d pip=%0d req=%b gnt=%b stb=%b sa=%h", dut.st, dut.csta, dut.pip, dma_req, dma_gnt, dma_stb, dut.sa); errors = errors + 1; end
            // controller returned both descriptors with OWN=0, F=1
            check("cmd desc returned", mr(COMM + 16 + ci*4 + 2) & 16'hC000, 16'h4000);
            check("rsp desc returned", mr(COMM + ri*4 + 2) & 16'hC000, 16'h4000);
            ci = (ci + 1) % 4; ri = (ri + 1) % 4;
            repeat (20) @(posedge clk);
        end
    endtask

    task show_rsp(input [127:0] name);
        integer n;
        begin
            n = rw(0) / 2 + 2;
            $write("  %-10s len=%0d hctc=%h opf=%h sts=%h |", name, rw(0), rw(1), rw(6), rw(7));
            for (j = 8; j < n; j = j + 1) $write(" %h", rw(j));
            $write("\n");
            $write("@RSP %0s", name);
            for (j = 0; j < n; j = j + 1) $write(" %h", rw(j));
            $write("\n");
        end
    endtask

    task rsp_common(input [127:0] name, input [15:0] opf, input [15:0] sts, input [15:0] lnt);
        begin
            show_rsp(name);
            check({name, " opf"}, rw(6), opf);
            check({name, " sts"}, rw(7), sts);
            check({name, " len"}, rw(0), lnt);
            check({name, " ref"}, rw(2), 16'h1234);
        end
    endtask

    task rwcmd(input [7:0] opc, input [31:0] bcnt, input [31:0] ba, input [31:0] lb);
        rwcmdu(opc, 0, bcnt, ba, lb);
    endtask
    task rwcmdu(input [7:0] opc, input [15:0] unit, input [31:0] bcnt, input [31:0] ba, input [31:0] lb);
        begin
            new_cmd(opc, unit, 0, 16'h1234);
            cw(8, bcnt[15:0]); cw(9, bcnt[31:16]);
            cw(10, ba[15:0]);  cw(11, ba[31:16]);
            cw(16, lb[15:0]);  cw(17, lb[31:16]);
            issue;
        end
    endtask

    task do_init(input wrap_test, input pp_test);
        begin
            wb_wr(0, 16'h0000);                          // IP write: init
            wb_rd(2, r); check("S1 SA", r, 16'h0B40);    // S1|Q22|DI|MP
            if (wrap_test) begin
                wb_wr(2, 16'hC0A5);                      // VL|WR: wrap mode
                repeat (5) @(posedge clk);
                wb_rd(2, r); check("wrap echo 1", r, 16'hC0A5);
                wb_wr(2, 16'h5A5A); repeat (5) @(posedge clk);
                wb_rd(2, r); check("wrap echo 2", r, 16'h5A5A);
                wb_wr(0, 16'h0000);
                wb_rd(2, r); check("S1 SA after wrap", r, 16'h0B40);
            end
            // step 1: VL, cmd ring 2^2, rsp ring 2^2, IE, vector 0524/4
            wb_wr(2, 16'h8000 | (2 << 11) | (2 << 8) | 16'h0080 | (9'o524 >> 2));
            wait_sa(16'h1000, r); check("S2 SA", r, 16'h1000 | 16'h0092);  // EC = s1dat<15:8>
            wb_wr(2, COMM[15:0] | (pp_test ? 1 : 0));    // step 2: comm low, PI
            wait_sa(16'h2000, r); check("S3 SA", r, 16'h2000 | 16'h00D5);  // EC = s1dat<7:0>
            for (j = -8; j < 32; j = j + 2) mw(COMM + j, 16'hFFFF);        // garbage in comm area
            if (pp_test) begin
                wb_wr(2, 16'h8000 | COMM[21:16]);        // step 3 with purge/poll test
                repeat (5) @(posedge clk);
                wb_rd(2, r); check("PP SA = 0", r, 16'h0000);
                wb_wr(2, 16'h0000);                      // host writes 0
                repeat (5) @(posedge clk);
                wb_rd(0, r);                             // then reads IP -> step 4
            end else
                wb_wr(2, COMM[21:16]);                   // step 3: comm high
            wait_sa(16'h4000, r); check("S4 SA", r, 16'h4133);  // S4 | (19<<4) | 3
            for (j = (pp_test ? -8 : -4); j < 32; j = j + 2) check("comm zeroed", mr(COMM + j), 0);
            if (!pp_test) check("comm-8 untouched", mr(COMM - 8), 16'hFFFF);
            wb_wr(2, 16'h0001);                          // step 4: GO
            repeat (5) @(posedge clk);
            wb_rd(2, r); check("UP SA", r, 16'h0000);
            ci = 0; ri = 0;
        end
    endtask

    integer a, b, errs_before;
    initial begin
        s_cyc = 0; s_stb = 0; s_we = 0; s_adr = 0; s_dat_w = 0;
        for (a = 0; a < MEMW; a = a + 1) mem[a] = 16'h0;
        for (a = 0; a < 4*NB*512; a = a + 1)
            card.img[a] = ((a % (NB*512)) * 7 + ((a % (NB*512)) >> 9) + 37 * (a / (NB*512))) & 8'hFF;
        repeat (5) @(posedge clk); rst <= 0;

        $display("== init: wrap mode + purge/poll test ==");
        do_init(1, 1);
        check("init irqs (S2,S3,S4)", nirq, 3);

        $display("== init: normal ==");
        do_init(0, 0);

        $display("== port-level commands ==");
        new_cmd(OP(4), 0, 0, 16'h1234);  cw(8, 0); cw(9, 16'h00E0); cw(10, 16'd30); issue;   // SCC (CF_THS off: no error-log datagrams)
        rsp_common("SCC", 8'h84, 16'h0000, 16'd32);
        check("SCC hctc (credits 15)", rw(1), 16'h000F);
        check("SCC cflgs", rw(9), 16'h80E0); check("SCC tmo", rw(10), 120);
        check("SCC ver", rw(11), 16'h0103); check("SCC cidd", rw(15), 16'h0113);

        new_cmd(OP(3), 0, 0, 16'h1234); issue;                                    // GUS
        rsp_common("GUS", 8'h83, 16'h0004, 16'd48);                               // ST_AVL
        check("GUS hctc (credits 2)", rw(1), 16'h0002);
        check("GUS ufl", rw(9), 16'h8000); check("GUS uidd", rw(15), 16'h020D);
        check("GUS medl", rw(16), 16'h4036); check("GUS medh", rw(17), 16'h2564);
        check("GUS trk", rw(20), 17); check("GUS grp", rw(21), 15); check("GUS cyl", rw(22), 1);
        check("GUS rcts", rw(24), 7); check("GUS rbsc", rw(25), 16'h0101);

        new_cmd(OP(9), 0, 0, 16'h1234); issue;                                    // ONL
        rsp_common("ONL", 8'h89, 16'h0000, 16'd44);
        check("ONL hctc (credits 1)", rw(1), 16'h0001);
        check("ONL sizl", rw(20), 16'hBFA0); check("ONL sizh", rw(21), 16'h0004);  // 311200
        check("ONL vsnl", rw(22), 16'o1234);
        new_cmd(OP(9), 0, 0, 16'h1234); issue;
        rsp_common("ONL again", 8'h89, 16'h0100, 16'd44);                         // SB_SUC_ON
        new_cmd(OP(9), 4, 0, 16'h1234); issue;
        rsp_common("ONL unit4", 8'h89, 16'h0003, 16'd44);                         // no unit 4                         // ST_OFL
        new_cmd(OP(5), 0, 0, 16'h1234); issue;
        rsp_common("bad opc", 8'h80, 16'h0801, 16'd12);                           // ST_CMD|I_OPCD
        new_cmd(OP(4), 0, 0, 16'h1234); cw(8, 1); issue;
        rsp_common("SCC v1", 8'h80, 16'h0C01, 16'd32);                            // I_VRSN
        new_cmd(OP(19), 0, 0, 16'h1234); issue;
        rsp_common("FLU", 8'h93, 16'h0000, 16'd12);
        new_cmd(OP(47), 0, 0, 16'h1234); issue;
        rsp_common("FMT", 8'hAF, 16'h0801, 16'd12);
        new_cmd(OP(2), 0, 0, 16'h1234); issue;
        rsp_common("GCS", 8'h82, 16'h0000, 16'd20);

        $display("== data transfers ==");
        for (a = 0; a < 1024; a = a + 2) mw(32'h4000 + a, (a * 16'h0101) ^ 16'h5AC3);
        rwcmd(OP(34), 1024, 32'h4000, 100);                                       // WR
        rsp_common("WR 1024", 8'hA2, 16'h0000, 16'd32); check("WR bc", rw(8), 1024);
        b = 0;
        for (a = 0; a < 1024; a = a + 1)
            if (card.img[100*512 + a] !== mem[(32'h4000 + a) >> 1][(a & 1) ? 15 : 7 -: 8]) b = b + 1;
        check("WR disk content", b, 0);

        for (a = 0; a < 2048; a = a + 2) mw(32'h8000 + a, 16'hEEEE);
        rwcmd(OP(33), 1024, 32'h8000, 100);                                       // RD
        rsp_common("RD 1024", 8'hA1, 16'h0000, 16'd32); check("RD bc", rw(8), 1024);
        b = 0; for (a = 0; a < 1024; a = a + 2) if (mr(32'h8000 + a) !== mr(32'h4000 + a)) b = b + 1;
        check("RD data", b, 0); check("RD no overrun", mr(32'h8000 + 1024), 16'hEEEE);

        for (a = 0; a < 1024; a = a + 2) mw(32'hA000 + a, 16'hEEEE);
        rwcmd(OP(33), 700, 32'hA000, 100);                                        // partial RD
        rsp_common("RD 700", 8'hA1, 16'h0000, 16'd32); check("RD700 bc", rw(8), 700);
        b = 0; for (a = 0; a < 700; a = a + 2) if (mr(32'hA000 + a) !== mr(32'h4000 + a)) b = b + 1;
        check("RD700 data", b, 0); check("RD700 stop", mr(32'hA000 + 700), 16'hEEEE);

        rwcmd(OP(34), 700, 32'h4000, 200);                                        // partial WR
        rsp_common("WR 700", 8'hA2, 16'h0000, 16'd32);
        b = 0; for (a = 700 - 512; a < 512; a = a + 1) if (card.img[201*512 + a] !== 0) b = b + 1;
        check("WR700 zero pad", b, 0);

        rwcmd(OP(32), 1024, 32'h4000, 100);                                       // CMP equal
        rsp_common("CMP eq", 8'hA0, 16'h0000, 16'd32); check("CMP bc", rw(8), 1024);
        mw(32'h4000 + 514, mr(32'h4000 + 514) ^ 16'h0100);                        // byte 515 differs
        rwcmd(OP(32), 1024, 32'h4000, 100);
        rsp_common("CMP ne", 8'hA0, 16'h0007, 16'd32); check("CMP ne bc", rw(8), 515);

        rwcmd(OP(18), 512, 32'h0, 300);                                           // ERS
        rsp_common("ERS", 8'h92, 16'h0000, 16'd32);
        b = 0; for (a = 0; a < 512; a = a + 1) if (card.img[300*512 + a] !== 0) b = b + 1;
        check("ERS zeros", b, 0);

        rwcmd(OP(16), 1024, 32'h1, 100);                                          // ACC, odd BA ok
        rsp_common("ACC", 8'h90, 16'h0000, 16'd32);

        rwcmd(OP(33), 512, 32'h3FFC0, 100);                                       // NXM after 64 bytes
        rsp_common("RD NXM", 16'h20A1, 16'h0069, 16'd32); check("RD NXM bc", rw(8), 64);
        rwcmd(OP(34), 512, 32'h3FFC0, 400);                                       // WR NXM
        rsp_common("WR NXM", 16'h20A2, 16'h0069, 16'd32); check("WR NXM bc", rw(8), 64);
        b = 0; for (a = 64; a < 512; a = a + 1) if (card.img[400*512 + a] !== 0) b = b + 1;
        check("WR NXM partial sector written+padded", b, 0);

        if (!$test$plusargs("xchk")) begin                                        // no fault injection in simh
            card.fail_next = 4; rwcmd(OP(33), 1024, 32'h8000, 100);                        // drive error
            rsp_common("RD DRV", 16'h20A1, 16'h000B, 16'd32); check("RD DRV bc", rw(8), 0);
        end

        rwcmd(OP(33), 513, 32'h8000, 100);
        rsp_common("odd bc", 8'hA1, 16'h0049, 16'd32); check("odd bc bc", rw(8), 0);
        rwcmd(OP(33), 512, 32'h8001, 100);
        rsp_common("odd ba", 8'hA1, 16'h0029, 16'd32);
        rwcmd(OP(33), 1024, 32'h8000, 311199);
        rsp_common("cross end", 8'hA1, 16'h0C01, 16'd32);
        rwcmd(OP(33), 512, 32'h8000, 311207);
        rsp_common("past RCT", 8'hA1, 16'h1C01, 16'd32);
        rwcmd(OP(33), 1024, 32'h8000, 311200);
        rsp_common("RCT 1024", 8'hA1, 16'h0C01, 16'd32);
        rwcmd(OP(33), 512, 32'h8000, 311200);
        rsp_common("RCT read", 8'hA1, 16'h0000, 16'd32);
        b = 0; for (a = 0; a < 512; a = a + 2)
            if (mr(32'h8000 + a) !== {card.img[4096*512 + a + 1], card.img[4096*512 + a]}) b = b + 1;
        check("RCT data", b, 0);
        rwcmd(OP(34), 512, 32'h8000, 311200);
        rsp_common("RCT write", 8'hA2, 16'h1C01, 16'd32);
        rwcmd(OP(33), 0, 32'h8000, 5);
        rsp_common("RD bc=0", 8'hA1, 16'h0000, 16'd32);

        new_cmd(OP(10), 0, 16'h0004, 16'h1234); cw(9, 16'h1000); issue;           // SUC: set WPS
        rsp_common("SUC WPS", 8'h8A, 16'h0000, 16'd44); check("SUC ufl", rw(9), 16'h9000);
        rwcmd(OP(34), 512, 32'h4000, 10);
        rsp_common("WR swlock", 8'hA2, 16'h1006, 16'd32);
        new_cmd(OP(10), 0, 16'h0000, 16'h1234); issue;
        rsp_common("SUC clr", 8'h8A, 16'h0000, 16'd44); check("SUC ufl2", rw(9), 16'h8000);

        new_cmd(OP(8), 0, 0, 16'h1234); issue;
        rsp_common("AVL", 8'h88, 16'h0000, 16'd12);
        rwcmd(OP(33), 512, 32'h8000, 100);
        rsp_common("RD avail", 8'hA1, 16'h0004, 16'd32);


        $display("== multiple units ==");
        new_cmd(OP(9), 0, 0, 16'h1234); issue;
        rsp_common("ONL u0", 8'h89, 16'h0000, 16'd44);
        new_cmd(OP(3), 3, 0, 16'h1234); issue;
        rsp_common("GUS u3", 8'h83, 16'h0004, 16'd48);                            // exists, available
        check("GUS u3 mlun", rw(8), 3); check("GUS u3 uida", rw(12), 3); check("GUS u3 shun", rw(18), 3);
        new_cmd(OP(3), 5, 16'h0001, 16'h1234); issue;                              // NXU past last unit
        rsp_common("GUS nxu5", 8'h83, 16'h0000, 16'd48);                          // unit 0 is online
        check("GUS nxu5 unit", rw(4), 0); check("GUS nxu5 mlun", rw(8), 0);
        new_cmd(OP(9), 2, 0, 16'h1234); issue;
        rsp_common("ONL u2", 8'h89, 16'h0000, 16'd44);
        check("ONL u2 vsn", rw(22), 16'o1236); check("ONL u2 uida", rw(12), 2);
        for (a = 0; a < 1024; a = a + 2) mw(32'h6000 + a, 16'hC000 + a);
        rwcmdu(OP(34), 2, 1024, 32'h6000, 100);
        rsp_common("WR u2", 8'hA2, 16'h0000, 16'd32);
        b = 0; for (a = 0; a < 1024; a = a + 2)
            if ({card.img[2*NB*512 + 100*512 + a + 1], card.img[2*NB*512 + 100*512 + a]} !== mr(32'h6000 + a)) b = b + 1;
        check("WR u2 landed in DU2 slot", b, 0);
        rwcmdu(OP(33), 0, 1024, 32'hC000, 100);                                   // DU0 LBN 100 unchanged
        rsp_common("RD u0", 8'hA1, 16'h0000, 16'd32);
        b = 0; for (a = 0; a < 1024; a = a + 2)
            if (mr(32'hC000 + a) !== {card.img[100*512 + a + 1], card.img[100*512 + a]}) b = b + 1;
        check("RD u0 unaffected", b, 0);
        rwcmdu(OP(33), 2, 1024, 32'hC000, 100);
        rsp_common("RD u2", 8'hA1, 16'h0000, 16'd32);
        b = 0; for (a = 0; a < 1024; a = a + 2) if (mr(32'hC000 + a) !== mr(32'h6000 + a)) b = b + 1;
        check("RD u2 data", b, 0);
        rwcmdu(OP(33), 1, 512, 32'hC000, 100);
        rsp_common("RD u1 avl", 8'hA1, 16'h0004, 16'd32);                        // not online
        new_cmd(OP(8), 2, 0, 16'h1234); issue;
        rsp_common("AVL u2", 8'h88, 16'h0000, 16'd12);
        new_cmd(OP(3), 2, 0, 16'h1234); issue;
        rsp_common("GUS u2", 8'h83, 16'h0004, 16'd48);
        new_cmd(OP(8), 0, 0, 16'h1234); issue;
        rsp_common("AVL u0", 8'h88, 16'h0000, 16'd12);

        $display("== ring full: response buffer supplied late ==");
        new_cmd(OP(3), 0, 0, 16'h1234);
        mw(COMM + 16 + ci*4, CMDP[15:0]); mw(COMM + 16 + ci*4 + 2, 16'hC000 | CMDP[21:16]);
        wb_rd(0, r);
        repeat (3000) @(posedge clk);
        check("cmd taken", mr(COMM + 16 + ci*4 + 2) & 16'h8000, 0);
        mw(COMM + ri*4, RSPP[15:0]); mw(COMM + ri*4 + 2, 16'hC000 | RSPP[21:16]);
        a = 0; while ((mr(COMM + ri*4 + 2) & 16'h8000) && a < 100000) begin @(posedge clk); a = a + 1; end
        repeat (50) @(posedge clk);          // let putdesc finish its prev-descriptor check
        ci = (ci + 1) % 4; ri = (ri + 1) % 4;
        rsp_common("GUS late", 8'h83, 16'h0004, 16'd48);

        $display("== ring interrupt: all response descriptors owned by port ==");
        for (a = 0; a < 4; a = a + 1) begin
            mw(COMM + a*4, RSPP[15:0]); mw(COMM + a*4 + 2, 16'hC000 | RSPP[21:16]);
        end
        mw(COMM - 2, 16'h0000); b = nirq;
        new_cmd(OP(3), 0, 0, 16'h1234);
        mw(COMM + 16 + ci*4, CMDP[15:0]); mw(COMM + 16 + ci*4 + 2, 16'hC000 | CMDP[21:16]);
        wb_rd(0, r);
        a = 0; while ((mr(COMM + ri*4 + 2) & 16'h8000) && a < 100000) begin @(posedge clk); a = a + 1; end
        repeat (50) @(posedge clk);
        check("rsp ring flag word (comm-2)", mr(COMM - 2), 1);
        check("ring interrupt taken", nirq - b, 1);
        ci = (ci + 1) % 4; ri = (ri + 1) % 4;
        rsp_common("GUS ringint", 16'h0083, 16'h0004, 16'd48);
        for (a = 0; a < 4; a = a + 1) mw(COMM + a*4 + 2, 16'h4000 | RSPP[21:16]);  // take them back

        $display("== fatal: invalid connection id ==");
        new_cmd(OP(3), 0, 0, 16'h1234); cw(1, 16'h0500);
        mw(COMM + 16 + ci*4, CMDP[15:0]); mw(COMM + 16 + ci*4 + 2, 16'hC000 | CMDP[21:16]);
        wb_rd(0, r); repeat (2000) @(posedge clk);
        wb_rd(2, r); check("fatal SA (PE_ICI)", r, 16'h800E);

        $display("== re-init after fatal, then READ ==");
        do_init(0, 0);
        new_cmd(OP(9), 0, 0, 16'h1234); issue; rsp_common("ONL", 8'h89, 16'h0000, 16'd44);
        rwcmd(OP(33), 1024, 32'h8000, 100); rsp_common("RD", 8'hA1, 16'h0000, 16'd32);

        // final state dump for the simh cross-check
        $write("@MEM 3ffc0"); for (a = 32'h3FFC0; a < 32'h40000; a = a + 2) $write(" %h", mr(a)); $write("\n");
        $write("@MEM 0a000"); for (a = 32'hA000; a < 32'hA2C0; a = a + 2) $write(" %h", mr(a)); $write("\n");
        $write("@MEM 08000"); for (a = 32'h8000; a < 32'h8402; a = a + 2) $write(" %h", mr(a)); $write("\n");
        $write("@DISK");
        for (a = 0; a < 512; a = a + 1) $write(" %h", card.img[100*512+a]);
        for (a = 0; a < 1024; a = a + 1) $write(" %h", card.img[200*512+a]);
        for (a = 0; a < 512; a = a + 1) $write(" %h", card.img[300*512+a]);
        for (a = 0; a < 512; a = a + 1) $write(" %h", card.img[400*512+a]);
        for (a = 0; a < 1024; a = a + 1) $write(" %h", card.img[2*NB*512 + 100*512+a]);
        $write("\n");
        $display("interrupts taken: %0d", nirq);
        if (errors == 0) $display("ALL TESTS PASSED"); else $display("%0d FAILURES", errors);
        $finish;
    end

    function [7:0] OP(input integer x); OP = x; endfunction

    initial begin #300_000_000; $display("FAIL: global timeout"); $finish; end
endmodule
`default_nettype wire
