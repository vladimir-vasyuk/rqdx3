// rqdx3_lite_defs.vh -- constants transcribed from open-simh
//   PDP11/pdp11_uqssp.h, PDP11/pdp11_mscp.h, PDP11/pdp11_rq.c
// Included inside module rqdx3

// ---------------- UQSSP (pdp11_uqssp.h) ----------------
localparam [15:0] SA_ER       = 16'h8000;
localparam [15:0] SA_S4       = 16'h4000;
localparam [15:0] SA_S3       = 16'h2000;
localparam [15:0] SA_S2       = 16'h1000;
localparam [15:0] SA_S1       = 16'h0800;
localparam [15:0] SA_S1C_Q22  = 16'h0200;
localparam [15:0] SA_S1C_DI   = 16'h0100;
localparam [15:0] SA_S1C_MP   = 16'h0040;
localparam [15:0] SA_S1H_VL   = 16'h8000;
localparam [15:0] SA_S1H_WR   = 16'h4000;
localparam [15:0] SA_S2C_PT   = 16'h0000;
localparam [15:0] SA_S2H_CLO  = 16'hFFFE;
localparam [15:0] SA_S3H_PP   = 16'h8000;
localparam integer SA_S4C_V_MOD = 4;
localparam integer SA_S4C_V_VER = 0;
localparam [15:0] SA_S4H_GO   = 16'h0001;

localparam [15:0] PE_PRE = 16'd1,  PE_PWE = 16'd2,  PE_QRE = 16'd6,  PE_QWE = 16'd7;
localparam [15:0] PE_ICI = 16'd14, PE_PIE = 16'd20, PE_PPF = 16'd21;

localparam [31:0] UQ_DESC_OWN = 32'h8000_0000;
localparam [31:0] UQ_DESC_F   = 32'h4000_0000;

localparam integer UQ_HLNT = 0;           // packet word indices (header at -4)
localparam integer UQ_HCTC = 1;
localparam [3:0]  UQ_TYP_SEQ  = 4'd0;
localparam [7:0]  UQ_CID_MSCP = 8'd0;
localparam [7:0]  UQ_CID_DUP  = 8'd2;

// ---------------- controller state (pdp11_rq.c CST_*) ----------------
localparam [3:0] CST_S1 = 4'd0, CST_S1_WR = 4'd1, CST_S2 = 4'd2, CST_S3 = 4'd3,
                 CST_S3_PPA = 4'd4, CST_S3_PPB = 4'd5, CST_S4 = 4'd6,
                 CST_UP = 4'd7, CST_DEAD = 4'd8;

// ---------------- controller identity (pdp11_rq.c) ----------------
localparam [15:0] RQ_CLASS = 16'd1, RQ_HVER = 16'd1, RQ_SVER = 16'd3;
localparam [15:0] RQ_DCTMO = 16'd120;
localparam [15:0] RQDX3_UQPM = 16'd19, RQDX3_MODEL = 16'd19;

// ---------------- RD54 (pdp11_rq.c drv_tab) ----------------
localparam [15:0] RD54_SECT = 16'd17, RD54_TPG = 16'd15, RD54_GPC = 16'd1;
localparam [31:0] RD54_LBN  = 32'd311200;
localparam [31:0] RD54_RCTS = 32'd7;
localparam [15:0] RD54_MOD  = 16'd13;
localparam [31:0] RD54_MED  = 32'h2564_4036;
localparam [15:0] UID_DISK  = 16'd2;

// ---------------- MSCP opcodes / modifiers / flags (pdp11_mscp.h) ----------------
localparam [7:0] OP_ABO = 8'd1,  OP_GCS = 8'd2,  OP_GUS = 8'd3,  OP_SCC = 8'd4,
                 OP_AVL = 8'd8,  OP_ONL = 8'd9,  OP_SUC = 8'd10, OP_DAP = 8'd11,
                 OP_ACC = 8'd16, OP_CCD = 8'd17, OP_ERS = 8'd18, OP_FLU = 8'd19,
                 OP_CMP = 8'd32, OP_RD  = 8'd33, OP_WR  = 8'd34, OP_FMT = 8'd47,
                 OP_END = 8'h80;
localparam [15:0] MD_SWP = 16'h0004, MD_NXU = 16'h0001;
localparam [7:0]  EF_LOG = 8'h20;
localparam [15:0] CF_RPL = 16'h8000;
localparam [15:0] UF_RPL = 16'h8000, UF_WPH = 16'h2000, UF_WPS = 16'h1000;
localparam [15:0] UF_MSK = 16'h0003;      // UF_CMR | UF_CMW (pdp11_rq.c)

// status codes: ST_V_SUB = 5, ST_V_INV = 8
localparam [15:0] ST_SUC = 16'd0, ST_CMD = 16'd1, ST_OFL = 16'd3, ST_AVL = 16'd4,
                  ST_WPR = 16'd6, ST_CMP = 16'd7, ST_HST = 16'd9, ST_DRV = 16'd11;
localparam [15:0] SB_SUC_ON  = 16'd8   << 5;
localparam [15:0] SB_OFL_NV  = 16'd1   << 5;
localparam [15:0] SB_WPR_SW  = 16'd128 << 5;
localparam [15:0] SB_WPR_HW  = 16'd256 << 5;
localparam [15:0] SB_HST_OA  = 16'd1   << 5;
localparam [15:0] SB_HST_OC  = 16'd2   << 5;
localparam [15:0] SB_HST_NXM = 16'd3   << 5;
localparam [15:0] I_OPCD = 16'd8  << 8;
localparam [15:0] I_BCNT = 16'd12 << 8;
localparam [15:0] I_LBN  = 16'd28 << 8;
localparam [15:0] I_VRSN = 16'd12 << 8;

// packet lengths (bytes, excluding 4-byte header)
localparam [15:0] RSP_LNT = 16'd12, ABO_LNT = 16'd16, AVL_LNT = 16'd12,
                  GCS_LNT = 16'd20, FMT_LNT = 16'd12, GUS_LNT_D = 16'd48,
                  ONL_LNT = 16'd44, SCC_LNT = 16'd32, SUC_LNT = 16'd44,
                  RW_LNT_D = 16'd32;

// field word indices into pkt[]
localparam integer CMD_UN = 4, CMD_OPC = 6, CMD_MOD = 7;
localparam integer RSP_UN = 4, RSP_OPF = 6, RSP_STS = 7;
localparam integer GCS_STSL = 10, GCS_STSH = 11;
localparam integer ONL_MLUN = 8,  ONL_UFL = 9,  ONL_RSVL = 10, ONL_RSVH = 11,
                   ONL_UIDA = 12, ONL_UIDB = 13, ONL_UIDC = 14, ONL_UIDD = 15,
                   ONL_MEDL = 16, ONL_MEDH = 17, ONL_SHUN = 18, ONL_SHST = 19,
                   ONL_SIZL = 20, ONL_SIZH = 21, ONL_VSNL = 22, ONL_VSNH = 23;
localparam integer GUS_SHUN = 18, GUS_SHST = 19, GUS_TRK = 20, GUS_GRP = 21,
                   GUS_CYL = 22,  GUS_UVER = 23, GUS_RCTS = 24, GUS_RBSC = 25;
localparam integer SCC_MSV = 8, SCC_CFL = 9, SCC_TMO = 10, SCC_VER = 11,
                   SCC_CIDA = 12, SCC_CIDB = 13, SCC_CIDC = 14, SCC_CIDD = 15,
                   SCC_MBCL = 16, SCC_MBCH = 17;
localparam integer SCC_VER_V_HVER = 8, SCC_VER_V_SVER = 0,
                   SCC_CIDD_V_CLS = 8, SCC_CIDD_V_MOD = 0;
localparam integer RW_BCL = 8, RW_BCH = 9, RW_BAL = 10, RW_BAH = 11,
                   RW_LBNL = 16, RW_LBNH = 17;
