int8_t ade_init(void);
uint8_t ade_read8(uint16_t);
uint16_t ade_read16(uint16_t);
uint32_t ade_read32(uint16_t);
int8_t ade_write8(uint16_t, uint8_t);
int8_t ade_write16(uint16_t, uint16_t);
int8_t ade_write32(uint16_t, uint32_t);
int8_t ade_set8(uint16_t, uint8_t);
int8_t ade_set16(uint16_t, uint16_t);
int8_t ade_set32(uint16_t, uint32_t);
int8_t ade_clear8(uint16_t, uint8_t);
int8_t ade_clear16(uint16_t, uint16_t);
int8_t ade_clear32(uint16_t, uint32_t);

// Calibration Data

#define CAL_CVRMS 41951
#define CAL_CIRMS 17687307
#define CAL_CPOW 18628


// Register addresses

#define AIGAIN		0x4380
#define AVGAIN		0x4381
#define BIGAIN		0x4382
#define BVGAIN		0x4383
#define CIGAIN		0x4384
#define CVGAIN		0x4385
#define NIGAIN		0x4386
#define AIRMSOS		0x4387
#define AVRMSOS		0x4388
#define BIRMSOS		0x4389
#define BVRMSOS		0x438A
#define CIRMSOS		0x438B
#define CVRMSOS		0x438C
#define NIRMSOS		0x438D
#define AVAGAIN		0x438E
#define BVAGAIN		0x438F
#define CVAGAIN		0x4390
#define AWGAIN		0x4391
#define AWATTOS		0x4392
#define BWGAIN		0x4393
#define BWATTOS		0x4394
#define CWGAIN		0x4395
#define CWATTOS		0x4396
#define AVARGAIN	0x4397
#define AVAROS		0x4398
#define BVARGAIN	0x4399
#define BVAROS		0x439A
#define CVARGAIN	0x439B
#define CVAROS		0x439C
#define AFWGAIN		0x439D
#define AFWATTOS	0x439E
#define BFWGAIN		0x439F
#define BFWATTOS	0x43A0
#define CFWGAIN		0x43A1
#define CFWATTOS	0x43A3
#define AFVARGAIN	0x439E
#define AFVAROS		0x43A4
#define BFVARGAIN	0x43A5
#define BFVAROS		0x43A6
#define CFVARGAIN	0x43A7
#define CFVAROS		0x43A8
#define VATHR1		0x43A9
#define VATHR0		0x43AA
#define WTHR1		0x43AB
#define WTHR0		0x43AC
#define VARTHR1		0x43AD
#define VARTHR0		0x43AE
#define VANOLOAD	0x43B0
#define APNOLOAD	0x43B1
#define VARNOLOAD	0x43B2
#define VLEVEL		0x43B3
#define DICOEFF		0x43B5
#define HPFDIS		0x43B6
#define ISUM		0x43BF
#define AIRMS		0x43C0
#define AVRMS		0x43C1
#define BIRMS		0x43C2
#define BVRMS		0x43C3
#define CIRMS		0x43C4
#define CVRMS		0x43C5
#define NIRMS		0x43C6
#define RUN			0xE228
#define AWATTHR		0xE400
#define BWATTHR		0xE401
#define CWATTHR		0xE402
#define AFWATTHR	0xE403
#define BFWATTHR	0xE404
#define CFWATTHR	0xE405
#define AVARHR		0xE406
#define BVARHR		0xE407
#define CVARHR		0xE408
#define AFVARHR		0xE409
#define BFVARHR		0xE40A
#define CFVARHR		0xE40B
#define AVAHR		0xE40C
#define BVAHR		0xE40D
#define CVAHR		0xE40E
#define IPEAK		0xE500
#define VPEAK		0xE501
#define STATUS0		0xE502
#define STATUS1		0xE503
#define AIMAV		0xE504
#define BIMAV		0xE505
#define CIMAV		0xE506
#define OILVL		0xE507
#define OVLVL		0xE508
#define SAGLVL		0xE509
#define MASK0		0xE50A
#define MASK1		0xE50B
#define IAWV		0xE50C
#define IBWV		0xE50D
#define ICWV		0xE50E
#define INWV		0xE50F
#define VAWV		0xE510
#define VBWV		0xE511
#define VCWV		0xE512
#define AWATT		0xE513
#define BWATT		0xE514
#define CWATT		0xE515
#define AVAR		0xE516
#define BVAR		0xE517
#define CVAR		0xE518
#define AVA			0xE519
#define BVA			0xE51A
#define CVA			0xE51B
#define CHECKSUM	0xE51F
#define VNOM		0xE520
#define PHSTATUS	0xE600
#define ANGLE0		0xE601
#define ANGLE1		0xE602
#define ANGLE2		0xE603
#define PERIOD		0xE607
#define PHNOLOAD	0xE608
#define LINECYC		0xE60C
#define ZXTOUT		0xE60D
#define COMPMODE	0xE60E
#define GAIN		0xE60F
#define CFMODE		0xE610
#define CF1DEN		0xE611
#define CF2DEN		0xE612
#define CF3DEN		0xE613
#define APHCAL		0xE614
#define BPHCAL		0xE615
#define CPHCAL		0xE616
#define PHSIGN		0xE617
#define CONFIG		0xE618
#define MMODE		0xE700
#define ACCMODE		0xE701
#define LCYCMODE	0xE702
#define PEAKCYC		0xE703
#define SAGCYC		0xE704
#define CFCYC		0xE705
#define HSDC_CFG	0xE706
#define VERSION		0xE707
#define LPOILVL		0xEC00
#define CONFIG2		0xEC01


// Register bit masks

// IPEAK
#define IPEAKVAL	0xFFFFFF
#define IPPHASE0	0x1000000
#define IPPHASE1	0x2000000
#define IPPHASE2	0x4000000

// VPEAK
#define VPEAKVAL	0xFFFFFF
#define VPPHASE0	0x1000000
#define VPPHASE1	0x2000000
#define VPPHASE2	0x4000000

// STATUS0, MASK0
#define AEHF		0x1
#define FAEHF		0x2
#define REHF		0x4
#define FREHF		0x8
#define VAEHF		0x10
#define LENERGY		0x20
#define REVAPA		0x40
#define REVAPB		0x80
#define REVAPC		0x100
#define REVPSUM1	0x200
#define REVRPA		0x400
#define REVRPB		0x800
#define REVRPC		0x1000
#define REVPSUM2	0x2000
#define CF1			0x4000
#define CF2			0x8000
#define CF3			0x10000
#define DREADY		0x20000
#define REVPSUM3	0x40000

// STATUS1, MASK1
#define NLOAD		0x1
#define FNLOAD		0x2
#define VANLOAD		0x4
#define ZXTOVA		0x8
#define ZXTOVB		0x10
#define ZXTOVC		0x20
#define ZXTOIA		0x40
#define ZXTOIB		0x80
#define ZXTOIC		0x100
#define ZXVA		0x200
#define ZXVB		0x400
#define ZXVC		0x800
#define ZXIA		0x1000
#define ZXIB		0x2000
#define ZXIC		0x4000
#define RSTDONE		0x8000
#define SAG			0x10000
#define OI			0x20000
#define OV			0x40000
#define SEQERR		0x80000
#define MISMTCH		0x100000
#define PKI			0x800000
#define PKV			0x1000000

// PHSTATUS
#define OIPHASE0	0x4
#define OIPHASE1	0x8
#define OIPHASE2	0x10
#define OVPHASE0	0x200
#define OVPHASE1	0x400
#define OVPHASE2	0x800
#define VSPHASE0	0x1000
#define VSPHASE1	0x2000
#define VSPHASE2	0x4000

// PHNOLOAD
#define NLPHASE0	0x1
#define NLPHASE1	0x2
#define NLPHASE2	0x4
#define FNLPHASE0	0x8
#define FNLPHASE1	0x10
#define FNLPHASE2	0x20
#define VANLPHASE0	0x40
#define VANLPHASE1	0x80
#define VANLPHASE2	0x100

// COMPMODE
#define TERMSEL10	0x1
#define TERMSEL11	0x2
#define TERMSEL12	0x4
#define TERMSEL20	0x8
#define TERMSEL21	0x10
#define TERMSEL22	0x20
#define TERMSEL30	0x40
#define TERMSEL31	0x80
#define TERMSEL32	0x100
#define ANGLESEL	0x600
#define VNOMAEN		0x800
#define VNOMBEN		0x1000
#define VNOMCEN		0x2000
#define SELFREQ		0x4000

// GAIN
#define PGA1		0x7
#define PGA2		0x38
#define PGA3		0x1C0
// GAIN set values
#define PGA1_1		0x0
#define PGA1_2		0x1
#define PGA1_4		0x2
#define PGA1_8		0x3
#define PGA1_16		0x4
#define PGA2_1		0x0
#define PGA2_2		0x8
#define PGA2_4		0x10
#define PGA2_8		0x18
#define PGA2_16		0x20
#define PGA3_1		0x0
#define PGA3_2		0x40
#define PGA3_4		0x80
#define PGA3_8		0xC0
#define PGA3_16		0x100

// CFMODE
#define CF1SEL		0x7
#define CF2SEL		0x38
#define CF3SEL		0x1C0
#define CF1DIS		0x200
#define CF2DIS		0x400
#define CF3DIS		0x800
#define CF1LATCH	0x1000
#define CF2LATCH	0x2000
#define CF3LATCH	0x4000
#define RSTDONE		0x8000

// APHCAL, BPHCAL, CPHCAL
#define PHCALVAL	0x3FF

// PHSIGN
#define AWSIGN		0x1
#define BWSIGN		0x2
#define CWSIGN		0x4
#define SUM1SIGN	0x8
#define AVARSIGN	0x10
#define BVARSIGN	0x20
#define CVARSIGN	0x40
#define SUM2SIGN	0x80
#define SUM3SIGN	0x100

// CONFIG
#define INTEN		0x1
#define SWAP		0x8
#define MOD1SHORT	0x10
#define MOD2SHORT	0x20
#define HSDCEN		0x40
#define SWRST		0x80
#define VTOIA		0x300
#define VTOIB		0xC00
#define VTOIC		0x3000
// VTOI set values
#define VTOIA_A		0x0
#define VTOIA_B		0x100
#define VTOIA_C		0x200
#define VTOIB_B		0x0
#define VTOIB_C		0x400
#define VTOIB_A		0x800
#define VTOIC_C		0x0
#define VTOIC_A		0x1000
#define VTOIC_B		0x2000

// MMODE
#define PERSEL		0x3
#define PEAKSEL0	0x4
#define PEAKSEL1	0x8
#define PEAKSEL2	0x10

// ACCMODE
#define WATTACC		0x3
#define VARACC		0xC
#define CONSEL		0x30
#define REVAPSEL	0x40
#define REVRPSEL	0x80

// LCYCMODE
#define LWATT		0x1
#define LVAR		0x2
#define LVA			0x4
#define ZXSEL0		0x8
#define ZXSEL1		0x10
#define ZXSEL2		0x20
#define RSTREAD		0x40

// HSDC_CFG
#define HCLK		0x1
#define HSIZE		0x2
#define HGAP		0x4
#define HXFER		0x18
#define HSAPOL		0x20

// LPOILVL
#define LPOIL		0x7
#define LPLINE		0xF8

// CONFIG2
#define EXTREFEN	0x1
#define I2C_LOCK	0x2

// RUN
#define STOP		0x0
#define START		0x1
