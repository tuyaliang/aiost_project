/*
 * Copyright (C) 2016 Socionext inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DTS_SC2000A_CLK_H
#define __DTS_SC2000A_CLK_H

#define OFFSET_1	0
#define OFFSET_2	32
#define OFFSET_3	64
#define OFFSET_4	96
#define OFFSET_5	128
#define OFFSET_6	160
#define OFFSET_7	192
#define OFFSET_8	224
#define OFFSET_9	256
#define OFFSET_10	288
#define OFFSET_11	320
#define OFFSET_12	352
#define OFFSET_13	384
#define OFFSET_14	416

/* CLKSEL */
#define bRCLK		(OFFSET_1 + 0)
#define bUHS1CLK0	(OFFSET_1 + 3)
#define bUHS1CLK1	(OFFSET_1 + 8)
#define bUHS1CLK2	(OFFSET_1 + 13)
#define bUHS2CLK	(OFFSET_1 + 18)
#define bNFCLK		(OFFSET_1 + 22)
#define bEMMCCLK	(OFFSET_1 + 28)
#define bELACLK		(OFFSET_2 + 0)
#define bJPGCLK		(OFFSET_2 + 4)
#define bGPUCLK		(OFFSET_2 + 8)
#define bIPUTMECLK	(OFFSET_2 + 12)
#define bIPUCLK		(OFFSET_2 + 16)
#define bMIFCLK		(OFFSET_2 + 20)
#define bHIFCLK		(OFFSET_2 + 24)
#define bRAWCLK		(OFFSET_2 + 28)
#define bVDFCLK		(OFFSET_3 + 0)
#define bPXFCLK		(OFFSET_3 + 4)
#define bIPPCLK		(OFFSET_3 + 8)
#define bPASCLK		(OFFSET_3 + 13)
#define bIIPCLK		(OFFSET_3 + 17)
#define bSENMSKCLK	(OFFSET_3 + 21)
#define bSENCLK		(OFFSET_3 + 23)
#define bPIPESEL	(OFFSET_3 + 27)
#define bCNR1CLK	(OFFSET_4 + 0)
#define bB2R1CLK	(OFFSET_4 + 7)
#define bLTM1CLK	(OFFSET_4 + 14)
#define bR2Y1CLK	(OFFSET_4 + 21)
#define bSRO1CLK_2	(OFFSET_4 + 28)
#define bLTMRBK1CLK	(OFFSET_5 + 0)
#define bB2B1CLK	(OFFSET_5 + 6)
#define bSRO1CLK	(OFFSET_5 + 12)
#define bCNR2CLK	(OFFSET_6 + 0)
#define bB2R2CLK	(OFFSET_6 + 7)
#define bLTM2CLK	(OFFSET_6 + 14)
#define bR2Y2CLK	(OFFSET_6 + 21)
#define bSRO2CLK_2	(OFFSET_6 + 28)
#define bLTMRBK2CLK	(OFFSET_7 + 0)
#define bB2B2CLK	(OFFSET_7 + 6)
#define bSRO2CLK	(OFFSET_7 + 12)
#define bDSPCLK		(OFFSET_8 + 0)
#define bSPICLK		(OFFSET_8 + 3)
#define bAUCLK		(OFFSET_8 + 6)
#define bAU0SEL		(OFFSET_8 + 9)
#define bAU2SEL		(OFFSET_8 + 11)
#define bAU3SEL		(OFFSET_8 + 13)
#define bNETAUSEL	(OFFSET_8 + 15)
#define bAPCLK		(OFFSET_8 + 17)
#define bAP0SEL		(OFFSET_8 + 23)
#define bAP1SEL		(OFFSET_8 + 25)
#define bAP2SEL		(OFFSET_8 + 27)
#define bAP3SEL		(OFFSET_8 + 29)
#define bPCLK		(OFFSET_9 + 0)
#define bHCLK		(OFFSET_9 + 7)
#define bHCLKBMH	(OFFSET_9 + 12)
#define bACLKEXS	(OFFSET_9 + 16)
#define bACLK		(OFFSET_9 + 20)
#define bACLK400	(OFFSET_10 + 0)
#define bMCLK200	(OFFSET_10 + 3)
#define bMCLK400	(OFFSET_10 + 7)
#define bDCHREQ		(OFFSET_11 + 0)
#define bACLK300	(OFFSET_12 + 0)
#define bGYROCLK	(OFFSET_12 + 2)
#define bFPT0CLK	(OFFSET_12 + 8)
#define bFPT1CLK	(OFFSET_12 + 12)
#define bMECLK		(OFFSET_12 + 16)
#define bNFBCHCLK	(OFFSET_12 + 19)
#define bRIBCLK		(OFFSET_12 + 21)
#define bSHDRCLK	(OFFSET_12 + 26)

/* PLLCNT */
#define bPL00ST		(OFFSET_1 + 0)
#define bPL01ST		(OFFSET_1 + 1)
#define bPL02ST		(OFFSET_1 + 2)
#define bPL03ST		(OFFSET_1 + 3)
#define bPL04ST		(OFFSET_1 + 4)
#define bPL05ST		(OFFSET_1 + 5)
#define bPL05AST	(OFFSET_1 + 6)
#define bPL06ST		(OFFSET_1 + 7)
#define bPL07ST		(OFFSET_1 + 8)
#define bPL08ST		(OFFSET_1 + 9)
#define bPL10ST		(OFFSET_1 + 10)
#define bPL10AST	(OFFSET_1 + 11)
#define bPL11ST		(OFFSET_1 + 12)
#define bDPL00ST	(OFFSET_1 + 13)
#define bDPL01ST	(OFFSET_1 + 14)
#define bDPL02ST	(OFFSET_1 + 15)
#define bDPL10ST	(OFFSET_1 + 16)
#define bDPL11ST	(OFFSET_1 + 17)
#define bDPL12ST	(OFFSET_1 + 18)
#define bPL00SEL	(OFFSET_2 + 0)
#define bPL01SEL	(OFFSET_2 + 1)
#define bPL02SEL	(OFFSET_2 + 2)
#define bPL03SEL	(OFFSET_2 + 3)
#define bPL04SEL	(OFFSET_2 + 4)
#define bPL05SEL	(OFFSET_2 + 5)
#define bPL05ASEL	(OFFSET_2 + 6)
#define bPL06SEL	(OFFSET_2 + 7)
#define bPL07SEL	(OFFSET_2 + 8)
#define bPL08SEL	(OFFSET_2 + 9)
#define bPL10SEL	(OFFSET_2 + 10)
#define bPL10ASEL	(OFFSET_2 + 11)
#define bPL11SEL	(OFFSET_2 + 12)
#define bPL01SSEN	(OFFSET_2 + 16)
#define bPL02SSEN	(OFFSET_2 + 17)
#define bP00POSTDIV	(OFFSET_3 + 0)
#define bP00PREDIV	(OFFSET_3 + 8)
#define bP00PLLDIV	(OFFSET_3 + 16)
#define bP00FNUM	(OFFSET_4 + 0)
#define bP00FDEN	(OFFSET_5 + 0)
#define bP01RATE	(OFFSET_6 + 0)
#define bP01FREQ	(OFFSET_6 + 10)
#define bP01MODE	(OFFSET_6 + 12)
#define bP02RATE	(OFFSET_6 + 16)
#define bP02FREQ	(OFFSET_6 + 26)
#define bP02MODE	(OFFSET_6 + 28)
#define bP03POSTDIV0	(OFFSET_7 + 0)
#define bP03POSTDIV1	(OFFSET_7 + 4)
#define bP03PREDIV	(OFFSET_7 + 8)
#define bP03PLLDIV	(OFFSET_7 + 16)
#define bP03OC0		(OFFSET_7 + 24)
#define bP03OC1		(OFFSET_7 + 25)
#define bD0XOC		(OFFSET_7 + 26)
#define bD1XOC		(OFFSET_7 + 27)
#define bDPLX8		(OFFSET_7 + 28)
#define bDSEL		(OFFSET_7 + 29)
#define bP04POSTDIV	(OFFSET_8 + 0)
#define bP04PREDIV	(OFFSET_8 + 8)
#define bP04PLLDIV	(OFFSET_8 + 16)
#define bP10APLLDIV	(OFFSET_9 + 0)
#define bP08PLLDIV	(OFFSET_9 + 8)
#define bP05CHG		(OFFSET_9 + 16)

/* CLKSTOP */
#define bDSPCK		(OFFSET_1 + 0)
#define bDSPAX		(OFFSET_1 + 2)
#define bSENCK		(OFFSET_1 + 4)
#define bSENAX		(OFFSET_1 + 6)
#define bSENAH		(OFFSET_1 + 8)
#define bSENAP		(OFFSET_1 + 10)
#define bGPIOAP		(OFFSET_1 + 12)
#define bAU0CK		(OFFSET_1 + 14)
#define bAU2CK		(OFFSET_1 + 16)
#define bAU3CK		(OFFSET_1 + 18)
#define bAU4CK		(OFFSET_1 + 20)
#define bAU5CK		(OFFSET_1 + 22)
#define bNETAUCK	(OFFSET_1 + 24)
#define bTEMPCK		(OFFSET_1 + 28)
#define bRCK		(OFFSET_2 + 0)
#define bUHS1CK0	(OFFSET_2 + 2)
#define bUHS1CK1	(OFFSET_2 + 4)
#define bUHS1CK2	(OFFSET_2 + 6)
#define bUHS2CK		(OFFSET_2 + 8)
#define bNFCK		(OFFSET_2 + 10)
#define bEMMCCK		(OFFSET_2 + 12)
#define bNETSECCK	(OFFSET_2 + 14)
#define bNETRCK		(OFFSET_2 + 16)
#define bEXSAX		(OFFSET_2 + 18)
#define bSPICK		(OFFSET_2 + 20)
#define bSLIMB00CK	(OFFSET_2 + 22)
#define bSLIMB01CK	(OFFSET_2 + 24)
#define bSLIMB10CK	(OFFSET_2 + 26)
#define bSLIMB11CK	(OFFSET_2 + 28)
#define bPCISUPPCK	(OFFSET_2 + 30)
#define bIIPCK		(OFFSET_3 + 0)
#define bIIPAP		(OFFSET_3 + 2)
#define bIIPAH		(OFFSET_3 + 4)
#define bIIPAX		(OFFSET_3 + 6)
#define bLCDCK		(OFFSET_3 + 8)
#define bHIFCK		(OFFSET_3 + 10)
#define bMIFCK		(OFFSET_3 + 12)
#define bDISPAP		(OFFSET_3 + 14)
#define bDISPAH		(OFFSET_3 + 16)
#define bDISPAX		(OFFSET_3 + 18)
#define bJPGCK		(OFFSET_3 + 20)
#define bJPGAX		(OFFSET_3 + 22)
#define bJPGAH		(OFFSET_3 + 24)
#define bJPGAP		(OFFSET_3 + 26)
#define bPDM0CK		(OFFSET_3 + 28)
#define bPDM1CK		(OFFSET_3 + 30)
#define bGPUCK		(OFFSET_4 + 0)
#define bGPUAP		(OFFSET_4 + 2)
#define bGPUAH		(OFFSET_4 + 4)
#define bGPUAX		(OFFSET_4 + 6)
#define bFPT0CK		(OFFSET_4 + 8)
#define bFPT0AX		(OFFSET_4 + 10)
#define bFPT0AH		(OFFSET_4 + 12)
#define bFPT0AP		(OFFSET_4 + 14)
#define bFPT1CK		(OFFSET_4 + 16)
#define bFPT1AP		(OFFSET_4 + 18)
#define bFPT1AH		(OFFSET_4 + 20)
#define bFPT1AX		(OFFSET_4 + 22)
#define bAPCK0		(OFFSET_4 + 24)
#define bAPCK1		(OFFSET_4 + 26)
#define bAPCK2		(OFFSET_4 + 28)
#define bAPCK3		(OFFSET_4 + 30)
#define bMICAX0		(OFFSET_5 + 0)
#define bMICAX1		(OFFSET_5 + 2)
#define bMICAX2		(OFFSET_5 + 4)
#define bMICAX3		(OFFSET_5 + 6)
#define bMICAX4		(OFFSET_5 + 8)
#define bMICAX5		(OFFSET_5 + 10)
#define bMICAX6		(OFFSET_5 + 12)
#define bMICAP0		(OFFSET_5 + 14)
#define bMICAP1		(OFFSET_5 + 16)
#define bMICAP2		(OFFSET_5 + 18)
#define bMICAP3		(OFFSET_5 + 20)
#define bMICAP4		(OFFSET_5 + 22)
#define bMICAP5		(OFFSET_5 + 24)
#define bMICAP6		(OFFSET_5 + 26)
#define bMICAH1		(OFFSET_6 + 0)
#define bMICAH2		(OFFSET_6 + 2)
#define bMICAH3		(OFFSET_6 + 4)
#define bIMGAX		(OFFSET_6 + 6)
#define bIMGAH0		(OFFSET_6 + 8)
#define bIMGAH1		(OFFSET_6 + 10)
#define bIMGAH3		(OFFSET_6 + 12)
#define bIMGAP3		(OFFSET_6 + 14)
#define bREGAP		(OFFSET_6 + 16)
#define bXCHAX		(OFFSET_6 + 18)
#define bXCHAP		(OFFSET_6 + 20)
#define bELACK		(OFFSET_6 + 22)
#define bELAAX		(OFFSET_6 + 24)
#define bELAAP		(OFFSET_6 + 26)
#define bIPUFDCK	(OFFSET_7 + 0)
#define bIPUVISCK	(OFFSET_7 + 2)
#define bIPUAX		(OFFSET_7 + 4)
#define bIPUAH		(OFFSET_7 + 6)
#define bRAWCK		(OFFSET_7 + 8)
#define bRAWAX		(OFFSET_7 + 10)
#define bRAWAP		(OFFSET_7 + 12)
#define bSHDRCK		(OFFSET_7 + 14)
#define bSHDRAX		(OFFSET_7 + 16)
#define bSHDRAH		(OFFSET_7 + 18)
#define bSHDRAP		(OFFSET_7 + 20)
#define bM0CK		(OFFSET_7 + 22)
#define bMECK		(OFFSET_7 + 24)
#define bMEAX		(OFFSET_7 + 26)
#define bMEAP		(OFFSET_7 + 28)
#define bRIBCK		(OFFSET_8 + 0)
#define bRIBAH		(OFFSET_8 + 2)
#define bHEVDFCK	(OFFSET_8 + 4)
#define bHEPXFCK	(OFFSET_8 + 6)
#define bHEIPPCK	(OFFSET_8 + 8)
#define bUMC0HEVCMX2	(OFFSET_8 + 10)
#define bUMC0HEVCMX4	(OFFSET_8 + 12)
#define bUMC0RBRMX4	(OFFSET_8 + 14)
#define bUMC1HEVCMX2	(OFFSET_8 + 16)
#define bUMC1HEVCMX4	(OFFSET_8 + 18)
#define bUMC1RBRMX4	(OFFSET_8 + 20)
#define bUMC0CMNAX	(OFFSET_8 + 22)
#define bUMC0MX1AX	(OFFSET_8 + 24)
#define bUMC0MX2AX	(OFFSET_8 + 26)
#define bUMC0MX3AX	(OFFSET_8 + 28)
#define bUMC0MX4AX	(OFFSET_8 + 30)
#define bUMC0MX5AX	(OFFSET_9 + 0)
#define bUMC1CMNAX	(OFFSET_9 + 2)
#define bUMC1MX1AX	(OFFSET_9 + 4)
#define bUMC1MX2AX	(OFFSET_9 + 6)
#define bUMC1MX3AX	(OFFSET_9 + 8)
#define bUMC1MX4AX	(OFFSET_9 + 10)
#define bUMC1MX5AX	(OFFSET_9 + 12)
#define bUMC0MX0AX3	(OFFSET_9 + 14)
#define bUMC0MX6AX4	(OFFSET_9 + 16)
#define bUMC0HEVCAX4	(OFFSET_9 + 18)
#define bUMC1MX0AX3	(OFFSET_9 + 20)
#define bUMC1MX6AX4	(OFFSET_9 + 22)
#define bUMC1HEVCAX4	(OFFSET_9 + 24)
#define bUMC0AP		(OFFSET_9 + 26)
#define bUMC1AP		(OFFSET_9 + 28)
#define bSRO1CK		(OFFSET_10 + 0)
#define bSRO1CK_2	(OFFSET_10 + 2)
#define bSRO1AX		(OFFSET_10 + 4)
#define bSRO1AH		(OFFSET_10 + 6)
#define bSRO1AP		(OFFSET_10 + 8)
#define bB2B1CK		(OFFSET_10 + 10)
#define bB2B1AX		(OFFSET_10 + 12)
#define bB2B1AH		(OFFSET_10 + 14)
#define bB2B1AP		(OFFSET_10 + 16)
#define bLTMRBK1CK	(OFFSET_10 + 18)
#define bB2R1CK		(OFFSET_10 + 20)
#define bB2R1AX		(OFFSET_10 + 22)
#define bB2R1AH		(OFFSET_10 + 24)
#define bB2R1AP		(OFFSET_10 + 26)
#define bLTM1CK		(OFFSET_11 + 0)
#define bLTM1AX		(OFFSET_11 + 2)
#define bLTM1AH		(OFFSET_11 + 4)
#define bLTM1AP		(OFFSET_11 + 6)
#define bR2Y1CK		(OFFSET_11 + 8)
#define bR2Y1AX		(OFFSET_11 + 10)
#define bR2Y1AH		(OFFSET_11 + 12)
#define bR2Y1AP		(OFFSET_11 + 14)
#define bCNR1CK		(OFFSET_11 + 16)
#define bCNR1AX		(OFFSET_11 + 18)
#define bCNR1AP		(OFFSET_11 + 20)
#define bAPAH		(OFFSET_11 + 22)
#define bDBGAP		(OFFSET_11 + 24)
#define bNFBCHCK	(OFFSET_11 + 26)
#define bSRO2CK		(OFFSET_12 + 0)
#define bSRO2CK_2	(OFFSET_12 + 2)
#define bSRO2AX		(OFFSET_12 + 4)
#define bSRO2AH		(OFFSET_12 + 6)
#define bSRO2AP		(OFFSET_12 + 8)
#define bB2B2CK		(OFFSET_12 + 10)
#define bB2B2AX		(OFFSET_12 + 12)
#define bB2B2AH		(OFFSET_12 + 14)
#define bB2B2AP		(OFFSET_12 + 16)
#define bLTMRBK2CK	(OFFSET_12 + 18)
#define bB2R2CK		(OFFSET_12 + 20)
#define bB2R2AX		(OFFSET_12 + 22)
#define bB2R2AH		(OFFSET_12 + 24)
#define bB2R2AP		(OFFSET_12 + 26)
#define bLTM2CK		(OFFSET_13 + 0)
#define bLTM2AX		(OFFSET_13 + 2)
#define bLTM2AH		(OFFSET_13 + 4)
#define bLTM2AP		(OFFSET_13 + 6)
#define bR2Y2CK		(OFFSET_13 + 8)
#define bR2Y2AX		(OFFSET_13 + 10)
#define bR2Y2AH		(OFFSET_13 + 12)
#define bR2Y2AP		(OFFSET_13 + 14)
#define bCNR2CK		(OFFSET_13 + 16)
#define bCNR2AX		(OFFSET_13 + 18)
#define bCNR2AP		(OFFSET_13 + 20)
#define bUMCVDFMX4	(OFFSET_13 + 22)
#define bUMCPXFMX4	(OFFSET_13 + 24)
#define bUMCVDFMX2	(OFFSET_13 + 26)
#define bUMCPXFMX2	(OFFSET_13 + 28)
#define bBMH1CK		(OFFSET_13 + 30)
#define bSTATAX		(OFFSET_14 + 0)
#define bSTATAH		(OFFSET_14 + 2)
#define bSTATAP		(OFFSET_14 + 4)
#define bPERIAH		(OFFSET_14 + 6)
#define bPERIAP		(OFFSET_14 + 8)
#define bSENMSKCK	(OFFSET_14 + 10)
#define bGYROCK		(OFFSET_14 + 12)
#define bEXSAH		(OFFSET_14 + 14)
#define bEXSAP		(OFFSET_14 + 16)
#define bPASCK		(OFFSET_14 + 18)
#define bBMH0CK		(OFFSET_14 + 20)
#define bBMH0AX		(OFFSET_14 + 22)
#define bRDMAAX		(OFFSET_14 + 26)
#define bRDMAAP		(OFFSET_14 + 28)
#define bBMH1AX		(OFFSET_14 + 30)

/* CRSWR */
#define bSRREQ		0

/* CRRRS */
#define bRIBSR		0
#define bHEIPPASYNCSR	1
#define bHEIPPSYNCSR	2
#define bHEVDFSR	3
#define bHEPXFSR	4
#define bDSPSR		5
#define bCPU0_SR	8
#define bCPU1_SR	9
#define bCPU2_SR	10
#define bCPU3_SR	11

/* CRRSM */
#define bWDRST		0
#define bSWRST		1
#define bSRST		2
#define bPORST		3

/* PLLFREQ1 */
#define bPLLIDIV	0

/* ODIVCH0 */
#define bODIVCH0NUM	0

/* CMDEN */
#define bCMDEN		0


#define MLB01_MAX_CLKS	68

#endif /* __DTS_SC2000A_CLK_H */
