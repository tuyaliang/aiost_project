/*
 * Copyright (C) 2015 Linaro Ltd.
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

#ifndef __DTS_MB86S27_CLK_H
#define __DTS_MB86S27_CLK_H

#define OFFSET_A	0
#define OFFSET_B	32
#define OFFSET_C	64
#define OFFSET_D	96
#define OFFSET_E	128
#define OFFSET_F	160
#define OFFSET_G	192
#define OFFSET_H	224

#define bPLL1512ST	0
#define bPLL864ST	2
#define bPLL594ST	4
#define bAPLLST		6
#define bAPLSEL		(32 * 8 + 16)
#define bDDRPLL0ST	(32 + 0)
#define bDDRPLL1ST	(32 + 2)
#define bDDRPLL2ST	(32 + 4)
#define bDDRPLL0SEL	(32 + 8)
#define bDDRPLL1SEL	(32 + 16)
#define bDDRPLL2SEL	(32 + 24)

#define bRCLK		(OFFSET_A + 4)
#define bSENCLK		(OFFSET_A + 7)
#define bSD1U2CLK	(OFFSET_A + 10)
#define bSD1U1CLK	(OFFSET_A + 16)
#define bSD0CLK		(OFFSET_A + 20)
#define bNFCLK		(OFFSET_A + 24)
#define bAPCLKO		(OFFSET_A + 26)
#define bJPGCLK		(OFFSET_B + 0)
#define bELA0CLK	(OFFSET_B + 4)
#define bELA1CLK	(OFFSET_B + 8)
#define bSPRCLK		(OFFSET_B + 12)
#define bWDRCLK		(OFFSET_B + 16)
#define bAPLCLK		(OFFSET_B + 25)
#define bJHDRCLK	(OFFSET_B + 28)
#define bHIFCLK		(OFFSET_C + 0)
#define bLCDCLK		(OFFSET_C + 4)
#define bLCDSEL		(OFFSET_C + 11)
#define bRIBCLK		(OFFSET_C + 16)
#define bB2YCLK		(OFFSET_C + 22)
#define bIIPCLK		(OFFSET_C + 28)
#define bPROCLK		(OFFSET_D + 0)
#define bPROMUX		(OFFSET_D + 7)
#define bSROCLK		(OFFSET_D + 12)
#define bSROSEL		(OFFSET_D + 16)
#define bAHFDCLK	(OFFSET_D + 24)
#define bAHIPCLK	(OFFSET_D + 28)
#define bCPUCLK		(OFFSET_E + 0)
#define bL2CLK		(OFFSET_E + 8)
#define bPERICLK	(OFFSET_E + 16)
#define bEAXICLK	(OFFSET_F + 0)
#define bAXICLK		(OFFSET_F + 8)
#define bAHBCLK		(OFFSET_F + 16)
#define bAPBCLK		(OFFSET_F + 24)
#define bXCLK		(OFFSET_H + 0)
#define bDSPCLK		(OFFSET_H + 4)
#define bDPPCLK		(OFFSET_H + 16)
#define bSRRCLK		(OFFSET_H + 20)
#define bAU0CKSEL	(OFFSET_H + 24)
#define bAU1CKSEL	(OFFSET_H + 26)
#define bAU2CKSEL	(OFFSET_H + 28)
#define bNETAUCKSEL	(OFFSET_H + 30)


#define bPDBSTP		(OFFSET_A + 1)
#define bHAPSTP		(OFFSET_A + 3)
#define bDSPCK		(OFFSET_A + 4)
#define bDSPAX		(OFFSET_A + 5)
#define bSENCK		(OFFSET_A + 10)
#define bSENAP		(OFFSET_A + 11)
#define bGPIOAP		(OFFSET_A + 14)
#define bCECCLK		(OFFSET_A + 18)
#define bRCK		(OFFSET_A + 19)
#define bAU0CK		(OFFSET_A + 20)
#define bAU1CK		(OFFSET_A + 21)
#define bAU2CK		(OFFSET_A + 22)
#define bNETAUCK	(OFFSET_A + 23)
#define bPERIAP		(OFFSET_A + 26)
#define bPERIAH		(OFFSET_A + 27)
#define bHDMEAH		(OFFSET_B + 0)
#define bXDMEAX		(OFFSET_B + 1)
#define bNFCK		(OFFSET_B + 4)
#define bNFAX		(OFFSET_B + 7)
#define bSD0CK		(OFFSET_B + 8)
#define bSD0AH		(OFFSET_B + 9)
#define bSD1U1CK	(OFFSET_B + 12)
#define bSD1U2CK	(OFFSET_B + 13)
#define bSD1AX		(OFFSET_B + 15)
#define bRELCAH		(OFFSET_B + 18)
#define bPCIE0AX	(OFFSET_B + 21)
#define bPCIE1AX	(OFFSET_B + 23)
#define bOSCCK		(OFFSET_B + 24)
#define bNETSAX		(OFFSET_B + 26)
#define bIIPCK		(OFFSET_C + 0)
#define bIIPAP		(OFFSET_C + 1)
#define bIIPAH		(OFFSET_C + 2)
#define bIIPAX		(OFFSET_C + 3)
#define bB2YCK		(OFFSET_C + 4)
#define bB2YAP		(OFFSET_C + 5)
#define bB2YAH		(OFFSET_C + 6)
#define bB2YAX		(OFFSET_C + 7)
#define bLCDCK		(OFFSET_C + 9)
#define bHIFCK		(OFFSET_C + 10)
#define bDISPAP		(OFFSET_C + 13)
#define bDISPAH		(OFFSET_C + 14)
#define bDISPAX		(OFFSET_C + 15)
#define bWDRCK		(OFFSET_C + 16)
#define bJPGCK		(OFFSET_C + 20)
#define bJPGAH		(OFFSET_C + 22)
#define bJPGAX		(OFFSET_C + 23)
#define bRIBCK		(OFFSET_C + 28)
#define bRIBAH		(OFFSET_C + 29)
#define bRIBAX		(OFFSET_C + 31)
#define bXCHAP		(OFFSET_D + 1)
#define bXCHAX		(OFFSET_D + 3)
#define bAHFDCK		(OFFSET_D + 8)
#define bAHIPCK		(OFFSET_D + 9)
#define bAHIPAH		(OFFSET_D + 10)
#define bAHIPAX		(OFFSET_D + 11)
#define bJHDRCK		(OFFSET_D + 16)
#define bJHDRAH		(OFFSET_D + 18)
#define bJHDRAX		(OFFSET_D + 19)
#define bSROCK		(OFFSET_D + 20)
#define bAPCK		(OFFSET_D + 21)
#define bSNCK		(OFFSET_D + 23)
#define bPROCK		(OFFSET_D + 24)
#define bPROAP		(OFFSET_D + 25)
#define bPROAH		(OFFSET_D + 26)
#define bPROAX		(OFFSET_D + 27)
#define bSPRCK		(OFFSET_D + 28)
#define bSPRAP		(OFFSET_D + 29)
#define bSPRAX		(OFFSET_D + 31)
#define bMICAX		(OFFSET_E + 4)
#define bMICAP		(OFFSET_E + 5)
#define bMICAH		(OFFSET_E + 6)
#define bSRAMAX		(OFFSET_E + 10)
#define bIMGAP		(OFFSET_E + 16)
#define bIDECAH		(OFFSET_E + 17)
#define bITOPAX		(OFFSET_E + 18)
#define bSDI0AX		(OFFSET_E + 20)
#define bSDI0AP		(OFFSET_E + 21)
#define bSDI1AX		(OFFSET_E + 24)
#define bSDI1AP		(OFFSET_E + 25)
#define bSDI2AX		(OFFSET_E + 28)
#define bSDI2AP		(OFFSET_E + 29)
#define bELA0CK		(OFFSET_F + 0)
#define bELA0AP		(OFFSET_F + 1)
#define bELA0AX		(OFFSET_F + 3)
#define bELA1CK		(OFFSET_F + 4)
#define bELA1AP		(OFFSET_F + 5)
#define bELA1AX		(OFFSET_F + 7)
#define bDPPCK		(OFFSET_F + 8)
#define bDPPAH		(OFFSET_F + 10)
#define bDPPAX		(OFFSET_F + 11)
#define bSRRCK		(OFFSET_F + 12)
#define bSRRAH		(OFFSET_F + 14)
#define bSRRAX		(OFFSET_F + 15)

#define M8M_MAX_CLKS	93

#endif /* __DTS_MB86S27_CLK_H */
