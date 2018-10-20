/*
 *  linux/arch/arm/mach-mb8ac0300/include/mach/f_usb20hdc.h
 *
 * Copyright (C) 2012-2015 SOCIONEXT
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MACH_F_USB20HDC_H
#define __MACH_F_USB20HDC_H

enum f_usb2otg_hcd_variants {
	FHDC_VARIANT_ORIG,
	FHDC_VARIANT_LAP
};


/* F_USB20HDC USB controller DMA channel count */
#define HDC_MAX_DMA_CH		2

/* F_USB20HDC C_HSEL area address offset */
#define F_USB20HDC_C_HSEL_ADDR		0x00000
/* F_USB20HDC D_HSEL area address offset */
#define F_USB20HDC_D_HSEL_ADDR		0x10000

/* endpoint channel symbolic constant */
#define EP0	0	/* endpoint 1 */
#define EP1	1	/* endpoint 1 */
#define EP2	2	/* endpoint 2 */
#define EP3	3	/* endpoint 3 */
#define EP4	4	/* endpoint 4 */
#define EP5	5	/* endpoint 5 */
#define EP6	6	/* endpoint 6 */
#define EP7	7	/* endpoint 7 */
#define EP8	8	/* endpoint 8 */
#define EP9	9	/* endpoint 9 */
#define EP10	10	/* endpoint 10 */
#define EP11	11	/* endpoint 11 */
#define EP12	12	/* endpoint 12 */
#define EP13	13	/* endpoint 13 */
#define EP14	14	/* endpoint 14 */
#define EP15	15	/* endpoint 15 */

/* endpoint counter count per one endpoint */
#define EP_BUF_CNT		2

/* endpoint configuration structure array table */
struct endpont_cb {
	/*
	 * endpoint name string
	 * [notice]:The following constant definition is used.
	 *	endpoint 0 is "ep0" fixation
	 *	unused	= ""
	 *	used	= "ep(number)(in or out)-(bulk or iso or int)"
	 *	[Example]
	 *	"ep1", "ep2", ... address is fixed, not direction or type
	 *	"ep1in, "ep2out", ... address and direction are fixed, not type
	 *	"ep1-bulk", "ep2-int", ... address and type are fixed, not dir
	 *	"ep1in-bulk", "ep2out-iso", ... all three are fixed
	 */
	char	*name;
	/*
	 * endpoint transfer maximum packet size for high-speed
	 * [notice]:unusing it is referred to as '0'
	 */
	u16	hs_maxpacket;
	/*
	 * endpoint transfer maximum packet size for full-speed
	 * [notice]:unusing it is referred to as '0'
	 */
	u16	fs_maxpacket;
	/*
	 * endpoint buffer size(x1[bytes])
	 * [notice]:unusing it is referred to as '0'
	 */
	u16	buffer_size;
	/*
	 * endpoint buffer count
	 * [notice]:unusing it is referred to as '0', and endpoint 0 is 1 fixed
	 */
	u8	buffers;
	/*
	 * PIO transfer auto change flag
	 * [notice]:unusing it is referred to as '0', and endpoint 0 is 0 fixed
	 */
	u8	pio_auto_change;
	/*
	 * IN transfer end notify timing to USB host flag
	 * [notice]:unusing it is referred to as '0', and endpoint 0 is 0 fixed
	 */
	u8	trans_end_timing;
	/*
	 * USB controller DMA channel for endpoint
	 * [notice]:unusing it is referred to as '-1'
	 */
	s8	dma_ch;
};

/* -- FHDC_VARIANT_ORIG -- */
/* F_USB20HDC controller register offset constant, six reg is read-only*/
#define HDC_CONF			0x0000	/* System Configuration */
#define BIT_BYTE_ORDER			0
#define BIT_BURST_WAIT			1
#define BIT_SOFT_RESET			2
#define HDC_MODE			0x0004	/* Oepration Mode */
#define BIT_HOST_EN			0
#define BIT_DEV_EN			1
#define BIT_DEV_INT_MODE		2
#define BIT_DEV_ADDR_LOAD_MODE		3
#define HDC_INTEN			0x0008	/* Global Interrupt Enable */
#define BIT_HOST_INTEN			0
#define BIT_DEV_INTEN			1
#define BIT_OTG_INTEN			2
#define BIT_PHY_ERR_INTEN		3
#define BIT_CMD_INTEN			4
#define BIT_DMA_INTEN(ch)		(8 + (ch))
#define BIT_DEV_EP_INTEN(n)		(16 + (n))
/* USB DMA channel symbolic constant */
#define HDC_DMA_CH1			0 /* USB DMA channel 1 */
#define HDC_DMA_CH2			1 /* USB DMA channel 2 */
#define HDC_INTS			0x000c /* Global Interrupt Status */
#define BIT_HOST_INT			0
#define BIT_DEV_INT			1
#define BIT_OTG_INT			2
#define BIT_PHY_ERR_INT			3
#define BIT_CMD_INT			4
#define BIT_DMA_INT(ch)			(8 + (ch))
#define BIT_DMA1_INT			8
#define BIT_DMA2_INT			9
#define BIT_DEV_EP_INT(ep)		(16 + (ep))
/* -- FHDC_VARIANT_LAP -- */

#define LAPREG_CLKCTRL			0x10000
#define  LAP_CLKCTRL__PCKEN__SHIFT	1
#define  LAP_CLKCTRL__HCKEN__SHIFT	0
#define LAPREG_RSTCTL			0x10004
#define  LAP_RSTCTL__SFTRST__SHIFT	0
#define LAPREG_ANPDCTL			0x10008
#define  LAP_ANPDCTL__ANPDCTL__SHIFT	0 /* set = phy powerdown */
#define LAPREG_IDVBUSSEL		0x11000
#define  LAP_IDVBUSSEL__ID_VBUS_SEL__SHIFT	0 /* 0=macro drives, 1=usb core */
#define LAPREG_IDVBUSCTL		0x11004
#define  LAP_IDVBUSCTL__ID_PULLUP__SHIFT	1
#define  LAP_IDVBUSCTL__VBUS_CLIP__SHIFT	0
#define LAPREG_IDVBUSDET		0x11008
#define  LAP_IDVBUSDET__ID_DET_STATE__SHIFT	1
#define  LAP_IDVBUSDET__VBUS_DET_STATE__SHIFT	0
#define LAPREG_HDMAC1			0x12000
#define  LAP_HDMAC__HDMAC_MS1__SHIFT	6
#define  LAP_HDMAC__HDMAC_BT1__SHIFT	4
#define  LAP_HDMAC__HDMAC_BT1__MASK	3
#define  LAP_HDMAC__HDMAC_BC1__SHIFT	0
#define  LAP_HDMAC__HDMAC_BC1__MASK	15
#define LAPREG_HDMAC2			0x12004
/* same constants for HDMAC1 */
#define LAPREG_DMAFSM1			0x12008
#define  LAPREG_DMAFSM__DMA_TANS_STATE	BIT(1)
#define  LAPREG_DMAFSM__FSMRST		BIT(0)
#define LAPREG_DNAFSM2			0x1200c
/* same constants for DMAFSM1 */

/* HDC_EPCMD has different bit function between two mode */
#define HDC_EPCMD(n)			(0x0040 + ((n) * 0x4))	/* EPx Cmd */
#define BIT_START			0
#define BIT_STOP			1
#define BIT_INIT			2
#define BIT_BUFWR			3
#define BIT_BUFRD			4
#define BIT_TOGGLE_SET			7
#define BIT_TOGGLE_CLR			8
#define BIT_WRITE_EN			11
#define BIT_ET				23
#define LEN_ET				2
#define TYPE_UNUSED			0x0
#define TYPE_CONTROL			0x0
#define TYPE_ISOCHRONOUS		0x1
#define TYPE_BULK			0x2
#define TYPE_INTERRUPT			0x3
#define BIT_BNUM			26
#define LEN_BNUM			2
#define BIT_NEXTLINK			12	/* host mode */
#define LEN_NEXTLINK			3	/* host mode */
#define BIT_NLINKINVALID		15	/* host mode */
#define BIT_SENDPID			16	/* host mode */
#define LEN_SENDPID			2	/* host mode */
#define PID_OUT				0	/* host mode */
#define PID_IN				1	/* host mode */
#define PID_SETUP			2	/* host mode */
#define PID_PING			3	/* host mode */
#define BIT_ERRCNT_CLR			18	/* host mode */
#define BIT_STATUS_CLR			19	/* host mode */
#define BIT_SC				25	/* host mode */
#define BIT_SPEED			28	/* host mode */
#define LEN_SPEED			2	/* host mode */
/* endpoint speed request symbolic constant */
#define SPEED_FULL_SPEED		0	/* host mode */
#define SPEED_LOW_SPEED			1	/* host mode */
#define SPEED_HIGH_SPEED		2	/* host mode */
#define BIT_STALL_SET			5	/* dev mode */
#define BIT_STALL_CLR			6	/* dev mode */
#define BIT_NULLRESP			9	/* dev mode */
#define BIT_NACKRESP			10	/* dev mode */
#define BIT_RDYI_RDY_INTEN		12	/* dev mode */
#define BIT_RDYO_EMPTY_INTEN		13	/* dev mode */
#define BIT_PING_INTEN			14	/* dev mode */
#define BIT_STALLED_INTEN		15	/* dev mode */
#define BIT_NACK_INTEN			16	/* dev mode */
/* readyi_int_clr [Ep 0], ready_int_clr [Ep 1-15] */
#define BIT_RDYI_RDY_INT_CLR	18	/* dev mode */
/* readyo_int_clr [Ep 0] ready_empty_clr [Ep 1-15] */
#define BIT_RDYO_EMPTY_INT_CLR	19	/* dev mode */
#define BIT_PING_INT_CLR		20	/* dev mode */
#define BIT_STALLED_INT_CLR		21	/* dev mode */
#define BIT_NACK_INT_CLR		22		/* dev mode */
#define BIT_DIR				25	/* dev mode */
#define BIT_HIBAND			28	/* dev mode */
#define LEN_HIBAND			2	/* dev mode */

#define HDC_DEVC			0x0200 /* Device Control */
#define BIT_REQSPEED			0
/* bus speed request symbolic constant */
#define REQ_SPEED_HIGH_SPEED		0	/* high-speed request */
#define REQ_SPEED_FULL_SPEED		1	/* full-speed request */
#define BIT_REQRESUME			2
#define BIT_RMTWKUP			3
#define BIT_DISCONNECT			5
#define BIT_PHYSUSP			16
#define BIT_SUSPENDE_INTEN		24
#define BIT_SUSPENDB_INTEN		25
#define BIT_SOF_INTEN			26
#define BIT_SETUP_INTEN			27
#define BIT_USBRSTE_INTEN		28
#define BIT_USBRSTB_INTEN		29
#define BIT_STATUS_OK_INTEN		30
#define BIT_STATUS_NG_INTEN		31

#define HDC_DEVS			0x0204 /* Device Status */
#define BIT_SUSPEND			0
#define BIT_BUSRESET			1
#define BIT_PHYRESET			16
#define BIT_CRTSPEED			17
/* bus speed symbolic constant */
#define CRT_SPEED_HIGH_SPEED		0	/* high-speed */
#define CRT_SPEED_FULL_SPEED		1	/* full-speed */
#define BIT_SUSPENDE_INT		24	/* suspende_int */
#define BIT_SUSPENDB_INT		25	/* suspendb_int */
#define BIT_SOF_INT			26	/* sof_int */
#define BIT_SETUP_INT			27	/* setup_int */
#define BIT_USBRSTE_INT			28	/* usbrste_int */
#define BIT_USBRSTB_INT			29	/* usbrstb_int */
#define BIT_STATUS_OK_INT		30	/* status_ok_int */
#define BIT_STATUS_NG_INT		31	/* status_ng_int */

#define HDC_FADDR			0x0208 /* Function Address */
#define BIT_FUNC_ADDR			0
#define LEN_FUNC_ADDR			7
#define BIT_DEV_CONFIGURED		8

#define HDC_TSTAMP_R_ONLY		0x020c /* Device Time Stamp */
#define BIT_TIMESTAMP			0
#define LEN_TIMESTAMP			11

#define HDC_PORTSC			0x0100 /* Port Status  Control */
#define BIT_OVER_CURRENT		0
#define BIT_POWER_CTL_RHS		1
#define BIT_POWER_RHS			2
#define BIT_FORCEFS_RHS			3
#define BIT_CONNECTION_RHS		4
#define BIT_RESET_RHS			5
#define BIT_ENABLE_RHS			6
#define BIT_LS_RHS			7
#define BIT_HS_RHS			8
#define BIT_WAKEUP_RHS			9
#define BIT_SUSPENDED_RHS		10
#define BIT_RESUMING_RHS		11
#define BIT_POWER_CTL_REQ		21
#define BIT_POWER_REQ			22
#define BIT_FORCEFS_REQ			23
#define BIT_RESET_REQ			25 /* port_reset_req */
#define BIT_ENABLE_REQ			26 /* port_enable_req */
#define BIT_WAKEUP_REQ			29
#define BIT_SUSPEND_REQ			30 /* port_suspend_req */
#define BIT_RESUME_REQ			31 /* port_resume_req */
#define HDC_PORTSTSC			0x0104 /* Port Status Change */
#define BIT_CONNECTION_C		0 /* port_conection */
#define BIT_ENABLE_C			1 /* port_enable_c */
#define BIT_SUSPEND_C			2 /* port_suspend_c */
#define BIT_OV_CURR_C			3 /* port_ov_curr_c */
#define BIT_RESET_C			4 /* port_reset_c */
#define BIT_LINE_STATE			5
#define LEN_LINE_STATE			2
#define LINESTATE_DP_LOW_DM_LOW		0x0 /* D+ low level and D- low  */
#define LINESTATE_DP_HIGH_DM_LOW	0x1 /* D+ high level and D- low */
#define LINESTATE_DP_LOW_DM_HIGH	0x2 /* D+ low level and D- high  */
#define LINESTATE_DP_HIGH_DM_HIGH	0x3 /* D+ high level and D- high */
#define HDC_HOSTEVENTSRC		0x0108 /* Host Event Factor */
#define BIT_SOFSTART			0 /* sofstart */
#define BIT_FRAMEOV			1 /* frameov */
#define BIT_PORT_STATUS_C		2
#define BIT_TRANS_DONE(ep_ch)		(8 + (ep_ch)) /* trans_done[0:7] */
#define HDC_HOSTINTEN			0x010c /* Host Interrupt Enable */
#define BIT_SOFSTART_INTEN		0
#define BIT_FRAMEOV_INTEN		1
#define BIT_CONNECT_C_INTEN		2
#define BIT_ENABLE_C_INTEN		3
#define BIT_SUSPEND_C_INTEN		4
#define BIT_OV_CURRENT_C_INTEN		5
#define BIT_RESET_C_INTEN		6
#define BIT_TRANS_DONE_INTEN(ep_ch)	(8 + (ep_ch))
#define BIT_SOF_INTERVAL		16
#define LEN_SOF_INTERVAL		3
#define FRAME_1U			0x0	/* 1uFrame(125[us] */
#define FRAME_2U			0x1	/* 2uFrame(250[us] */
#define FRAME_4U			0x2	/* 4uFrame(500[us] */
#define FRAME_8U			0x3	/* 8uFrame(1[ms] */
#define HDC_HCFRMIDX			0x0110 /* HC Frame Index */
#define HDC_HCFRMINIT			0x0114 /* HC Frame Init */
#define HDC_HCCTRL			0x0118 /* HC Control */
#define HDC_HCSTLINK			0x011c /* HC Start Link */
#define HDC_OTGC			0x0300 /* OTG Control */
#define BIT_DM_PULL_DOWN		7
#define BIT_DP_PULL_DOWN		8
#define BIT_ID_PULL_UP			9
#define HDC_OTGSTS_R_ONLY		0x0310 /* OTG Status */
#define BIT_OTG_TMROUT			0
#define BIT_ID				6
#define BIT_VBUS_VLD			10
#define HDC_OTGSTSC			0x0314 /* OTG Status Change */
#define BIT_TMROUT_C			0
#define BIT_ID_C			6
#define BIT_VBUS_VLD_C			10
#define HDC_OTGSTSFALL			0x0318 /* OTG Status Fall Detect */
#define BIT_ID_FEN			6
#define BIT_VBUS_VLD_FEN		10
#define HDC_OTGSTSRISE			0x031c /* OTG Status Rise Detect */
#define BIT_TMROUT_REN			0
#define BIT_ID_REN			6
#define BIT_VBUS_VLD_REN		10
#define HDC_OTGTC			0x0320 /* OTG Timer Control */
#define BIT_START_TMR			0
#define HDC_OTGT			0x0324 /* OTG Timer */
#define BIT_TMR_INIT_VAL		0
#define LEN_TMR_INIT_VAL		16
#define HDC_DMAC(n)			(0x0400 + ((n) * 0x20)) /* n: 0~1 */
#define BIT_DMA_ST			0

#define BIT_DMA_MODE			2
#define MODE_DEMAND			0	/* demand transter */
#define MODE_BLOCK			1	/* block transter */

#define BIT_DMA_SENDNULL		3
#define BIT_DMA_INT_EMPTY		4
#define BIT_DMA_SPR			5
#define BIT_DMA_EP			8
#define LEN_DMA_EP			4
#define BIT_DMA_BLKSIZE			16
#define LEN_DMA_BLKSIZE			11
#define HDC_DMAS_R_ONLY(n)		(0x0404 + ((n) * 0x20)) /* DMA Status */
#define BIT_DMA_SP			1
/* DMA Total Trans Bytes */
#define HDC_DMATCI(n)			(0x0408 + ((n) * 0x20))
/* DMA Total Trans Bytes Counter */
#define HDC_DMATC_R_ONLY(n)		(0x040c + ((n) * 0x20))
#define HDC_TESTC			0x0500	/* Test Control */
#define BIT_TEST_P			0
#define BIT_TEST_J			1
#define BIT_TEST_K			2
#define BIT_TEST_SE0NACK		3
/* host mode, n is 0~7*/
#define HDC_HCEPCTRL2(n)		(0x8004 + ((n) * 0x8))
#define BIT_STARTBIT			1
#define BIT_HUB_PORT_NUM		4
#define LEN_HUB_PORT_NUM		4
#define BIT_HUBADDR			8
#define LEN_HUBADDR			4
#define BIT_EP_NUMBER			12
#define LEN_EP_NUMBER			4
#define BIT_FUNCADDR			16
#define LEN_FUNCADDR			4
#define BIT_INTERVAL			20
#define LEN_INTERVAL			10
#define BIT_FMTSEL			30
#define BIT_INTERVALSEL			31
#define HDC_HCEPCTRL1(n)		(0x8000 + ((n) * 0x8))
#define BIT_TRANS_EN			0
#define BIT_HCPTR			8
#define LEN_HCPTR			2
#define BIT_EMPTY			10
#define BIT_FULL			11
#define BIT_HCEPCTRL1_SC		12
#define BIT_TOGGLE			13
#define BIT_ERRCNT			14
#define LEN_ERRCNT			2
#define BIT_STATUS_HALT			16
#define BIT_STATUS_STALL		17
#define BIT_STATUS_MISSED_UFRAME	18
#define BIT_NYET_CNT			20
#define LEN_NYET_CNT			2
#define BIT_HCEPCTRL1_SENDPID		24
#define LEN_HCEPCTRL1_SENDPID		2
#define BIT_HCEPCTRL1_SPEED		26
#define LEN_HCEPCTRL1_SPEED		2
#define BIT_HCEPCTRL1_NEXTLINK		28
#define LEN_HCEPCTRL1_NEXTLINK		3
#define BIT_HCEPCTRL1_NLINKINVALID	31
/*dev mode*/
#define HDC_EPCTRL(n)			(0x8000 + ((n) * 0x4)) /* max n is 15 */
#define BIT_EP_EN			0
#define BIT_FULLI			9
#define BIT_EMPTYO_EMPTY		10
#define BIT_FULLO_FULL			11
#define BIT_STALL			12
#define BIT_EPCTRL_NACKRESP		17
#define BIT_READY_INTEN			18
#define BIT_EMPTY_INTEN			19
#define BIT_STALL_INTEN			21
#define BIT_EPCTRL_NACK_INTEN		22
#define BIT_RDYI_RDY_INT		26
#define BIT_RDYO_EMPTY_INT		27
#define BIT_STALLED_INT			29
#define BIT_NACK_INT			30

/*shared between HDC_EPCTRL & HDC_HCEPCTRL1*/
#define BIT_EPCTRL_ET			1
#define LEN_EPCTRL_ET			2
#define BIT_EPCTRL_BNUM			4
#define LEN_EPCTRL_BNUM			2
#define BIT_APPPTR			6
#define LEN_APPPTR			2

#define HDC_EPCONF(n)			(0x8040 + ((n) * 0x4)) /* max n is 15*/
#define BIT_BASE			0
#define LEN_BASE			13
#define BIT_SIZE			13
#define LEN_SIZE			11
#define BIT_COUNTIDX			24
#define LEN_COUNTIDX			5
#define HDC_EPCOUNT(n)			(0x8080 + ((n) * 0x4)) /* max n is 31*/
#define BIT_APPCNT			0
#define LEN_APPCNT			11
#define BIT_PHYCNT			16
#define LEN_PHYCNT			11
#define HDC_EPBUF			0x8100 /* HCEP Buffer  */
/* DMA 1 Addr Convert Area */
#define HDC_DMA_BUFFER(n)		(0x10000 + ((n) * 0x8000))

#endif /* __MACH_F_USB20HDC_H */
