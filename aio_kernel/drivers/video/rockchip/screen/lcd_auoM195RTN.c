#ifndef _LCD_HJ080NA__
#define _LCD_HJ080NA__

/* Base */
#define SCREEN_TYPE		SCREEN_RGB
#define LVDS_FORMAT      	LVDS_8BIT_2
#define OUT_FACE		OUT_P888
#define DCLK			108000000
#define LCDC_ACLK       	500000000//312000000           //29 lcdc axi DMA ÆµÂÊ

/* Timing */
#define H_PW			32
#define H_BP			80
#define H_VD			1600
#define H_FP			48

#define V_PW			5
#define V_BP			18
#define V_VD			900
#define V_FP			3

#define LCD_WIDTH      	 	1600
#define LCD_HEIGHT     	 	900
/* Other */
#define DCLK_POL                0
#define DEN_POL			0
#define VSYNC_POL		0
#define HSYNC_POL		0

#define SWAP_RB			0
#define SWAP_RG			0
#define SWAP_GB			0 

#endif
