#ifndef __DA217_H__
#define __DA217_H__

#include <stdint.h>
#include <stdbool.h>


/*REG definiton ------------------------------------------------------------------------------
 * */
#define DA217_REG_SPI_CFG				0x00	// RW
#define DA217_REG_CHIP_ID				0x01	// R
#define DA217_REG_ACC_X_LSB				0x02	// R
#define DA217_REG_ACC_X_MSB				0x03	// R
#define DA217_REG_ACC_Y_LSB				0x04	// R
#define DA217_REG_ACC_Y_MSB				0x05	// R
#define DA217_REG_ACC_Z_LSB				0x06	// R
#define DA217_REG_ACC_Z_MSB				0x07	// R
#define DA217_REG_FIFO_STATUS			0x08	// R
#define DA217_REG_MOTION_FLAG			0x09	// R
#define DA217_REG_NEWDATA_FLAG			0x0A	// R
#define DA217_REG_TAP_ACTIVE_STATUS		0x0B	// R
#define DA217_REG_ORIENT_STATUS			0x0C	// R
#define DA217_REG_STEPS_MSB				0x0D	// R
#define DA217_REG_STEPS_LSB				0x0E	// R
#define DA217_REG_RES_RANGE				0x0F	// RW
#define DA217_REG_ODR_AXIS				0x10	// RW
#define DA217_REG_MODE_BW				0x11	// RW
#define DA217_REG_SWAP_POLARITY			0x12	// RW
#define DA217_REG_FIFO_CTRL				0x14	// RW
#define DA217_REG_INT_SET0				0x15	// RW
#define DA217_REG_INT_SET1				0x16	// RW
#define DA217_REG_INT_SET2				0x17	// RW
#define DA217_REG_INT_MAP1				0x19	// RW
#define DA217_REG_INT_MAP2				0x1A	// RW
#define DA217_REG_INT_MAP3				0x1B	// RW
#define DA217_REG_INT_CONFIG			0x20	// RW
#define DA217_REG_INT_LATCH				0x21	// RW
#define DA217_REG_FREEFALL_DUR			0x22	// RW
#define DA217_REG_FREEFALL_THS			0x23	// RW
#define DA217_REG_FREEFALL_HYST			0x24	// RW
#define DA217_REG_ACTIVE_DUR			0x27	// RW
#define DA217_REG_ACTIVE_THS			0x28	// RW
#define DA217_REG_TAP_DUR				0x2A	// RW
#define DA217_REG_TAP_THS				0x2B	// RW
#define DA217_REG_ORIENT_HYST			0x2C	// RW
#define DA217_REG_Z_BLOCK				0x2D	// RW
#define DA217_REG_RESET_STEP			0x2E	// RW
#define DA217_REG_STEP_FILTER			0x33	// RW
#define DA217_REG_SM_THRESHOLD			0x34	// RW


/*REG STRUCT -----------------------------------------------------------------------------------
 * */
/* REG 01 */
#define DA217_CHIP_ID					0x13

/* REG 08 ~ 0CH */

// Sign of the first active & triggered interrupt
typedef enum
{
	DA217_SIGN_POSITIVE		= 0, // positive
	DA217_SIGN_NEGATIVE		= 1, // negative
}
da217_sign_t;

// orientation value of ‘z’ axis
typedef enum
{
	DA217_ORIENT_Z_UPWARD_LOOKING		= 0, // 0: upward looking,
	DA217_ORIENT_Z_DOWNWORD_LOOKING		= 1, // 1: downward looking
}
da217_orient_z_t;

// orientation value of ‘x’, ‘y’ axes
typedef enum
{
	DA217_ORIENT_XY_PORTRAIT_UPRIGHT		= 0,
	DA217_ORIENT_XY_PORTRAIT_UPSIDE_DOWN	= 1,
	DA217_ORIENT_XY_LANDSCAPE_LEFT			= 2,
	DA217_ORIENT_XY_LANDSCAPE_RIGHT			= 3,
}
da217_orient_xy_t;

// step status
typedef enum
{
	DA217_STEP_STATUS_IDLE	= 0, // 00/11: idle
	DA217_STEP_STATUS_WALK	= 1, // 01: walk
	DA217_STEP_STATUS_RUN	= 2, // 10: run
}da217_step_status_t;

typedef struct
{
	// fifo status reg @ 0x08
	uint8_t fifo_cnt	: 6; // FIFO_entries[5:0] reports how many data stored in the FIFO
	bool fifo_full		: 1; // 1: the FIFO is full
	bool watermark		: 1; // 1: the FIFO entries exceed the water mark level

	// motion status reg @ 0x09
	bool freefall	: 1; // 1: free fall interrupt has occurred
	bool tilt		: 1; // 1: tilt interrupt has occurred
	bool acitve		: 1; // 1: active interrupt has occurred
	bool sm			: 1; // 1: significant motion has be detected
	bool d_tap		: 1; // 1: double tap interrupt has occurred
	bool s_tap		: 1; // 1: single tap interrupt has occurred
	bool orient		: 1; // 1: orient interrupt has occurred
	bool step		: 1; // 1: one step detected

	// new data flag reg @ 0x0A
	bool newdata	: 1;	// 1: new_data interrupt has occurred
	uint8_t 		: 7;	// unused

	// tap active status reg @ 0x0B
	bool active_first_z			: 1; // 1: indicate Z is the triggering axis of the active interrupt.
	bool active_first_y			: 1; // 1: indicate Y is the triggering axis of the active interrupt.
	bool active_first_x			: 1; // 1: indicate X is the triggering axis of the active interrupt.
	da217_sign_t active_sign	: 1; // Sign of the first active interrupt. 1: negative, 0: positive
	bool tap_first_z			: 1; // 1: indicate Z is the triggering axis of the tap interrupt.
	bool tap_first_y			: 1; // 1: indicate Y is the triggering axis of the tap interrupt.
	bool tap_first_x			: 1; // 1: indicate X is the triggering axis of the tap interrupt.
	da217_sign_t tap_sign		: 1; // Sign of the first tap that triggered interrupt, 1: negative, 0: positive

	// orient status reg @ 0x0C
	da217_step_status_t step_status	: 2; // step status, idle / walk / run
	uint8_t 						: 2; // unused
	da217_orient_xy_t orient_xy		: 2; // orientation value of ‘x’, ‘y’ axes
	da217_orient_z_t orient_z		: 1; // orientation value of ‘z’ axis
	bool 							: 1; // unused
}
da217_status_t;

/* REG 08 */
//typedef struct
//{
//	uint8_t fifo_cnt	: 6; // FIFO_entries[5:0] reports how many data stored in the FIFO
//	bool fifo_full		: 1; // 1: the FIFO is full
//	bool watermark		: 1; // 1: the FIFO entries exceed the water mark level
//}
//da217_fifo_status_reg_t;

/* REG 09 */
//typedef struct
//{
//	bool freefall	: 1; // 1: free fall interrupt has occurred
//	bool tilt		: 1; // 1: tilt interrupt has occurred
//	bool acitve		: 1; // 1: active interrupt has occurred
//	bool sm			: 1; // 1: significant motion has be detected
//	bool d_tap		: 1; // 1: double tap interrupt has occurred
//	bool s_tap		: 1; // 1: single tap interrupt has occurred
//	bool orient		: 1; // 1: orient interrupt has occurred
//	bool step		: 1; // 1: one step detected
//}
//da217_motion_status_reg_t;

/* REG 0A */
//typedef struct
//{
//	bool	newdata	: 1;	// 1: new_data interrupt has occurred
//	uint8_t 		: 7;	// unused
//}
//da217_newdata_status_reg_t;

/* REG 0B */
//typedef struct
//{
//	bool active_first_z			: 1; // 1: indicate Z is the triggering axis of the active interrupt.
//	bool active_first_y			: 1; // 1: indicate Y is the triggering axis of the active interrupt.
//	bool active_first_x			: 1; // 1: indicate X is the triggering axis of the active interrupt.
//	da217_sign_t active_sign	: 1; // Sign of the first active interrupt. 1: negative, 0: positive
//	bool tap_first_z			: 1; // 1: indicate Z is the triggering axis of the tap interrupt.
//	bool tap_first_y			: 1; // 1: indicate Y is the triggering axis of the tap interrupt.
//	bool tap_first_x			: 1; // 1: indicate X is the triggering axis of the tap interrupt.
//	da217_sign_t tap_sign		: 1; // Sign of the first tap that triggered interrupt, 1: negative, 0: positive
//}
//da217_tap_active_status_reg_t;

/* REG 0CH ORIENT_STATUS */
// need to do...

/* REG 0FH */
typedef enum
{
	DA217_RANGE_2G  = 0,
	DA217_RANGE_4G  = 1,
	DA217_RANGE_8G  = 2,
	DA217_RANGE_16G = 3,
}
da217_range_t;

typedef enum
{
	DA217_RES_14BIT = 0,
	DA217_RES_12BIT = 1,
	DA217_RES_10BIT = 2,
	DA217_RES_8BIT  = 3,
}
da217_res_t;

#define DA217_HIGHPASS_FILTER_DISABLE	0
#define DA217_HIGHPASS_FILTER_ENABLE	1

#define DA217_WDT_DISABLE				0
#define DA217_WDT_ENABLE				1

#define DA217_WDT_1_MS					0
#define DA217_WDT_50_MS					1

typedef struct
{
	da217_range_t full_scale	: 2; // ref: da217_range_t
	da217_res_t resolution		: 2; // ref: da217_res_t
	bool wdt_time				: 1; // 1: 50ms, 0: 1ms
	bool wdt_en					: 1; // 1: enable watch dog
	bool hp_en					: 1; // 1: enable high pass filter
}
da217_range_reg_t;

/* REG 10H */
typedef enum
{
	DA217_ODR_1_HZ			= 0,	// 0000: 1Hz
	DA217_ODR_1_95_HZ		= 1,	// 0001: 1.95Hz
	DA217_ODR_3_9_HZ		= 2,	// 0010: 3.9Hz
	DA217_ODR_7_81_HZ		= 3,	// 0011: 7.81Hz
	DA217_ODR_15_63_HZ		= 4,	// 0100: 15.63Hz
	DA217_ODR_31_25_HZ		= 5,	// 0101: 31.25Hz
	DA217_ODR_62_5_HZ		= 6,	// 0110: 62.5Hz
	DA217_ODR_125_HZ		= 7,	// 0111: 125Hz
	DA217_ODR_250_HZ		= 8,	// 1000: 250Hz
	DA217_ODR_500_HZ		= 9,	// 1001: 500Hz
	DA217_ODR_1000_HZ		= 15,	// 1100-1111: 1000Hz
}
da217_odr_t;

typedef struct
{
	da217_odr_t odr		: 4;	// output data rate
	bool 				: 1;	// unused
	bool z_axis_dis		: 1;	// 1: disable Z axis
	bool y_axis_dis		: 1;	// 1: disable Y axis
	bool x_axis_dis		: 1;	// 1: disable X axis
}
da217_odr_axis_reg_t;

/* REG 10H */
#define DA217_PWR_NORMAL	0	// 0: normal mode
#define DA217_PWR_SUSPEND	1	// 1: suspend mode

typedef enum
{
	DA217_BW_100HZ = 2,	// 10:100hz
	DA217_BW_250HZ = 1,	// 01:250hz
	DA217_BW_500HZ = 0,	// 00/11:500hz
}
da217_bw_t;

typedef struct
{
	/* 0: working the current ODR state all the way
	 * 1: Working at 12.5hz in inactive state, automatic switched to normal mode during active state */
	bool auto_sleep_en	: 1;
	da217_bw_t bw		: 2; // ref: da217_bw_t
	uint8_t 			: 4; // unused
	bool power			: 1; // DA217_PWR_SUSPEND or DA217_PWR_NORMAL
}
da217_mode_reg_t;

/* REG 14H */
#define DA217_FIFO_CNT		32	// fifo count number

typedef enum
{
	DA217_FIFO_BYPASS	= 0, // 00: bypass mode
	DA217_FIFO_FIFO		= 1, // 01: FIFO mode
	DA217_FIFO_STREAM	= 2, // 10: stream mode
	DA217_FIFO_TRIGGER	= 3, // 11: trigger mode
}
da217_fifo_mode_t;

typedef struct
{
	uint8_t watermark_samples		: 5; // Indicate how many data entries needed to trig a water mark interrupt
	bool 							: 1; // usused
	da217_fifo_mode_t fifo_mode		: 2; // fifo mode, ref: da217_fifo_mode_t
}
da217_fifo_ctrl_reg_t;

/* REG 15H ~ 17H */
#define DA217_INT_EN_TILT			0x0001	// tilt
#define DA217_INT_EN_WATERMARK		0x0002	// water mark
#define DA217_INT_EN_FIFO_FULL		0x0004	// FIFO full
#define DA217_INT_EN_SM				0x0008	// significant motion
#define DA217_INT_EN_STEP			0x0010 	// step
#define DA217_INT_EN_S_TAP			0x0020	// single tap
#define DA217_INT_EN_D_TAP			0x0040	// double tap
#define DA217_INT_EN_ORIENT			0x0080	// orient
#define DA217_INT_EN_ACTIVE_Z		0x0100	// active z axis
#define DA217_INT_EN_ACTIVE_Y		0x0200	// active y axis
#define DA217_INT_EN_ACTIVE_X		0x0400	// active x axis
#define DA217_INT_EN_NEW_DATA		0x0800	// new data
#define DA217_INT_EN_FREE_FALL		0x1000	// free fall

/* REG 15H */
typedef struct
{
	bool step_int_en		: 1; // 1:enable step counter interrupt
	bool sm_int_en			: 1; // 1:enable SM(significant motion) interrupt
	bool fifo_full_int_en	: 1; // 1:enable FIFO_full interrupt
	bool watermark_int_en	: 1; // 1:enable water mark interrupt
	bool tilt_int_en		: 1; // 1:enable tilt interrupt
	uint8_t 				: 3; // unused
}
da217_int_set0_reg_t;

/* REG 16H */
typedef enum
{
	DA217_INTSRC_OVERSAPLING = 0, // 00: over sampling data (ODR_period =ODR*OSR)
	DA217_INTSRC_UNFILTERD	 = 1, // 01: unfiltered data (ODR_period =ODR)
	DA217_INTSRC_FILTERD	 = 2, // 10/11: filtered data (ODR_period =ODR IIR)
}
da217_intsource_t;

typedef struct
{
	bool active_int_x_en		: 1; // 1: enable the active interrupt for the x axis.
	bool active_int_y_en		: 1; // 1: enable the active interrupt for the y axis.
	bool active_int_z_en		: 1; // 1: enable the active interrupt for the z axis.
	bool orient_int_en			: 1; // 1: enable the orient interrupt.
	bool d_tap_int_en			: 1; // 1: enable the double tap interrupt.
	bool s_tap_int_en			: 1; // 1: enable the single tap interrupt.
	da217_intsource_t int_src	: 2; // ref: da217_intsource_t
}
da217_int_set1_reg_t;

/* REG 17H */
typedef enum
{
	DA217_TEMP_DIS_TIME_100MS	= 0, // 00:100ms
	DA217_TEMP_DIS_TIME_1S		= 1, // 01:1s
	DA217_TEMP_DIS_TIME_2S		= 2, // 10:2s
	DA217_TEMP_DIS_TIME_4S		= 3, // 11:4s
}
da217_temp_dis_time_t;

typedef struct
{
	uint8_t 								: 3;
	bool freefall_int_en					: 1; // 1: enable the free fall interrupt
	bool newdata_int_en						: 1; // 1: enable the new data interrupt.
	// temporary disable all interrupts for a short time(configured by temp_dis_time)
	da217_temp_dis_time_t temp_dis_time		: 2;
	bool temporary_disable					: 1;
}
da217_int_set2_reg_t;

/* REG 19H */
typedef enum
{
	DA217_SELECT_INT1_PIN,	// select INT1 pin
	DA217_SELECT_INT2_PIN,	// select INT2 pin
}
da217_int_pin_t;

typedef struct
{
	bool int1_freefall	: 1; // 1: mapping free fall interrupt to INT1
	bool int1_step		: 1; // 1: mapping step counter interrupt to INT1
	bool int1_active	: 1; // 1: mapping active interrupt to INT1
	bool int1_tilt		: 1; // 1: mapping tilt interrupt to INT1
	bool int1_d_tap		: 1; // 1: mapping double tap interrupt to INT1
	bool int1_s_tap		: 1; // 1: mapping single tap interrupt to INT1
	bool int1_orient	: 1; // 1: mapping orient interrupt to INT1
	bool int1_sm		: 1; // 1: mapping SM interrupt to INT1
}
da217_intmap1_reg_t;

/* REG 1AH */
typedef struct
{
	bool int1_newdata	: 1; // 1: mapping new data interrupt to INT1
	bool int1_watermark	: 1; // 1: mapping water mark interrupt to INT1
	bool int1_fifo_full	: 1; // 1: mapping FIFO full interrupt to INT1
	uint8_t 			: 2; // unused
	bool int2_fifo_full	: 1; // 1: mapping FIFO full interrupt to INT2
	bool int2_watermark	: 1; // 1: mapping water mark interrupt to INT2
	bool int2_newdata	: 1; // 1: mapping new data interrupt to INT2
}
da217_intmap2_reg_t;

/* REG 1BH */
typedef struct
{
	bool int2_freefall	: 1; // 1: mapping free fall interrupt to INT2
	bool int2_step		: 1; // 1: mapping step counter interrupt to INT2
	bool int2_active	: 1; // 1: mapping active interrupt to INT2
	bool int2_tilt		: 1; // 1: mapping tilt interrupt to INT2
	bool int2_d_tap		: 1; // 1: mapping double tap interrupt to INT2
	bool int2_s_tap		: 1; // 1: mapping single tap interrupt to INT2
	bool int2_orient	: 1; // 1: mapping orient interrupt to INT2
	bool int2_sm		: 1; // 1: mapping SM interrupt to INT2
}
da217_intmap3_reg_t;

/* REG 20H */
typedef enum
{
	DA217_INT_PUSH_PULL		= 0, // 0: select push-pull output for INTx
	DA217_INT_OPEN_DRAIN	= 1, // 1: selects OD output for INTx
}
da217_int_pp_od_t;

typedef enum
{
	DA217_INT_ACTIVE_LOW	= 1, // 1: selects active level low for pin INTx
	DA217_INT_ACTIVE_HIGH	= 0, // 0: selects active level high for pin INTx
}
da217_int_level_t;

typedef struct
{
	da217_int_level_t int1_level	: 1; // DA217_INT_ACTIVE_HIGH or DA217_INT_ACTIVE_LOW
	da217_int_pp_od_t int1_od_pp	: 1; // DA217_INT_PUSH_PULL or DA217_INT_OPEN_DRAIN
	da217_int_level_t int2_level	: 1; // DA217_INT_ACTIVE_HIGH or DA217_INT_ACTIVE_LOW
	da217_int_pp_od_t int2_od_pp	: 1; // DA217_INT_PUSH_PULL or DA217_INT_OPEN_DRAIN
	uint8_t 						: 3; // unused
	uint8_t reset_int				: 1; // Write '1' to reset all latched int
}
da217_intcfg_reg_t;

/* REG 21H */
typedef enum
{
	DA217_INT_LATCH_NONE		= 0, // 0000b / 1000b: non-latched
	DA217_INT_LATCH_250MS		= 1, // 0001b: temporary latched 250ms
	DA217_INT_LATCH_500MS		= 2, // 0010b: temporary latched 500ms
	DA217_INT_LATCH_1S			= 3, // 0011b: temporary latched 1s
	DA217_INT_LATCH_2S			= 4, // 0100b: temporary latched 2s
	DA217_INT_LATCH_4S			= 5, // 0101b: temporary latched 4s
	DA217_INT_LATCH_8S			= 6, // 0110b: temporary latched 8s
	DA217_INT_LATCH_FOREVER		= 7, // 0111b: latched
	DA217_INT_LATCH_1MS			= 9, // 1001b / 1010b: temporary latched 1ms
	DA217_INT_LATCH_2MS			= 11, // 1011: temporary latched 2ms
	DA217_INT_LATCH_25MS		= 12, // 1100: temporary latched 25ms
	DA217_INT_LATCH_50MS		= 13, // 1101: temporary latched 50ms
	DA217_INT_LATCH_100MS		= 14, // 1110: temporary latched 100ms
}
da217_int_latch_time_t;

typedef struct
{
	da217_int_latch_time_t latch_int1	: 4;
	da217_int_latch_time_t latch_int2	: 4;
}
da217_int_latch_reg_t;

/* REG 24H */
#define DA217_FREEFALL_SINGLE_MODE		0
#define DA217_FREEFALL_SUM_MODE			1

typedef enum
{
	DA217_FREEFALL_SYSTERESIS_NONE		= 0,
	DA217_FREEFALL_SYSTERESIS_125MS		= 1,
	DA217_FREEFALL_SYSTERESIS_250MS		= 2,
	DA217_FREEFALL_SYSTERESIS_375MS		= 3,
}
da217_freefall_hy_time_t;

typedef struct
{
	da217_freefall_hy_time_t hy		: 2; // the hysteresis for free fall detection
	bool freefall_mode				: 1; // DA217_FREEFALL_SUM_MODE or DA217_FREEFALL_SINGLE_MODE
	uint8_t 						: 5; // unused
}
da217_freefall_hyst_reg_t;

/* REG 27H */
typedef struct
{
	uint8_t active_dur		: 4; // Active duration time = (Active_dur + 1)* ODR_period
	uint8_t inactive_dur	: 4; // inactive duration time = (Inactive_dur + 1)* ODR_period
}
da217_active_dur_reg_t;

/* REG 2AH TAP_DUR */
/* REG 2BH TAP_THS */
/* REG 2CH ORIENT_HYST */
/* REG 2DH Z_BLOCK */
/* REG 2EH RESET_STEP */
/* REG 33H STEP_FILTER */
/* REG 34H SM_THRESHOLD */
// need to do...


/*! @name Resolution related parameters */
#define DA217_8_BIT_RES_MAX_VAL			UINT8_C(127)
#define DA217_8_BIT_RES_NEG_VAL			UINT8_C(256)
#define DA217_8_BIT_RES_SHIFT_VAL		UINT8_C(8)
#define DA217_10_BIT_RES_MAX_VAL		UINT8_C(511)
#define DA217_10_BIT_RES_NEG_VAL		UINT8_C(1024)
#define DA217_10_BIT_RES_SHIFT_VAL		UINT8_C(6)
#define DA217_12_BIT_RES_MAX_VAL		UINT8_C(2047)
#define DA217_12_BIT_RES_NEG_VAL		UINT8_C(4096)
#define DA217_12_BIT_RES_SHIFT_VAL		UINT8_C(4)
#define DA217_14_BIT_RES_MAX_VAL		UINT8_C(8191)
#define DA217_14_BIT_RES_NEG_VAL		UINT8_C(16384)
#define DA217_14_BIT_RES_SHIFT_VAL		UINT8_C(2)


/*-------------------------------------------------------------------------------------------
 * */
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}
da217_accel_data_t;

typedef da217_accel_data_t	da217_accel_mg_t;


/*REG read & write FUNC ------------------------------------------------------------------------
 * */
typedef bool (*da217_reg_read_t) (uint8_t reg_addr, uint8_t *data, uint32_t len);
typedef bool (*da217_reg_read_byte_t) (uint8_t reg_addr, uint8_t *data);
typedef bool (*da217_reg_write_byte_t) (uint8_t reg_addr, uint8_t val);
typedef void (*da217_sleep_ms_t) (uint32_t ms);

typedef struct
{
	da217_reg_read_t		reg_read;
	da217_reg_read_byte_t	reg_read_byte;
	da217_reg_write_byte_t	reg_write_byte;
	da217_sleep_ms_t		sleep_ms;
}
da217_rw_func_t;

typedef struct
{
	bool				is_inited;
	da217_range_t		range;
	da217_res_t			resolution;
	da217_bw_t			bw;
	float				odr;
	da217_fifo_mode_t	fifo_mode;

	/* mG per LSB
	 * if range 2G, resolution 14 BIT, then
	 * lsb_mg = 2000 / (2^14 / 2)
	 */
	float				lsb_mg;

	/* factor for Threshold of active interrupt
	 * */
	float				active_th_k;

	/*! Variables to determine maximum value, negating value and shift value
     *  to calculate raw accel x, y and z axis data
     */
    uint16_t maximum_val;
    uint16_t negating_val;
    uint8_t shift_value;
}
da217_cfg_t;


/*-------------------------------------------------------------------------------------------
 * */
bool da217_reset(void);

bool da217_dump_regs(uint8_t reg_addr_start, uint8_t reg_addr_end, uint8_t *reg_val);

bool da217_set_range_res_power_bw(da217_range_t range, da217_res_t resolution,
		bool power_mode, da217_bw_t bw);

bool da217_set_odr(bool en_x_axis, bool en_y_axis, bool en_z_axis, da217_odr_t odr);

bool da217_set_fifo(da217_fifo_mode_t mode, uint8_t watermark);

bool da217_int_config(uint16_t int_en_mask, da217_intsource_t int_src,
		da217_int_pin_t int_pin, da217_int_pp_od_t pp_od,
		da217_int_level_t active_level, da217_int_latch_time_t latch);

bool da217_set_active_int(uint16_t threshold_mg, uint8_t active_dur, uint8_t inactive_dur);

bool da217_get_all_status(da217_status_t *status);

bool da217_read_accel_mg(da217_accel_mg_t *accel);

bool da217_read_accel_fifo_mg(uint8_t fifo_cnt, da217_accel_mg_t *fifo_mg);

bool da217_init(da217_rw_func_t *rw_func);

#endif // __DA217_H__
