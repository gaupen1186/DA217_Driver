#include <string.h>
#include <math.h>
#include "da217.h"
#include "esp_err.h"

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"


static const char *TAG = "DA217";

#define DA217_SET_REG_BIT(REG, BIT)		((REG) |= (1 << (BIT)))
//#define DA217_CHECK_REG_BIT(REG, BIT)	(((REG) >> (BIT)) & 0x01)
//#define DA217_CLEAR_REG_BIT(REG, BIT)   ((REG) &= ~(1 << (BIT)))

/*-------------------------------------------------------------------------------------------
 * */
static da217_cfg_t da217_cfg = { .is_inited = false };
static da217_rw_func_t	da217 = { NULL };

/*-------------------------------------------------------------------------------------------
 * */
static void da217_raw_data_convert(const uint8_t *data_array, da217_accel_data_t *accel)
{
	uint32_t reg_data;

	/* Accel X axis data */
	reg_data = (uint32_t)(((uint16_t)(data_array[1] << 8) | data_array[0]) >> da217_cfg.shift_value);
	if (reg_data > da217_cfg.maximum_val)
	{
		/* Computing accel data negative value */
		accel->x = (int16_t)(reg_data - da217_cfg.negating_val);
	}
	else
	{
		accel->x = (int16_t)reg_data;
	}

	/* Accel Y axis data */
	reg_data = (uint32_t)(((uint16_t)(data_array[3] << 8) | data_array[2]) >> da217_cfg.shift_value);
	if (reg_data > da217_cfg.maximum_val)
	{
		/* Computing accel data negative value */
		accel->y = (int16_t)(reg_data - da217_cfg.negating_val);
	}
	else
	{
		accel->y = (int16_t)reg_data;
	}

	/* Accel Z axis data */
	reg_data = (uint32_t)(((uint16_t)(data_array[5] << 8) | data_array[4]) >> da217_cfg.shift_value);
	if (reg_data > da217_cfg.maximum_val)
	{
		/* Computing accel data negative value */
		accel->z = (int16_t)(reg_data - da217_cfg.negating_val);
	}
	else
	{
		accel->z = (int16_t)reg_data;
	}
	/*
	// 转为 int16_t x y z
	int16_t *tmp = (int16_t *)data;
	tmp[0] = (data_array[1] << 8) | data_array[0];	// x
	tmp[1] = (data_array[3] << 8) | data_array[2];	// y
	tmp[2] = (data_array[5] << 8) | data_array[4];	// z

	uint8_t i;
	for(i = 0; i < 3; i++)
	{
		// 若高位为 1 则为负数, 需要将其从补码转回原码
		if(CHECK_BIT(tmp[i], 16) == 1)
		{
			tmp[i] -= 1;			// 补码 -1 变反码
			tmp[i] = ~tmp[i];		// 反码取反变原码(此处同时取反了最高符号位，符号位不能修改)
			SET_BIT(tmp[i], 16);	// 重新改回符号位
		}
		// 根据精度右移去除未使用的低位
		tmp[i] >>= da217_cfg.unused_bits;
	}
	 */
}

static void da217_convert_data_to_mg(const da217_accel_data_t *data, da217_accel_mg_t *mg)
{
	mg->x = (int16_t)(da217_cfg.lsb_mg * data->x);
	mg->y = (int16_t)(da217_cfg.lsb_mg * data->y);
	mg->z = (int16_t)(da217_cfg.lsb_mg * data->z);
}

/*-------------------------------------------------------------------------------------------
 * */
bool da217_reset(void)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	int8_t ret = 0;
	uint8_t val;

	// read back, set soft reset bit, and write back
	ret += ! da217.reg_read_byte(DA217_REG_SPI_CFG, &val);
	DA217_SET_REG_BIT(val, 2);
	DA217_SET_REG_BIT(val, 5);
	ret += ! da217.reg_write_byte(DA217_REG_SPI_CFG, val);

	da217.sleep_ms(20);

	if(ret != 0)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	ESP_LOGI(TAG, "[%s] OK!", __func__);
	return true;
}

bool da217_dump_regs(uint8_t reg_addr_start, uint8_t reg_addr_end, uint8_t *reg_val)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	int8_t reg_len = 1 + reg_addr_end - reg_addr_start;
	if(reg_len > 0)
		return da217.reg_read(reg_addr_start, reg_val, reg_len);
	else
		return false;
}

bool da217_set_range_res_power_bw(da217_range_t range, da217_res_t resolution,
		bool power_mode, da217_bw_t bw)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	int8_t ret = 0;
	uint8_t reg_val;

	reg_val = 0x40; // reset default value
	da217_range_reg_t *reg_range = (da217_range_reg_t *)&reg_val;
	reg_range->hp_en = false;
	reg_range->wdt_en = false;
	reg_range->wdt_time = 0;
	reg_range->resolution = resolution;
	reg_range->full_scale = range;
	ret += ! da217.reg_write_byte(DA217_REG_RES_RANGE, reg_val);

	reg_val = 0x9E;	// reset default value
	da217_mode_reg_t *reg_mode = (da217_mode_reg_t *)&reg_val;
	reg_mode->power = power_mode;
	reg_mode->bw = bw;
	reg_mode->auto_sleep_en = 0;
	ret += ! da217.reg_write_byte(DA217_REG_MODE_BW, reg_val);

	if(ret != 0)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	da217_cfg.range = range;
	da217_cfg.bw = bw;
	da217_cfg.resolution = resolution;

	uint16_t range_mg;
	if(range == DA217_RANGE_2G)
		range_mg = 2000;
	else if(range == DA217_RANGE_4G)
		range_mg = 4000;
	else if(range == DA217_RANGE_8G)
		range_mg = 8000;
	else if(range == DA217_RANGE_16G)
		range_mg = 16000;
	else
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	/* Resolution based data conversion */
	if(resolution == DA217_RES_8BIT)
	{
		da217_cfg.maximum_val = DA217_8_BIT_RES_MAX_VAL;
		da217_cfg.negating_val = DA217_8_BIT_RES_NEG_VAL;
		da217_cfg.shift_value = DA217_8_BIT_RES_SHIFT_VAL;
		da217_cfg.lsb_mg = (float)range_mg / DA217_8_BIT_RES_MAX_VAL;
	}
	else if(resolution == DA217_RES_10BIT)
	{
		da217_cfg.maximum_val = DA217_10_BIT_RES_MAX_VAL;
		da217_cfg.negating_val = DA217_10_BIT_RES_NEG_VAL;
		da217_cfg.shift_value = DA217_10_BIT_RES_SHIFT_VAL;
		da217_cfg.lsb_mg = (float)range_mg / DA217_10_BIT_RES_MAX_VAL;;
	}
	else if(resolution == DA217_RES_12BIT)
	{
		da217_cfg.maximum_val = DA217_12_BIT_RES_MAX_VAL;
		da217_cfg.negating_val = DA217_12_BIT_RES_NEG_VAL;
		da217_cfg.shift_value = DA217_12_BIT_RES_SHIFT_VAL;
		da217_cfg.lsb_mg = (float)range_mg / DA217_12_BIT_RES_MAX_VAL;;
	}
	else if(resolution == DA217_RES_14BIT)
	{
		da217_cfg.maximum_val = DA217_14_BIT_RES_MAX_VAL;
		da217_cfg.negating_val = DA217_14_BIT_RES_NEG_VAL;
		da217_cfg.shift_value = DA217_14_BIT_RES_SHIFT_VAL;
		da217_cfg.lsb_mg = (float)range_mg / DA217_14_BIT_RES_MAX_VAL;;
	}
	else
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	ESP_LOGI(TAG, "[%s] OK!", __func__);
	return true;
}

bool da217_set_odr(bool en_x_axis, bool en_y_axis, bool en_z_axis, da217_odr_t odr)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	if(odr == DA217_ODR_1_HZ)
		da217_cfg.odr = 1.0f;
	else if(odr == DA217_ODR_1_95_HZ)
		da217_cfg.odr = 1.95f;
	else if(odr == DA217_ODR_3_9_HZ)
		da217_cfg.odr = 3.9f;
	else if(odr == DA217_ODR_7_81_HZ)
		da217_cfg.odr = 7.81f;
	else if(odr == DA217_ODR_15_63_HZ)
		da217_cfg.odr = 15.63f;
	else if(odr == DA217_ODR_31_25_HZ)
		da217_cfg.odr = 31.25f;
	else if(odr == DA217_ODR_62_5_HZ)
		da217_cfg.odr = 62.5f;
	else if(odr == DA217_ODR_125_HZ)
		da217_cfg.odr = 125.0f;
	else if(odr == DA217_ODR_250_HZ)
		da217_cfg.odr = 250.0f;
	else if(odr == DA217_ODR_500_HZ)
		da217_cfg.odr = 500.0f;
	else if(odr == DA217_ODR_1000_HZ)
		da217_cfg.odr = 1000.0f;
	else
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	uint8_t val = 0x0F;	// set default value
	da217_odr_axis_reg_t *reg = (da217_odr_axis_reg_t *) &val;
	reg->x_axis_dis = ! en_x_axis;
	reg->y_axis_dis = ! en_y_axis;
	reg->z_axis_dis = ! en_z_axis;
	reg->odr = odr;

	bool ret = da217.reg_write_byte(DA217_REG_ODR_AXIS, val);
	if(ret != true)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	ESP_LOGI(TAG, "[%s] OK!", __func__);
	return true;
}

bool da217_set_fifo(da217_fifo_mode_t mode, uint8_t watermark)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	uint8_t reg_val = 0x00;

	da217_fifo_ctrl_reg_t *reg = (da217_fifo_ctrl_reg_t *)&reg_val;
	reg->fifo_mode = mode;
	if(watermark > (DA217_FIFO_CNT - 1))
		reg->watermark_samples = DA217_FIFO_CNT - 1;
	else
		reg->watermark_samples = watermark;

	bool ret = da217.reg_write_byte(DA217_REG_FIFO_CTRL, reg_val);

	if(ret != true)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	da217_cfg.fifo_mode = mode;
	ESP_LOGI(TAG, "[%s] OK!", __func__);
	return true;
}

/* 实测发现开启 watermark 和 fifo full 的管脚映射但未使能 watermark 和 fifo full 中断后，
 * 会导致 active 中断失效无反应 */
bool da217_int_config(uint16_t int_en_mask, da217_intsource_t int_src,
		da217_int_pin_t int_pin, da217_int_pp_od_t pp_od,
		da217_int_level_t active_level, da217_int_latch_time_t latch)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	uint8_t val[8];
	memset(val, 0x00,8);

	da217_int_set0_reg_t *int_set_reg0 = (da217_int_set0_reg_t *) &val[0];
	da217_int_set1_reg_t *int_set_reg1 = (da217_int_set1_reg_t *) &val[1];
	da217_int_set2_reg_t *int_set_reg2 = (da217_int_set2_reg_t *) &val[2];
	da217_intmap1_reg_t *int_map_reg1 = (da217_intmap1_reg_t *) &val[3];
	da217_intmap2_reg_t *int_map_reg2 = (da217_intmap2_reg_t *) &val[4];
	da217_intmap3_reg_t *int_map_reg3 = (da217_intmap3_reg_t *) &val[5];
	da217_intcfg_reg_t *int_cfg_reg = (da217_intcfg_reg_t *) &val[6];
	da217_int_latch_reg_t *int_latch_reg = (da217_int_latch_reg_t *) &val[7];

	if((int_en_mask & DA217_INT_EN_TILT) !=0)
	{
		int_set_reg0->tilt_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_tilt = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_tilt = 1;
	}
	if((int_en_mask & DA217_INT_EN_WATERMARK) !=0)
	{
		int_set_reg0->watermark_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg2->int1_watermark = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg2->int2_watermark = 1;
	}
	if((int_en_mask & DA217_INT_EN_FIFO_FULL) !=0)
	{
		int_set_reg0->fifo_full_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg2->int1_fifo_full = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg2->int2_fifo_full = 1;
	}
	if((int_en_mask & DA217_INT_EN_SM) !=0)
	{
		int_set_reg0->sm_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_sm = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_sm = 1;
	}
	if((int_en_mask & DA217_INT_EN_STEP) !=0)
	{
		int_set_reg0->step_int_en =1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_step = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_step = 1;
	}
	if((int_en_mask & DA217_INT_EN_S_TAP) !=0)
	{
		int_set_reg1->s_tap_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_s_tap = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_s_tap = 1;
	}
	if((int_en_mask & DA217_INT_EN_D_TAP) !=0)
	{
		int_set_reg1->d_tap_int_en =1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_d_tap = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_d_tap = 1;
	}
	if((int_en_mask & DA217_INT_EN_ORIENT) !=0)
	{
		int_set_reg1->orient_int_en =1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_orient = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_orient = 1;
	}
	if((int_en_mask & DA217_INT_EN_ACTIVE_Z) !=0)
	{
		int_set_reg1->active_int_z_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_active = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_active = 1;
	}
	if((int_en_mask & DA217_INT_EN_ACTIVE_Y) !=0)
	{
		int_set_reg1->active_int_y_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_active = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_active = 1;
	}
	if((int_en_mask & DA217_INT_EN_ACTIVE_X) !=0)
	{
		int_set_reg1->active_int_x_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_active = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_active = 1;
	}
	if((int_en_mask & DA217_INT_EN_NEW_DATA) !=0)
	{
		int_set_reg2->newdata_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg2->int1_newdata = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg2->int2_newdata = 1;
	}
	if((int_en_mask & DA217_INT_EN_FREE_FALL) !=0)
	{
		int_set_reg2->freefall_int_en = 1;
		if(int_pin == DA217_SELECT_INT1_PIN)
			int_map_reg1->int1_freefall = 1;
		else if(int_pin == DA217_SELECT_INT2_PIN)
			int_map_reg3->int2_freefall = 1;
	}

	if(int_pin == DA217_SELECT_INT1_PIN)
	{
		int_cfg_reg->int1_od_pp = pp_od;
		int_cfg_reg->int1_level = active_level;
		int_latch_reg->latch_int1 = latch;
	}
	else if(int_pin == DA217_SELECT_INT2_PIN)
	{
		int_cfg_reg->int2_od_pp = pp_od;
		int_cfg_reg->int2_level = active_level;
		int_latch_reg->latch_int2 = latch;
	}

	int_set_reg1->int_src = int_src;
	int_set_reg2->temporary_disable = 0;

	int8_t ret = 0;
	ret += ! da217.reg_write_byte(DA217_REG_INT_SET0, val[0]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_SET1, val[1]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_SET2, val[2]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_MAP1, val[3]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_MAP2, val[4]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_MAP3, val[5]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_CONFIG, val[6]);
	ret += ! da217.reg_write_byte(DA217_REG_INT_LATCH, val[7]);
	if(ret != 0)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}
	ESP_LOGI(TAG, "[%s] OK!", __func__);
	return true;
}

bool da217_set_active_int(uint16_t threshold_mg, uint8_t active_dur, uint8_t inactive_dur)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	if(active_dur > 0x0F || inactive_dur > 0x0F)	// 0b1111, 4bits max
		return false;

	int8_t ret = 0;
	uint8_t val;

	da217_active_dur_reg_t *reg1 = (da217_active_dur_reg_t *) &val;
	reg1->active_dur = active_dur;
	reg1->inactive_dur = inactive_dur;
	ret += ! da217.reg_write_byte(DA217_REG_ACTIVE_DUR, val);

	float k;
	if(da217_cfg.range == DA217_RANGE_2G)
		k = 3.91;
	else if(da217_cfg.range == DA217_RANGE_4G)
		k = 7.81;
	else if(da217_cfg.range == DA217_RANGE_8G)
		k = 15.625;
	else if(da217_cfg.range == DA217_RANGE_16G)
		k = 31.25;
	else
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	val = (uint8_t)round(threshold_mg / k);
	ret += ! da217.reg_write_byte(DA217_REG_ACTIVE_THS, val);

	if(ret != 0)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}
	ESP_LOGI(TAG, "[%s] OK!", __func__);
	return true;
}

bool da217_get_all_status(da217_status_t *status)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	if(status == NULL)
		return false;

	bool ret = da217.reg_read(DA217_REG_FIFO_STATUS, (uint8_t *)status, sizeof(da217_status_t));
	if(ret != true)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}
	return true;
}

// read one frame accel data
bool da217_read_accel_mg(da217_accel_mg_t *accel)
{
	if(da217_cfg.is_inited != true)
	{
		ESP_LOGE(TAG, "DA217 NOT init yet! line %d", __LINE__);
		return false;
	}

	// read raw data to buffer
	uint8_t raw[6];
	bool ret = da217.reg_read(DA217_REG_ACC_X_LSB, raw, 6);
	if(ret != true)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	da217_accel_data_t data;
	da217_raw_data_convert(raw, &data);
	da217_convert_data_to_mg(&data, accel);
	return true;
}

bool da217_read_accel_fifo_mg(uint8_t fifo_cnt, da217_accel_mg_t *fifo_mg)
{
	if(fifo_cnt == 0 || fifo_mg == NULL)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	uint8_t i;
	for(i = 0; i < fifo_cnt; i++)
	{
		if(da217_read_accel_mg(&fifo_mg[i]) != true)
		{
			ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		}
	}
	return true;
}

bool da217_init(da217_rw_func_t *rw_func)
{
	if(rw_func == NULL)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}

	da217.reg_read = rw_func->reg_read;
	da217.reg_read_byte = rw_func->reg_read_byte;
	da217.reg_write_byte = rw_func->reg_write_byte;
	da217.sleep_ms = rw_func->sleep_ms;

	uint8_t chipid;
	bool ret = da217.reg_read_byte(DA217_REG_CHIP_ID, &chipid);
	if(ret != true)
	{
		ESP_LOGE(TAG, "[%s] error, line %d", __func__, __LINE__);
		return false;
	}
	if(chipid == DA217_CHIP_ID)
	{
		ESP_LOGI(TAG, "[%s] da217 chip id OK", __func__);
	}
	else
	{
		ESP_LOGE(TAG, "[%s] DA217_CHIP_ID error, line %d", __func__, __LINE__);
		return false;
	}

	da217_cfg.is_inited = true;

	return da217_reset();
}
