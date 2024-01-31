#include "bmp280.h"
#include <zephyr/drivers/i2c.h>

static struct bmp280_t *p_bmp280; /**< pointer to BMP280 */
u32 v_actual_press_combined_u32;
s32 v_actual_temp_combined_s32;
//  I2C init
#define I2C1_NODE DT_NODELABEL(bmp280)
static const struct i2c_dt_spec bmp_spec =  I2C_DT_SPEC_GET(I2C1_NODE);

struct bmp280_t bmp280;

BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_INIT_VALUE;
	u8 v_chip_id_read_count = BMP280_CHIP_ID_READ_COUNT;

	p_bmp280 = bmp280;/* assign BMP280 ptr */

	while (v_chip_id_read_count > 0) {
		/* read chip id */
		com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(BMP280_CHIP_ID_REG, &v_data_u8,
				BMP280_GEN_READ_WRITE_DATA_LENGTH);
		/* Check for the correct chip id */
		if ((v_data_u8 == BMP280_CHIP_ID1)
			|| (v_data_u8 == BMP280_CHIP_ID2)
			|| (v_data_u8 == BMP280_CHIP_ID3))
			break;
		v_chip_id_read_count--;
		/* Delay added concerning the low speed of power up system to
		facilitate the proper reading of the chip ID */
		p_bmp280->delay_msec(BMP280_REGISTER_READ_DELAY);
	}

	/*assign chip ID to the global structure*/
	p_bmp280->chip_id = v_data_u8;
	/*com_rslt status of chip ID read*/
	com_rslt = (v_chip_id_read_count == BMP280_INIT_VALUE) ?
			BMP280_CHIP_ID_READ_FAIL : BMP280_CHIP_ID_READ_SUCCESS;

	if (com_rslt == BMP280_CHIP_ID_READ_SUCCESS) {
		/* readout bmp280 calibration parameter structure */
		com_rslt += bmp280_get_calib_param();
	}
	return com_rslt;
}
s32 bmp280_compensate_temperature_int32(s32 v_uncomp_temperature_s32)
{
	s32 v_x1_u32r = BMP280_INIT_VALUE;
	s32 v_x2_u32r = BMP280_INIT_VALUE;
	s32 temperature = BMP280_INIT_VALUE;
	/* calculate true temperature*/
	/*calculate x1*/
	v_x1_u32r = ((((v_uncomp_temperature_s32
			>> BMP280_SHIFT_BIT_POSITION_BY_03_BITS)
			- ((s32)p_bmp280->calib_param.dig_T1
			<< BMP280_SHIFT_BIT_POSITION_BY_01_BIT)))
			* ((s32)p_bmp280->calib_param.dig_T2))
			>> BMP280_SHIFT_BIT_POSITION_BY_11_BITS;
	/*calculate x2*/
	v_x2_u32r = (((((v_uncomp_temperature_s32
			>> BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
			- ((s32)p_bmp280->calib_param.dig_T1))
			* ((v_uncomp_temperature_s32
			>> BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
			- ((s32)p_bmp280->calib_param.dig_T1)))
			>> BMP280_SHIFT_BIT_POSITION_BY_12_BITS)
			* ((s32)p_bmp280->calib_param.dig_T3))
			>> BMP280_SHIFT_BIT_POSITION_BY_14_BITS;
	/*calculate t_fine*/
	p_bmp280->calib_param.t_fine = v_x1_u32r + v_x2_u32r;
	/*calculate temperature*/
	temperature = (p_bmp280->calib_param.t_fine * 5 + 128)
			>> BMP280_SHIFT_BIT_POSITION_BY_08_BITS;

	return temperature;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure(
		s32 *v_uncomp_pressure_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	/* Array holding the MSB and LSb value
	 a_data_u8[0] - Pressure MSB
	 a_data_u8[1] - Pressure LSB
	 a_data_u8[2] - Pressure LSB
	 */
	u8 a_data_u8[BMP280_PRESSURE_DATA_SIZE] = {BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE};
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
		com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
				BMP280_PRESSURE_MSB_REG, a_data_u8,
				BMP280_PRESSURE_DATA_LENGTH);
		*v_uncomp_pressure_s32 = (s32)((((u32)(
				a_data_u8[BMP280_PRESSURE_MSB_DATA]))
				<< BMP280_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((u32)(a_data_u8[BMP280_PRESSURE_LSB_DATA]))
				<< BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((u32)a_data_u8[BMP280_PRESSURE_XLSB_DATA]
				>> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));
	}
	return com_rslt;
}
u32 bmp280_compensate_pressure_int32(s32 v_uncomp_pressure_s32)
{
	s32 v_x1_u32r = BMP280_INIT_VALUE;
	s32 v_x2_u32r = BMP280_INIT_VALUE;
	u32 v_pressure_u32 = BMP280_INIT_VALUE;
	/* calculate x1*/
	v_x1_u32r = (((s32)p_bmp280->calib_param.t_fine)
			>> BMP280_SHIFT_BIT_POSITION_BY_01_BIT) - (s32)64000;
	/* calculate x2*/
	v_x2_u32r = (((v_x1_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
			* (v_x1_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS))
			>> BMP280_SHIFT_BIT_POSITION_BY_11_BITS)
			* ((s32)p_bmp280->calib_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r *
			((s32)p_bmp280->calib_param.dig_P5))
			<< BMP280_SHIFT_BIT_POSITION_BY_01_BIT);
	v_x2_u32r = (v_x2_u32r >> BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
			+ (((s32)p_bmp280->calib_param.dig_P4)
			<< BMP280_SHIFT_BIT_POSITION_BY_16_BITS);
	/* calculate x1*/
	v_x1_u32r = (((p_bmp280->calib_param.dig_P3
			* (((v_x1_u32r
			>> BMP280_SHIFT_BIT_POSITION_BY_02_BITS) * (v_x1_u32r
			>> BMP280_SHIFT_BIT_POSITION_BY_02_BITS))
			>> BMP280_SHIFT_BIT_POSITION_BY_13_BITS))
			>> BMP280_SHIFT_BIT_POSITION_BY_03_BITS)
			+ ((((s32)p_bmp280->calib_param.dig_P2)
			* v_x1_u32r)
			>> BMP280_SHIFT_BIT_POSITION_BY_01_BIT))
			>> BMP280_SHIFT_BIT_POSITION_BY_18_BITS;
	v_x1_u32r = ((((32768 + v_x1_u32r))
			* ((s32)p_bmp280->calib_param.dig_P1))
			>> BMP280_SHIFT_BIT_POSITION_BY_15_BITS);
	/* calculate pressure*/
	v_pressure_u32 = (((u32)(((s32)1048576) - v_uncomp_pressure_s32)
			- (v_x2_u32r >> BMP280_SHIFT_BIT_POSITION_BY_12_BITS)))
			* 3125;
	/* check overflow*/
	if (v_pressure_u32 < 0x80000000)
		/* Avoid exception caused by division by zero */
		if (v_x1_u32r != BMP280_INIT_VALUE)
			v_pressure_u32 = (v_pressure_u32
					<< BMP280_SHIFT_BIT_POSITION_BY_01_BIT)
					/ ((u32)v_x1_u32r);
		else
			return BMP280_INVALID_DATA;
	else
	/* Avoid exception caused by division by zero */
	if (v_x1_u32r != BMP280_INIT_VALUE)
		v_pressure_u32 = (v_pressure_u32 / (u32)v_x1_u32r) * 2;
	else
		return BMP280_INVALID_DATA;
	/* calculate x1*/
	v_x1_u32r = (((s32)p_bmp280->calib_param.dig_P9) * ((s32)(
			((v_pressure_u32
			>> BMP280_SHIFT_BIT_POSITION_BY_03_BITS)
			* (v_pressure_u32
			>> BMP280_SHIFT_BIT_POSITION_BY_03_BITS))
			>> BMP280_SHIFT_BIT_POSITION_BY_13_BITS)))
			>> BMP280_SHIFT_BIT_POSITION_BY_12_BITS;
	/* calculate x2*/
	v_x2_u32r = (((s32)(v_pressure_u32 >>
			BMP280_SHIFT_BIT_POSITION_BY_02_BITS))
			* ((s32)p_bmp280->calib_param.dig_P8))
			>> BMP280_SHIFT_BIT_POSITION_BY_13_BITS;
	/* calculate true pressure*/
	v_pressure_u32 = (u32)((s32)v_pressure_u32 + ((v_x1_u32r + v_x2_u32r
			+ p_bmp280->calib_param.dig_P7)
			>> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));

	return v_pressure_u32;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uncomp_pressure_temperature(
		s32 *v_uncomp_pressure_s32, s32 *v_uncomp_temperature_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[BMP280_ALL_DATA_FRAME_LENGTH] = {BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE};
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
                        com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
				BMP280_PRESSURE_MSB_REG, a_data_u8,
				BMP280_DATA_FRAME_SIZE);
                /*Pressure*/
		*v_uncomp_pressure_s32 = (s32)((((u32)(
				a_data_u8[BMP280_DATA_FRAME_PRESSURE_MSB_BYTE]))
				<< BMP280_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((u32)(
				a_data_u8[BMP280_DATA_FRAME_PRESSURE_LSB_BYTE]))
				<< BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((u32)a_data_u8[
				BMP280_DATA_FRAME_PRESSURE_XLSB_BYTE]
				>> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));

		/* Temperature */
		*v_uncomp_temperature_s32 = (s32)((((u32)(a_data_u8[
				BMP280_DATA_FRAME_TEMPERATURE_MSB_BYTE]))
				<< BMP280_SHIFT_BIT_POSITION_BY_12_BITS)
				| (((u32)(a_data_u8[
				BMP280_DATA_FRAME_TEMPERATURE_LSB_BYTE]))
				<< BMP280_SHIFT_BIT_POSITION_BY_04_BITS)
				| ((u32)a_data_u8[
				BMP280_DATA_FRAME_TEMPERATURE_XLSB_BYTE]
				>> BMP280_SHIFT_BIT_POSITION_BY_04_BITS));
	}
	return com_rslt;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_read_pressure_temperature(
		u32 *v_pressure_u32, s32 *v_temperature_s32)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	s32 v_uncomp_pressure_s32 = BMP280_INIT_VALUE;
	s32 v_uncomp_temperature_s32 = BMP280_INIT_VALUE;
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
		/* read uncompensated pressure and temperature*/
		com_rslt = bmp280_read_uncomp_pressure_temperature(
				&v_uncomp_pressure_s32,
				&v_uncomp_temperature_s32);
		/* read true pressure and temperature*/
		*v_temperature_s32 = bmp280_compensate_temperature_int32(
				v_uncomp_temperature_s32);
		*v_pressure_u32 = bmp280_compensate_pressure_int32(
				v_uncomp_pressure_s32);
	}
	return com_rslt;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param(void)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 a_data_u8[BMP280_CALIB_DATA_SIZE] = {BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE, BMP280_INIT_VALUE,
			BMP280_INIT_VALUE, BMP280_INIT_VALUE};
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
				
                        com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,
				a_data_u8,
				BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH);
		/* read calibration values*/
		p_bmp280->calib_param.dig_T1 = (u16)((((u16)((u8)a_data_u8[
					BMP280_TEMPERATURE_CALIB_DIG_T1_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_TEMPERATURE_CALIB_DIG_T1_LSB]);
		p_bmp280->calib_param.dig_T2 = (s16)((((s16)((s8)a_data_u8[
					BMP280_TEMPERATURE_CALIB_DIG_T2_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_TEMPERATURE_CALIB_DIG_T2_LSB]);
		p_bmp280->calib_param.dig_T3 = (s16)((((s16)((s8)a_data_u8[
					BMP280_TEMPERATURE_CALIB_DIG_T3_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_TEMPERATURE_CALIB_DIG_T3_LSB]);
		p_bmp280->calib_param.dig_P1 = (u16)((((u16)((u8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P1_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P1_LSB]);
		p_bmp280->calib_param.dig_P2 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P2_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P2_LSB]);
		p_bmp280->calib_param.dig_P3 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P3_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P3_LSB]);
		p_bmp280->calib_param.dig_P4 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P4_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P4_LSB]);
		p_bmp280->calib_param.dig_P5 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P5_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P5_LSB]);
		p_bmp280->calib_param.dig_P6 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P6_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P6_LSB]);
		p_bmp280->calib_param.dig_P7 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P7_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P7_LSB]);
		p_bmp280->calib_param.dig_P8 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P8_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P8_LSB]);
		p_bmp280->calib_param.dig_P9 = (s16)((((s16)((s8)a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P9_MSB]))
					<< BMP280_SHIFT_BIT_POSITION_BY_08_BITS)
					| a_data_u8[
					BMP280_PRESSURE_CALIB_DIG_P9_LSB]);
	}
	return com_rslt;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_set_power_mode(u8 v_power_mode_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_mode_u8 = BMP280_INIT_VALUE;
        
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
		if (v_power_mode_u8 <= BMP280_NORMAL_MODE) {
			/* write the power mode*/
			v_mode_u8 = (p_bmp280->oversamp_temperature
					<< BMP280_SHIFT_BIT_POSITION_BY_05_BITS)
					+ (p_bmp280->oversamp_pressure
					<< BMP280_SHIFT_BIT_POSITION_BY_02_BITS)
					+ v_power_mode_u8;
                                        //u8 arraY[2] = {BMP280_CTRL_MEAS_REG_POWER_MODE__REG,v_mode_u8};
                            com_rslt = p_bmp280->BMP280_BUS_WRITE_FUNC(
					BMP280_CTRL_MEAS_REG_POWER_MODE__REG,
					&v_mode_u8,
					BMP280_GEN_READ_WRITE_DATA_LENGTH);
		            //com_rslt = nrf_drv_twi_rx(&m_twi,p_bmp280->dev_addr,&v_mode_u8,BMP280_GEN_READ_WRITE_DATA_LENGTH);
		} else {
			com_rslt = E_BMP280_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_set_filter(u8 v_value_u8)
{
	BMP280_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;
	u8 v_data_u8 = BMP280_INIT_VALUE;
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
		/* write filter*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
				BMP280_CONFIG_REG_FILTER__REG, &v_data_u8,
				BMP280_GEN_READ_WRITE_DATA_LENGTH);

		if (com_rslt == SUCCESS) {
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
					BMP280_CONFIG_REG_FILTER, v_value_u8);
			com_rslt += p_bmp280->BMP280_BUS_WRITE_FUNC(
					BMP280_CONFIG_REG_FILTER__REG,
					&v_data_u8,
					BMP280_GEN_READ_WRITE_DATA_LENGTH);
		}
	}
	return com_rslt;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_set_standby_durn(u8 v_standby_durn_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_INIT_VALUE;
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
		/* write the standby duration*/
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
				BMP280_CONFIG_REG_STANDBY_DURN__REG, &v_data_u8,
				BMP280_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
					BMP280_CONFIG_REG_STANDBY_DURN,
					v_standby_durn_u8);
			com_rslt += p_bmp280->BMP280_BUS_WRITE_FUNC(
					BMP280_CONFIG_REG_STANDBY_DURN__REG,
					&v_data_u8,
					BMP280_GEN_READ_WRITE_DATA_LENGTH);
		}
	}
	return com_rslt;
}
BMP280_RETURN_FUNCTION_TYPE bmp280_set_work_mode(u8 v_work_mode_u8)
{
	/* variable used to return communication result*/
	BMP280_RETURN_FUNCTION_TYPE com_rslt = ERROR;
	u8 v_data_u8 = BMP280_INIT_VALUE;
	/* check the p_bmp280 structure pointer as NULL*/
	if (p_bmp280 == BMP280_NULL) {
		com_rslt = E_BMP280_NULL_PTR;
	} else {
	if (v_work_mode_u8 <= BMP280_ULTRA_HIGH_RESOLUTION_MODE) {
			com_rslt = p_bmp280->BMP280_BUS_READ_FUNC(
				BMP280_CTRL_MEAS_REG, &v_data_u8,
				BMP280_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			switch (v_work_mode_u8) {
			/* write work mode*/
			case BMP280_ULTRA_LOW_POWER_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
					BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_LOW_POWER_MODE:
				p_bmp280->oversamp_temperature =
					BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
					BMP280_LOWPOWER_OVERSAMP_PRESSURE;
				break;
			case BMP280_STANDARD_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_HIGH_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
				p_bmp280->oversamp_temperature =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
				p_bmp280->oversamp_pressure =
				BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
				break;
			}
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE,
				p_bmp280->oversamp_temperature);
			v_data_u8 = BMP280_SET_BITSLICE(v_data_u8,
				BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE,
				p_bmp280->oversamp_pressure);
			com_rslt += p_bmp280->BMP280_BUS_WRITE_FUNC(BMP280_CTRL_MEAS_REG,
				&v_data_u8, BMP280_GEN_READ_WRITE_DATA_LENGTH);
		}
	} else {
		com_rslt = E_BMP280_OUT_OF_RANGE;
	}
	}
	return com_rslt;
}
void bmp280_read(u32 *temp, u32 *press)
{
    // Signal on LED that something is going on.
    s32 com_rslt = ERROR;
    v_actual_press_combined_u32 = BMP280_INIT_VALUE;
    v_actual_temp_combined_s32 = BMP280_INIT_VALUE;
    com_rslt = bmp280_read_pressure_temperature(&v_actual_press_combined_u32, &v_actual_temp_combined_s32);
    v_actual_temp_combined_s32 /= 100;
	*temp = v_actual_temp_combined_s32;
	*press = v_actual_press_combined_u32;
}

s8 BMP280_I2C_bus_write(u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BMP280_INIT_VALUE;
	array[BMP280_INIT_VALUE] = reg_addr;
	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		array[stringpos + BMP280_DATA_INDEX] = *(reg_data + stringpos);
	}
	i2c_write_dt(&bmp_spec,array,cnt+1);
     if (reg_addr < 0xF7) {
         //printk("W %X %X %X %X\r\n", dev_addr, reg_addr, cnt, err_code);
         //NRF_LOG_HEXDUMP_INFO(reg_data, cnt);
         //NRF_LOG_FLUSH();
     }

	return (s8)iError;
}

s8 BMP280_I2C_bus_read(u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	u8 arraY[I2C_BUFFER_LEN] = {BMP280_INIT_VALUE};
	u8 stringpos = BMP280_INIT_VALUE;
	arraY[BMP280_INIT_VALUE] = reg_addr;

	i2c_write_read_dt(&bmp_spec,&reg_addr,sizeof(reg_addr),arraY,sizeof(arraY));
    //APP_ERROR_CHECK(err_code);
     if (reg_addr < 0xF7) {
         //printk("R %X %X %X %X\r\n", dev_addr, reg_addr, cnt, err_code);
         //NRF_LOG_HEXDUMP_INFO(array, cnt);
         //NRF_LOG_FLUSH();
     }

	for (stringpos = BMP280_INIT_VALUE; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = arraY[stringpos];
	}

	return (s8)iError;
}

void BMP280_delay_msek(u32 msek)
{
	k_msleep(msek);
}

bool bmp280_config(void)
{
	if(i2c_is_ready_dt(&bmp_spec)){ // Start of check i2c initilization
	    char array[1] = {NULL};
		if(i2c_write_dt(&bmp_spec,array,sizeof(array))){
			return false;
		}
		else{}
    }
    else{
		return false;
	} // End of check i2c initilization
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = BMP280_I2C_ADDRESS1;
	bmp280.delay_msec = BMP280_delay_msek;

    s32 com_rslt = ERROR;
    com_rslt = bmp280_init(&bmp280); //important to check this
    com_rslt += bmp280_set_power_mode(BMP280_SLEEP_MODE);
    com_rslt += bmp280_set_work_mode(BMP280_ULTRA_HIGH_RESOLUTION_MODE);
    com_rslt += bmp280_set_filter(BMP280_FILTER_COEFF_16);
    com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
    com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);/// I am here now 18/08/2023//04:45PM
    //APP_ERROR_CHECK(com_rslt);

     u8 data[4];
	 unsigned char write_value = 0xF2;
	 i2c_write_read_dt(&bmp_spec,&write_value,BMP280_GEN_READ_WRITE_DATA_LENGTH,&data,sizeof(data));
	 return true;
}