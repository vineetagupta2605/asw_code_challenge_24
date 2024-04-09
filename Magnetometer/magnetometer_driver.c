/*
 * lis3mdl.c
 *
 * This file contains the implementation of functions for interacting with
 * the LIS3MDLTR 3-Axis Magnetometer sensor.
 *
 * Created on: April 9, 2024
 * Author: Vineeta Gupta
 */

/******************************************************************************
 * #1 Include Header Files
 ******************************************************************************/
#include "../Magnetometer/magnetometer_driver.h"

/******************************************************************************
 * #8 Extern Function Definitions
 ******************************************************************************/
extern status_t mmtDrvGetFullScaleConfig(mmt_drv_configEn * fullScaleConfigValue_pen)
{
	uint8_t readBuffer_u8;

	status_t status = i2c_read(MMT_DRV_I2C_BUS_ADDRESS, MMT_DRV_CTRL_REG2, 1u, &readBuffer_u8);
	if (status == STATUS_OK)
	{
		uint8_t FullScaleConfigValue = (readBuffer_u8 >> 6) & 0x03;
		switch(FullScaleConfigValue)
		{
		case 0x00:
			*fullScaleConfigValue_pen = MMT_DRV_CONFIG_4G;
			break;
		case 0x01:
			*fullScaleConfigValue_pen = MMT_DRV_CONFIG_8G;
			break;
		case 0x10:
			*fullScaleConfigValue_pen = MMT_DRV_CONFIG_12G;
			break;
		case 0x11:
			*fullScaleConfigValue_pen = MMT_DRV_CONFIG_16G;
			break;
		default:
			*fullScaleConfigValue_pen = MMT_DRV_CONFIG_ERROR;
			break;
	}

	}
	return status;
}


extern status_t mmtDrvGetDataRate(mmt_drv_SpeedConfigurationSt * dataRate_pst)
{
	uint8_t registerBuffer_u8;

	status_t status = i2c_read(MMT_DRV_I2C_BUS_ADDRESS, MMT_DRV_CTRL_REG1, 1u, &registerBuffer_u8);
	if (status == STATUS_OK)
	{
		dataRate_pst->dataRate = (registerBuffer_u8 & MMT_DRV_SPEED_MASK) >> 2;
		dataRate_pst->mode = (registerBuffer_u8 & 0x03);
		dataRate_pst->fastOdr = (registerBuffer_u8 & MMT_DRV_FAST_ODR_MASK);
	}

	return status;
}


extern status_t mmtDrvSetDataRate(mmt_drv_SpeedConfigurationSt dataRate_pst)
{
	uint8_t registerBuffer_u8 = ((dataRate_pst.dataRate << 2) & MMT_DRV_SPEED_MASK) | (dataRate_pst.mode & 0x03) | (dataRate_pst.fastOdr & MMT_DRV_FAST_ODR_MASK);
	status_t status = i2c_write(MMT_DRV_I2C_BUS_ADDRESS, MMT_DRV_CTRL_REG1, 1u, &registerBuffer_u8);

	return status;
}


extern status_t mmtDrvSetInterrupt(bool userInterruptState_b)
{
	uint8_t readBuffer_u8;
	status_t status = i2c_read(MMT_DRV_I2C_BUS_ADDRESS, MMT_DRV_INT_CFG, 1u, &readBuffer_u8);

	if (status == STATUS_OK)
	{
		bool interruptStatus_u8 = (readBuffer_u8 & 0x01) != 0u;

		if (userInterruptState_b == true && interruptStatus_u8 == false)
		{
			readBuffer_u8 |= 0x01;
			status = i2c_write(MMT_DRV_I2C_BUS_ADDRESS, MMT_DRV_INT_CFG, 1u, &readBuffer_u8);
		}

		else if (userInterruptState_b == false && interruptStatus_u8 == true)
		{
			readBuffer_u8 &= ~0x01;
			status = i2c_write(MMT_DRV_I2C_BUS_ADDRESS, MMT_DRV_INT_CFG, 1u, &readBuffer_u8);
		}
		else
		{
			status = STATUS_ERROR;
		}
	}

	return status;
}

extern status_t mmtDrvReadOutputData(mmt_drv_AxisEn axis_en, int16_t *outputData)
{
    uint8_t lowReg_u8, highReg_u8;
    status_t status = STATUS_DEFAULT;
    uint8_t regAddress_u8;

    switch (axis_en) {
        case MMT_DRV_X_AXIS:
        	regAddress_u8 = MMT_DRV_X_L;
            break;
        case MMT_DRV_Y_AXIS:
        	regAddress_u8 = MMT_DRV_Y_L;
            break;
        case MMT_DRV_Z_AXIS:
        	regAddress_u8 = MMT_DRV_Z_L;
            break;
        default:
            // Return error if axis is invalid
    }

    status |= i2c_read(MMT_DRV_I2C_BUS_ADDRESS, regAddress_u8, 1u, &lowReg_u8);
    status |= i2c_read(MMT_DRV_I2C_BUS_ADDRESS, regAddress_u8 + 1, 1u, &highReg_u8);

    if (status == STATUS_OK)
    {
    	*outputData = (int16_t)((highReg_u8 << 8) | lowReg_u8);
    }

    return status;
}
