/*
 * magnetometer_driver.h
 *
 * This file contains the definitions for interacting with
 * the LIS3MDLTR 3-Axis Magnetometer sensor.
 *
 * Created on: April 9, 2024
 * Author: Vineeta Gupta
 */


#ifndef MAGNETOMETER_DRIVER_H_
#define MAGNETOMETER_DRIVER_H_

/******************************************************************************
 * #1 Include Header Files
 ******************************************************************************/
#include <stdlib.h>
#include <stdbool.h>
#include <main.h>

/******************************************************************************
 * #2 Macro Declarations
 ******************************************************************************/
/* Dummy bus address */
#define MMT_DRV_I2C_BUS_ADDRESS		0x50

/* Necessary bit masks */
#define MMT_DRV_SPEED_MASK			0x1C
#define MMT_DRV_FAST_ODR_MASK		0x01

/* Register mapping of required registers */
#define MMT_DRV_CTRL_REG1	0x20
#define MMT_DRV_CTRL_REG2	0x21
#define MMT_DRV_X_L			0x28
#define MMT_DRV_X_H			0x29
#define MMT_DRV_Y_L			0x2A
#define MMT_DRV_Y_H			0x2B
#define MMT_DRV_Z_L			0x2C
#define MMT_DRV_Z_H			0x2D
#define MMT_DRV_INT_CFG		0x30

/******************************************************************************
 * #3 Type Declarations
 ******************************************************************************/
typedef enum {
	MMT_DRV_CONFIG_4G,
	MMT_DRV_CONFIG_8G,
	MMT_DRV_CONFIG_12G,
	MMT_DRV_CONFIG_16G,
	MMT_DRV_CONFIG_ERROR,
} mmt_drv_configEn;

typedef enum {
	MMT_DRV_RATE_0_625_Hz,
	MMT_DRV_RATE_1_25_Hz,
	MMT_DRV_RATE_2_25_Hz,
	MMT_DRV_RATE_5_Hz,
	MMT_DRV_RATE_10_Hz,
	MMT_DRV_RATE_20_Hz,
	MMT_DRV_RATE_40_Hz,
	MMT_DRV_RATE_80_Hz
} mmt_drv_OutputDataRateEn;

typedef enum {
	MMT_DRV_LP_MODE,
	MMT_DRV_MP_MODE,
	MMT_DRV_HP_MODE,
	MMT_DRV_UHP_MODE
} mmt_drv_OperatingModeEn;

typedef enum {
	MMT_DRV_X_AXIS,
	MMT_DRV_Y_AXIS,
	MMT_DRV_Z_AXIS,
} mmt_drv_AxisEn;

typedef struct {
	mmt_drv_OutputDataRateEn dataRate;
	mmt_drv_OperatingModeEn mode;
    uint8_t fastOdr;
} mmt_drv_SpeedConfigurationSt;

/******************************************************************************
 * #8 Extern Function Declaration
 ******************************************************************************/
/**
 * @brief Get the full-scale configuration of the magnetometer.
 *
 * @param[out] fullScaleConfigValuePen Pointer to a variable to store the full-scale configuration value.
 *
 * @return STATUS_OK if successful.
 */
extern status_t mmtDrvGetFullScaleConfig(mmt_drv_configEn * fullScaleConfigValuePen);


/**
 * @brief Get the data rate configuration of the magnetometer.
 *
 * @param[out] dataRatePst Pointer to a SpeedConfiguration struct to store the data rate configuration.
 *
 * @return STATUS_OK if successful.
 */
extern status_t mmtDrvGetDataRate(mmt_drv_SpeedConfigurationSt * dataRatePst);


/**
 * @brief Set the data rate configuration of the magnetometer.
 *
 * @param[in] dataRatePst SpeedConfiguration struct containing the desired data rate configuration.
 *
 * @return STATUS_OK if successful
 */
extern status_t mmtDrvSetDataRate(mmt_drv_SpeedConfigurationSt dataRatePst);


/**
 * @brief Enable or disable the interrupt pin of the magnetometer.
 *
 * @param[in] userInterruptStateB Boolean value indicating whether to enable (true) or disable (false) the interrupt pin.
 *
 * @return STATUS_OK if successful.
 */
extern status_t mmtDrvSetInterrupt(bool userInterruptStateB);


/**
 * @brief Read the output data of a specified axis from the magnetometer.
 *
 * @param[in] axisEn Axis for which to read the output data (X, Y, or Z).
 * @param[out] outputDataP16 Pointer to a variable to store the output data.
 *
 * @return STATUS_OK if successful.
 */
extern status_t mmtDrvReadOutputData(mmt_drv_AxisEn axisEn, int16_t * outputDataP16);

#endif /* MAGNETOMETER_DRIVER_H_ */
