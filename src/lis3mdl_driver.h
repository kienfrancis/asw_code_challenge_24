#ifndef LIS3MDL_DRIVER_H
#define LIS3MDL_DRIVER_H

#include "i2c.h"
#include <math.h>
#include <stdbool.h>

/****************************************************************************
 * Public Macro Definitions                                                 *
 ****************************************************************************/
#define DEVICE_ADDR      ( 0x1E ) // Assuming SDO/SA1 pin is connected to VCC.

#define CTRL_REG1_REG_BASE_ADDR     ( 0x20 )
#define CTRL_REG1_REG_LENGTH        ( 0x01 )
#define CTRL_REG1_FAST_ODR_BIT_POS  ( 0x01 )
#define CTRL_REG1_FAST_ODR_BIT_MASK ( 1 << CTRL_REG1_FAST_ODR_BIT_POS )
#define CTRL_REG1_DO0_BIT_POS       ( 0x02 )
#define CTRL_REG1_DO0_BIT_MASK      ( 1 << CTRL_REG1_DO0_BIT_POS )
#define CTRL_REG1_DO1_BIT_POS       ( 0x03 )
#define CTRL_REG1_DO1_BIT_MASK      ( 1 << CTRL_REG1_DO1_BIT_POS )
#define CTRL_REG1_DO2_BIT_POS       ( 0x04 )
#define CTRL_REG1_DO2_BIT_MASK      ( 1 << CTRL_REG1_DO2_BIT_POS )
#define CTRL_REG1_DO_MASK           ( CTRL_REG1_DO2_BIT_MASK | CTRL_REG1_DO1_BIT_MASK | CTRL_REG1_DO0_BIT_MASK )
#define CTRL_REG1_OM0_BIT_POS       ( 0x05 )
#define CTRL_REG1_OM0_BIT_MASK      ( 1 << CTRL_REG1_OM0_BIT_POS )
#define CTRL_REG1_OM1_BIT_POS       ( 0x06 )
#define CTRL_REG1_OM1_BIT_MASK      ( 1 << CTRL_REG1_OM1_BIT_POS )
#define CTRL_REG1_OM_BIT_MASK       ( CTRL_REG1_OM1_BIT_MASK | CTRL_REG1_OM0_BIT_MASK )

#define CTRL_REG2_REG_BASE_ADDR ( 0x30 )
#define CTRL_REG2_REG_LENGTH    ( 0x01 )
#define CTRL_REG2_FS0_BIT_POS   ( 0x05 )
#define CTRL_REG2_FS0_BIT_MASK  ( 1 << CTRL_REG2_FS0_BIT_POS )
#define CTRL_REG2_FS1_BIT_POS   ( 0x06 )
#define CTRL_REG2_FS1_BIT_MASK  ( 1 << CTRL_REG2_FS1_BIT_POS )
#define CTRL_REG2_FS_MASK       ( CTRL_REG2_FS1_BIT_MASK | CTRL_REG2_FS0_BIT_MASK )

#define CTRL_REG4_REG_BASE_ADDR ( 0x23 )
#define CTRL_REG4_REG_LENGTH    ( 0x01 )
#define CTRL_REG4_OMZ0_BIT_POS  ( 0x02 )
#define CTRL_REG4_OMZ0_BIT_MASK ( 1 << CTRL_REG4_OMZ0_BIT_POS )
#define CTRL_REG4_OMZ1_BIT_POS  ( 0x03 )
#define CTRL_REG4_OMZ1_BIT_MASK ( 1 << CTRL_REG4_OMZ1_BIT_POS )
#define CTRL_REG4_OMZ_BIT_MASK  ( CTRL_REG4_OMZ1_BIT_MASK | CTRL_REG4_OMZ0_BIT_MASK )

#define INT_CFG_REG_BASE_ADDR ( 0x30 )
#define INT_CFG_REG_LENGTH    ( 0x01 )
#define INT_CFG_IEN_BIT_POS   ( 0x00 )
#define INT_CFG_IEN_BIT_MASK  ( 1 << INT_CFG_IEN_BIT_POS )

#define OUT_X_L_REG_BASE_ADDR    (0x28)
#define OUT_X_H_REG_BASE_ADDR    (0x29)
#define OUT_Y_L_REG_BASE_ADDR    (0x2A)
#define OUT_Y_H_REG_BASE_ADDR    (0x2B)
#define OUT_Z_L_REG_BASE_ADDR    (0x2C)
#define OUT_Z_H_REG_BASE_ADDR    (0x2D)
#define OUT_REGS_REG_LENGTH      (0x01)
#define OUT_REGS_HIGHBYTE_OFFSET (0x08)

#define FULL_SCALE_MAX_GAIN      (6842U)

/****************************************************************************
 * Public Type Definition                                                   *
 ****************************************************************************/
typedef enum{
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_MAX,
} tAxis;

typedef enum{
    FULL_SCALE_GAUSS_4 = 0,
    FULL_SCALE_GAUSS_8,
    FULL_SCALE_GAUSS_12,
    FULL_SCALE_GAUSS_16,
    FULL_SCALE_MAX,
} tFullScaleConfig;

typedef enum{
    ODR_HZ_0U625 = 0,
    ODR_HZ_1U25,
    ODR_HZ_2U5,
    ODR_HZ_5,
    ODR_HZ_10,
    ODR_HZ_20,
    ODR_HZ_40,
    ODR_HZ_80,
    ODR_HZ_155,
    ODR_HZ_300,
    ODR_HZ_560,
    ODR_HZ_1000,
    ODR_HZ_MAX,
} tDataRateHz;

typedef enum{
    ODR_FAST_LP = 0,
    ODR_FAST_MP,
    ODR_FAST_HP,
    ODR_FAST_UHP,
    ODR_FAST_MAX,
} tPerformanceMode;

/****************************************************************************
 * Public Function Definition                                               *
 ****************************************************************************/
/**
 * @brief Get Full-scale configuration of the sensor
 * @param[out] pFullScaleConfig - pointer to full-scale configuration in Gauss
 * @return i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t lis3mdlDriver_getFullScaleConfiguration( uint16_t *pFullScaleConfig );

/**
 * @brief Set Data Output Rate
 * @param[in] data_rate - output data rate in Hz (or fast ODR that needs to set performance mode)
 * @return i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t lis3mdlDriver_setOutputDataRate( const tDataRateHz data_rate );

/**
 * @brief Set Data Output Rate
 * @param[in] axis - selected axis in tAxis
 * @param[out] pOutputDataRate - pointer to output data rate in Hz (tDataRateHz)
 * @return i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t lis3mdlDriver_getOutputDataRate( const tAxis axis, tDataRateHz* pOutputDataRate );

/**
 * @brief Set Performance Mode for the selected axis
 * @param[in] axis - selected axis to update the performance mode ( X & Y axes shares the same performance mode )
 * @param[in] mode - selected performance mode (ODR_FAST_LP, ODR_FAST_MP, ODR_FAST_HP, ODR_FAST_UHP)
 * @return i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t lis3mdlDriver_setPerformanceMode( const tAxis axis, const tPerformanceMode mode );

/**
 * @brief Enable/Disable Sensor's interrupt pin
 * @param[in] enable - True enables the interrupt pin, otherwise disables the interrupt pin.
 * @return i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t lis3mdlDriver_setInterruptPinState( bool enable );

/**
 * @brief Read Axis Output Data in Gauss, this also gets the current full-scale config to scale down the read output data for the axis
 * @param[in] axis - selected axis in tAxis
 * @param[out] pOutputData - pointer to output data rate in Gauss
 * @return i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t lis3mdlDriver_readAxisOutputData( const tAxis axis, float_t* pOutputData );

#endif // LIS3MDL_DRIVER_H