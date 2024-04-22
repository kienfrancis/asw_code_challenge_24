/*
 * LIS3MDL Magnetometer Sensor I2C Driver File
 */
#include <stdbool.h>
#include "i2c.h"
#include "lis3mdl_driver.h"

/**
 * @brief Get Full-scale configuration of the sensor
 * @param[out] full-scale configuration in Gauss
 * @param[out] i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t getFullScaleConfiguration( uint16_t *pFullScaleConfig )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t buffer[1];

    i2c_status = i2c_read( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, &buffer ); 
    pFullScaleConfig = buffer[0]
}


/**
 * @brief Enable/Disable Sensor's interrupt pin
 * @param[in] enable - True enables the interrupt pin, otherwise disables the interrupt pin.
 * @param[out] i2c_status - returns STATUS_OK if there's no error encountered, otherwise returns STATUS_ERROR
 */
status_t setInterruptPinState( bool enable )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t buffer[1];
    // Read first the existing config to disable/enable interrupt pin without affecting the current config
    i2c_status = i2c_read( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, &buffer );
    if( STATUS_OK == i2c_status )
    {
        if( enable )
        {
            buffer[0] |= ( 1 << INT_CFG_IEN_BIT_POS );
        }
        else
        {
            buffer[0] &= ~( 1 << INT_CFG_IEN_BIT_POS );
        }
        i2c_status = i2c_write( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, &buffer ); 
    }

    return i2c_status;
}

