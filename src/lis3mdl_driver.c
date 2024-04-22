/*
 * LIS3MDL Magnetometer Sensor I2C Driver File
 */
#include "lis3mdl_driver.h"

/****************************************************************************
 * Private Macro Definitions                                                *
 ****************************************************************************/
#define GET_FULL_SCALE_GAIN( fullScaleConfig ) ( ( fullScaleConfig / 4 ) * FULL_SCALE_MAX_GAIN )

/****************************************************************************
 * Private Variable Declarations                                            *
 ****************************************************************************/
#ifdef UNIT_TEST
    uint8_t lastWrittenData = 0xFF;
#endif

/****************************************************************************
 * Public Function Declarations                                             *
 ****************************************************************************/
status_t lis3mdlDriver_getFullScaleConfiguration( uint16_t *pFullScaleConfig )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t data;

    i2c_status = i2c_read( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &data );
    tFullScaleConfig fullScaleConfigRaw = ( data & CTRL_REG2_FS_MASK ) >> CTRL_REG2_FS0_BIT_POS;
    if( STATUS_OK == i2c_status )
    {
        switch( fullScaleConfigRaw )
        {
            case FULL_SCALE_GAUSS_4:
            {
                *pFullScaleConfig = 4U;
            }
            break;

            case FULL_SCALE_GAUSS_8:
            {
                *pFullScaleConfig = 8U;
            }
            break;

            case FULL_SCALE_GAUSS_12:
            {
                *pFullScaleConfig = 12U;
            }
            break;

            case FULL_SCALE_GAUSS_16:
            {
                *pFullScaleConfig = 16U;
            }
            break;

            default:
            {
                // undefined value, return STATUS_ERROR
                i2c_status = STATUS_ERROR;
            }
            break;
        }
    }

    return i2c_status;
}

status_t lis3mdlDriver_setOutputDataRate( const tDataRateHz data_rate )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t data;

    // Read the register value first to just modify the bits related to output data rate
    i2c_status = i2c_read( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &data );
    if( STATUS_OK == i2c_status )
    {
        if( data_rate <= ODR_HZ_80 )
        {
            data |= ( (uint8_t) data_rate << CTRL_REG1_DO0_BIT_POS );
        }
        else
        {
            data |= CTRL_REG1_FAST_ODR_BIT_MASK;
        }
    }

    i2c_status = i2c_write( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &data );
#ifdef UNIT_TEST
    lastWrittenData = data;
#endif
    return i2c_status;
}

status_t lis3mdlDriver_setPerformanceMode( const tAxis axis, const tPerformanceMode mode )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t data;
    uint8_t min_bit_pos, reg_addr, length;
    if( axis < AXIS_Z )
    {
        min_bit_pos = CTRL_REG1_OM0_BIT_POS;
        reg_addr = CTRL_REG1_REG_BASE_ADDR;
        length = CTRL_REG1_REG_LENGTH;
    }
    else // axis == AXIS_Z
    {
        min_bit_pos = CTRL_REG4_OMZ0_BIT_POS;
        reg_addr = CTRL_REG4_REG_BASE_ADDR;
        length = CTRL_REG4_REG_LENGTH;
    }
    // Read the register value first to just modify the bits related to output data rate
    i2c_status = i2c_read( DEVICE_ADDR, reg_addr, length, &data );
    if( STATUS_OK == i2c_status )
    {
        // Set the performance mode
        data |= ( (uint8_t) mode << min_bit_pos );

        // Write new configuration
        i2c_status = i2c_write( DEVICE_ADDR, reg_addr, length, &data );
    }

#ifdef UNIT_TEST
    lastWrittenData = data;
#endif
    return i2c_status;
}

status_t lis3mdlDriver_getOutputDataRate( const tAxis axis, tDataRateHz* pOutputDataRate )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t data;

    // Read CTRL_REG1 value
    i2c_status = i2c_read( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &data );
    if( STATUS_OK == i2c_status )
    {
        // Check if FAST ODR is enabled
        if( data & CTRL_REG1_FAST_ODR_BIT_MASK )
        {
            uint8_t mode_val = 0;
            // If selected axis is Z, read CTRL_REG4 to get its performance mode
            if( axis == AXIS_Z )
            {
                i2c_status = i2c_read( DEVICE_ADDR, CTRL_REG4_REG_BASE_ADDR, CTRL_REG4_REG_LENGTH, &data );
                mode_val = ( data & CTRL_REG4_OMZ_BIT_MASK ) >> CTRL_REG4_OMZ0_BIT_POS ;
            }
            else // get X and Y performance mode in CTRL_REG1
            {
                mode_val = ( data & CTRL_REG1_OM_BIT_MASK ) >> CTRL_REG1_OM0_BIT_POS ;
            }

            *pOutputDataRate = ODR_HZ_155 + mode_val;
        }
        else // Not Fast ODR, check OMX bits to get the output data rate
        {
            *pOutputDataRate = (tDataRateHz)( ( data & CTRL_REG1_DO_MASK ) >> CTRL_REG1_DO0_BIT_POS );
        }
    }

    return i2c_status;
}

status_t lis3mdlDriver_setInterruptPinState( bool enable )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t data;

    // Read first the existing config to disable/enable interrupt pin without affecting the current config
    i2c_status = i2c_read( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, &data );

    if( STATUS_OK == i2c_status )
    {
        if( enable )
        {
            // Set IEN bit
            data |= INT_CFG_IEN_BIT_MASK;
        }
        else
        {
            // Clear IEN bit
            data &= ~INT_CFG_IEN_BIT_MASK;
        }

        i2c_status = i2c_write( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, &data );
#ifdef UNIT_TEST
        lastWrittenData = data;
#endif
    }

    return i2c_status;
}

status_t lis3mdlDriver_readAxisOutputData( const tAxis axis, float_t* pOutputData )
{
    status_t i2c_status = STATUS_ERROR;
    uint8_t data, lowbyte_addr;
    int16_t raw_data;
    uint16_t fullScaleConfig = 0;

    // This currently assumes that BLE is set to 1. (Big-endian)
    switch( axis )
    {
        case AXIS_X:
        {
            lowbyte_addr = OUT_X_L_REG_BASE_ADDR;
        }
        break;

        case AXIS_Y:
        {
            lowbyte_addr = OUT_Y_L_REG_BASE_ADDR;
        }
        break;

        case AXIS_Z:
        {
            lowbyte_addr = OUT_Z_L_REG_BASE_ADDR;
        }
        break;

        default:
        {
            // do nothing
        }
        break;
    }

    // Read the lowbyte value first
    i2c_status = i2c_read( DEVICE_ADDR, lowbyte_addr, OUT_REGS_REG_LENGTH, &data );
    if( STATUS_OK == i2c_status )
    {
        // Read the highbyte
        raw_data = data;
        i2c_status = i2c_read( DEVICE_ADDR, lowbyte_addr + 1, OUT_REGS_REG_LENGTH, &data );
    }

    if( STATUS_OK == i2c_status )
    {
        raw_data |= ( data << OUT_REGS_HIGHBYTE_OFFSET );
        // Get Full Scale Configuration to get the needed gain
        i2c_status = lis3mdlDriver_getFullScaleConfiguration( &fullScaleConfig );
    }

    if( STATUS_OK == i2c_status )
    {
        *pOutputData = ( (float_t)raw_data / GET_FULL_SCALE_GAIN( fullScaleConfig ) ) ;
    }

#ifdef UNIT_TEST
    lastWrittenData = data;
#endif

    return i2c_status;
}