/*
 * Mock I2C for example use
 */

#include <stdint.h>

uint8_t lis3mdl_i2c_read( )
{
    /* Setting the output to some arbitrary value */
    for (size_t i = 0; i < length; ++i) {
        buffer[i] = 0xFF;
    }
}