#ifndef LIS3MDL_DRIVER_H
#define LIS3MDL_DRIVER_H

/****************************************************************************
 * Macro Definition                                                         *
 ****************************************************************************/
#define DEVICE_ADDR      (0x1E) // Assuming SDO/SA1 pin is connected to VCC.

#define CTRL_REG2_REG_BASE_ADDR (0x30)
#define CTRL_REG2_REG_LENGTH    (0x01)
#define CTRL_REG2_FS0_BIT_POS   (0x01)
#define CTRL_REG2_FS1_BIT_POS

#define INT_CFG_REG_BASE_ADDR (0x30)
#define INT_CFG_REG_LENGTH    (0x01)
#define INT_CFG_IEN_BIT_POS   (0x00)

#endif // LIS3MDL_DRIVER_H