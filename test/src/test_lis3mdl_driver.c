#ifdef UNIT_TEST

#include <stdbool.h>

#include "unity.h"
#include "mock_i2c.h"

#include "lis3mdl_driver.h"

#define ENABLE_INTERRUPT_PIN    ( true  )
#define DISABLE_INTERRUPT_PIN   ( false )

extern uint8_t lastWrittenData;

uint8_t expectForSettingInterruptPinState( status_t expectedI2CStatus, uint8_t* currentRegisterValue, bool enable )
{
    uint8_t expectedData;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, currentRegisterValue, expectedI2CStatus );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( currentRegisterValue );
    
    if( enable )
    {
        expectedData = *currentRegisterValue | INT_CFG_IEN_BIT_MASK;
    }
    else
    {
        expectedData = *currentRegisterValue & ~INT_CFG_IEN_BIT_MASK;
    }
    
    i2c_write_ExpectAndReturn( DEVICE_ADDR, INT_CFG_REG_BASE_ADDR, INT_CFG_REG_LENGTH, &expectedData, STATUS_OK );
    i2c_write_IgnoreArg_buffer();

    return expectedData;
}

void setUp(void)
{
    lastWrittenData = 0xFF;
}

void tearDown(void)
{
}

void test_lis3mdlDriver_getFullScaleConfiguration_successful(void)
{
    uint8_t rawTestData;
    uint16_t testFullScaleConfig = 0xFF;
    status_t testI2CStatus;

    // Expected return value of 4 GAUSS
    rawTestData = 0x00;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    // Execute Function
    testI2CStatus = lis3mdlDriver_getFullScaleConfiguration( &testFullScaleConfig );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( 4U, testFullScaleConfig );

    // Expected return value of 8 GAUSS
    rawTestData = 0x20;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    // Execute Function
    testI2CStatus = lis3mdlDriver_getFullScaleConfiguration( &testFullScaleConfig );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( 8U, testFullScaleConfig );

    // Expected return value of 12 GAUSS
    rawTestData = 0x40;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    // Execute Function
    testI2CStatus = lis3mdlDriver_getFullScaleConfiguration( &testFullScaleConfig );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( 12U, testFullScaleConfig );

    // Expected return value of 16 GAUSS
    rawTestData = 0x60;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    // Execute Function
    testI2CStatus = lis3mdlDriver_getFullScaleConfiguration( &testFullScaleConfig );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( 16U, testFullScaleConfig );
}

void test_lis3mdlDriver_getFullScaleConfiguration_error(void)
{
    uint8_t rawTestData;
    uint16_t testFullScaleConfig = 0xFF;
    status_t testI2CStatus;
    
    // Expected I2C Error
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_ERROR );
    i2c_read_IgnoreArg_buffer();

    // Execute Function
    testI2CStatus = lis3mdlDriver_getFullScaleConfiguration( &testFullScaleConfig );
    TEST_ASSERT_EQUAL( STATUS_ERROR, testI2CStatus );
    TEST_ASSERT_EQUAL( 0xFF, testFullScaleConfig ); // value should not be updated
}

void test_lis3mdlDriver_setInterruptPinState_enabledSuccessfully(void)
{
    uint8_t rawTestData, expectedData;
    status_t testI2CStatus;
    
    // Assuming interrupt generation for all axes are already configured, active low config, and not latching interrupt
    rawTestData = 0xEA;
    testI2CStatus = STATUS_ERROR;
    expectedData = expectForSettingInterruptPinState( STATUS_OK, &rawTestData, ENABLE_INTERRUPT_PIN );

    // Execute Function
    testI2CStatus = lis3mdlDriver_setInterruptPinState( ENABLE_INTERRUPT_PIN );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( expectedData, lastWrittenData );
}

void test_lis3mdlDriver_setInterruptPinState_disabledSuccessfully( void )
{
    uint8_t rawTestData, expectedData;
    status_t testI2CStatus;
    
    // Assuming interrupt generation for all axes are already configured, active low config, and not latching interrupt
    rawTestData = 0xEA;
    testI2CStatus = STATUS_ERROR;
    expectedData = expectForSettingInterruptPinState( STATUS_OK, &rawTestData, DISABLE_INTERRUPT_PIN );
    // Execute Function
    testI2CStatus = lis3mdlDriver_setInterruptPinState( DISABLE_INTERRUPT_PIN );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( expectedData, lastWrittenData );
}

void test_lis3mdlDriver_setOutputDataRate_10Hz( void )
{
    uint8_t rawTestData, expectedData;
    status_t testI2CStatus;
    const tDataRateHz testDataRate = ODR_HZ_10;
    testI2CStatus = STATUS_ERROR;

    // Assuming temperature sensor is enabled
    rawTestData = 0x80;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    // Expecting 10Hz ( D2-D0 = 0b100 )
    expectedData = rawTestData | ( 0b100 << CTRL_REG1_DO0_BIT_POS );

    i2c_write_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &expectedData, STATUS_OK );
    i2c_write_IgnoreArg_buffer();

    testI2CStatus = lis3mdlDriver_setOutputDataRate( testDataRate );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( expectedData, lastWrittenData );
}

void test_lis3mdlDriver_setOutputDataRateAndPerformanceMode_560HzForAllAxes( void )
{
    uint8_t rawTestData, expectedData;
    status_t testI2CStatus;
    const tDataRateHz testDataRate = ODR_HZ_560;
    const tPerformanceMode testPerformanceMode = ODR_FAST_MP;
    testI2CStatus = STATUS_ERROR;

    // Set Output Data First ( Assuming temperature sensor is enabled )
    rawTestData = 0x80;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    // Expecting FAST_ODR = 1
    expectedData = rawTestData | CTRL_REG1_FAST_ODR_BIT_MASK;

    i2c_write_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &expectedData, STATUS_OK );
    i2c_write_IgnoreArg_buffer();

    testI2CStatus = lis3mdlDriver_setOutputDataRate( testDataRate );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( expectedData, lastWrittenData );

    // Set Performance Mode to MP for X and Y Axes ( last written data should not be altered )
    rawTestData = expectedData;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    // Expecting MP ( OM1-OM0 = 0b01 )
    expectedData = rawTestData | ( 0b01 << CTRL_REG1_OM0_BIT_POS );

    i2c_write_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &expectedData, STATUS_OK );
    i2c_write_IgnoreArg_buffer();

    testI2CStatus = lis3mdlDriver_setPerformanceMode( AXIS_X, ODR_FAST_MP );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( expectedData, lastWrittenData );

    // Set Performance Mode to MP for Z Axis
    rawTestData = 0x00;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG4_REG_BASE_ADDR, CTRL_REG4_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    // Expecting MP ( OMZ1-OMZ0 = 0b01 )
    expectedData = rawTestData | ( 0b01 << CTRL_REG4_OMZ0_BIT_POS );

    i2c_write_ExpectAndReturn( DEVICE_ADDR, CTRL_REG4_REG_BASE_ADDR, CTRL_REG4_REG_LENGTH, &expectedData, STATUS_OK );
    i2c_write_IgnoreArg_buffer();

    testI2CStatus = lis3mdlDriver_setPerformanceMode( AXIS_Z, ODR_FAST_MP );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( expectedData, lastWrittenData );
}


void test_lis3mdlDriver_getOutputDataRate_20Hz( void )
{
    uint8_t rawTestData, expectedData;
    status_t testI2CStatus;
    tDataRateHz testOutputDataRate = 0xFF;
    tAxis testAxis = AXIS_X;
    testI2CStatus = STATUS_ERROR;

    // Expecting ODR is 20Hz and temp sensor is enabled
    rawTestData = 0x94;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );

    testI2CStatus = lis3mdlDriver_getOutputDataRate( testAxis, &testOutputDataRate );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( ODR_HZ_20, testOutputDataRate );
}

void test_lis3mdlDriver_getOutputDataRate_300HzAllAxes( void )
{
    uint8_t rawTestData, rawTestData2, expectedData;
    status_t testI2CStatus;
    tDataRateHz testOutputDataRate = 0xFF;
    testI2CStatus = STATUS_ERROR;

    // For X-axis
    // Expecting FAST ODR is set, X-Y performance mode is HP, and temp sensor is enabled
    rawTestData = 0xB6;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );

    testI2CStatus = lis3mdlDriver_getOutputDataRate( AXIS_X, &testOutputDataRate );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( ODR_HZ_300, testOutputDataRate );

    // For Y-axis
    // Expecting FAST ODR is set, X-Y performance mode is HP, and temp sensor is enabled
    rawTestData = 0xB6;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );

    testI2CStatus = lis3mdlDriver_getOutputDataRate( AXIS_Y, &testOutputDataRate );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( ODR_HZ_300, testOutputDataRate );

    // For Z-axis
    // Expecting FAST ODR is set, X-Y performance mode is HP, and temp sensor is enabled
    rawTestData = 0xB6;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG1_REG_BASE_ADDR, CTRL_REG1_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );

    // Expecting Z performance mode is HP
    rawTestData2 = 0x04;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG4_REG_BASE_ADDR, CTRL_REG4_REG_LENGTH, &rawTestData2, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData2 );

    testI2CStatus = lis3mdlDriver_getOutputDataRate( AXIS_Z, &testOutputDataRate );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL( ODR_HZ_300, testOutputDataRate );
}

void test_lis3mdlDriver_readAxisOutputData_allAxes( void )
{
    uint8_t rawTestData, lowByteData, highByteData, expectedData;
    status_t testI2CStatus;
    float_t testOutputData;
    testI2CStatus = STATUS_ERROR;

    // Read output data for X-axis
    lowByteData = 0xE8;
    highByteData = 0x6A;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, OUT_X_L_REG_BASE_ADDR, OUT_REGS_REG_LENGTH, &lowByteData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &lowByteData );
    
    i2c_read_ExpectAndReturn( DEVICE_ADDR, OUT_X_H_REG_BASE_ADDR, OUT_REGS_REG_LENGTH, &highByteData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &highByteData );

    // Expected Full-scale config is 4 Gauss
    rawTestData = 0x00;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    testI2CStatus = lis3mdlDriver_readAxisOutputData( AXIS_X, &testOutputData );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL_FLOAT( 4, testOutputData );

    // Read output data for Y-axis
    lowByteData = 0xBA;
    highByteData = 0x1A;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, OUT_Y_L_REG_BASE_ADDR, OUT_REGS_REG_LENGTH, &lowByteData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &lowByteData );
    
    i2c_read_ExpectAndReturn( DEVICE_ADDR, OUT_Y_H_REG_BASE_ADDR, OUT_REGS_REG_LENGTH, &highByteData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &highByteData );

    // Expected Full-scale config is 4 Gauss
    rawTestData = 0x00;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    testI2CStatus = lis3mdlDriver_readAxisOutputData( AXIS_Y, &testOutputData );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL_FLOAT( 1, testOutputData );

    // Read output data for Z-axis
    lowByteData = 0x8C;
    highByteData = 0xCA;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, OUT_Z_L_REG_BASE_ADDR, OUT_REGS_REG_LENGTH, &lowByteData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &lowByteData );
    
    i2c_read_ExpectAndReturn( DEVICE_ADDR, OUT_Z_H_REG_BASE_ADDR, OUT_REGS_REG_LENGTH, &highByteData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &highByteData );

    // Expected Full-scale config is 4 Gauss
    rawTestData = 0x00;
    testI2CStatus = STATUS_ERROR;
    i2c_read_ExpectAndReturn( DEVICE_ADDR, CTRL_REG2_REG_BASE_ADDR, CTRL_REG2_REG_LENGTH, &rawTestData, STATUS_OK );
    i2c_read_IgnoreArg_buffer();
    i2c_read_ReturnThruPtr_buffer( &rawTestData );
    
    testI2CStatus = lis3mdlDriver_readAxisOutputData( AXIS_Z, &testOutputData );
    TEST_ASSERT_EQUAL( STATUS_OK, testI2CStatus );
    TEST_ASSERT_EQUAL_FLOAT( -2, testOutputData );
}

#endif // TEST
