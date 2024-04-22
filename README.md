# AAC-Clydespace Avionics Software Challenge
You are given the LIS3MDLTR 3-Axis Magnetometer sensor. The data sheet is
located at `doc/lism3mdl.pdf`

## Task Description
You must use the C API defined in `i2c.h` to operate on an I2C bus.

We provide example stub implementations of this API in `i2c.c`. You are free to
extend the provided stub functions for the API.

Please write a device driver that implements the following API:
- Get the full-scale configuration
- Get and set the output data rate
- Enable or disable the interrupt pin
- Read the output data of a specified axis

## Development
You are provided this minimal repository, `aac-code-challenge`, please perform
your work on branch `<candidate_name>`

Documentation can be found in `aac-code-challenge/doc`

Once complete, please repackage and email back this repo to your interviewers

## Scope
You shouldn't need to spend more than a couple of hours on the task.

This is not a closed book assessment.

## Extra Thoughts
If you have time, answer the following questions:
- What changes you would make to this interfaces for use in an RTOS
environment?
1. To use in RTOS environment the following can be added for this interfaces. 
   1. Make a common HAL interface for the I2C driver for different I2C peripherals usage. 
   2. If Magnetometer Sensor shares a bus with other device, it is also needed to set up some scheduling of the transaction to poll both devices.
   3. Timing of the read should be defined to optimize the MCU CPU Utilization. We don't need to over read the sensor and just dispose the data.
- How might the I2C API be improved
1. I2C API can be improved by making it the higher level API that links to the HAL of the I2C.
2. This I2C API can also manage the scheduling of the i2c transactions if there is multiple I2C devices connected.

## Added Changes
- Added unit tests for the LIS3MDL Driver APIs (mostly sunny test cases). This can be run using ceedling test:all ( Ruby-based Unit Testing Framework )