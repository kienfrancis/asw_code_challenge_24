/*
 * example usage of the LIS3MDL driver
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum
{
    STATE_START = 0,
    STATE_GET_CONFIG = 1,
    STATE_GET_DATA_RATE = 2,
    STATE_SET_DATA_RATE = 3,
    STATE_ENABLE_INTERRUPT = 4,
    STATE_DISABLE_INTERRUPT = 5,
    STATE_GET_AXIS_DATA = 6,
    STATE_WAIT = 7,
} tOptionState;

tOptionState option_state = STATE_START;
uint8_t mainOption = 0;
uint8_t dataOption = 0;

void main()
{
    printf("Welcome to LIS3MDL Driver Example Application\n");
    printf("This example app will guide the user for the available APIs of the LIS3MDL driver\n");
    printf("---------------------------------------------\n");

    while(1)
    {
        switch(option_state)
        {
            case STATE_START:
            {
                printf("\nOptions:\n1.) Get full-scale configuration\n2.) Get output data rate\n3.) Set output data rate\n4.) Enable interrupt pin\n5.) Disable interrupt pin\n6.) Read the output data of a specified axis\n");
                printf("Enter desired option: ");
                // reset options
                mainOption = 0;
                dataOption = 0;

                scanf("%d", &mainOption);
                if( ( mainOption != 0 ) && 
                    ( mainOption < STATE_WAIT ) )
                {
                    option_state = (tOptionState)mainOption;
                }
                else
                {
                    printf("Invalid Option!");
                    option_state = STATE_WAIT;
                }
            }
            break;

            case STATE_GET_CONFIG:
            {
                printf("GET CONFIG\n");
                option_state = STATE_WAIT;
            }
            break;

            case STATE_GET_DATA_RATE:
            {
                printf("GET DATA RATE\n");
                option_state = STATE_WAIT;
            }
            break;

            case STATE_SET_DATA_RATE:
            {
                printf("SET DATA RATE\n");
                option_state = STATE_WAIT;
            }
            break;

            case STATE_ENABLE_INTERRUPT:
            {
                printf("ENABLE INTERRUPT\n");
                option_state = STATE_WAIT;
            }
            break;

            case STATE_DISABLE_INTERRUPT:
            {
                printf("DISABLE INTERRUPT\n");
                option_state = STATE_WAIT;
            }
            break;

            case STATE_GET_AXIS_DATA:
            {
                printf("GET AXIS_DATA\n");
                option_state = STATE_WAIT;
            }
            break;

            case STATE_WAIT:
            {
                getchar();
                printf("Press Any Key to Continue...\n");
                getchar();
                option_state = STATE_START;
            }
            break;

            default:
            {
                // do nothing
            }
            break;
        }
    }
}