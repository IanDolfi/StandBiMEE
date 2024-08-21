/*
 ********************************************************************************
 * @file    ${odrive.c}
 * @author  ${Ian Dolfi}
 * @date    ${06/10/2024}
 * @brief   Contains definitions for odrive UART functions
 ********************************************************************************
 */

#include "odrive.h"

const char lookup[][4] = {"\x00\x00\x00\x01", "\x00\x00\x00\x02", "\x00\x00\x00\x04",
                          "\x00\x00\x00\x08", "\x00\x00\x01\x06", "\x00\x00\x03\x02", "\x00\x00\x06\x04",
                          "\x00\x01\x02\x08", "\x00\x02\x05\x06", "\x00\x05\x01\x02", "\x01\x00\x02\x04",
                          "\x02\x00\x04\x08", "\x04\x00\x09\x06"};


char setVelocityControl[] = "w odrv0.axis0.controller.config.control_mode ControlMode.VELOCITY_CONTROL"; //74
char setRampString[] = "w axis0.controller.config.vel_ramp_rate " //40
        "";
char setRampedVelocityString[] = "w odrv0.axis0.controller.config.input_mode InputMode.VEL_RAMP"; //62
char setVelocityString[44] = "w odrv0.axis0.controller.input_vel "; //36

void odrive_setVelocity(char *strPointer, int vel)
{
    char numStr[] = {0,0,0,0,0};

    strPointer[0] = 'v';
    strPointer[1] = ' ';
    strPointer[2] = '0';
    strPointer[3] = ' ';

    if(vel == 0)
    {
        strPointer[4] = '0';
        strPointer[5] = 0;
    }
    else if(vel < 0)
    {
        strPointer[4] = '-';
        intToStr((unsigned int)abs(vel), numStr);
        strPointer[5] = numStr[0];
        strPointer[6] = numStr[1];
        strPointer[7] = '.';
        strPointer[8] = numStr[2];
        strPointer[9] = numStr[3];
        strPointer[10] = 0;
    }
    else
    {
        intToStr((unsigned int)vel, numStr);
        strPointer[4] = numStr[0];
        strPointer[5] = numStr[1];
        strPointer[6] = '.';
        strPointer[7] = numStr[2];
        strPointer[8] = numStr[3];
        strPointer[9] = 0;
    }
}

void odrive_setVelocityW(char *strPointer, int vel)
{
    char numStr[] = {0,0,0,0,0};

    int i = 0;
    for (i = 0; i < 37; i++)    //Copy the set velocity string
    {
        strPointer[i] = setVelocityString[i];
    }

    if(vel == 0)
    {
        strPointer[37] = '0';
    }
    else if(vel < 0)
    {
        strPointer[37] = '-';
        intToStr((unsigned int)abs(vel), numStr);
        strPointer[38] = numStr[0];
        strPointer[39] = numStr[1];
        strPointer[40] = '.';
        strPointer[41] = numStr[2];
        strPointer[42] = numStr[3];
        strPointer[43] = 0;
    }
    else
    {
        intToStr((unsigned int)vel, numStr);
        strPointer[37] = numStr[0];
        strPointer[38] = numStr[1];
        strPointer[39] = '.';
        strPointer[40] = numStr[2];
        strPointer[41] = numStr[3];
        strPointer[42] = 0;
    }
}



void odrive_setControlRamped(char *strPointer)
{
    int i = 0;
    for (i = 0; i < 64; i++)    //Copy the set control ramped velocity string
    {
        strPointer[i] = setRampedVelocityString[i];
    }
}

void odrive_setControlVelocity(char *strPointer)
{
    int i = 0;
    for (i = 0; i < 76; i++)    //Copy the set control ramped velocity string
    {
        strPointer[i] = setVelocityControl[i];
    }
}

void odrive_setTorque(char *strPointer, int torque)
{
    if(!torque)
    {
        strPointer[0] = 'c';
        strPointer[1] = ' ';
        strPointer[2] = '0';
        strPointer[3] = ' ';
        strPointer[4] = '0';
        strPointer[5] = 0;
    }
}

void odrive_setRamp(char *strPointer, int ramp)
{
    char numStr[] = {0,0,0,0,0};

    int i = 0;
    for (i = 0; i < 64; i++)    //Copy the set control ramped velocity string
    {
        strPointer[i] = setRampString[i];
    }

    intToStr((unsigned int)abs(ramp), numStr);
    strPointer[40] = numStr[0];
    strPointer[41] = numStr[1];
    strPointer[42] = '.';
    strPointer[43] = numStr[2];
    strPointer[44] = numStr[3];
    strPointer[45] = 0;

}

//void odrive_setRamp(char *strPointer, int ramp);



void intToStr(unsigned int num, char* str)
{
    //Implementation inspired by:
    //https://www.reddit.com/r/learnprogramming/comments/2qva3h/most_efficient_way_to_go_from_int_to_ascii_code/

    unsigned int numShift = num;
    char bit, sum, car;
    char res[] = {0,0,0,0,0};

    //Go through each bit in the desired number, and use a lookup table to add the corresponding
    //power of 2 to the result
    for(bit = 0; bit < 12; bit++)
    {
        if(numShift & 1)
        {
            car = 0;
            int i;
            for (i = 3; i >= 0; i--)
            {
                sum = res[i] + lookup[bit][i] + car;
                if (sum > 9) //Take care of overflow
                {
                    car = 1;
                    res[i] = sum - 10;
                }
                else
                {
                    car = 0;
                    res[i] = sum;
                }
            }
        }
        numShift = numShift >> 1;
    }

    int i;
    for (i = 0; i < 4; i++)
    {
        str[i] = res[i] + 48;
    }
}
