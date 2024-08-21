#ifndef ODRIVE_H
#define ODRIVE_H
/*
 ********************************************************************************
 * @file    ${odrive.5}
 * @author  ${Ian Dolfi}
 * @date    ${06/10/2024}
 * @brief   Contains declarations for odrive UART functions
 ********************************************************************************
 */


//This uses the ASCII protocol for ODrive.
// Documentation found at https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html#sending-commands

/********************************************************************************
* @brief    Gives the string needed to set a motor velocity
* @param[0] strPointer: Where to put this string
* @param[1] vel: Velocity, in turns/sec * 100
* @return   N/A
*********************************************************************************
*/
void odrive_setVelocity(char *strPointer, int vel);


/********************************************************************************
* @brief    Gives the string needed to set a motor velocity by directly changing
*           the velocity attribute
* @param[0] strPointer: Where to put this string
* @param[1] vel: Velocity, in turns/sec * 100
* @return   N/A
*********************************************************************************
*/
void odrive_setVelocityW(char *strPointer, int vel);


/********************************************************************************
* @brief    Gives the string needed to set the control mode to ramped velocity
* @param    strPointer: Where to put this string
* @return   N/A
*********************************************************************************
*/
void odrive_setControlRamped(char *strPointer);


/********************************************************************************
* @brief    Gives the string needed to set the control mode to velocity
* @param[0] strPointer: Where to put this string
* @param[1] vel: Velocity, in turns/sec
* @return   N/A
*********************************************************************************
*/
void odrive_setControlVelocity(char *strPointer);


/********************************************************************************
* @brief    Gives the string needed to set a motor velocity
* @param[0] strPointer: Where to put this string
* @param[1] vel: Velocity, in turns/sec
* @return   N/A
*********************************************************************************
*/
void odrive_setRamp(char *strPointer, int ramp);


/********************************************************************************
* @brief    Gives the string needed to set the motor torque
* @param    strPointer: Where to put this string
* @return   N/A
*********************************************************************************
*/
void odrive_setTorque(char *strPointer, int torque);


/********************************************************************************
* @brief    Convert an integer to a string
* @param[0] num: number to be turned into a string * 100
* @param[1] str: Where to put this string
* @return   N/A
*********************************************************************************
*/
void intToStr(unsigned int num, char* str);


#endif
