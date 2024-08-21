#ifndef ROAM_FUNCTIONS_H
#define ROAM_FUNCTIONS_H
/*
 ************************************************************************************
 * @file    ${Stand_Functions.h}
 * @author  ${Ian Dolfi}
 * @date    ${05/27/2024}
 * @brief   Contains declarations for all main functions in the ABVI/D3 ROAM project
 ************************************************************************************
 */

#include <stdint.h>
#include "msp430fr2355.h"
#include <msp430.h>

#define RF_ADAFRUIT 1 //Comment this if using the other RF controller type


/*********************************************************************************/
/************************************MACROS***************************************/
/*********************************************************************************/

#define SAFE_ZONE       0
#define STOP_PLAYING    0
#define CHARGE_BATTERY  1
#define LOW_BATTERY     2
#define MEDIUM_BATTERY  3
#define PLUGGED_IN      3
#define FULL_BATTERY    4
#define DANGER_ZONE     5
#define WARNING_ZONE    6
#define FAULT_CHIME     7
#define IR_WARNING      8
#define RESUME          9

#define MCLK_FREQ_MHZ 8 // MCLK = 8MHz

//Measured in ADC value, these are the lower limits of the voltage zones indicated
#define BATTERY_THRESHOLD_HIGH  118 //39.9V
#define BATTERY_THRESHOLD_MED   106 //35.9V
#define BATTERY_THRESHOLD_LOW   94  //31.8V

#define MAX_RX_SIZE 77
#define MAX_TX_SIZE 77
#define RX_END_CHAR '\n'
#define TX_END_CHAR '\n'

#define IR_THRESHOLD_LOW    2400
#define IR_THRESHOLD_HIGH   3100

#define TRIM_FACTOR 100

#define JOY_DEAD    512
#define JOY_DEAD_L  1536
#define JOY_DEAD_H  2560


/*********************************************************************************/
/*****************************FUNCTION DECLARATIONS*******************************/
/*********************************************************************************/


/********************************************************************************
* @brief    Links a test variable here to a test variable in main
* @param[0] N/A
* @return   N/A
*********************************************************************************
*/
void setTestVar(volatile char* var);


/********************************************************************************
* @brief    Initializes general IO pins, not including those associated with other
*           initialized peripherals
* @param[0] IRENABLE: Pointer to IREnable flag in main
* @return   N/A
*********************************************************************************
*/
void initIO(char *IRENABLE, char *IRWARNING);


/********************************************************************************
* @brief    Initializes UART0 and UART1, including baud rates and pin selection
* @param[0] N/A
* @return   N/A
*********************************************************************************
*/
void initUART();


/********************************************************************************
* @brief    Initialize timers
*           Timer 0: Checks IR sensors and joysticks routinely
*           Timer 1: Controls PWM for that style of motor controller
*           Timer 2: Blinks the LEDs and changes the note if a sequence is playing
*           Timer 3: Buzzer output
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void initTimers();


/********************************************************************************
* @brief    Initialize RTC, 5s intervals
*           On interrupt, checks the battery level and the speed control knob
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void initRTC();


/********************************************************************************
* @brief    Set MCLK speed to 8MHz
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void calibrateCLK();


/********************************************************************************
* @brief    Used by calibrakeCLK()
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void Software_Trim();


/********************************************************************************
* @brief    Initialize ADC by linking variables to main code, set references,
*           and set sampling times
* @param[0] bat: pointer to battery sample storage variable in main
* @param[1] speed: pointer to speed limit sample storage variable in main
* @param[2] joyx: pointer to joystick X sample storage variable in main
* @param[3] joyy: pointer to joystick Y sample storage variable in main
* @param[4] irl: pointer to left IR sensor sample storage variable in main
* @param[5] irr: pointer to right IR sensor sample storage variable in main
* @return   N/A
*********************************************************************************
*/
void initADC(volatile int *bat, volatile int *speed, volatile int *joyx, volatile int *joyy, volatile int *irl, volatile int *irr);


/********************************************************************************
* @brief    Read a specific channel on the ADC and do not return until it is obtained
* @param    channel: channel to sample
* @return   The raw ADC reading (12-bit)
*********************************************************************************
*/
int readADC_Blocking(char channel);


/********************************************************************************
* @brief    Read the battery channel then the speed channel from ADC.
*           Will return before conversion is done.
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void readBattSpeed();


/********************************************************************************
* @brief    Read the IRL channel then the IRR channel from ADC.
*           Stops any conversion that is currently going
*           Will return before conversion is done.
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void readIR();


/********************************************************************************
* @brief    Determine if the stander should stop due to IR readings
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void interpretIR();


/********************************************************************************
* @brief    Read the Joystick X channel then the Joystick Y channel from ADC.
*           Will return before conversion is done.
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void readJoys();


/********************************************************************************
* @brief    Converts the raw ADC battery reading to a voltage (in V*10)
* @param    sample: Raw ADC sample of battery channel
* @return   N/A
*********************************************************************************
*/
inline unsigned int convertVoltage(unsigned int sample);


/********************************************************************************
* @brief    Formats the input register that holds the button information
* @param    N/A
* @return   A char with the first 6 bits each corresponding to 1 = pressed
*           X-X-FWD-BCK-LFT-RGT-VLFT-VR
*********************************************************************************
*/
inline unsigned char getButtons();


/********************************************************************************
* @brief    Formats the input register that holds the RF pin information
*           For the Adafruit RF board only
* @param    N/A
* @return   A char with the first 4 bits corresponding to 1 = toggled
*           X-X-X-X-A-B-C-D
*********************************************************************************
*/
inline unsigned char getRF();


/********************************************************************************
* @brief    Formats the input register that holds the button information
* @param    N/A
* @return   A char with the first 6 bits each corresponding to 1 = pressed
*           X-X-FWD-BCK-LFT-RGT-VLFT-VR
*********************************************************************************
*/
inline unsigned char getBump();


/********************************************************************************
* @brief    When an end character is received, act on the full message that
*           was received
* @param    port: UART port to the end character came from
* @return   N/A
*********************************************************************************
*/
void interpretString(unsigned char port);


/********************************************************************************
* @brief    When an end character is received, act on the full message that
*           was received
* @param[0] string: pointer to string to send
* @param[1] port: port to send the string on
* @return   N/A
*********************************************************************************
*/
void sendString(char* string, unsigned char port);


/********************************************************************************
* @brief    Initialize communication with ODrives and link relevent variables
* @params   Pointers to speed and direction variables in main
* @return   N/A
*********************************************************************************
*/
void initMotors(int *moter0Speed, int *moter1Speed, char *Dir0, char *Dir1, int *Trim, char *brake, char *stop);


/********************************************************************************
* @brief    Set the speed limit from the ADC reading
* @params   N/A
* @return   N/A
*********************************************************************************
*/
void setLimit();


/********************************************************************************
* @brief    Apply the speed limit to a speed value
* @params   in: the input speed
* @return   Resultant speed
*********************************************************************************
*/
inline unsigned int applyLimit(unsigned int in);


/********************************************************************************
* @brief    Sets PWM for both motors
* @params   motor 0 and motor 1 speeds (0-2047)
* @return   N/A
*********************************************************************************
*/
void setMotorSpeed(int motor0, int motor1);


/********************************************************************************
* @brief    Apply a coordinate system change
*           from (speed, rotation) to (velocity 0, velocity 1)
*           Will edit motor0speed, motor1speed, dir0, and dir1
*           Has applyLimit() built-in
* @params   N/A
* @return   N/A
*********************************************************************************
*/
void calcMotors(int speed, int direction, char DirX, char DirY);


/********************************************************************************
* @brief    Will turn on, turn off, or blink the LEDs
* @param[0] red:    red LED value. 1 = ON, 0 = OFF, -1 = blink, -2 = blink fast
* @param[1] green:  green LED value. 1 = ON, 0 = OFF, -1 = blink
* @return   N/A
*********************************************************************************
*/
void setLEDs(signed char red, signed char green);


/********************************************************************************
* @brief    Intentionally get stuck in an infinite loop. Error handling
* @params   N/A
* @return   N/A
*********************************************************************************
*/
void faultTrap();


/********************************************************************************
* @brief    Plays a sequence of notes on the buzzer and vibrations on the motors+
* @param[0] seqNum: Which sequence to play
* @param[1] repeat: Does this sequence continue to repeat? (T/F)
* @return   N/A
*********************************************************************************
*/
void playSequence(char seqNum, char repeat);


/********************************************************************************
* @brief    Start the note change timer and PWM timer
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void startSequence(void);


/********************************************************************************
* @brief    Stop the note change timer and the PWM timer.
* @param    N/A
* @return   N/A
*********************************************************************************
*/
inline void stopSequence(void);


/********************************************************************************
* @brief    Add trim left or right to the direction vector
* @param    dir: direction to apply trim to
* @return   N/A
*********************************************************************************
*/
void applyTrim(char dir);


/********************************************************************************
* @brief    Toggles if the IR sensors are active or not. For outdoor use.
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void toggleIR();


/********************************************************************************
* @brief    Emergency stop. Lock the motors.
* @param    N/A
* @return   N/A
*********************************************************************************
*/
void ESTOP();

#endif
