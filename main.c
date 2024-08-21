/*
 ********************************************************************************
 * @file    ${main.c}
 * @author  ${Ian Dolfi}
 * @date    ${05/30/2024}
 * @brief   Main code execution in the RIT StandBiMEE project
 ********************************************************************************
 */

#include "msp430fr2355.h"
#include <msp430.h>
#include "Stand_Functions.h"
#include "odrive.h"

char* testString = "TESTab1\n"
        "w config.dc_max_positive_current 10\n"
        "w axis0.current_soft_max 9\n"
        "w axis0.config.motor.current_hard_max 10\n"
        "w axis0.controller.config.vel_integrator_gain 5\n"
        "w axis0.controller.config.vel_gain 3\n"
        "w config.dc_max_positive_current 10\n"
        "w axis0.config.motor.current_soft_max 9\n"
        "w axis0.current_hard_max 10\n"
        "w axis0.controller.config.vel_integrator_gain 5\n"
        "w axis0.controller.config.vel_gain 3\n"
        ;
char A[9] = {0b1, 0b11, 0b111, 0b1111, 0b11111, 0b111111, 0b1111111, 0b11111111, 0};
//char* A = "ABCDEFGHIJKLMNOP";

volatile char test = 0;
volatile char* testPointer;

volatile int Bat, Speed, JoyX, JoyY, IRL, IRR;
unsigned int lastSpeed = 0;

int speedX, speedY;
char dirX, dirY, dir0, dir1;
int motor0Speed = 0, motor1Speed = 0;
char motor0Dir, motor1Dir;
int trim;
char brake, stop, IREnable = 1, IRWarning = 0;

char nextButton, lastButton;

char tx0busy = 0, tx1busy = 0;

char tstStr[75] = {};

unsigned char joyControl; //Flag for the joystick being the domminant control on this frame
char message0[MAX_TX_SIZE], message1[MAX_TX_SIZE];

char stopString[] = "\nw axis0.controller.config.vel_ramp_rate 9999\nv 0 0";

signed char red, green;

void backout();
void normalMove();


int main(void){
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	PM5CTL0 &= ~LOCKLPM5;   //Disable the GPIO power-on default high-impedance mode
	                        //to activate previously configured port settings

	//P4OUT = 0;

	//Initialize all subsystems
	initIO(&IREnable, &IRWarning);
    calibrateCLK();
	initUART(&tx0busy, &tx1busy);
	initADC(&Bat, &Speed, &JoyX, &JoyY, &IRL, &IRR);
	initTimers();
	initRTC();
	initMotors(&motor0Speed, &motor1Speed, &dir0, &dir1, &trim, &brake, &stop);

	//For debug. Internally connect RX1 to TX1
	//(does not affect TX1 output from the ucontroller)
	UCA1STATW |= UCLISTEN;

    __enable_interrupt();
    //playSequence(DANGER_ZONE, 0);

    //Get initial readings on all of the analog inputs
    Bat = readADC_Blocking(4);
    Speed = readADC_Blocking(5);
    JoyX = readADC_Blocking(1);
    JoyY = readADC_Blocking(2);
    IRL = readADC_Blocking(10);
    IRR = readADC_Blocking(11);

    //Wait 5 seconds before going further
    int waitNum = 5;
    for (waitNum = waitNum; waitNum > 0; )
    {
        //brake is set by RTC
        if (brake)
        {
            waitNum--;
            brake = 0;
        }
    }

    if(Bat > BATTERY_THRESHOLD_HIGH)
    {
        playSequence(FULL_BATTERY, 0);
    }
    else if(Bat > BATTERY_THRESHOLD_MED)
    {
        playSequence(MEDIUM_BATTERY, 0);
    }
    else if(Bat > BATTERY_THRESHOLD_LOW)
    {
        playSequence(LOW_BATTERY, 0);
    }
    else
    {
        playSequence(LOW_BATTERY, 1);
        faultTrap();
    }


    //__disable_interrupt();



/*
    testPointer = &test;
    setTestVar(testPointer);

	sendString(testString, 0);
	while(1)
	{
	    while(!test)
	    {
	        ;
	    }

	    UCA1CTLW0 |= UCTXBRK;
        P4SEL0 &= ~BIT2; //Inverted UART RX on 4.2
        P4SEL0 &= ~BIT3; //Noninverted TX on 4.3
        P4SEL1 |= BIT2; //Inverted UART RX on 4.2
        P4SEL1 |= BIT3; //Noninverted TX on 4.3
        P4SEL0 = 0;
        P4SEL1 = 0;

        sendString(A,0);
        test = 0;
        while(!test)
        {
            ;
        }


        UCA1CTLW0 |= UCTXBRK;
        P4SEL1 &= ~BIT2; //Inverted UART RX on 4.2
        P4SEL1 &= ~BIT3; //Noninverted TX on 4.3
        P4SEL0 |= BIT2; //Inverted UART RX on 4.2
        P4SEL0 |= BIT3; //Noninverted TX on 4.3

        sendString(A,0);
        test = 0;
	}*/

	//_BIS_SR(CPUOFF | GIE); //Turn off CPU

    RTCCTL &= ~RTCIE;

    /*while(1)
    {
        if(!(ADCCTL1 & ADCBUSY) && !(tx1busy) && !(tx0busy))
        {
            readBattSpeed();
            //Speed = Speed + 15;
            //if (Speed > 5000) Speed = 0;
            if(Speed + 10 < lastSpeed || Speed - 10 > lastSpeed)
            {
                lastSpeed = Speed;
                odrive_setVelocity(tstStr, applyLimit((unsigned)Speed) >> 2);
                sendString(tstStr, 1);
                sendString(tstStr, 0);
                //setMotorSpeed(Speed<<3, Speed<<3);
            }
        }
    }*/

    //----------------------
    //  Initialize ODrives
    //----------------------

    //Send a useless test string
    sendString(testString, 0);
    sendString(testString, 1);

    while ((tx1busy) || (tx0busy))
    {
        _nop();
    }

    sendString(testString, 0);
    sendString(testString, 1);

    while ((tx1busy) || (tx0busy))
    {
        _nop();
    }

    //Set the ramp speed that the odrives work in
    odrive_setRamp(message0, 100);
    odrive_setRamp(message1, 100);
    sendString(message0, 0);
    sendString(message1, 1);


    while ((tx1busy) || (tx0busy))
    {
        _nop();
    }

    //Send again just in case
    odrive_setRamp(message0, 100);
    odrive_setRamp(message1, 100);
    sendString(message0, 0);
    sendString(message1, 1);


    while ((tx1busy) || (tx0busy))
    {
        _nop();
    }

    _nop();
    stop = 0;
    IREnable = 0;

    while (1)
    {
        //For debug. usually checked periodicaly by the RTC interrupt
        readBattSpeed();
        readJoys();

        //Check battery voltage
        if(convertVoltage(Bat) < BATTERY_THRESHOLD_LOW)
        {
            //faultTrap();
        }

        //Check IR drop sensors
        if (IREnable && IRWarning)
        {
            //If a drop or an object is detected, only allow backing up slowly
            backout();
        }
        else if (getBump()) //Check if the bump sensors are pressed
        {
            //If currently pumping into an object is detected, only allow backing up slowly
            backout();
        }
        else    //Move normally
        {
            normalMove();
        }

        //Create the messages to be sent to the ODrives
        odrive_setVelocity(message0, applyLimit(motor0Speed >> 3) * (dir0 ? 1 : -1));
        odrive_setVelocity(message1, applyLimit(motor1Speed >> 3) * (dir1 ? -1 : 1));

        //Bottleneck for waiting for UART transaction to complete.
        //If this is an issue, change baud rate in initUART()
        while ((tx1busy) || (tx0busy))
        {
            _nop();
        }
        if(!stop && !brake)   //Send message generated by controls
        {
            sendString(message1, 1);
            sendString(message0, 0);
        }
        else        //Send brake string
        {
            sendString(stopString, 1);
            sendString(stopString, 0);
        }

        //Set LED colors accordingly
        red = 0; green = 0;
        if(stop)
        {
            red = -2;
            green = -1;
        }
        else if(motor0Speed == 0 && motor1Speed == 0)
        {
            green = -1;
        }
        else
        {
            if(dir0 == 0 || dir1 == 0)
            {
                red = 1;
            }
            else
            {
                red = 0;
            }

            if(dir0 == 1 || dir1 == 1)
            {
                green = 1;
            }
            else
            {
                green = 0;
            }
        }

        setLEDs(red, green);

        //IRL = readADC_Blocking(11);
    }

	return 0;
}

void normalMove()
{
    joyControl = 0;
    //Check joystick first


    //When moving, backwards is half the speed as forwards
    if (JoyX < JOY_DEAD_L)
    {
        joyControl = 1;
        speedX = (JoyX^2047) - JOY_DEAD; //Inverse then subtract the deadband
        dirX = 0;
        ;
    }
    else if (JoyX > JOY_DEAD_H)
    {
        joyControl = 1;
        speedX = (JoyX - 2048) - JOY_DEAD; //Divide by 2 then subtract the deadband
        dirX = 1;
    }
    else
    {
        speedX = 0;
    }

    if (JoyY < JOY_DEAD_L)
    {
        joyControl = 1;
        speedY = ((JoyY^2047) - JOY_DEAD) >> 1; //Inverse then subtract the deadband
        dirY = 0;
    }
    else if (JoyY > JOY_DEAD_H)
    {
        joyControl = 1;
        speedY = (JoyY - 2048) - JOY_DEAD; //Divide by 2 then subtract the deadband
        dirY = 1;
    }
    else
    {
        speedY = 0;
    }
    //dirY = 0;
    //speedY = 2047;
    //joyControl = 0;

    if(joyControl)  //Calculate speeds based on joystick
    {
        calcMotors(speedY, speedX, dirX, dirY);
    }
    else    //Use button control
    {
        lastButton = nextButton;
        nextButton = getButtons();
        if(nextButton & 0b1)           //Forward
        {
            if(trim > 0)
            {
                motor0Speed = 2047;
                motor1Speed = 2047-trim;
            }
            else
            {
                motor0Speed = 2047+trim;
                motor1Speed = 2047;
            }
            dir0 = 1;
            dir1 = 1;
            nextButton = 0b1;
        }
        else if(nextButton & 0b10)     //Backward
        {
            if(trim > 0)
            {
                motor1Speed = 1023;
                motor0Speed = 1023-trim;
            }
            else
            {
                motor1Speed = 1023+trim;
                motor0Speed = 1023;
            }
            dir0 = 0;
            dir1 = 0;
            nextButton = 0b10;
        }
        else if(nextButton & 0b100)    //Left
        {
            motor0Speed = 2047;
            motor1Speed = 2047;
            dir0 = 0;
            dir1 = 1;
            nextButton = 0b100;
        }
        else if(nextButton & 0b1000)   //Right
        {
            motor0Speed = 2047;
            motor1Speed = 2047;
            dir0 = 1;
            dir1 = 0;
            nextButton = 0b1000;
        }
        else if(nextButton & 0b10000)  //Veer left
        {
            motor0Speed = 1023+trim;
            motor1Speed = 2047;
            dir0 = 1;
            dir1 = 1;
            nextButton = 0b10000;
        }
        else if(nextButton & 0b100000) //Veer right
        {
            motor0Speed = 2047;
            motor1Speed = 1023-trim;
            dir0 = 1;
            dir1 = 1;
            nextButton = 0b100000;
        }
        else
        {
            motor0Speed = 0;
            motor1Speed = 0;
            nextButton = 0;
            //stop = 1;
            trim = 0;
            if (lastButton != 0)
            {
                brake = 0;
                RTCCTL |= RTCSR;
            }
        }
    }

}

void backout()
{

    if (JoyY < JOY_DEAD_L)
    {
        joyControl = 1;
        speedY = ((JoyY^2047) - JOY_DEAD) >> 1; //Inverse then subtract the deadband
        dirY = 0;
    }
    else
    {
        speedY = 0;
    }

    if(joyControl)  //Calculate speeds based on joystick
    {
        calcMotors(speedY, speedX, dirX, dirY);
    }
    else    //Use button control
    {
        lastButton = nextButton;
        nextButton = getButtons();
        if(nextButton & 0b10)     //Backward
        {
            if(trim > 0)
            {
                motor1Speed = 1023;
                motor0Speed = 1023-trim;
            }
            else
            {
                motor1Speed = 1023+trim;
                motor0Speed = 1023;
            }
            dir0 = 0;
            dir1 = 0;
            nextButton = 0b10;
        }
        else
        {
            motor0Speed = 0;
            motor1Speed = 0;
            nextButton = 0;
        }
    }
}
