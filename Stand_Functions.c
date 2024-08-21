/*
 ***********************************************************************************
 * @file    ${Stand_Functions.c}
 * @author  ${Ian Dolfi}
 * @date    ${05/27/2024}
 * @brief   Contains definitions for all main functions in the ABVI/D3 ROAM project
 ***********************************************************************************
 */

// For more information on programming the MSP430FR2355 access the datasheet and user's guide:
//https://www.ti.com/lit/ds/symlink/msp430fr2355.pdf
//https://www.ti.com/lit/ug/slau445i/slau445i.pdf

#include "Stand_Functions.h"
#include "odrive.h"

/*********************************************************************************/
/************************************MACROS***************************************/
/*********************************************************************************/

#define ADCINCH_BATT 4
#define ADCINCH_SPEED 5
#define ADCINCH_JOYX 1
#define ADCINCH_JOYY 2
#define ADCINCH_IRL 10
#define ADCINCH_IRR 11

/*********************************************************************************/
/***********************************Variables*************************************/
/*********************************************************************************/

//The values in the seq lists correspond to timer CCR0 values
//CCR0 value is equal to (1/f)/(0.000008), where f is the desired frequency. Examples:
//A3=568; A#3=536; B3=506; C4=478; C#4=451; D4=426; D#4=402; E4=379; F4=358; F#4=338; G4=317; A4=284; A#4=268; C5=239; D5=213

int* sequences[8];
int seq0[] = {478, 238, 0, -1};                   //Charge Battery
int seq1[] = {478, 0, 379, 0, 317, 0, 0, -1};           //Medium battery
int seq2[] = {478, 0, 379, 0, 317, 0, 239, 0, 0, -1};   //High Battery
int seq3[] = {238, 201, 159, -1};//{317, 268, 213, -1};                       //Danger Zone
int seq4[] = {201, 0, 159, 0, 150, 0, 0, 0, -1};//{402, 0, 536, 0, 402, 0, 0, 0, -1};        //Warning Zone
int seq5[] = {150, 159, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1};//{451, 506, 0, 0, 0, 0, -1};                //Fault Chime
int seq6[] = {150, 0, 150, -1};         //IR Warning
int seq7[] = {159, -1};                 //Resume

int* seq;                   //Sequence being used
volatile char seqIndex = 0; //Which note in the sequence is being played
volatile char repeatSeq = 0;//Does this sequence repeat?
volatile int note = 0;      //What note is playing?
volatile char isPlaying = 0;//Is a sequence playing?

volatile char *TX0Busy, *TX1Busy;
char *string0, *string1;    //String buffers for UART 0 and 1 respectively
char nullString[2] = {0,0};
char RXString0[MAX_RX_SIZE] = {0}, RXString1[MAX_RX_SIZE] = {0};
unsigned int RXIndex0 = 0, RXIndex1 = 0;
unsigned int batteryVoltage;

char rx1Char, rx0Char;

char ADCNextCh = 0, converting = 0;;
volatile int *ADC_Bat, *ADC_Speed, *ADC_JoyX, *ADC_JoyY, *ADC_IRL, *ADC_IRR;

int *mot0Speed, *mot1Speed;
char *drX, *drY, *dr0, *dr1;
int limitMask = 0xFFFF, *trm;
char limitNum = 2;

volatile char *testVar, *IREn;
signed char Red = 1, Green = 0;

char *STOP, *BRAKE, *IRWar = 0, stopStr[] = "\nw axis0.controller.input_vel 9999\nv 0 0",
                    restartString[] = "\nw axis0.controller.config.vel_ramp_rate 5\n"
                                        "w axis0.controller.config.vel_ramp_rate 5";


/*********************************************************************************/
/***********************************FUNCTIONS*************************************/
/*********************************************************************************/

void setTestVar(volatile char* var)
{
    testVar = var;
}

void initIO(char *IRENABLE, char *IRWARNING)
{
    //Initialize Digital I/O
    P3DIR |= BIT1 | BIT2;    //Outputs:          P3.1, P3.2
    P3DIR |= BIT1;           //Output:           P5.1

    P2REN = 0b11111100;     // Directional buttons:
    P2OUT = 0b11111100;     //Input w/ pull-up: P2.2, P2.3, P2.4, P2.5, P2.6, P2.7

    P4REN = 0b11;
    P4OUT = 0b11;           //Input w/ pull-up: P4.0, P4.1


    //Enable interrupts for bump sensors
    P4IE |= 0b00000011;
    P4IES |= 0b11;

    P5DIR |= BIT1 | BIT0;

    //Enable interrupts for RF only if using the adafruit board
    P4IE |= 0b11110000;

    IREn = IRENABLE;
    IRWar = IRWARNING;
}

void initUART(char *tx0, char *tx1)
{
    TX0Busy = tx0;
    TX1Busy = tx1;
    string0 = nullString;
    string1 = nullString;
    // Configure UART0 pins
    P1SEL0 |= BIT6 | BIT7; // set 2-UART pin as second function

    // Configure UART0
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;

    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 17-4: UCBRSx = 0x49
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA0BR0 = 52; // 8000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0x4900 | UCOS16 | UCBRF_1;

    UCA0CTLW0 &= ~UCSWRST; // Initialize eUSCI
    UCA0IE |= UCRXIE | UCTXIE;    // Enable USCI_A0 RX and TX interrupt

    //-----------------------------------------------
    //Same thing but with UART1
    //-----------------------------------------------
    // Configure UART1 pins
    //P1SEL0 |= BIT6 | BIT7; // set 2-UART pin as second function

    P4SEL0 |= BIT2 | BIT3;

    // Configure UART1
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;

    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 17-4: UCBRSx = 0x49
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA1BR0 = 52; // 8000000/16/9600
    UCA1BR1 = 0x00;
    UCA1MCTLW = 0x5000 | UCOS16 | UCBRF_1;

    UCA1CTLW0 &= ~UCSWRST; // Initialize eUSCI
    UCA1IE |= UCRXIE | UCTXIE;    // Enable USCI_A0 RX and TX interrupt

}

void initADC(volatile int *bat, volatile int *speed, volatile int *joyx, volatile int *joyy, volatile int *irl, volatile int *irr)
{
    //A4 = Battery; A5 = Speed Control; A1,A2 = Joystick X,Y; A10,A11 = IR Left,Right

    ADC_Bat = bat;
    ADC_Speed = speed;
    ADC_JoyX = joyx;
    ADC_JoyY = joyy;
    ADC_IRL = irl;
    ADC_IRR = irr;

    //Initialize Analog Inputs
    P1SEL0 |= 0b00110110;
    P1SEL1 |= 0b00110110;   //Analog Inputs:    A1, A2, A4, A5
    P5SEL0 |= BIT2 | BIT3;
    P5SEL1 |= BIT2 | BIT3;  //Analog Inputs:    A10, A11

    // Set the reference voltage to be 1.5 Volts
    PMMCTL2 |= REFVSEL_1;
    PMMCTL2 |= INTREFEN_1;

    // ADC Settings
    ADCCTL0 |= ADCSHT_14 | ADCON; // 32 ADCCLK cycles, changes once. ADC is now on

    P1SEL0 |= BIT0;
    P1SEL1 |= BIT0;
    ADCMCTL0 |= ADCSREF_3;  // ADCSREF (VR+ = VREF and  VR- = AVSS
    ADCCTL1 |= ADCSSEL_2 | ADCSHP | ADCDIV_7;

    //ADCCTL1 |= ADCSSEL_2 | ADCSHP | ADCSHS1 | ADCCONSEQ1; // SMCLK for the ADC clock.
        //Sample is sourced from the sampling timer.
        //Uses TB1.1B as a trigger
        // Single-Channel Repeat-Conversion

    ADCCTL2 = ADCRES_2; // 12 bit conversion or 14 clock cycles

    //ADCMCTL0 |= ADCINCH_11; // A11 input

    ADCIE |= ADCIE0; // Interrupt enable for checking ADC value

    __enable_interrupt(); // Allows for continuous ADC testing.
}


void initTimers(void)
{
    //Timer B0: Trigger routine interrupt every Xs to check non-emergency readings.
    TB0CTL |= TBCLR;

    //Set CLK input to be 512Hz
    TB0CTL |= TBSSEL0 | ID0 | ID1;  //Use ACLK/8 As source
    TB0EX0 |= TBIDEX_7;             //Further divide the source clock by 8

    TB0CCR0 = 64;      //0.25 second period
    TB0CCTL0 |= CCIE;   //Enable interrupts on CCR0

    sequences[0] = seq0;    //Point the sequences[] elements to the correct locations
    sequences[1] = seq1;
    sequences[2] = seq2;
    sequences[3] = seq3;
    sequences[4] = seq4;
    sequences[5] = seq5;
    sequences[6] = seq6;
    sequences[7] = seq7;

    //-----------------------------------------------------------------------------------------

    //Timer B1: Motor PWM at 50Hz
    TB1CTL |= TBCLR;

    //Set CLK input to be SMCLK/8. 1count = 0.001ms
    TB1CTL |= TBSSEL1 | ID0 | ID1;  //Use SMCLK/8 As source

    TB1CCTL1 = OUTMOD_7;    //Rest/Set mode: High then low
    TB1CCR0 = 20000;        //Frequency = 50Hz
    TB1CCR1 = 1;

    P2DIR |= BIT1 | BIT0;
    P2SEL0 |= BIT1 | BIT0;

    //-----------------------------------------------------------------------------------------

    //Timer B2: LED Blink at 1Hz
    TB2CTL |= TBCLR;

    //Set CLK input to be SMCLK/8. 1count = 0.001ms
    TB2CTL |= TBSSEL0 | ID0 | ID1;  //Use ACLK/8 As source

    TB2CCTL1 = OUTMOD_7;    //Rest/Set mode: High then low
    TB2CCR0 = 4096;        //Frequency = 1Hz
    TB2CCR1 = 2048+512;
    TB2CCR2 = 2048-512;
    TB2CCTL0 |= CCIE;   //Enable interrupts on CCR0

    P5DIR |= BIT0 | BIT1;
    P5SEL0 |= BIT0 | BIT1;

    TB2CTL |= MC0 | MC1;

    //-----------------------------------------------------------------------------------------

    //Timer B3: PWM for buzzer (TB3.4)
    //Buzzer needs to be between 30Hz and ~2000 Hz
    //With SMCLK input (8MHz), every count = 0.000125ms
    TB3CTL |= TBSSEL1 | ID0 | ID1;  //SMCLK/8
    TB3EX0 |= TBIDEX_7;             //Further divide source by 8
    TB3CCTL1 = OUTMOD_7;            //Reset/Set; For easy PWM

    P6SEL0 |= BIT0; //Select timer outputs as these pin functions
    P6DIR |= BIT0;
}

void initRTC()
{
    RTCMOD = 10000;//50000;
    RTCCTL |= RTCSS_3 | RTCSR | RTCIE;
}

int readADC_Blocking(char channel)
{
    //Don't try to read while another read is in progress
    while(converting)
    {
        _nop();
    }
    converting = 1;
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ~ADCINCH_15; //Clear ADCINCHx
    ADCMCTL0 |= channel; // Set input channel (ADCINCHx)
    ADCCTL0 |= ADCENC | ADCSC;
    _BIS_SR(CPUOFF | GIE); //Turn off CPU
    ADCCTL0 &= ~ADCENC;
    return ADCMEM0;
}


void readBattSpeed()
{
    //Don't try to read while another read is in progress
    while(converting)
    {
        _nop();
    }
    converting = 1;
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ~ADCINCH_15;    //Clear channel select bits
    ADCMCTL0 |= ADCINCH_BATT;   //Select channel as battery
    ADCNextCh = ADCINCH_SPEED;  //Set next channel to be speed
    ADCCTL0 |= ADCENC | ADCSC;  //Start conversion
}

void readIR()
{
    ADCCTL0 &= ~ADCENC;
    converting = 1;
    ADCMCTL0 &= ~ADCINCH_15;    //Clear channel select bits
    ADCMCTL0 |= ADCINCH_IRL;   //Select channel as battery
    ADCNextCh = ADCINCH_IRR;  //Set next channel to be speed
    ADCCTL0 |= ADCENC | ADCSC;  //Start conversion
}


void interpretIR()
{
    if (*IREn && ((*ADC_IRL < IR_THRESHOLD_LOW || *ADC_IRL > IR_THRESHOLD_HIGH)
           || (*ADC_IRR < IR_THRESHOLD_LOW || *ADC_IRR > IR_THRESHOLD_HIGH)))
    {
        if (!*IRWar)
        {
            *mot0Speed = 0;
            *mot1Speed = 0;
            sendString(stopStr, 0); //Tell ODrive 0 to stop
            sendString(stopStr, 1); //Tell ODrive 1 to stop
            sendString(stopStr, 0); //Tell ODrive 0 to stop
            sendString(stopStr, 1); //Tell ODrive 1 to stop
            setLEDs(-2,0); //Fast Blink Red LED
            playSequence(IR_WARNING, 0);

            *IRWar = 1;
        }
    }
    else
    {
        if (*IRWar)
        {
            if((*ADC_IRL > IR_THRESHOLD_LOW+100) && (*ADC_IRR > IR_THRESHOLD_LOW+100))
            {
                playSequence(RESUME, 0);
                *IRWar = 0;
            }
        }
    }
}


void readJoys()
{
    //Don't try to read while another read is in progress
    while(converting)
    {
        _nop();
    }
    converting = 1;
    ADCCTL0 &= ~ADCENC;
    ADCMCTL0 &= ~ADCINCH_15;    //Clear channel select bits
    ADCMCTL0 |= ADCINCH_JOYX;   //Select channel as Joystick X
    ADCNextCh = ADCINCH_JOYY;   //Set next channel to be Joystick Y
    ADCCTL0 |= ADCENC | ADCSC;  //Start conversion
}

void calibrateCLK()
{
    __bis_SR_register(SCG0);    // disable FLL
    //CSCTL3 |= SELREF__REFOCLK;// Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;  // DCOFTRIM=3, DCO Range = 8MHz
    CSCTL2 = FLLD_0 + 243;      // DCODIV = 8MHz
    __delay_cycles(3);          //Delay for 3 machine cycles
    __bic_SR_register(SCG0);    // enable FLL
    Software_Trim();            // Software Trim to get the best DCOFTRIM value

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;  // set default REFO(~32768Hz) as
                                                // ACLK source, ACLK = 32768Hz
                                                // Select DCODIV as MCLK and SMCLK source
}


void Software_Trim()
{
    //I do not know who wrote this code -Ian
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100; // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;      // Clear DCO fault flag
        } while (CSCTL7 & DCOFFG);  // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ); // Wait FLL lock status
                                                            // (FLLUNLOCK) to be stable

        // Suggest to wait 24 cycles of divided FLL reference clock
        while ((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0; // Read CSCTL0
        csCtl1Read = CSCTL1; // Read CSCTL1

        oldDcoTap = newDcoTap;                      // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;            // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070) >> 4;   // Get DCOFTRIM value

        if (newDcoTap < 256) // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;                      // Delta value between DCPTAP and 256
            if ((oldDcoTap != 0xffff) && (oldDcoTap >= 256))    // DCOTAP cross 256
                endLoop = 1;                                    // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim << 4);
            }
        }
        else // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;  // Delta value between DCPTAP and 256
            if (oldDcoTap < 256)            // DCOTAP cross 256
                endLoop = 1;                // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim << 4);
            }
        }

        if (newDcoDelta < bestDcoDelta) // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }
    } while (endLoop == 0); // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy; // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy; // Reload locked DCOFTRIM

    while (CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}

inline unsigned int convertVoltage(unsigned int sample)
{
    //A11 should see VDD_BAT * 0.4048
    //ADC should return a sample of A11*2730 (A11/1.5 * 2^12)
    return sample*1.206348;
}


inline unsigned char getButtons()
{
    return (P6IN >> 1)^0b111111; //Return a value that only uses the first 6 bits and
                                 //A bit is high if that button is pressed
}

inline unsigned char getRF()
{
#ifdef RF_ADAFRUIT
    return  P4IN >> 4;
#endif

}

inline unsigned char getBump()
{
    return (P4IN & 0b11) ^ 0b11;
}


void sendString(char* string, unsigned char port)
{
    if(port)
    {
        *TX1Busy = 1;
        string1 = string;
        UCA1TXBUF = *string1;
    }
    else
    {
        *TX0Busy = 1;
        string0 = string;
        UCA0TXBUF = *string0;
    }
}

void interpretString(unsigned char port)
{
    _nop();
    if (port) RXIndex1 = 0;
    else        RXIndex0 = 0;
}

void playSequence(char seqNum, char repeat)
{
    switch (seqNum)
    {
        case STOP_PLAYING:      //Stop Playing
            stopSequence();
            break;

        case CHARGE_BATTERY:    //Charge Battery
            seq = sequences[0];
            repeatSeq = repeat;
            startSequence();
            break;

        case LOW_BATTERY:       //Low Battery
            seq = sequences[0];
            repeatSeq = repeat;
            startSequence();
            break;

        case MEDIUM_BATTERY:    //Medium Battery
            seq = sequences[1];
            repeatSeq = repeat;
            startSequence();
            break;

        case FULL_BATTERY:      //Full Battery
            seq = sequences[2];
            repeatSeq = repeat;
            startSequence();
            break;

        case DANGER_ZONE:       //Danger Zone
            seq = sequences[3];
            repeatSeq = repeat;
            startSequence();
            break;

        case WARNING_ZONE:      //Warning Zone
            seq = sequences[4];
            repeatSeq = repeat;
            startSequence();
            break;

        case FAULT_CHIME:       //Fault Chime
            seq = sequences[5];
            repeatSeq = repeat;
            startSequence();
            break;

        case IR_WARNING:      //Warning Zone
            seq = sequences[6];
            repeatSeq = repeat;
            startSequence();
            break;

        case RESUME:       //Fault Chime
            seq = sequences[7];
            repeatSeq = repeat;
            startSequence();
            break;

        default:
            break;
    }

    /* Debug stuff
    if (seqNum == FAULT_CHIME)
    {
        __no_operation();
    }

    if (seqNum == DANGER_ZONE)
    {
        P6OUT |= BIT6;
    }
    else
    {
        P6OUT &= ~BIT6;
    }
    */
}

void startSequence(void)
{

    P6SEL0 |= BIT0; //Select TB3.4 as pin output to enable the buzzer
    P6DIR |= BIT0;

    seqIndex = 0;   //Point to beginning of sequence
    note = seq[0];  //Load the first note

    TB3CCR0 = note;     //Change PWM period to the first note value in the sequence
    TB3CCR4 = TB3CCR2 = TB3CCR1 = note >> 1;    //Divide the period 2. This sets the duty cycle at 50%
    TB3CCTL1 = OUTMOD_7;    //Reset/Set; For easy PWM
    TB3CCTL2 = OUTMOD_7;
    TB3CCTL4 = OUTMOD_7;
    TB3CTL |= MC0;          //Start the PWM timer

    TB0CTL |= TBCLR;        //Clear the note change timer
    TB0CTL |= MC0;          //Start the note change timer
    isPlaying = 1;
}

inline void stopSequence(void)
{
    TB3CTL = TBSSEL1 | ID0 | ID1 | TBCLR;   //Turn off PWM timer
    TB3CCTL1 = 0;                           //Ensure that timer outputs are all low
    TB3CCTL2 = 0;
    TB3CCTL4 = 0;
    //TB0CTL = TBSSEL0 | ID0 | ID1 | TBCLR;   //Turn off note change timer

    isPlaying = 0;

    //P6OUT &= ~BIT6; //FOR DEBUG ONLY
}

void setMotorSpeed(int motor0, int motor1)
{
    //Motor speed should be out of 20000
    TB1CTL &= ~MC0;
    TB1CTL |= TBCLR;
    if(motor0)
    {
        TB1CCR1 = motor0;
        TB1CTL |= MC0;
    }

    if(motor1)
    {
        TB1CCR2 = motor1;
        TB1CTL |= MC0;
    }
}

inline unsigned int applyLimit(unsigned int in)
{
    //return in >> limitNum;    //Less intensive, but gives low precision for speed control
    return in * (*ADC_Speed / 4095.0);
}

void calcMotors(int speed, int direction, char DirX, char DirY)
{
    int res0 = (DirX ? 1 : -1)*(direction>>1) + speed * (DirY ? 1 : -1);
    *mot0Speed = abs(res0);
    *dr0 = (res0<0 ? 0 : 1);


    int res1 = (DirX ? -1 : 1)*(direction>>1) + speed * (DirY ? 1 : -1);
    *mot1Speed = abs(res1);
    *dr1 = (res1<0 ? 0 : 1);
    /*
    //11 bits of speed and direction are being used at this point
    //speed = speed >> limitNum;  //Possibly temporary way of using the speed limit, just divide by 2 every power of 2

    //If direction is less than half of max,
        //If drX > 0 (turn right)
            //Then: Move left motor at max speed
                //  Scale right motor between max-0 based on direction 0-max
        //If drX < 0 (turn left)
            //Then: Move right motor at max speed
                //  Scale left motor between max-0 based on direction 0-max

    //Motor 0 = left; Motor 1 = right
    if(DirY)    //Forward
    {
        if(direction < 1024)    //Veer
        {
            if(DirX)    //Veer right
            {
                *mot0Speed = speed;
                *mot1Speed = (speed/2047.0 * ((direction^1023) << 1));
                *dr0 = 1;
                *dr1 = 1;
            }
            else            //Veer left
            {
                *mot1Speed = speed;
                *mot0Speed = (speed/2047.0 * ((direction^1023) << 1));
                *dr0 = 1;
                *dr1 = 1;
            }
        }
        else    //Turn
        {
            if(DirX)    //Turn right forward
            {
                *mot0Speed = speed;
                *mot1Speed = (speed/2047.0 * ((direction-1024) << 1));
                *dr0 = 1;
                *dr1 = 0;
            }
            else            //Turn left forward
            {
                *mot1Speed = speed;
                *mot0Speed = (speed/2047.0 * ((direction-1024) << 1));
                *dr0 = 0;
                *dr1 = 1;
            }
        }
    }
    else    //Reverse
    {
        if(direction < 1024)    //Veer
        {
            if(DirX)    //Veer right back
            {
                *mot1Speed = speed;
                *mot0Speed = (speed/2047.0 * ((direction^1023) << 1));
                *dr0 = 0;
                *dr1 = 0;
            }
            else            //Veer left back
            {
                *mot0Speed = speed;
                *mot1Speed = (speed/2047.0 * ((direction^1023) << 1));
                *dr0 = 0;
                *dr1 = 0;
            }
        }
        else    //Turn
        {
            if(DirX)    //Turn right back
            {
                *mot1Speed = speed;
                *mot0Speed = (speed/2047.0 * ((direction-1024) << 1));
                *dr0 = 1;
                *dr1 = 0;
            }
            else            //Turn left back
            {
                *mot0Speed = speed;
                *mot1Speed = (speed/2047.0 * ((direction-1024) << 1));
                *dr0 = 0;
                *dr1 = 1;
            }
        }
    }
    */
}

void setLimit()
{
    int i = 0;
    for (i = 0; !(*ADC_Speed << i & 0x800) && i < 12; i++)
    {
        _nop();
    }
    limitNum = i;
}

void setLEDs(signed char red, signed char green)
{
    TB2CCR0 = 0x2000;        //Frequency = 50Hz
    TB2CCR1 = 0xA00;
    TB2CCR2 = 0x600;
    if (red < 0)
    {
        if (red == -2)
        {
            TB2CCR0 = 0x1000;
            TB2CCR1 = 0xE00;
            TB2CCR2 = 0x200;
        }
        TB2CCTL1 |= OUTMOD_2; //Set red LED output to SET/RESET mode
    }
    else
    {
        TB2CCTL1 = 0; //Set red LED output to OUT mode
        if(!red)
        {
            TB2CCTL1 |= OUT;
        }
    }

    if (green < 0)
    {
        TB2CCTL2 |= OUTMOD_6; //Set red LED output to SET/RESET mode
    }
    else
    {
        TB2CCTL2 = 0; //Set green LED output to OUT mode
        if(!green)
        {
            TB2CCTL2 |= OUT;
        }
    }
}

void initMotors(int *motor0Speed, int *motor1Speed, char *Dir0, char *Dir1, int *Trim, char *brake, char *stop)
{
    mot0Speed = motor0Speed;
    mot1Speed = motor1Speed;
    dr0 = Dir0;
    dr1 = Dir1;
    BRAKE = brake;
    STOP = stop;
    trm = Trim;

}

void applyTrim(char dir)
{
    if(dir)
    {
        *trm += TRIM_FACTOR;
    }
    else
    {
        *trm -= TRIM_FACTOR;
    }
}

void ESTOP()
{
    *mot0Speed = 0;
    *mot1Speed = 0;
    *STOP = 1;
    *BRAKE = 1;
    sendString(stopStr, 0); //Tell ODrive 0 to stop
    sendString(stopStr, 1); //Tell ODrive 1 to stop
    setLEDs(1,0); //Solid Red LED
}

void faultTrap()
{
    //Sets the red LED to blink and the fault chime to play until the device is turned off
    setLEDs(-2,0);
    playSequence(FAULT_CHIME, 1);
    _BIS_SR(CPUOFF | GIE);
    while(1)
    {
        _nop();
    }
}


/*********************************************************************************/
/***********************************INTERRUPTS*************************************/
/*********************************************************************************/



/***********************************************************************************
 * @brief   UART 0 interrupt. For both receive and transmit.
 ***********************************************************************************/
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    unsigned char flags = UCA0IV;
    if(flags & 0x04)  //Transmit next character
    {
        if(*TX0Busy)
        {
            if(*string0)
            {
                string0 = ++string0;    //Procede to next character
                if (*string0)   //If we aren't at the end of the string
                {
                    UCA0TXBUF = *string0;
                }
                else
                {
                    UCA0TXBUF = TX_END_CHAR;
                }
            }
            else
            {
                //UCA0TXBUF = TX_END_CHAR;
                *TX0Busy = 0;
                *testVar = 1;
            }
        }
    }
    if(flags & 0x02) //Receive character
    {
        rx0Char = UCA0RXBUF;
        RXString0[++RXIndex0] = rx0Char;
        if(rx0Char == RX_END_CHAR) //If an end character is received
        {
            interpretString(0);
        }
    }
}


/***********************************************************************************
 * @brief   UART 1 interrupt. For both receive and transmit.
 ***********************************************************************************/
#pragma vector = USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    unsigned char flags = UCA1IV;
    if(flags & 0x04)  //Transmit next character
    {
        if(*TX1Busy)
        {
            if(*string1)
            {
                string1 = ++string1;    //Procede to next character
                if (*string1)   //If we aren't at the end of the string
                {
                    UCA1TXBUF = *string1;
                }
                else
                {
                    UCA1TXBUF = TX_END_CHAR;
                }
            }
            else
            {
                //UCA1TXBUF = TX_END_CHAR;
                *TX1Busy = 0;
                *testVar = 1;
            }
        }
    }
    if(flags & 0x02) //Receive character
    {
        rx1Char = UCA1RXBUF;
        RXString1[++RXIndex1] = rx1Char;
        if(rx1Char == RX_END_CHAR) //If an end character is received
        {
            interpretString(1);
        }
    }
}



/***********************************************************************************
 * @brief   ADC interrupt. Converts the sample to a voltage and compares that to the
 *          voltage thresholds. Will beep if battery is low and fault if very low.
 ***********************************************************************************/
#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch ((ADCMCTL0 & ADCINCH_15))
    {
        case ADCINCH_BATT:
            *ADC_Bat = ADCMEM0;
            break;
        case ADCINCH_SPEED:
            *ADC_Speed = ADCMEM0;
            setLimit();
            break;
        case ADCINCH_JOYX:
            *ADC_JoyX = ADCMEM0;
            _nop();
            break;
        case ADCINCH_JOYY:
            *ADC_JoyY = ADCMEM0;
            _nop();
            break;
        case ADCINCH_IRL:
            *ADC_IRL = ADCMEM0;
            break;
        case ADCINCH_IRR:
            *ADC_IRR = ADCMEM0;
            interpretIR();
            break;
    }

    if(ADCNextCh)
    {
        ADCCTL0 &= ~ADCENC;
        //ADCMCTL0 &= ~ADCENC
        ADCMCTL0 &= ~ADCINCH_15;
        ADCMCTL0 |= ADCNextCh;
        ADCNextCh = 0;
        ADCCTL0 |= ADCENC | ADCSC;
    }
    else
    {
        converting = 0;
    }
    __bic_SR_register_on_exit(CPUOFF);
    ADCIFG &= ~ADCIFG0;
}

/***********************************************************************************
 * @brief   Timer B2 interrupt. Important periodic sensor checks.
 ***********************************************************************************/
#pragma vector = TIMER2_B0_VECTOR
__interrupt void Timer_B2_CCR0_ISR(void)
{
    _nop();
}


/***********************************************************************************
 * @brief   Timer B0 interrupt. Progress to next note in the sequence.
 ***********************************************************************************/
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B0_CCR0_ISR(void)
{
    readIR();
    if(isPlaying)
    {
        if ((note = seq[++seqIndex]) < 0) //If the end of the sequence is reached (-1)
        {
            if (repeatSeq == 1)
            {
                seqIndex = 0;
                note = seq[0];
            }
            else
            {
                stopSequence();
                return;
            }
        }

        TB3CTL = TBSSEL1 | ID0 | ID1 | TBCLR;   //Stop PWM timer and clear it

        if (note != 0)  //Hard coded for 50% duty cycle
        {
            TB3CCTL1 = OUTMOD_7;
            TB3CCR0 = note;
            TB3CCR1 = note >> 1;
            TB3CTL |= MC0;          //Start PWM timer
        }
        else
        {
            TB3CCTL1 = 0;
        }
    }

    /*
    if (readBattery() < BATTERY_THRESHOLD_LOW)
    {
        playSequence(CHARGE_BATTERY, 0);
    }
    */
}


/***********************************************************************************
 * @brief   Port 4 GPIO interrupt. Handles RF and bump sensors
 ***********************************************************************************/
#pragma vector = PORT4_VECTOR
__interrupt void P4IFG_ISR(void)
{
    int iv = P4IV;
    if(iv == 2 || iv == 4)       //4.0 or 4.1
    {
        ESTOP();
        /*
        *mot0Speed = 0;
        *mot1Speed = 0;
        sendString(stopStr, 0);
        sendString(stopStr, 1);*/
    }
    else if(iv == 0X0A)  //4.4  D
    {
        setLEDs(1, 0);
        applyTrim(1);
        if(P4IN & BIT4)
        {
            P4IES |= BIT4;
        }
        else
        {
            P4IES &= ~BIT4;
        }
    }
    else if(iv == 0x0C)  //4.5  C
    {
        setLEDs(1, 0);
        applyTrim(0);
        if(P4IN & BIT5)
        {
            P4IES |= BIT5;
        }
        else
        {
            P4IES &= ~BIT5;
        }
    }
    else if(iv == 0x0E)  //4.6  B
    {
        setLEDs(1, 0);
        if(*STOP)   //Exit stopping
        {
            *STOP = 0;
            sendString(restartString, 0);
            sendString(restartString, 1);
            playSequence(MEDIUM_BATTERY, 0);
        }
        else        //Stop
        {
            ESTOP();
            playSequence(LOW_BATTERY, 0);
        }
        if(P4IN & BIT6)
        {
            P4IES |= BIT6;
        }
        else
        {
            P4IES &= ~BIT6;
        }
    }
    else if(iv == 0x10)  //4.7  A
    {
        setLEDs(1, 0);
        *IREn ^= 1;
        //RFD
        _nop();
    }
}



/***********************************************************************************
 * @brief   RTC Interrupt. Triggers ADC read for battery monitor and speed control
 ***********************************************************************************/
#pragma vector = RTC_VECTOR
__interrupt void RTCIFG_ISR(void)
{
    if (RTCIV)
    {
        /*
        if(++Red > 1) Red = -1;
        if(++Green > 1) Green = -1;
        setLEDs(Red, Green);
        */
        //readBattSpeed();
        //readJoys();
        //*ADC_JoyY += 200;
        //if (*ADC_JoyY > 4096) *ADC_JoyY = 0;

        if (*mot0Speed == 0 && *mot1Speed == 0) *BRAKE = 1;
    }
}
