/**********************************************************************
 * © 2015 Instituto Politécnico de Leiria.
 *   ESTG - DEE
 *
 * FileName:        main.c
 * Dependencies:    Header (.h) files if applicable, see below
 * Processor:       dsPIC30FXXXX
 * Compiler:        MPLAB® XC16 v1.24 or higher
 * Tested On:	    dsPIC30F3011
 *
 *
 * You agree that you are solely responsible for testing the code and 
 * determining its suitability.  Microchip has no obligation to modify, test, 
 * certify, or support the code.
 *
 * REVISION HISTORY:
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author            	Date      Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Cristiano Justino	04/11/2015	  First release of source file
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 **********************************************************************/

#if defined(__dsPIC30F__)
#include <p30Fxxxx.h>
#else
#error "Include file does not match processor setting"
#endif



// Confiuration Register Settings
// Internal FRC Oscillator
_FOSC(FRC_PLL8); // FRC Oscillator 
_FWDT(WDT_OFF); // Watchdog Timer Enabled/disabled by user software
_FGS(CODE_PROT_OFF & GWRP_OFF);
_FBORPOR(PWRT_64 & MCLR_EN & PBOR_ON & BORV20);
/* Clock related Settings */
#define FCY 14000000UL
#define MIPS 14.75
#define MATCH_VALUE 1006    // IntT1 Period = 0.0044sec.

/* libraries used on the program */
#include <libpic30.h>
#include <xc.h>
#include <timer.h>
#include <qei.h>
#include <ports.h>
#include <i2c.h>

/* Functions Declaration*/
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SI2CInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void I2CInit(void);
void QEIInit(void);
void PWMInit(void);
void Homing(void);
int AngularPositionCalculation(void);
void AngularSpeedCalculation(void);
int clipValue(int value, int min, int max);

/* Constants definition*/
#define ZERO_SPEED_PWM 500
#define PI 3.14159
#define TRUE    1
#define FALSE   0
#define CCW     1
#define CW      0
#define USE_I2C_Clock_Stretch


/* Constants and variables used for Speed Calculation */
#define MAXSPEED 271000 
#define HALFMAXSPEED (MAXSPEED>>1)
#define COUNT_PER_REV 1000 // 500 CPR
long Speed = 0;             // Speed (pulses) calculation
float real_speed = 0;       // Speed (rad/s) calculation
long AngPos[2] = {0, 0}; // Two variables are used for Speed Calculation


/* I2C management variables*/
struct FlagType {
    unsigned char AddrFlag : 1;
    unsigned char DataFlag : 1;
};

char i2c_value = 0, i2c_instr = 0;  // used to process the i2c messages
struct FlagType Flag;               // used to process the i2c messages
int secondByteFlag = FALSE;         // used to process the i2c messages
float pwm_value = 0;    // converted angular velocity to pwm scale
float ang_vel = 0;      // angular velocity received on the i2c bus from CPU


/* other global variables used*/
int counter_overflow = 0;       // used to manage the QEI counter
int homing_call = FALSE;        // used to call the homing function
int homing_flag = FALSE;        // used to flag active homing function 
int homing_cycle = FALSE;       // flag active homing function, used to process i2c messages
int speed_update = FALSE;       // used to flag new speed value available



// /*Declare constants/coefficients/calibration data to be stored in DataEEPROM*/
// int _EEDATA(32) fooArrayInDataEE[] = {101};
// /*Declare variables to be stored in RAM*/
// int fooArray1inRAM[] = {0xAAAA};
// int fooArray2inRAM[1];
// int rot = 0;
//_prog_addressT dst = 0x0FFF;



//Begin the Main Fucntion

int main(void) {

 //   _prog_addressT EE_addr = 0x0FFF;
    //    /* initialize a variable to represent the Data EEPROM address */
    // _init_prog_address(EE_addr, fooArrayInDataEE);

    //    /*Copy array "fooArrayinDataEE" from DataEEPROM to "fooArray2inRAM" in RAM*/
 //   _memcpy_p2d16(fooArray2inRAM, EE_addr, _EE_WORD);

    //    /*Write a row to Data EEPROM from array "fooArray1inRAM" */
 //   _write_eedata_row(EE_addr, fooArray1inRAM);

 //   rot = fooArray2inRAM[0];



    TRISDbits.TRISD0 = 1; // configures pin RDO of port D as input
    TRISDbits.TRISD1 = 0; // DIRECTION output // configures pin RD1 of port D as output
    TRISDbits.TRISD2 = 0; // LED onboard      // configures pin RD2 of port D as output
    TRISDbits.TRISD3 = 0; // LED onboard      // configures pin RD3 of port D as output            
    TRISEbits.TRISE0 = 0; // PWM      // Set RE1 as a digital output
    TRISEbits.TRISE8 = 1; // INDEX PIN    //Set RE8/INTO as a digital input
    TRISFbits.TRISF6 = 0; // PWM      // Set RE1 as a digital output
    LATEbits.LATE0 = 0;
    LATFbits.LATF6 = 0;

    #define INDEX LATEbits.LATE8    /* Define Pin RD1 as DIRECTION */     
    #define DIRECTION LATDbits.LATD1    /* Define Pin RD1 as DIRECTION */ 
        
    DIRECTION = CCW; /* Set Rotation DIRECTION => 0 = CW  |   1 = CCW  */

    /* Modules initialization*/
    I2CInit();
    PWMInit();
    QEIInit();
    ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_2 & EXT_INT_ENABLE);
    ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
    OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_64 & T1_SYNC_EXT_OFF &
            T1_SOURCE_INT, MATCH_VALUE);

    while (1) {
        LATDbits.LATD2 = 0;
        __delay_ms(50);
        LATDbits.LATD2 = 1;
        __delay_ms(100);
        


        if (homing_call) {            
            homing_call = 0;
            Homing();
        }


        if (speed_update) {
            // convert speed count to speed motor, scaling to  -MAX_MOTOR_RPS <= Speed <= MAX_MOTOR_RPS
            real_speed = (float) ((Speed / 0.0044)/ COUNT_PER_REV); // *60 for rpm |  0.0044 is the Timer period 
            //convert motor speed (rpm) to real ang_vel (rad/min)
            real_speed = real_speed * 0.0018 / 0.1047; // /60 for seconds            
            speed_update = FALSE;
        }
    }
}


/* Function used to set the wheels position on index*/
void Homing(void) {    
    PDC1 = ZERO_SPEED_PWM; /* PWM Duty-Cycle minimum is 10% for the used controller */
    homing_cycle = 1; // Homing started. variable used to change I2C routine
    DisableIntT1;
    DisableINT0;
    
    if (DIRECTION == CCW) 
        ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_2 & EXT_INT_ENABLE);
    else
        ConfigINT0(FALLING_EDGE_INT & EXT_INT_PRI_2 & EXT_INT_ENABLE);
    
    homing_flag = 1;
    PDC1 = (ZERO_SPEED_PWM << 1);   // Slow movement for homing rotation
    while (homing_flag);
    homing_cycle = 0;
    EnableIntT1;
}






/* Interruptions *****************************************************
 ********************************************************************/

/*****************************************************************
        I2C Bus Interruption
 *****************************************************************/
void __attribute__((interrupt, no_auto_psv)) _SI2CInterrupt(void) {

    unsigned char Temp; //used for dummy read
    int value = 0;
    
    if ((I2CSTATbits.R_W == 0)&&(I2CSTATbits.D_A == 0)) //Address matched
    {
        Temp = I2CRCV; //dummy read
        Flag.AddrFlag = 1; //next byte will be address           
    } else if ((I2CSTATbits.R_W == 0)&&(I2CSTATbits.D_A == 1)) //check for data	
    {
        if (Flag.AddrFlag) {
            Flag.AddrFlag = 0;
            Flag.DataFlag = 1; //next byte is data
            i2c_instr = I2CRCV;
    #if defined(USE_I2C_Clock_Stretch)
            I2CCONbits.SCLREL = 1; //Release SCL1 line
    #endif
        } else if (Flag.DataFlag) {


            if (!homing_cycle) {

                if (secondByteFlag == FALSE) {
                    i2c_value = I2CRCV; // store data into RAM                
                    Flag.AddrFlag = 0; //end of tx
                    Flag.DataFlag = 0;


                    if (i2c_instr == 'v') // Velocity command received
                    {
                        value = (int) i2c_value;
                        value = clipValue(value, -120, 120);
                        ang_vel = (float) value * 0.01; // convert from char to ang_vel

                        if (ang_vel == 0) // angular velocity is ZERO
                            PDC1 = ZERO_SPEED_PWM;

                        else // angular velocity is Positive
                            if (ang_vel > 0) {
                            DIRECTION = CCW;
                            ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_2 & EXT_INT_ENABLE);
                        } else // angular velocity is Negative
                        {
                            ang_vel = -ang_vel;
                            DIRECTION = CW;
                            ConfigINT0(FALLING_EDGE_INT & EXT_INT_PRI_2 & EXT_INT_ENABLE);
                        }


                        /* Speed conversion: rad/s to PWM value*/
                        // (pwm = ang_vel*59.3403+10.165;)
                        // (PDC1 = abs(pwm*28.892-109.19);)
                       // pwm_value = (ang_vel * 1714.456) + 184.4972; (OSC = 4MHz)
                        pwm_value = ang_vel * 4019 + 580;
                        if(pwm_value < ZERO_SPEED_PWM)
                            PDC1 = ZERO_SPEED_PWM;
                        else
                            PDC1 = (unsigned int) pwm_value;

                    }
                    else if (i2c_instr == 's') // STOP command received
                    {

                        PDC1 = ZERO_SPEED_PWM;
                    } 
                    else if (i2c_instr == 'h') // Homing command received
                    {
                        homing_call = 1;
                    }
                }
#if defined(USE_I2C_Clock_Stretch)
                I2CCONbits.SCLREL = 1; //Release SCL1 line
#endif                   
            }
            else { // Discarding 2 bytes
                i2c_value = I2CRCV; // store data into RAM 
                I2CCONbits.SCLREL = 1; //Release SCL1 line
                i2c_value = I2CRCV; // store data into RAM  
                I2CCONbits.SCLREL = 1; //Release SCL1 line
                Flag.AddrFlag = 0; //end of tx
            }
        }
    } else if ((I2CSTATbits.R_W == 1)&&(I2CSTATbits.D_A == 0)) {
        
        
        int aux;


        if (!homing_cycle) // Normal message returns rotation in radians
        {

            Temp = I2CRCV;
            I2CCONbits.STREN = 1; /* SCL clock stretch enable bit */
            aux = 'R';
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */ 
            
            
            
            
            // sending position, separated in 2 bytes
            value = AngularPositionCalculation(); // position rad. *10000

            aux = (unsigned char) (value >> 8); 
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */


            aux = (unsigned char) value;
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */
 //           LATDbits.LATD3 = 1;
            I2CCONbits.SCLREL = 1; //Release SCL1 line    
            while (I2CSTATbits.TBF); //Wait till all 
            
            
            
           
            // sending real speed, separated in 2 bytes
            value = (int)(real_speed * 1000); // convert ang_vel (*1000) to int value
 
            aux = (char) (value >> 8); 
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */
            
            aux = (char) value; 
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */            
        } 
        
        else // Homing message returns 'H' character
        {
            Temp = I2CRCV;
            I2CCONbits.STREN = 1; /* SCL clock stretch enable bit */
            aux = 'H';
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */

            I2CTRN = 0; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */

            I2CTRN = 0; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */
            I2CTRN = 0; /* data transferred to I2CTRN reg */
             
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */

            I2CTRN = 0; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */
        }
    }

    _SI2CIF = 0; //clear I2C1 Slave interrupt flag  
}

/*****************************************************************
       External Interruption INT0 
 *****************************************************************/
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {

      LATDbits.LATD3 = 1;
    if (DIRECTION == CCW) {
        counter_overflow = 0;
        POSCNT = 0;
    } else {
        counter_overflow = 4;
        POSCNT = 8856; // TOTAL  271000 count
    }

    if (homing_flag) {
        PDC1 = ZERO_SPEED_PWM; //MOTOR 0 RPM
        homing_flag = 0;
    }
    LATDbits.LATD3 = 0;
    IFS0bits.INT0IF = 0; // Clear INT0 interrupt flag
}

/*****************************************************************
       QEI Interruption
 *****************************************************************/
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void) {
    if (QEICONbits.UPDN)
        counter_overflow++;
    else
        counter_overflow--;

    IFS2bits.QEIIF = 0; // Clear QEI interrupt flag
}

/*****************************************************************
       TIMER1 Interruption
 *****************************************************************/
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    
    LATFbits.LATF6 = 1;
    AngularSpeedCalculation();  
    speed_update = TRUE;
    IFS0bits.T1IF = 0; // Clear timer 1 interrupt flag
    LATFbits.LATF6 = 0;
    return;
}


/*****************************************************************
 ******************************************************************
       Modules Initial Config  
 *****************************************************************		
 *****************************************************************/

/*****************************************************************
        Init PWM module 
 *****************************************************************/
void PWMInit(void) {
    /*~~~~~~~~~~~~~~~~~~~~~~~ PWM1 Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* PWM1 I/O Control Register register */
    PWMCON1bits.PEN1H = 0; /* PWM1H is controlled by GPIO module */
    PWMCON1bits.PEN1L = 1; /* PWM1L is controlled by PWM module */
    PWMCON1bits.PTMOD1 = 1; /* Select Independent Output PWM mode */

    /* Load PDC1 register with initial Duty Cycle value */
    PDC1 = ZERO_SPEED_PWM; /* PWM Duty-Cycle minimum is 10% for the used controller */
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /* Configure PTPER register to produce 5kHz PWM frequency */
    PTPER = 2947; /* PWM freq 5kHz - this value on PDC1 represents 50% */

    /* PWM Time Base Control Register */
    PTCONbits.PTEN = 1; /* Enable the PWM Module */
}

/*****************************************************************
        Init I2C1 Bus
 *****************************************************************/
void I2CInit(void) {
#if !defined(USE_I2C_Clock_Stretch)
    I2CCON = 0x8000; //Enable I2C module        
#else
    I2CCON = 0x9040; //Enable I2C module, enable clock stretching
#endif

    I2CADD = 0x10; // 7-bit I2C slave address must be initialised here. 
    IFS0 = 0;

    Flag.AddrFlag = 0; //Initlize AddFlag
    Flag.DataFlag = 0; //Initlize DataFlag

    _SI2CIE = 1;
}

/*****************************************************************
        Init QEI module 
 *****************************************************************/
void QEIInit(void) {
    ADPCFG |= 0x0038; // Configure QEI pins as digital inputs
    QEICONbits.QEIM = 0; // Disable QEI Module
    QEICONbits.CNTERR = 0; // Clear any count errors
    QEICONbits.QEISIDL = 0; // Continue operation during sleep
    QEICONbits.SWPAB = 1; // QEA and QEB swapped
    QEICONbits.PCDOUT = 0; // Normal I/O pin operation
    QEICONbits.POSRES = 1; // Index pulse do not reset position counter
    DFLTCONbits.CEID = 1; // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1; // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK = 4; // 1:32 clock divide for digital filter for QEn
    QEICONbits.QEIM = 5; //(2x mode) with position counter reset by match (MAXCNT)           
    //7 = QEI enabled (4x mode) with position counter reset by match (MAXCNT)
    //6 = QEI enabled (4x mode) with index pulse reset of position counter
    //5 = QEI enabled (2x mode) with position counter reset by match (MAXCNT)
    //4 = QEI enabled (2x mode) with index pulse reset of position counter
    //3 = Unused (module disabled)
    //2 = Unused (module disabled)
    //1 = Starts 16-bit Timer
    //0 = QEI/Timer off


    // instead of resetting the counter, EEPROM reading should be considered here
    counter_overflow = 0;
    POSCNT = 0; // Reset position counter
    MAXCNT = 0xFFFF;
    
   ConfigIntQEI(QEI_INT_ENABLE & QEI_INT_PRI_3); 
   

}

/*****************************************************************
 ******************************************************************
        Other General Functions
 *****************************************************************		
 *****************************************************************/

int clipValue(int value, int min, int max) {
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else return value;
}

/* Angular Position - Orientation [-PI; PI] rad. Returns radian*1000.  */
int AngularPositionCalculation(void) {
    float total_position, part_position, quadrant_value;

    if (counter_overflow == 4)
        quadrant_value = 6.1004;
    if (counter_overflow < 4)
        quadrant_value = 1.525092 * counter_overflow;


    part_position = (POSCNT * 1.525069) / 65535;

    total_position = part_position + quadrant_value; // position rad. [0;2PI]

    if (total_position > PI)
        total_position -= (2 * PI); // position rad. [-PI,PI]

    total_position = total_position * 1000; // position rad. *10000

    return (int) total_position;
}

/* Angular speed calculation. Returns pulses */
void AngularSpeedCalculation(void) {        
    
    long total_counter; // quadrant_counter;

    total_counter = POSCNT + (0xFFFF * counter_overflow); //quadrant_counter; 
    
    AngPos[1] = AngPos[0];
    AngPos[0] = total_counter;
    
    Speed = AngPos[0] - AngPos[1];

    if (Speed >= 0) {
        if (Speed >= (HALFMAXSPEED))
            Speed = Speed - MAXSPEED;
    } else {
        if (Speed < -(HALFMAXSPEED))
            Speed = Speed + MAXSPEED;
    }            
    return;
}
