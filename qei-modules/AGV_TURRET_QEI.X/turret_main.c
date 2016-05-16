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
 * Cristiano Justino	13/11/2015	  First release of source file
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 **********************************************************************/

#if defined(__dsPIC30F__)
#include <p30Fxxxx.h>
#else
#error "Include file does not match processor setting"
#endif

/* THIS CODE IS USED TO CONTROL THE ROBOT'S TURRET  */
/* DO NOT USED IT FOR OTHER PURPOSES.               */
/* IT MAY CAUSE DAMAGE OR INJURY                    */

// Confiuration Register Settings
// Internal FRC Oscillator
_FOSC(FRC_PLL8); // FRC Oscillator 
_FWDT(WDT_OFF); // Watchdog Timer Enabled/disabled by user software
_FGS(CODE_PROT_OFF & GWRP_OFF);
_FBORPOR(PWRT_64 & MCLR_EN & PBOR_ON & BORV20);
/* Clock related Settings */
#define FCY 14000000UL
#define MIPS 14.75

/* libraries used on the program */
#include <libpic30.h>
#include <stdlib.h>
#include <xc.h>
#include <timer.h>
#include <qei.h>
#include <ports.h>
#include <i2c.h>
#include <math.h>

/* Functions Declaration*/
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SI2CInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _QEIInterrupt(void);
void I2CInit(void);
void QEIInit(void);
void PWMInit(void);
void Homing(void);
int AngularPositionCalculation(void);
void NewPositionCommand(float rad_value);
int clipValue(int value, int min, int max);

/* Constants definition*/
#define PI 3.14159
#define TRUE    1
#define FALSE   0
#define CCW     0
#define CW      1
#define USE_I2C_Clock_Stretch
#define ZERO_SPEED_PWM 530
#define MAX_COUNT 83600
#define MAX_RAMP 5000   // 21.5º
#define MIN_RAMP 2000   //  8.61º
#define DEAD_ZONE 500   //  2.15º
#define STD_VEL 0.1
#define MAX_STD_VEL 1
#define VEL_RAMP ((MAX_STD_VEL - STD_VEL)/(MAX_RAMP - MIN_RAMP))
#define RAMP_COMP ((MAX_RAMP * VEL_RAMP) - MAX_STD_VEL)

/* I2C management variables*/
struct FlagType {
    unsigned char AddrFlag : 1;
    unsigned char DataFlag : 1;
};

char i2c_value = 0, i2c_instr = 0; // used to process the i2c messages
struct FlagType Flag; // used to process the i2c messages
int secondByteFlag = FALSE; // used to process the i2c messages
unsigned int pwm_value = 0; // converted angular velocity to pwm scale
int snd_value = 0;
int rcv_value = 0;
float radi = 0;



/* other global variables used*/
int counter_overflow = 0; // used to manage the QEI counter
int homing_call = FALSE; // used to call the homing function
int homing_flag = FALSE; // used to flag active homing function 
int homing_cycle = FALSE; // flag active homing function, used to process i2c messages



/*Declare constants/coefficients/calibration data to be stored in DataEEPROM*/
//int _EEDATA(32) fooArrayInDataEE[] = {101};
/*Declare variables to be stored in RAM*/
//int fooArray1inRAM[] = {0xAAAA};
//int fooArray2inRAM[1];
//int rot = 0;
//_prog_addressT dst = 0x0FFF;




//Begin the Main Fucntion
int main(void) {
    
/* work to do: save position on eeprom */    
//    _prog_addressT EE_addr = 0x0FFF;
//    //    /* initialize a variable to represent the Data EEPROM address */
//    // _init_prog_address(EE_addr, fooArrayInDataEE);
//
//    //    /*Copy array "fooArrayinDataEE" from DataEEPROM to "fooArray2inRAM" in RAM*/
//    _memcpy_p2d16(fooArray2inRAM, EE_addr, _EE_WORD);
//
//    //    /*Write a row to Data EEPROM from array "fooArray1inRAM" */
//    _write_eedata_row(EE_addr, fooArray1inRAM);
//
//    rot = fooArray2inRAM[0];



    TRISDbits.TRISD0 = 1; // configures pin RDO of port D as input
    TRISDbits.TRISD1 = 0; // DIRECTION output // configures pin RD1 of port D as output
    TRISDbits.TRISD2 = 0; // LED onboard      // configures pin RD2 of port D as output
    TRISDbits.TRISD3 = 0; // LED onboard      // configures pin RD3 of port D as output            
    TRISEbits.TRISE0 = 0; // PWM      // Set RE1 as a digital output
    TRISEbits.TRISE8 = 1; // INDEX PIN    //Set RE8/INTO as a digital input
    //    TRISFbits.TRISF6 = 0; // PWM      // Set RE1 as a digital output
    LATEbits.LATE0 = 0;
    //    LATFbits.LATF6 = 0;


#define INDEX LATEbits.LATE8    /* Define Pin RD1 as DIRECTION */     
#define DIRECTION LATDbits.LATD1    /* Define Pin RD1 as DIRECTION */ 

    DIRECTION = CCW; /* Set Rotation DIRECTION => 0 = CW  |   1 = CCW  */

    /* Modules initialization*/
    I2CInit();
    PWMInit();
    QEIInit();
    ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);

    
    

    while (1) {
        LATDbits.LATD2 = 0;
        __delay_ms(50);
        LATDbits.LATD2 = 1;
        __delay_ms(100);


        if (homing_call) {
            homing_call = 0;
            Homing();
            LATDbits.LATD3 = 0;
        }
    }
}

/* Function used to set the wheels position on index*/
void Homing(void) {
    PDC1 = ZERO_SPEED_PWM; /* PWM Duty-Cycle minimum is 10% for the used controller */
    homing_cycle = 1; // Homing started. variable used to change I2C routine

    long actual_position, desired_position, delta_position;

    actual_position = POSCNT + (counter_overflow * 65535);

    // convert rad to counter values           
    desired_position = 0;

    // calculate displacement's sense and value
    if (actual_position <= (MAX_COUNT / 2)) {
        delta_position = desired_position - actual_position;
    } else {
        delta_position = MAX_COUNT - actual_position + desired_position;
    }

    if (delta_position == 0) // angular velocity is ZERO
        PDC1 = ZERO_SPEED_PWM;

    else // displacement is Positive
        if (delta_position > 0) {
        DIRECTION = CCW;
        ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);
    } else // angular velocity is Negative
    {
        DIRECTION = CW;
        ConfigINT0(FALLING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);
    }

    homing_flag = 1;
    PDC1 = ((ZERO_SPEED_PWM << 1)-(ZERO_SPEED_PWM / 4)); // Slow movement for homing rotation
    while (homing_flag);
    homing_cycle = 0;
    return;
}






/* Interruptions *****************************************************
 ********************************************************************/

/*****************************************************************
        I2C Bus Interruption
 *****************************************************************/
void __attribute__((interrupt, auto_psv)) _SI2CInterrupt(void) {

    unsigned char Temp; //used for dummy read


    if ((I2CSTATbits.R_W == 0)&&(I2CSTATbits.D_A == 0)) //Address matched
    {
        Temp = I2CRCV; //dummy read
        Flag.AddrFlag = 1; //next byte will be address           
    }
    else if ((I2CSTATbits.R_W == 0)&&(I2CSTATbits.D_A == 1)) //check for data	
    {
        if (Flag.AddrFlag) {
            Flag.AddrFlag = 0;
            Flag.DataFlag = 1; //next byte is data
            i2c_instr = I2CRCV;
#if defined(USE_I2C_Clock_Stretch)
            I2CCONbits.SCLREL = 1; //Release SCL1 line
#endif

        } else if (Flag.DataFlag) {


            if (!homing_cycle) { // Normal operation. Deals with messages 

                if (secondByteFlag == FALSE) { // first byte after instruct

                    i2c_value = I2CRCV; // store data into RAM    
                    Flag.AddrFlag = 0; //
                    Flag.DataFlag = 0; //end of tx                     


                    if (i2c_instr == 'k') // Velocity command received
                    {

                        rcv_value = (int) (i2c_value << 8);

                    }
                    else if (i2c_instr == 'v') // Velocity command received
                    {
                        rcv_value = (int) (rcv_value | ((unsigned char) (i2c_value)));
                        radi = (float) (rcv_value / 1000.00); // convert to rad
                        NewPositionCommand(radi);
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

            } else { // Discarding 2 bytes
                i2c_value = I2CRCV; // store data into RAM    
                I2CCONbits.SCLREL = 1; //Release SCL1 line
                i2c_value = I2CRCV; // store data into RAM  
                Flag.AddrFlag = 0; //end of tx
                Flag.DataFlag = 0;
                I2CCONbits.SCLREL = 1; //Release SCL1 line
                secondByteFlag = FALSE;
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
            snd_value = AngularPositionCalculation();

            aux = (char) (snd_value >> 8);
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */


            aux = (char) snd_value;
            I2CTRN = aux; /* data transferred to I2CTRN reg */
            I2CCONbits.SCLREL = 1; /* Release the clock */
            while (I2CSTATbits.TBF); /* wait till the transmit buffer is clear */
            while (!IFS0bits.SI2CIF); /* Wait till the ACK from master is received */
            I2CCONbits.SCLREL = 1; //Release SCL1 line    
            while (I2CSTATbits.TBF); //Wait till all 


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
        }
    }

    _SI2CIF = 0; //clear I2C1 Slave interrupt flag  
}

/*****************************************************************
       External Interruption INT0 
 *****************************************************************/
void __attribute__((interrupt, auto_psv)) _INT0Interrupt(void) {


    if (DIRECTION == CCW) {
        counter_overflow = 0;
        POSCNT = 0;

    } else {
        counter_overflow = 1;
        POSCNT = 18065; //(MAX_COUNT - 65535); // TOTAL 83600 count
    }

    if (homing_flag) {
        PDC1 = ZERO_SPEED_PWM; //MOTOR 0 RPM
        homing_flag = 0;
    }

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

    /* Configure PTPER register to produce 5.33kHz PWM frequency */
    PTPER = 2947; /* PWM freq 5.33kHz */
    
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

    I2CADD = 0x11; // 7-bit I2C slave address must be initialised here. 
    IFS0 = 0;

    Flag.AddrFlag = 0; //Initlize AddFlag
    Flag.DataFlag = 0; //Initlize DataFlag


    ConfigIntI2C(SI2C_INT_PRI_2);

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

    ConfigIntQEI(QEI_INT_ENABLE & QEI_INT_PRI_7);


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
    int answer;

    quadrant_value = 4.9255 * counter_overflow;
    part_position = (POSCNT * 4.9255) / 65535;
    total_position = part_position + quadrant_value; // position rad. [0;2PI]

    if (total_position > PI)
        total_position -= (2 * PI); // position rad. [-PI,PI]

    answer = (int) (total_position * 1000); // position rad. *1000

    return answer;
}


/* New Position - Movement.  */
//  /* Speed conversion: rad/s to PWM value*/
// (pwm = ang_vel*59.3403+10.165;)
// (PDC1 = abs(pwm*28.892-109.19);)
// pwm_value = 2 * (STD_VEL * 1714.456) + 184.4972;  (OSC = 4MHz)
// pwm_value = STD_VEL * 3556 + 580;             

void NewPositionCommand(float rad_value) {

    unsigned long actual_position, desired_position;
    long delta_position;
    int situation = 0;
    
    LATDbits.LATD3 = 1;
    actual_position = POSCNT + (counter_overflow * 65535);

    // convert rad [-PI;PI] to rad [0;2PI]
    if (rad_value < 0) {
        rad_value = (rad_value) + (2 * PI);
    }
    // convert rad to counter values           
    desired_position = (unsigned long) (rad_value * 65535 / 4.9255);

    if ((actual_position <= (MAX_COUNT / 2)) && (desired_position <= (MAX_COUNT / 2)))
        situation = 1;

    else if ((actual_position > (MAX_COUNT / 2)) && (desired_position > (MAX_COUNT / 2)))
        situation = 1;

    else if ((actual_position <= (MAX_COUNT / 2)) && (desired_position > (MAX_COUNT / 2)))
        situation = 2;

    else if ((actual_position > (MAX_COUNT / 2)) && (desired_position <= (MAX_COUNT / 2)))
        situation = 3;

    while (situation > 0) {

        switch (situation) {

            case 1:                
                delta_position = (long) desired_position - actual_position;
                if (delta_position >= 0) { // angular velocity is positive
                    DIRECTION = CCW;
                    ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);
                } else { //angular velocity is Negative 
                    DIRECTION = CW;
                    ConfigINT0(FALLING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);
                }
                
                pwm_value = MAX_STD_VEL * 1078 + ZERO_SPEED_PWM;             
                PDC1 = (unsigned int) pwm_value;


                while ((fabs(delta_position)) > MAX_RAMP) { //0.376 rad = 21.5º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                }

                while ((fabs(delta_position)) > MIN_RAMP) { //0.1503 rad = 8.61º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
   
                    pwm_value = ((fabs(delta_position) * VEL_RAMP) - RAMP_COMP) * 1078  + ZERO_SPEED_PWM;                       
                    PDC1 = (unsigned int) pwm_value;
                }
                
                pwm_value = STD_VEL * 1078 + ZERO_SPEED_PWM;
                PDC1 = (unsigned int) pwm_value;

                while ((fabs(delta_position)) > DEAD_ZONE) { //0.0187 rad = 1.08º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                }

                situation = 0;
                break;

            case 2:
                delta_position = (long) (-(MAX_COUNT - desired_position + actual_position));

                DIRECTION = CW;
                ConfigINT0(FALLING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);
                
                pwm_value = MAX_STD_VEL * 1078 + ZERO_SPEED_PWM;    
                PDC1 = (unsigned int) pwm_value;

                while (actual_position <= (MAX_COUNT / 2)) { // semiplano diferente
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                }

                while ((fabs(delta_position)) > MAX_RAMP) {  //0.376 rad = 21.5º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                }


                while ((fabs(delta_position)) > MIN_RAMP) { //0.1503 rad = 8.61º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                    
                    pwm_value = ((fabs(delta_position) * VEL_RAMP) - RAMP_COMP) * 1078  + ZERO_SPEED_PWM;                       
                    PDC1 = (unsigned int) pwm_value;
                }
                
                pwm_value = STD_VEL * 1078 + ZERO_SPEED_PWM;              
                PDC1 = (unsigned int) pwm_value;
                
                while ((fabs(delta_position)) > DEAD_ZONE) { //0.0187 rad = 1.08º

                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;

                }


                situation = 0;                
                break;


            case 3:
                delta_position = (long) (MAX_COUNT - actual_position + desired_position);

                DIRECTION = CCW;
                ConfigINT0(RISING_EDGE_INT & EXT_INT_PRI_6 & EXT_INT_ENABLE);

                pwm_value = MAX_STD_VEL * 1078 + ZERO_SPEED_PWM;                
                PDC1 = (unsigned int) pwm_value;

                while (actual_position > (MAX_COUNT / 2)) {     //semiplano diferente
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                }

                while ((fabs(delta_position)) > MAX_RAMP) { //0.376 rad = 21.5º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                }


                while ((fabs(delta_position)) > MIN_RAMP) { //0.1503 rad = 8.61º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                    
                    pwm_value = ((fabs(delta_position) * VEL_RAMP) - RAMP_COMP) * 1078  + ZERO_SPEED_PWM;                       
                    PDC1 = (unsigned int) pwm_value;
                }

                pwm_value = STD_VEL * 1078 + ZERO_SPEED_PWM;
                PDC1 = (unsigned int) pwm_value;
                LATDbits.LATD3 = 1;

                while ((fabs(delta_position)) > DEAD_ZONE) { //0.0187 rad = 1.08º
                    actual_position = (unsigned long) (POSCNT + (counter_overflow * 65535));
                    delta_position = (long) desired_position - actual_position;
                }


                situation = 0;
                break;

            default:
                delta_position = 0;
                situation = 0;
        }

    } // fim ciclo while (situation < 0)
    LATDbits.LATD3 = 0;
    PDC1 = ZERO_SPEED_PWM;

    return;
}