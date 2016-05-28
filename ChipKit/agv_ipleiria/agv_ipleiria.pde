#include <plib.h> 
/*
  MEE -  AGV para ambiente interior
  Código para implementação do controlo de baixo nível do NOMAD200 recuperado.
 */

// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif





#include <ros.h>
#include <agv_ipleiria/AgvCommand.h>
#include <agv_ipleiria/AgvStatus.h>
#include <agv_ipleiria/RangeArray.h>
#include <agv_ipleiria/Agv_sensors.h>
#include <agv_ipleiria/Agv_main.h>
#include <Wire.h>        // standard library for I2C use.
// the board_defs.h file needs to be modified in order to configure the I2C2 Module




// #########################################################################################################
// comment/uncomment each line in order to activate respective function on the robot's low level
// 
//#define BUMPER
//#define SONAR
//#define INFRARED
//
// #########################################################################################################


#define READY_ASSERT_VALUE LOW     // Controllers READY output ASSERT - The controller input is configurable on the ESCON Studio Software (Maxon Motor)
#define ENABLE_ASSERT_VALUE HIGH     // Controllers ENABLE input ASSERT - The controller input is configurable on the ESCON Studio Software (Maxon Motor)
#define ENABLE_DEASSERT_VALUE LOW      // Controllers ENABLE input DEASSERT - The controller input is configurable on the ESCON Studio Software (Maxon Motor)
#define ZERO_SPEED_PWM 20            // PWM MINIMUM duty-cycle. 20 => 10% => ZERO SPEED


#define TURRET_ADDRESS 0x11
#define WHEELS_ADDRESS 0x10



// General I/O Pin assigning
// Pin definition is for the "Chipkit Interface board" V2.1 - July 2015
//
const short MOTORS_READY = 3;    // Motor Controllers Ready Signal
const short MOTORS_ENABLE = 73;  // Motor Controllers Enable output
const short ACTIVATE = 74;       // Activate OUTPUT - enables the power supply (Motor 24V)
const short BATTERY_LEVEL = A6;		// Battery Charge Level



// Motor 1 Controller Pin assigning
const short PWM_LIN = 10;        // Speed PWM output
const short DIR_LIN = 70;        // Direction pin selection
const short ActualCurrent1 = A0;    // Actual Current Analog Input (0V - 3.3V)
const short ActualSpeed1 = A1;    // Actual Speed Analog Input (0V - 3.3V)




#ifdef BUMPER
const short BUMPER_S0 = 75;       // Bumper LSB selection input
const short BUMPER_S1 = 8;       // Bumper selection input
const short BUMPER_S2 = 4;       // Bumper MSB selection input
const short BUMPER_Y = 2;       // Bumper Output (interrupt1)
volatile short bumper_index = 0;
#endif



#ifdef SONAR

// Sonar global variables
volatile float sonar_echo = 0; //echo tine value
volatile float sonar_trig = 0; //trig time value
volatile float sonar_distance[16]; // calculated distance
volatile short sonar_index = 0;
volatile bool sonar_updated = true;
volatile bool sonar_array_done = false;
const short sonar_sequence[16] = {0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15}; //defines scanning sequence

#define SONAR_MIN_RANGE 0.2       //# minimum range value [m]
#define SONAR_MAX_RANGE 6.5       //# maximum range value [m]


// Sonar board Pin assigning
const short SONAR_ADD0 = 84;   // ADD0 - Sensor Selection LSB
const short SONAR_ADD1 = 83;   // ADD1 - Sensor Selection
const short SONAR_ADD2 = 80; // ADD2 - Sensor Selection
const short SONAR_ADD3 = 79; // ADD3 - Sensor Selection MSB
const short SONAR_INIT = 78;  // 82; // INIT
const short SONAR_ENABLE = 9; // ENABLE
const short SONAR_ECHO = 20; // ECHO (interrupt4)
#endif

#ifdef INFRARED
// Infrared global variables
volatile bool infrared_updated = true;
volatile bool infrared_cal = true;      //false turns IR LEDs ON.
volatile bool infrared_array_done = true;
volatile long infrared_trig = 0; //trig time value
volatile long infrared_done = 0;
volatile long infrared_count = 0;
volatile short overflow_count = 0;
volatile short infrared_index = 0;
volatile float infrared_distance[16]; // calculated distance
const short infrared_sequence[16] = {0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15}; //defines scanning sequence

#define IR_MIN_RANGE 0.04       //# minimum range value [m]
#define IR_MAX_RANGE 0.60       //# maximum range value [m]


// Infrared board Pin assigning
const short IR_ADD1 = 38; // ADD1 - Sensor Selection LSB
const short IR_ADD2 = 39; // ADD2 - Sensor Selection
const short IR_ADD3 = 40; // ADD3 - Sensor Selection
const short IR_ADD4 = 41; // ADD4 - Sensor Selection MSB
const short IR_RESA = 42; // CONN A on the board. Resolution selection LSB
const short IR_RESB = 43; // CONN B on the board. Resolution selection
const short IR_RESC = 44; // CONN C on the board. Resolution selection MSB
const short IR_TRIG = 45; // TRIG - triggers the aquisition of selected sensor
const short IR_DONE = 7; // (interrupt2) OUTCC Pin1 on the board.
const short IR_OUT1 = 37; // Conversion output(7 bit) LSB
const short IR_OUT2 = 36; // Conversion output(7 bit)
const short IR_OUT3 = 35; // Conversion output(7 bit)
const short IR_OUT4 = 34; // Conversion output(7 bit)
const short IR_OUT5 = 33; // Conversion output(7 bit)
const short IR_OUT6 = 32; // Conversion output(7 bit)
const short IR_OUT7 = 21; // (interrupt3) Conversion output(7 bit) MSB
const short IR_CAL = 31; // CAL is used to set ON the Emitting Diodes for calibration/compensation purposes
#endif





// CONTROL GLOBAL VARIABLES
bool  power_on = false;
bool  motor_enable_state = false;
bool  collision_detected = false;
bool  linear_velocity = false;
short user_defined_update_rate = 50; // Period in [ms]
short pwm_value = ZERO_SPEED_PWM;
short batt_count = 0;  // auxiliar variable for battery voltage management

float desired_turret_pose = 0;
float desired_lin_vel = 0;
float desired_ang_vel = 0;


float true_lin_vel = 0;
float true_turret_pose = 0;
float true_wheel_pose = 0;
float true_ang_vel = 0;

//time related variables
unsigned long now = 0;
unsigned long dt = 0;
unsigned long dt_calc = 0;
unsigned long last = millis();
unsigned long last_calc = millis();

// position related variables
float x = 0;
float y = 0;
float x_position = 0;
float y_position = 0;
float last_x = 0;
float last_y = 0;
float last_lin_vel = 0;
short startup = 0;




// SENSORS CONFIG VARIABLES
bool bumper_on = true;
bool sonar_on = true;
bool infrared_on = true;
short infrared_resolution = 3; // resolution is between 0 (000) and 7 (111), from lowest to highest resolution.
bool sonar_state[16] = {true, true, true, true, true, true, true, true, false, true, true,  true, true, true, true, true}; // all sonars are active     false, false, false, false, false, false, false, false, false, false, true, false, false, false, true};
bool infrared_state[16] = {true, true, true, true, true, true, true, true, true, true, true,  true, true, true, true, true}; // all infrareds are active



// ################################################################
// #################    ROS SERIAL RELATED CODE   #################
// ################################################################

/// Setup topics subscribers and Service
ros::NodeHandle  nh;
using agv_ipleiria::Agv_sensors;
using agv_ipleiria::Agv_main;
agv_ipleiria::AgvStatus estado;
ros::Publisher Status("/AGVipleiria/Status", &estado);
agv_ipleiria::RangeArray sonar_range;
ros::Publisher Sonar_Ranges("/AGVipleiria/Sonar", &sonar_range);
agv_ipleiria::RangeArray infrared_range;
ros::Publisher Infrared_Ranges("/AGVipleiria/Infrared", &infrared_range);
agv_ipleiria::RangeArray bumper_range;
ros::Publisher Bumper_Ranges("/AGVipleiria/Bumper", &bumper_range);


/* AgvCommand topic Callback function */
// ################################################################
void commandmessage(const agv_ipleiria::AgvCommand& msg){

    nh.logdebug("commandmessage Callback");

    desired_turret_pose = (float) msg.turret_angle;
    desired_lin_vel = (float) msg.linear;
    desired_ang_vel = (float) msg.angular;

    Turret_Rotation(desired_turret_pose);
    Wheel_Rotation(desired_ang_vel);
    speedCommand(desired_lin_vel);
}
// ################################################################


/* Main Service Callback function */
// ################################################################

void MainCallback(const Agv_main::Request & request, Agv_main::Response & response)
{
    nh.logdebug("Main Service Callback");



    switch(request.Motor_power){

    case 0:
        digitalWrite(ACTIVATE, request.Motor_power);
        power_on = digitalRead(ACTIVATE);
        break;

    case 1:
        digitalWrite(ACTIVATE, request.Motor_power);
        power_on = digitalRead(ACTIVATE);
#ifdef BUMPER
        Bumper_Init();
#endif

        break;


    case 2:
        break;


    default:
        return;
    }

    
    response.Motor_power = power_on;



    switch(request.Motor_enable){

    case 0:
        motor_enable_state = Disable_Controllers();
        break;

    case 1:
        motor_enable_state = Enable_Controllers();
        break;

    case 2:
        break;


    default:
        return;
    }


    response.Motor_enable = motor_enable_state;




    if (request.Update_freq <1)
        user_defined_update_rate = 50;
    else
        user_defined_update_rate = 1000/request.Update_freq; //convert [Hz] to [ms]
    
    response.Update_freq = 1000/user_defined_update_rate;


    switch(request.Homing){


    case 1:

        nh.logdebug("HOMING Service Callback");

        speedCommand(0);
        Turret_Stop();
        Wheel_Homing();
        response.Homing = 1;
        break;

    case 2:
        nh.logdebug("HOMING Service Callback");
        speedCommand(0);
        Wheel_Stop();
        Turret_Homing();
        response.Homing = 2;
        break;

    default:
        response.Homing = 0;

    }


}
// ################################################################


/* Sensors Service Callback function */
// ################################################################

void SensorsCallback(const Agv_sensors::Request & request, Agv_sensors::Response & response)
{
    nh.logdebug("Sensors Service Callback");

    short aux;
    short i;

    aux = request.radiation_type;

    switch(aux)
    {


#ifdef SONAR
    case 0:  // SONAR


        switch(request.radiation_state)
        {

        case 0:
            sonar_on = request.radiation_state;
            break;

        case 1:
            sonar_on = request.radiation_state;

            for(i=0;i<16;i++)
            {
                sonar_state[i] = request.ranges[i];
            }


            for(i=0;i<16;i++)
            {
                response.ranges[i] = sonar_state[i];
            }
            break;

        case 2:
            for(i=0;i<16;i++)
            {response.ranges[i] = sonar_state[i];}
            break;

        default:
            return;

        }

        response.radiation_type = 2;
        response.radiation_state = sonar_on;


        break;
#endif        


#ifdef INFRARED
    case 1:  // INFRARED


        switch(request.radiation_state)
        {

        case 0:
            infrared_on = request.radiation_state;
            break;

        case 1:
            infrared_on = request.radiation_state;
            infrared_resolution = request.resolution;
            infrared_resolution = clipValue(infrared_resolution,0,7);
            

            for(i=0;i<16;i++)
            {
                infrared_state[i] = request.ranges[i];
            }


            response.resolution = infrared_resolution;

            for(i=0;i<16;i++)
            {
                response.ranges[i] = infrared_state[i];
            }
            break;

        case 2:
            for(i=0;i<16;i++)
            {response.ranges[i] = infrared_state[i];}
            break;

        default:
            return;
        }

        digitalWrite(IR_RESC, infrared_resolution & 0x01);
        digitalWrite(IR_RESB, infrared_resolution & 0x02);
        digitalWrite(IR_RESA, infrared_resolution & 0x04);


        response.radiation_type = 1;
        response.radiation_state = infrared_on;


        break;

#endif


#ifdef BUMPER      
    case 2:  // BUMPER

        switch(request.radiation_state)
        {

        case 0:

        case 1:
            bumper_on = request.radiation_state;
            break;

        case 2:
            break;
        default:
            return;

        }


        response.radiation_type = 0;
        response.radiation_state = bumper_on;
        break;
#endif

        
    }


}
// ################################################################

ros::ServiceServer<Agv_sensors::Request, Agv_sensors::Response> sensors_server("/AGVipleiria/Agv_sensors", &SensorsCallback );
ros::ServiceServer<Agv_main::Request, Agv_main::Response> main_server("/AGVipleiria/Agv_main", &MainCallback );
ros::Subscriber<agv_ipleiria::AgvCommand> command_sub("/AGVipleiria/Command", &commandmessage );

// ################################################################
// ################################################################




// ################################################################
// #################         SETUP FUNCTION       #################
// ################################################################

void setup() {

    Wire.begin();


    //ROS related Initialization code
    nh.initNode();
    nh.subscribe(command_sub);
    nh.advertise(Status);
    

#ifdef SONAR
    nh.advertise(Sonar_Ranges);

    // set transducer values
    sonar_range.radiation_type = 0;
    sonar_range.field_of_view = 5;
    sonar_range.min_range = 0.2;       //# minimum range value [m]
    sonar_range.max_range = 6.5;       //# maximum range value [m]
    sonar_range.ir_res = 0;  	//	# ir resolution value

#endif

#ifdef INFRARED
    nh.advertise(Infrared_Ranges);
    
    // set transducer values
    infrared_range.radiation_type = 0;
    infrared_range.field_of_view = 5;
    infrared_range.min_range = 0.04;                     //# minimum range value [m]
    infrared_range.max_range = 0.60;                     //# maximum range value [m]
    infrared_range.ir_res = infrared_resolution;  	 //# ir resolution value

    Infrared_Ranges.publish( &infrared_range );

#endif

#ifdef BUMPER 
    nh.advertise(Bumper_Ranges);
#endif 


    nh.advertiseService(sensors_server);
    nh.advertiseService(main_server);

    while(!nh.connected()) nh.spinOnce();
    nh.logdebug("Startup complete");

    nh.loginfo("\t");
    nh.loginfo("\t\t IPLEIRIA  - ESTG (c) 2015");
    nh.loginfo("\t\t AGV para ambiente interior");
    nh.loginfo("\t\t http://www.ipleiria.pt");
    nh.loginfo("\t");



    // Defining PinModes
    /** Defining pinModes for all the signals**/

    pinMode(MOTORS_ENABLE,OUTPUT);    	// Enable OUTPUT - enables the controllers (Motor 24V)
    pinMode(ACTIVATE,OUTPUT);    		// Activate OUTPUT - enables the power supply (Motor 24V)
    pinMode(MOTORS_READY,INPUT);		// Ready INTPUT - info from controllers (Motor 24V)

    pinMode(PWM_LIN,OUTPUT);
    pinMode(DIR_LIN,OUTPUT);
    pinMode(ActualCurrent1,INPUT);
    pinMode(ActualSpeed1,INPUT);



#ifdef BUMPER
    /** Defining pinModes for all the signals**/
    // Bumper board
    pinMode(BUMPER_S0,OUTPUT);      // Bumper LSB selection input
    pinMode(BUMPER_S1,OUTPUT);      // Bumper selection input
    pinMode(BUMPER_S2,OUTPUT);      // Bumper MSB selection input
    pinMode(BUMPER_Y,INPUT);      // Bumper output


    /*put all Bumper outputs to high level - "Listen Mode" */
    digitalWrite(BUMPER_S0, HIGH);
    digitalWrite(BUMPER_S1, HIGH);
    digitalWrite(BUMPER_S2, HIGH);

    nh.logdebug("Bumper initialized");

#endif



#ifdef SONAR
    /** Defining pinModes for all the signals**/
    // Sonar board
    pinMode(SONAR_ADD0, OUTPUT);  // ADD0
    pinMode(SONAR_ADD1, OUTPUT);  // ADD1
    pinMode(SONAR_ADD2, OUTPUT);  // ADD2
    pinMode(SONAR_ADD3, OUTPUT);  // ADD3
    pinMode(SONAR_INIT, OUTPUT);  // INIT
    pinMode(SONAR_ENABLE, OUTPUT);  // ENABLE
    pinMode(SONAR_ECHO, INPUT);  // ECHO

    /*put all Sonar outputs to low level*/
    digitalWrite(SONAR_ENABLE, LOW);
    digitalWrite(SONAR_INIT, LOW);
    digitalWrite(SONAR_ADD0, LOW);
    digitalWrite(SONAR_ADD1, LOW);
    digitalWrite(SONAR_ADD2, LOW);
    digitalWrite(SONAR_ADD3, LOW);


    nh.logdebug("Sonar initialized");


#endif


#ifdef INFRARED
    // Infrared board
    pinMode(IR_ADD1, OUTPUT);  // ADD1 - Sensor Selection LSB
    pinMode(IR_ADD2, OUTPUT);  // ADD2 - Sensor Selection
    pinMode(IR_ADD3, OUTPUT);  // ADD3 - Sensor Selection
    pinMode(IR_ADD4, OUTPUT);  // ADD4 - Sensor Selection MSB
    pinMode(IR_RESA,OUTPUT);  // CONN A on the board. Resolution selection MSB
    pinMode(IR_RESB,OUTPUT);  // CONN B on the board. Resolution selection
    pinMode(IR_RESC,OUTPUT);  // CONN C on the board. Resolution selection LSB
    pinMode(IR_TRIG, OUTPUT);  // TRIG - triggers the aquisition of selected sensor
    pinMode(IR_CAL,OUTPUT);    // CAL is used to set ON the Emitting Diodes for calibration/compensation purposes
    pinMode(IR_DONE,INPUT);    // (interrupt) OUTCC Pin1 on the board.
    pinMode(IR_OUT1,INPUT);    // Conversion output(7 bit) LSB
    pinMode(IR_OUT2,INPUT);    // Conversion output(7 bit)
    pinMode(IR_OUT3,INPUT);    // Conversion output(7 bit)
    pinMode(IR_OUT4,INPUT);    // Conversion output(7 bit)
    pinMode(IR_OUT5,INPUT);    // Conversion output(7 bit)
    pinMode(IR_OUT6,INPUT);    // Conversion output(7 bit)
    pinMode(IR_OUT7,INPUT);    // (interrupt) Conversion output(7 bit) MSB



    /*put all infrared outputs to low level*/
    digitalWrite(IR_ADD1, LOW);
    digitalWrite(IR_ADD2, LOW);
    digitalWrite(IR_ADD3, LOW);
    digitalWrite(IR_ADD4, LOW);
    digitalWrite(IR_RESA, LOW);
    digitalWrite(IR_RESB, LOW);
    digitalWrite(IR_RESC, LOW);
    digitalWrite(IR_TRIG, LOW);
    digitalWrite(IR_CAL, LOW);


    digitalWrite(IR_RESC, infrared_resolution & 0x01);
    digitalWrite(IR_RESB, infrared_resolution & 0x02);
    digitalWrite(IR_RESA, infrared_resolution & 0x04);

    nh.logdebug("Infra-red initialized");
#endif




    // Initialize Motor1 PWM
    digitalWrite(DIR_LIN,HIGH);
    analogWrite(PWM_LIN, ZERO_SPEED_PWM);    // set PWM duty-cycle. 20-255 => 10%-100%
    digitalWrite(ACTIVATE, LOW);
    delay(500);


#ifdef BUMPER
    Bumper_Init();
#endif

}



// ################################################################
// #################          LOOP FUNCTION       #################
// ################################################################



void loop()
{

#ifdef SONAR

    if( sonar_updated == true )
        Sonar_Init();
#endif

#ifdef INFRARED

    if( infrared_updated == true )
        Infrared_Init();
#endif




    now = millis();
    dt = now-last;
    dt_calc = now-last_calc;


    // calculating and updating pose
    if ( (dt_calc >= 100) && linear_velocity) //(motor_enable_state)  ) // 100 Hz atualization
    {
        
        true_lin_vel = speedCalc();  // get true_lin_vel
        readbus(WHEELS_ADDRESS);    //get true_wheel_pose

        x = (float)((((true_lin_vel+last_lin_vel)/2)*dt_calc/1000)*cos(true_wheel_pose));  // x -displacement
        x_position += x;
        
        y = (float)((((true_lin_vel+last_lin_vel)/2)*dt_calc/1000)*sin(true_wheel_pose));  // y -displacement
        y_position += y;

        last_lin_vel = true_lin_vel;
        last_calc = now; // millis();

        
        if(startup <= 50) //avoiding initialization errors
        {
            x_position = 0;
            y_position = 0;
            last_lin_vel = 0;
            true_lin_vel = 0;
            startup++;
        }

        estado.pose_x = x_position;
        estado.pose_y = y_position;
        
    }




    // publishing ROS messages
    if(dt >= 100) //50ms = 20Hz
    {
        last = now;
        estado.Motor_power = power_on;
        estado.Motor_ready =  digitalRead(MOTORS_READY);
        estado.Battery = Battery_Check();
        estado.Collision = collision_detected;

        readbus(WHEELS_ADDRESS);
        readbus(TURRET_ADDRESS);

        estado.angular = true_ang_vel;
        estado.base_angle = -true_wheel_pose;
        estado.turret_angle = true_turret_pose;
        estado.linear = speedCalc();

        Status.publish( &estado );


#ifdef SONAR
        if(sonar_array_done == true)
        {

            digitalWrite(DIR_LIN,HIGH);
            Sonar_Ranges.publish( &sonar_range );
            digitalWrite(DIR_LIN,LOW);
            nh.logdebug("sonar publish");
            sonar_array_done = false;
        }
#endif


#ifdef INFRARED
        if(infrared_array_done == true)
        {
            Infrared_Ranges.publish( &infrared_range );
            nh.logdebug("infrared publish");
            infrared_array_done = false;
        }
#endif

    }


    nh.spinOnce();

    if(!nh.connected())
        connection_lost();
}
// ################################################################




/* ROS connection  lost - stops the robot and waits new connection with high-level control */
// ################################################################

void connection_lost()
{
    speedCommand(0);
    Wheel_Stop();
    Turret_Stop();
    Disable_Controllers();
    while(!nh.connected()) nh.spinOnce();
}
// ################################################################





/* ClipValue - limits the value (16bit) in a desired range*/
// ################################################################

short clipValue(short value, short min, short max) {
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else return value;
}
// ################################################################




/* power battery check - convert analog read to voltage value */
// ################################################################
float Battery_Check()
{
    short batt_level = 0;
    


    batt_level = analogRead(BATTERY_LEVEL);    // Reading the battery level
    float voltage = 0;
    voltage = (float) (batt_level*25/1023.0);

    if(voltage < 18)
        batt_count ++;

    if (batt_count == 1)
        nh.logwarn(" === BATTERY LOW ===> You may want to charge batteries!! ");

    if (batt_count == 100)
        batt_count = 0;



    return voltage;
}
// ################################################################



/* Enable Motor Controllers - makes max 3 attempts */
// ################################################################
short Enable_Controllers()  // returns 1 on success. if not returns 0
{
    short aux = 0;
    nh.logdebug(" === Controllers enable func === ");

    digitalWrite(MOTORS_ENABLE, ENABLE_DEASSERT_VALUE); // for the Controllers boot, better put on disable state
    delay(500);
    digitalWrite(MOTORS_ENABLE, ENABLE_ASSERT_VALUE);
    delay(1000);              // wait
    aux = digitalRead(MOTORS_READY);
    delay(50);

    nh.logdebug(" === Controllers enable done === ");
    
    // The following code is commented until the 'ready' signal from controllers is working properly.
    // Check logic circuit on the 'new slipring board'
    
    
    //    if(aux == READY_ASSERT_VALUE)
    //            return 1;
    //
    //    else
    //    {
    //        nh.logwarn(" === FAIL: Controllers enable attempt #1 === ");
    //        digitalWrite(MOTORS_ENABLE, ENABLE_DEASSERT_VALUE); // for the Controllers boot, better put on disable state
    //        delay(500);
    //        digitalWrite(MOTORS_ENABLE, ENABLE_ASSERT_VALUE);
    //        delay(1000);              // wait
    //        aux = digitalRead(MOTORS_READY);
    //        delay(50);
    //    }
    //
    //    if(aux == READY_ASSERT_VALUE)
    //        return 1;
    //    else
    //    {
    //        nh.logwarn(" === FAIL: Controllers enable attempt #2 === ");
    //        digitalWrite(MOTORS_ENABLE, ENABLE_DEASSERT_VALUE); // for the Controllers boot, better put on disable state
    //        delay(50);
    //        digitalWrite(MOTORS_ENABLE, ENABLE_ASSERT_VALUE);
    //        delay(50);              // wait
    //        aux = digitalRead(MOTORS_READY);
    //        delay(100);
    //    }
    //
    //    if(aux == READY_ASSERT_VALUE)
    //        return 1;
    //    else
    //    {
    //
    //        nh.logwarn(" === FAIL: Controllers enable attempt #3 === ");
    //        return 0;
    //    }



}
// ################################################################




/* Disaable Motor Controllers -  */
// ################################################################
short Disable_Controllers()
{
    nh.logdebug(" === Controllers disable func === ");
    digitalWrite(MOTORS_ENABLE, ENABLE_DEASSERT_VALUE); // for the Controllers boot, better put on disable state
    linear_velocity = false;
    return 0;
}



/* Linear Velocity command - receives lin_vel [m/s]*/
// ################################################################
void speedCommand(float cmd_lin_vel) {


    
    if(fabs(cmd_lin_vel) < 0.03)
    {
        analogWrite(PWM_LIN, ZERO_SPEED_PWM);
        linear_velocity = false;
    }
    
    else {
        if (cmd_lin_vel > 0) {
            digitalWrite(DIR_LIN, HIGH);
            nh.logdebug(". Forward .");
        }
        if (cmd_lin_vel < 0) {
            digitalWrite(DIR_LIN, LOW);
            cmd_lin_vel = abs(cmd_lin_vel);
            nh.logdebug(". Backward .");
        }

        float rpm_vel = cmd_lin_vel * 153.28 * 31.8;
        pwm_value = (rpm_vel * 0.0667) + ZERO_SPEED_PWM;
        pwm_value = clipValue(pwm_value, ZERO_SPEED_PWM, 250);
        analogWrite(PWM_LIN, pwm_value);
        linear_velocity = true;
    }
}
// ################################################################


/* Linear Speed Calculation - returns lin_vel [m/s]*/
// ################################################################
float speedCalc(void) {
    
    short total = 0;
    short average = 0; // the average
    short i;
    
    
    // read from the sensor:
    if(motor_enable_state)
    {
        nh.logdebug("analogread x10");

        for(i=0;i<10;i++)
        {
            total +=  analogRead(ActualSpeed1);
        }

        // calculate the average:
        average = total / 10;

        float vel = (float) ((6000.0 / 1014 * average) - 3000);  // range is 2x MAX_MOTOR_SPEED = 2X 3000 rpm.

        if (abs(vel) < 48) //rpm value - lower than this system will not move
            vel = 0;

        else
            vel = vel / (31.8 * 153.28); // Convert Motor RPM to Robot m/s


        return -vel; //return vel[m/s]


    }

    return 0;
}
// ################################################################


/* Wheel Rotation command - receives ang_vel [rad/s]*/
// ################################################################
void Wheel_Rotation(float desired_ang_vel) {

    if(fabs(desired_ang_vel) < 0.01)
        desired_ang_vel = 0;


    short aux = (short)(desired_ang_vel*100);
    aux = clipValue(aux,-120,120);

    nh.logdebug(". Wheels Rotation .");

    Wire.beginTransmission(WHEELS_ADDRESS); // transmit to device
    Wire.write('v');
    Wire.write(aux);
    Wire.endTransmission(); // stop transmitting
}
// ################################################################


/* Wheel STOP command - */
// ################################################################
void Wheel_Stop() {

    nh.logdebug(". Wheels Stop .");
    Wire.beginTransmission(WHEELS_ADDRESS); // transmit to device
    Wire.write('s');
    Wire.write(1);
    Wire.endTransmission(); // stop transmitting
}
// ################################################################


/* Wheel Homing command -*/
// ################################################################
void Wheel_Homing() {

    nh.logdebug(". Wheels Homing .");
    Wire.beginTransmission(WHEELS_ADDRESS); // transmit to device
    Wire.write('h');
    Wire.write(1);
    Wire.endTransmission(); // stop transmitting
}
// ################################################################



/* Turret Rotation command - receives pose [rad]*/
// ################################################################
void Turret_Rotation(float desired_turret_pose) {

    short value;
    short aux = (short) (desired_turret_pose * 1000);
    aux = clipValue(aux,-3142,3142);

    nh.logdebug(". Turret rotation .");
    

    Wire.beginTransmission(TURRET_ADDRESS); // transmit to device
    Wire.write('k'); // sends byte
    value = (char) (aux >> 8);
    Wire.write(value); // sends byte
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(TURRET_ADDRESS); // transmit to device
    Wire.write('v'); // sends byte
    value = (unsigned char) (aux);
    Wire.write(value); // sends byte
    Wire.endTransmission(); // stop transmitting
}
// ################################################################



/* Turret STOP command - */
// ################################################################
void Turret_Stop() {

    nh.logdebug(". Turret Stop .");
    Wire.beginTransmission(TURRET_ADDRESS); // transmit to device
    Wire.write('s');
    Wire.write(1);
    Wire.endTransmission(); // stop transmitting
}
// ################################################################



/* Turret Homing command -*/
// ################################################################
void Turret_Homing() {

    nh.logdebug(". Turret Homing .");
    Wire.beginTransmission(TURRET_ADDRESS); // transmit to device
    Wire.write('h');
    Wire.write(1);
    Wire.endTransmission(); // stop transmitting
}
// ################################################################




/* Read I2C bus command - returns true on success, false otherwise*/
// ################################################################

bool readbus(short Address) {

    short ii = 0;
    short numBytes;
    short aux1;
    short aux2;
    nh.logdebug("I2C bus read");


    switch(Address)
    {
    case TURRET_ADDRESS:

        numBytes = 3;
        break;

    case WHEELS_ADDRESS:

        numBytes = 5;
        break;

    default:

        return false; // returns 0 when not successfull read
    }


    Wire.requestFrom(Address, numBytes); // request n bytes from slave device

    while (Wire.available()) // slave may send less than requested
    {

        if (ii == 0) {
            char c = Wire.receive(); // receive a byte as character
            ii++;
        }
        else {

            int n = Wire.receive(); // receive a byte as character

            if (ii < numBytes) {

                
                if (ii == 1) {     // most significant byte of the orientation 16bit integer (short)
                    aux1 = (n << 8);
                }
                if (ii == 2) {    // less significant byte of the orientation 16bit integer (short)
                    aux1 = aux1 | n;
                }


                if (ii == 3) {     // most significant byte of the angular speed 16bit integer (short)
                    aux2 = (n << 8);

                }
                if (ii == 4) {    // less significant byte of the angular speed 16bit integer (short)
                    aux2 = aux2 | n;
                }

                ii++;

            }
        }
    }
    
    
    
    if (ii == numBytes) {

        switch(Address)
        {

        case TURRET_ADDRESS:

            true_turret_pose = (float) (aux1 / 1000.00);
            break;
            
            
        case WHEELS_ADDRESS:

            true_wheel_pose = (float) (aux1 / 1000.00);
            true_ang_vel = (float)(aux2 /1000.00);
            break;
        }

        ii = 0;

        return true; // returns 1 when successfull read
    }
    return false; // returns 0 when no read
}
// ################################################################





#ifdef BUMPER

/* Init the bumper interruption and send ROS initial message -*/
// ################################################################
void Bumper_Init()
{ 
    /*put all Bumper outputs to high level - "Listen Mode" */
    digitalWrite(BUMPER_S0, HIGH);
    digitalWrite(BUMPER_S1, HIGH);
    digitalWrite(BUMPER_S2, HIGH);
    
    bumper_range.radiation_type = 0;
    bumper_range.ranges[0] = 99;
    collision_detected = false;

    Bumper_Ranges.publish( &bumper_range );
    nh.logdebug("bumper publish");
    nh.spinOnce();


    attachInterrupt(1, BumperPressed, RISING);

}

// ################################################################



/* interrupt handle on the bumper event -*/
// ################################################################
void BumperPressed()
{


    short sum = 99;  // stores the sensor # detected



    nh.logdebug(". Bumper Pressed .");

    Disable_Controllers();


    for(bumper_index = 0; bumper_index <= 4; bumper_index ++)
    {
        digitalWrite(BUMPER_S0, bitRead(bumper_index,0));
        digitalWrite(BUMPER_S1, bitRead(bumper_index,1));
        digitalWrite(BUMPER_S2, bitRead(bumper_index,2));

        delay(10);
        if(digitalRead(BUMPER_Y))
            sum += pow(2,bumper_index);
    }

    sum -= 99;
    bumper_range.ranges[1] = sum*1.0;

    switch(sum)
    {
    case 17:   // 0º
        bumper_range.ranges[0] = 0.0;
        break;

    case 18:   // 18º
        bumper_range.ranges[0] = 0.314;
        break;

    case 13:   // 36º
        bumper_range.ranges[0] = 0.628;
        break;

    case 14:   // 54º
        bumper_range.ranges[0] = 0.942;
        break;

    case 9:   // 72º
        bumper_range.ranges[0] = 1.256;
        break;

    case 10:   // 90º
        bumper_range.ranges[0] = 1.57;
        break;


    case 5:   // 108º
        bumper_range.ranges[0] = 1.884;
        break;

    case 6:   // 126º
        bumper_range.ranges[0] = 2.198;
        break;

    case 1:   // 144º
        bumper_range.ranges[0] = 2.512;
        break;

    case 2:   // 162º
        bumper_range.ranges[0] = 2.826;
        break;

    case 0:   // 180º
        bumper_range.ranges[0] = 3.14;
        break;


    case 19:   // -18º
        bumper_range.ranges[0] = -0.314;
        break;

    case 16:   // -36º
        bumper_range.ranges[0] = -0.628;
        break;

    case 15:   // -54º
        bumper_range.ranges[0] = -0.942;
        break;

    case 12:   // -72º
        bumper_range.ranges[0] = -1.256;
        break;

    case 11:   // -90º
        bumper_range.ranges[0] = -1.57;
        break;


    case 8:   // -108º
        bumper_range.ranges[0] = -1.884;
        break;

    case 7:   // -126º
        bumper_range.ranges[0] = -2.198;
        break;

    case 4:   // -144º
        bumper_range.ranges[0] = -2.512;
        break;

    case 3:   // 162º
        bumper_range.ranges[0] = -2.826;
        break;

    default:
        bumper_range.ranges[0] = 999;
    }

    collision_detected = true;
    bumper_range.radiation_type = 0;




    Bumper_Ranges.publish( &bumper_range );
    nh.logdebug(" == Bumper updated");


    /*put all Bumper outputs to high level - "Listen Mode" */
    digitalWrite(BUMPER_S0, HIGH);
    digitalWrite(BUMPER_S1, HIGH);
    digitalWrite(BUMPER_S2, HIGH);
    delay(100);

}

#endif
// ################################################################





#ifdef INFRARED
/* initialize the infrared board for a sensor reading -*/
// ################################################################
void Infrared_Init()
{

    nh.logdebug("IR Trig");

    digitalWrite(IR_CAL, infrared_cal);

    infrared_updated = false;

    if (infrared_state[ (infrared_sequence[infrared_index]) ] == true)
    {
        digitalWrite(IR_ADD1, infrared_sequence[infrared_index] & 0x01);
        digitalWrite(IR_ADD2, infrared_sequence[infrared_index] & 0x02);
        digitalWrite(IR_ADD3, infrared_sequence[infrared_index] & 0x04);
        digitalWrite(IR_ADD4, infrared_sequence[infrared_index] & 0x08);

        delay(50);
        infrared_trig = micros();

        digitalWrite(IR_TRIG,HIGH);

        attachInterrupt(3, Infrared_overflow, RISING);
        attachInterrupt(2, Infrared_done, FALLING);
    }

    else
    {
        infrared_distance[infrared_sequence[infrared_index]] = 0;
        infrared_updated = true;
        infrared_index++;
        
        if(infrared_index==16)
        {
            infrared_cal = !infrared_cal;
            infrared_index=0;
            infrared_array_done = true;
        }
    }
}
// ################################################################


/* interrupt handle on infrared overflow -*/
// ################################################################
void Infrared_overflow()
{
    overflow_count ++;
}
// ################################################################


/* interrupt handle on end of infrared count -*/
// ################################################################
void Infrared_done()
{
    float distance = 0;
    infrared_count = 0;

    //  infrared_done = micros()-infrared_trig;

    infrared_count = infrared_count + (digitalRead(IR_OUT1)  * 0x01);
    infrared_count = infrared_count + (digitalRead(IR_OUT2)  * 0x02);
    infrared_count = infrared_count + (digitalRead(IR_OUT3)  * 0x04);
    infrared_count = infrared_count + (digitalRead(IR_OUT4)  * 0x08);
    infrared_count = infrared_count + (digitalRead(IR_OUT5)  * 0x10);
    infrared_count = infrared_count + (digitalRead(IR_OUT6)  * 0x20);
    infrared_count = infrared_count + (digitalRead(IR_OUT7)  * 0x40);

    digitalWrite(IR_TRIG,LOW);
    infrared_count = infrared_count + (127*overflow_count);
    overflow_count = 0;
    
    switch (infrared_resolution){   // values ar calculated in [cm]

    case 0:
        //  y = -42,167x+362,7
        distance = (infrared_count - 362.7)/(-42.167);
        break;
    case 1:
        //  y = -65,567x+616,1
        distance = (infrared_count - 616.1)/(-65.567);
        break;

    case 2:
        //  y = -217,3x+2067,7
        distance = (infrared_count - 2067.7)/(-217.3);
        break;
    case 3:
        //  y = -151,43x+1750,4
        distance = (infrared_count - 1750.4)/(151.43);
        break;

    case 4:
        //  y = -420,7x+5911,2
        distance = (infrared_count - 5911.2)/(-420.7);
        break;
    case 5:
        //  y = -375,37x + 5774,8
        distance = (infrared_count - 5774.8)/(-375.37);
        break;

    case 6:
        //  y = -708,4x + 10987
        distance = (infrared_count - 10987)/(-708.4);
        break;
    case 7:
        //  y = -3018,3x + 28436
        distance = (infrared_count - 28436)/(-3018.3);
        break;

    }


    // convert [cm] -> [m]
    distance = distance * 0.01;

    if(distance < IR_MIN_RANGE)        //# minimum range value [m]
        distance = IR_MIN_RANGE;

    if(distance > IR_MAX_RANGE)        //# maximum range value [m]
        distance = IR_MAX_RANGE;

    infrared_distance[infrared_sequence[infrared_index]] = distance;  //infrared_count;

    infrared_range.radiation_type = 1;
    infrared_range.ranges[infrared_sequence[infrared_index]] = infrared_distance[infrared_sequence[infrared_index]];

    infrared_updated = true;
    infrared_index++;
    if(infrared_index==16)

    {
        // infrared_cal = !infrared_cal;  // calibration pin. LOW level asserted. We'll leave it turned off = HIGH. To implement on future work.
        infrared_index=0;
        infrared_array_done = true;
    }

    detachInterrupt(2);
    detachInterrupt(3);
    nh.logdebug(" IR detach");
}

#endif
// ################################################################







#ifdef SONAR
/* triggering the sonar board for a sensor reading -*/
// ################################################################
void Sonar_Init()
{
    nh.logdebug(" == Sonar Init");

    sonar_updated = false;

    if (sonar_state[ (sonar_sequence[sonar_index]) ] == true)
    {
        digitalWrite(SONAR_ADD0, sonar_sequence[sonar_index] & 0x01);
        digitalWrite(SONAR_ADD1, sonar_sequence[sonar_index] & 0x02);
        digitalWrite(SONAR_ADD2, sonar_sequence[sonar_index] & 0x04);
        digitalWrite(SONAR_ADD3, sonar_sequence[sonar_index] & 0x08);

        //digitalWrite(SONAR_ENABLE, HIGH);
        sonar_trig = micros();
        digitalWrite(SONAR_INIT, HIGH);
        attachInterrupt(4, Sonar_Echo, RISING);
    }

    else
    {
        sonar_distance[sonar_sequence[sonar_index]] = 0;
        sonar_updated = true;
        sonar_index++;
        if(sonar_index==16)
            sonar_index=0;
    }

}
// ################################################################



/* interrupt handle on the sonar echo -*/
// ################################################################
void Sonar_Echo()
{
    nh.logdebug(" == Sonar Echo");

    sonar_echo = micros()-sonar_trig;
    digitalWrite(SONAR_INIT, LOW);
    detachInterrupt(4);

    sonar_distance[sonar_sequence[sonar_index]] = (sonar_echo/1000000*343/2);

    sonar_range.ranges[sonar_sequence[sonar_index]] = sonar_distance[sonar_sequence[sonar_index]];

    sonar_updated = true;
    sonar_index++;
    if(sonar_index==16)
    {
        sonar_index=0;
        sonar_array_done = true;
    }

    nh.logdebug(" == Sonar Echo done");
}

#endif



