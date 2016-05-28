
// Headers to read and write from the terminal
#include <iostream>
#include <iomanip>
#include <termios.h>
#include <stdio.h>
#include <vector>

// ROS API
#include <ros/ros.h>
#include <agv_ipleiria/AgvCommand.h>
#include <agv_ipleiria/AgvStatus.h>
#include <agv_ipleiria/RangeArray.h>
#include <agv_ipleiria/Agv_sensors.h>
#include <agv_ipleiria/Agv_main.h>
#include <geometry_msgs/Pose2D.h> // Velocity messages
#include <tf/tf.h> // Geometry transformations
#include <sensor_msgs/LaserScan.h>

#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees



#define VEL_BWD_MAX_VAL -0.6
#define VEL_FWD_MAX_VAL 0.6
#define INC_DEC_VEL_FACTOR 0.01
#define INC_DEC_STEER_FACTOR 0.05


struct termios org_tios;


ros::Publisher pub_command;
agv_ipleiria::Agv_sensors sensor_srv;
agv_ipleiria::Agv_main main_srv;
agv_ipleiria::AgvCommand msg;



geometry_msgs::Pose2D true_pose;
double true_lin_vel, true_ang_vel;
bool odom_updated = false;


/// laser information
bool laser_updated = false;
double closest_front_obstacle, closest_left_obstacle, closest_right_obstacle;
bool manual_mode = true; // If true, navigate in automatic mode
double stop_front_dist, min_front_dist, min_right_dist;

float   i_lin_vel = 0,
        i_ang_vel = 0,
        i_turret_angle = 0;


void statusCallback(const agv_ipleiria::AgvStatus& msg)
{
  // Store updated values
  true_pose.x = msg.pose_x;
  true_pose.y = msg.pose_y;
  true_pose.theta = msg.base_angle;

  true_lin_vel = msg.linear;
  true_ang_vel = msg.angular;
    i_turret_angle = true_pose.theta;

  // show pose estimated from odometry
  std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
            << "Robot estimated pose = "
            << true_pose.x << " [m], " << true_pose.y << " [m], "
            << true_pose.theta << " [º]\n";

//            << RAD2DEG(true_pose.theta) << " [º]\n";

  // Show estimated velocity
  std::cout << "Robot estimated velocity = "
            << true_lin_vel << " [m/s], "
            << true_ang_vel << " [º/s]\n";


  odom_updated = true;
}



/// ##################################################
/// Laser callback function --  obstacles detection
/// ##################################################
void laserCallback(const sensor_msgs::LaserScan& msg)
{
    double angle;
    unsigned int i;
    /// Update distance to closest obstacles
    // Right obstacle
    angle = -1.571; // DEG2RAD(-90)
    i = round((angle - msg.angle_min)/msg.angle_increment);
    closest_right_obstacle = msg.range_max;
    while( angle < -1.309 ) // DEG2RAD(-90+15)
    {
        if( (msg.ranges[i] < msg.range_max) &&
                (msg.ranges[i] > msg.range_min) &&
                (msg.ranges[i] < closest_right_obstacle) )
            closest_right_obstacle = msg.ranges[i];
        i++;
        angle += msg.angle_increment;
    }
    // Front obstacle
    angle = -0.785; // DEG2RAD(-45)
    i = round((angle - msg.angle_min)/msg.angle_increment);
    closest_front_obstacle = msg.range_max;
    while( angle < 0.785 ) // DEG2RAD(45)
    {
        if( (msg.ranges[i] < msg.range_max) &&
                (msg.ranges[i] > msg.range_min) &&
                (msg.ranges[i] < closest_front_obstacle) )
            closest_front_obstacle = msg.ranges[i];
        i++;
        angle += msg.angle_increment;
    }
    // Left obstacle
    angle = 1.309; // DEG2RAD(90-15)
    i = round((angle - msg.angle_min)/msg.angle_increment);
    closest_left_obstacle = msg.range_max;
    while( angle < 1.571 ) // DEG2RAD(90)
    {
        if( (msg.ranges[i] < msg.range_max) && (msg.ranges[i] > msg.range_min) &&
                (msg.ranges[i] < closest_left_obstacle) )
            closest_left_obstacle = msg.ranges[i];
        i++;
        angle += msg.angle_increment;
    }
    laser_updated = true;
    return;
}



/**
 * Restore the terminal to its original settings
 */
void restoreTerminal()
{
  std::cout << "restoreTerminal called"<< std::endl;

  //Clear output values


    msg.linear = 0;
    msg.angular = 0;
//    msg.turret_angle = 0;

  pub_command.publish(msg);  //Publish when changes happen

  tcsetattr(STDIN_FILENO, TCSANOW, &org_tios);
}
/**
 * Checks if there a key was pressed, and returns a positive number if true.
 */
int checkForKey()
{
  struct timeval tv = { 0L, 0L };
  fd_set fds;
  FD_SET(0, &fds);
  return select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
}


/**
 * Main function
 * Controls the robot using the keyboard keys and outputs posture and velocity
 * related information.
 */
int main(int argc, char** argv)
{
  // Set terminal settings for capturing the keyboard
  struct termios new_tios;
  tcgetattr( STDIN_FILENO, &org_tios );
  tcgetattr( STDIN_FILENO, &new_tios );
  atexit(restoreTerminal); // Restore on exit
  new_tios.c_iflag &= ~(IGNCR | INLCR);
  new_tios.c_iflag |= ICRNL;
  new_tios.c_oflag |= ONLCR;
  new_tios.c_lflag &= ~ICANON;
  new_tios.c_lflag &= ~(ECHO|ECHOE|ECHOK|ECHONL); // No echo
  tcsetattr( STDIN_FILENO, TCSANOW, &new_tios );

  // Init ROS
  ros::init(argc, argv, "agv_keyboard_teleop");

  // ROS variables/objects
  ros::NodeHandle nh; // Node handle


  // Output usage information
  std::cout << "Reading from keyboard\n"
            << "Use i, j, k, l and space to move front, left, back, right and stop, respectively\n"
            << " a: turret pose = 2.1;\n"
            << " s: turret pose = 0;\n"
            << " d: turret pose = -3\n"
            << "Press q to quit.\n"
            << "---------------------------" << std::endl;




  /// Setup subscribers
  // Odometry
  ros::Subscriber sub_status = nh.subscribe("/AGVipleiria/Status", 1, statusCallback);

  // Laser scans
  ros::Subscriber sub_laser = nh.subscribe("/AGVipleiria/Laser", 1, laserCallback);

  pub_command = nh.advertise<agv_ipleiria::AgvCommand>("/AGVipleiria/Command", 1);
    ros::Rate loop_rate(2);


  msg.linear = i_lin_vel;
  msg.angular = i_ang_vel;
//  msg.turret_angle = i_turret_angle;


  sleep((2));
  pub_command.publish(msg);

  // Infinite loop
  ros::Rate cycle(10.0); // Rate when no key is being pressed
  bool bKeyPressed; // Run the publish initially

  int i = 0;


  char cChar = ' ';
  while(ros::ok())
  {
    bKeyPressed = false;

    ros::spinOnce();

    // Read from the keyboard
    if(checkForKey())
    {
      cChar = getchar_unlocked();
      if( cChar == 'q')
      {

        std::cout << "Key pressed:'q' ---> Restore Values and quit" << std::endl;
         bKeyPressed = true;
      }
      else
      {
        switch(cChar)
        {


        case ' ':
          // Change Mode
          i_lin_vel = 0;
          i_ang_vel = 0;

          bKeyPressed = true;
          break;


        case 'k':
          // Decrease velocity

            if(i_lin_vel > VEL_BWD_MAX_VAL){
                i_lin_vel -= INC_DEC_VEL_FACTOR;
            }

          bKeyPressed = true;
          break;
        case 'i':
          // Increase velocity
            if(i_lin_vel < VEL_FWD_MAX_VAL){
                i_lin_vel += INC_DEC_VEL_FACTOR;


                std::cout << "/n LIN_VEL OUT =" << i_lin_vel<< " |" << std::endl;

            }
          bKeyPressed = true;
          break;
        case 'j':
          // Steer left
            i_ang_vel += INC_DEC_STEER_FACTOR;
          bKeyPressed = true;
          break;
        case 'l':
          // Steer right
            i_ang_vel -= INC_DEC_STEER_FACTOR;
          bKeyPressed = true;
          break;

        case 'a':
          // turret pose 1
            i_turret_angle = 2.1;


          bKeyPressed = true;
          break;

        case 's':
          // turret pose 1
            i_turret_angle = 0;

          bKeyPressed = true;
          break;
        case 'd':
          // turret pose 1
            i_turret_angle = -3;

          bKeyPressed = true;
          break;


        }
      }
    }
    if (bKeyPressed)
    {
      if (cChar=='q' )
      {
          std::cout << "Key pressed:'" << cChar <<"' ---> Clear output values" << std::endl;

          // Clear Variables
        msg.linear = 0;
        msg.angular = 0;
       }
      else
      {
        std::cout << "Key pressed:'" << cChar << std::endl;

      }

          //set values to message
      msg.linear = i_lin_vel;
      msg.angular = i_ang_vel;
      msg.turret_angle = i_turret_angle;


      pub_command.publish(msg);  //Publish when changes happen



      // Only change navigation controls if laser was updated
//      if( laser_updated == false )
//        continue;



      if (cChar=='q') break; //Close program if (q)uit
    }
    else
    {
      cycle.sleep(); // Limit the amount of messages when no key is pressed
    }
  }

  restoreTerminal();

  std::cout << "Exiting..." << std::endl;
  return 1;
}






























///*
//Copyright (c) 2013, Hugo Costelha
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:

//    * Redistributions of source code must retain the above copyright notice,
//      this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright notice,
//      this list of conditions and the following disclaimer in the documentation
//      and/or other materials provided with the distribution.
//    * Neither the name of the Player Project nor the names of its contributors
//      may be used to endorse or promote products derived from this software
//      without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//// Headers to read and write from the terminal
//#include <iostream>
//#include <iomanip>
//#include <stdio.h>

//// ROS API
//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h> // Velocity messages
//#include <geometry_msgs/Pose2D.h> // Velocity messages
////#include <nav_msgs/Odometry.h> // Odometry messages
////#include <sensor_msgs/LaserScan.h> // Laser sensor messages
////#include <tf/tf.h> // Geometry transformations
// #include <agv_ipleiria/AgvStatus.h>

//#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
//#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees

//geometry_msgs::Pose2D true_pose;

//agv_ipleiria::AgvStatus msg;
//double true_lin_vel, true_ang_vel;
//bool odom_updated = false, laser_updated = false;
//double closest_front_obstacle, closest_left_obstacle, closest_right_obstacle;

//double clipValue(double value, double min, double max)
//{
//  if( value > max )
//    return max;
//  else if( value < min )
//    return min;
//  else return value;
//}


//void StatusCallback(const agv_ipleiria::AgvStatus& msg)
//{

////    std::cout << "status callback" << std::endl;
////              << "---------------------------" << std::endl;
//    // Store updated values
////  true_pose.x = msg.Base.pose.pose.position.x;
////  true_pose.y = msg.Base.pose.pose.position.y;
////  true_pose.theta = msg.Turret.w; //tf::getYaw(msg.pose.pose.orientation);

////  true_lin_vel = msg.Base.twist.twist.linear.x; //          twist.twist.linear.x;
////  true_ang_vel = msg.Base.twist.twist.angular.z;
//  ROS_INFO("Motor Power: [%d]", msg.Motor_power);
//  ROS_INFO("Motor ready: [%d]", msg.Motor_ready);
//  odom_updated = true;
//}


////void odomCallback(const nav_msgs::Odometry& msg)
////{
////  // Store updated values
////  true_pose.x = msg.pose.pose.position.x;
////  true_pose.y = msg.pose.pose.position.y;
////  true_pose.theta = tf::getYaw(msg.pose.pose.orientation);

////  true_lin_vel = msg.twist.twist.linear.x;
////  true_ang_vel = msg.twist.twist.angular.z;

////  odom_updated = true;
////}


////void laserCallback(const sensor_msgs::LaserScan& msg)
////{
////  double angle;
////  unsigned int i;

////  /// Update distance to closest obstacles

////  // Right obstacle
////  angle = -1.571; // DEG2RAD(-90)
////  i = round((angle - msg.angle_min)/msg.angle_increment);
////  closest_right_obstacle = msg.range_max;
////  while( angle < -1.309 ) // DEG2RAD(-90+15)
////  {
////    if( (msg.ranges[i] < msg.range_max) &&
////        (msg.ranges[i] > msg.range_min) &&
////        (msg.ranges[i] < closest_right_obstacle) )
////      closest_right_obstacle = msg.ranges[i];
////    i++;
////    angle += msg.angle_increment;
////  }

////  // Front obstacle
////  angle = -0.785; // DEG2RAD(-45)
////  i = round((angle - msg.angle_min)/msg.angle_increment);
////  closest_front_obstacle = msg.range_max;
////  while( angle < 0.785 ) // DEG2RAD(45)
////  {
////    if( (msg.ranges[i] < msg.range_max) &&
////        (msg.ranges[i] > msg.range_min) &&
////        (msg.ranges[i] < closest_front_obstacle) )
////      closest_front_obstacle = msg.ranges[i];
////    i++;
////    angle += msg.angle_increment;
////  }

////  // Left obstacle
////  angle = 1.309; // DEG2RAD(90-15)
////  i = round((angle - msg.angle_min)/msg.angle_increment);
////  closest_left_obstacle = msg.range_max;
////  while( angle < 1.571 ) // DEG2RAD(90)
////  {
////    if( (msg.ranges[i] < msg.range_max) &&
////        (msg.ranges[i] > msg.range_min) &&
////        (msg.ranges[i] < closest_left_obstacle) )
////      closest_left_obstacle = msg.ranges[i];
////    i++;
////    angle += msg.angle_increment;
////  }

////  laser_updated = true;
////  return;
////}

///**
// * Main function
// * Controls the robot using the keyboard keys and outputs posture and velocity
// * related information.
// */
//int main(int argc, char** argv)
//{
//  //
//  // Create robot related objects
//  //
//  // Linear and angular velocities for the robot (initially stopped)
//  double lin_vel=0, ang_vel=0;
//  // Navigation variables
//  bool avoid, rotating_right = false, rotating_left = true;
//  double stop_front_dist, min_front_dist;

//  // Init ROS
//  ros::init(argc, argv, "kb_control");

//  // ROS variables/objects
//  ros::NodeHandle nh; // Node handle
////  ros::Publisher vel_pub; // Velocity commands publisher
////  geometry_msgs::Twist vel_cmd; // Velocity commands

//  std::cout << "Random navigation with obstacle avoidance" << std::endl
//            << "---------------------------" << std::endl;

//  // Get parameters
//  ros::NodeHandle n_private("~");
//  n_private.param("min_front_dist", min_front_dist, 0.8);
//  n_private.param("stop_front_dist", stop_front_dist, 0.6);

//  /// Setup subscribers
//  // Status
//  ros::Subscriber sub_status = nh.subscribe("AGVipleiriaStatus", 100, StatusCallback);

////  // Odometry
////  ros::Subscriber sub_odom = nh.subscribe("odom", 1, odomCallback);
////  // Laser scans
////  ros::Subscriber sub_laser = nh.subscribe("base_scan", 1, laserCallback);

//  // Setup publisher
////  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

//  // Infinite loop
//  ros::Rate cycle(10.0); // Rate when no key is being pressed
//  while(ros::ok())
//  {
//    // Get data from the robot and print it if available
//    ros::spinOnce();

//    // Only change navigation controls if laser was updated
//    if( laser_updated == false )
//      continue;

//    // show pose estimated from odometry
//    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
//              << "Robot estimated pose = "
//              << true_pose.x << " [m], " << true_pose.y << " [m], "
//              << RAD2DEG(true_pose.theta) << " [º]\n";

//    // Show estimated velocity
//    std::cout << "Robot estimated velocity = "
//              << true_lin_vel << " [m/s], "
//              << RAD2DEG(true_ang_vel) << " [º/s]\n";

//    avoid = false;
//    lin_vel = 0.4;
//    ang_vel = 0;

//    if( closest_front_obstacle < stop_front_dist )
//    {
//      avoid = true;
//      lin_vel = -0.100;
//    } else if( closest_front_obstacle < min_front_dist )
//    {
//      avoid = true;
//      lin_vel = 0;
//    }

//    if(avoid)
//    {
//      if( rotating_right )
//        ang_vel = DEG2RAD(30);
//      else if ( rotating_left )
//        ang_vel = DEG2RAD(-30);
//      else if(closest_left_obstacle < closest_right_obstacle )
//      {
//        rotating_left = true;
//        ang_vel = DEG2RAD(-30);
//      } else
//      {
//        rotating_right = false;
//        ang_vel = DEG2RAD(30);
//      }
//    } else
//    {
//      rotating_left = false;
//      rotating_right = false;
//    }

//    // Limit maximum velocities
//    // (not needed here)
////    lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
////    ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

//    // Show desired velocity
//    std::cout << "Robot desired velocity = "
//              << lin_vel << " [m/s], "
//              << RAD2DEG(lin_vel) << " [º/s]" << std::endl;

//    // Send velocity commands
////    vel_cmd.angular.z = ang_vel;
////    vel_cmd.linear.x = lin_vel;
////    vel_pub.publish(vel_cmd);

//    // Proceed at desired framerate
//    cycle.sleep();
//  }

//  // If we are quitting, stop the robot
////  vel_cmd.angular.z = 0;
////  vel_cmd.linear.x = 0;
////  vel_pub.publish(vel_cmd);

//  return 1;
//}
