
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

// ROS API
//#include <geometry_msgs/Twist.h> // Velocity messages
//#include <nav_msgs/Odometry.h> // Odometry messages
//#include <sensor_msgs/Joy.h>
//#include <sensor_msgs/JoyFeedbackArray.h>
//#include <sensor_msgs/JoyFeedback.h>
//#include <wiimote/State.h>
//#include <p2os_msgs/MotorState.h>


#define DEG2RAD(x) x*M_PI/180.0 // Transform from degrees to radians
#define RAD2DEG(x) x*180.0/M_PI // Transform from radians to degrees



#define VEL_BWD_MAX_VAL -0.2
#define VEL_FWD_MAX_VAL 0.2
#define INC_DEC_VEL_FACTOR 0.01
#define INC_DEC_STEER_FACTOR 0.05


struct termios org_tios;


ros::Publisher pub_command;
agv_ipleiria::Agv_sensors sensor_srv;
agv_ipleiria::Agv_main main_srv;
agv_ipleiria::AgvCommand msg;



geometry_msgs::Pose2D robot_pose;
double true_lin_vel, true_ang_vel;
bool odom_updated = false;
double lin_vel=0, ang_vel=0;
double turret_angle=0;
double last_robot_theta=0;
double l_scale_, a_scale_;

/// laser information
bool laser_updated = false;
double closest_front_obstacle, closest_left_obstacle, closest_right_obstacle;
bool manual_mode = false; // If true, navigate in automatic mode
double stop_front_dist, min_front_dist, min_right_dist;




double clipValue(double value, double min, double max)
{
  if( value > max )
    return max;
  else if( value < min )
    return min;
  else return value;
}




void statusCallback(const agv_ipleiria::AgvStatus& msg)
{
  // Store updated values
  robot_pose.x = msg.pose_x;
  robot_pose.y = msg.pose_y;
  robot_pose.theta = msg.base_angle;   // tf::getYaw(msg.base_angle);

  true_lin_vel = msg.linear;
  true_ang_vel = msg.angular;



  // show pose estimated from odometry
  std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
            << "Robot estimated pose = "
            << robot_pose.x << " [m], " << robot_pose.y << " [m], "
            << robot_pose.theta << " [rad]\n";

  // Show estimated velocity
  std::cout << "Robot estimated velocity = "
            << true_lin_vel << " [m/s], "
            << true_ang_vel << " [rad/s]\n";


  odom_updated = true;
}






/// ##################################################
/// Laser callback function --  obstacles detection
/// ##################################################
void laserCallback(const sensor_msgs::LaserScan& msg)
{

    std::cout << "lasrcallback ---"<< std::endl;

    ROS_INFO("laser callback");

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
    msg.turret_angle = 0;

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


        //
      // Create robot related objects
      //
      // Linear and angular velocities for the robot (initially stopped)
      double last_ang_vel = DEG2RAD(20);
      // Navigation variables
      bool avoid, new_rotation = false;
      double stop_front_dist, min_front_dist;

      // Init ROS
      ros::init(argc, argv, "agvipleiria_demo");


      // ROS variables/objects
      ros::NodeHandle nh; // Node handle


      // Get parameters
      ros::NodeHandle n_private("~");
      n_private.param("min_front_dist", min_front_dist, 0.9);
      n_private.param("stop_front_dist", stop_front_dist, 0.4);
      n_private.param("scale_angular", a_scale_, 0.05);
      n_private.param("scale_linear", l_scale_, 0.02);

      sleep(5);



      std::cout << "Random navigation with obstacle avoidance \n"
                << "---------------------------" << std::endl;

      /// Setup publishers
      pub_command = nh.advertise<agv_ipleiria::AgvCommand>("/AGVipleiria/Command", 1);


      /// Setup subscribers
      // Odometry
      ros::Subscriber sub_status = nh.subscribe("/AGVipleiria/Status", 1, statusCallback);
      // Laser scans
      ros::Subscriber sub_laser = nh.subscribe("/AGVipleiria/scan", 1, laserCallback);
      /// Setup Subscribers (for Future use)


      // Infinite loop
      ros::Rate cycle(10.0); // Rate when no key is being pressed

  msg.linear = 0.05;
  msg.angular = 0;
//  msg.turret_angle = 0;


  sleep(2);
  pub_command.publish(msg);

  bool bKeyPressed; // Run the publish initially

  int i = 0;


  char cChar = ' ';



  while(ros::ok())
  {


     bKeyPressed = false;

     // Get data from the robot and print it if available
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


}

//      Only change navigation controls if laser was updated
//     if( laser_updated == false )
//       continue;

     // show pose estimated from odometry
     std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3)
               << "Robot estimated pose = "
               << robot_pose.x << " [m], " << robot_pose.y << " [m], "
               << RAD2DEG(robot_pose.theta) << " [º]\n";

     // Show estimated velocity
     std::cout << "Robot estimated velocity = "
               << true_lin_vel << " [m/s], "
               << RAD2DEG(true_ang_vel) << " [º/s]\n";

     // Check for obstacles near the front  of the robot
     if( manual_mode == false ) // Autonomous mode
     {
       avoid = false;
       if( closest_front_obstacle < min_front_dist )
       {
         if( closest_front_obstacle < stop_front_dist )
         {
           avoid = true;
           lin_vel = -0.100;
//           new_rotation = false;
         } else
         {
           avoid = true;
           lin_vel = 0.05;
         }
       } else
       {
         lin_vel = 0.1;
         ang_vel = 0;
         new_rotation = false;
       }

       // Rotate to avoid obstacles
       if(avoid)
       {
         if( new_rotation == false )
         {
           double rnd_point = drand48();
           if( rnd_point >= 0.9 )
           {
             last_ang_vel = -last_ang_vel;
             last_robot_theta = robot_pose.theta;
           }
         }
         ang_vel = last_ang_vel;
         new_rotation = true;

//         if(fabs(last_robot_theta - robot_pose.theta)>0.6)
//             ang_vel = 0;

       }
     } else // Manual mode
     {
       if( closest_front_obstacle < stop_front_dist )
            lin_vel = 0;

       ROS_INFO("manual mode");


     }

     // Limit maximum velocities
     // (not needed here)
 //    lin_vel = clipValue(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL);
 //    ang_vel = clipValue(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL);

     // Show desired velocity
     std::cout << "Robot desired velocity = "
               << lin_vel << " [m/s], "
               << RAD2DEG(ang_vel) << " [º/s],  "
               << robot_pose.theta << " [rad] "<< std::endl;

     if (bKeyPressed)
     {
       if (cChar=='q' ) //|| (cChar==' ' && iMode==MODE_OFF))
       {
           std::cout << "Key pressed:'" << cChar <<"' ---> Clear output values" << std::endl;

           // Clear Variables
         lin_vel = 0;
         ang_vel = 0;
        }
       else
       {
         std::cout << "Key pressed:'" << cChar << std::endl;
        }

}

     // Send velocity commands
     msg.linear = lin_vel;
     msg.angular = ang_vel;
     msg.turret_angle = robot_pose.theta;


     pub_command.publish(msg);  //Publish when changes happen




//     if (cChar=='q') break; //Close program if (q)uit

//   else
//   {
     cycle.sleep(); // Limit the amount of messages when no key is pressed
//   }
 }

   // If we are quitting, stop the robot


   // Send velocity commands
   msg.linear = 0;
   msg.angular = 0;
   pub_command.publish(msg);  //Publish when changes happen
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


