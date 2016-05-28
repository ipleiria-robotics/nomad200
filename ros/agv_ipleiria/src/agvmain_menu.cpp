

// Headers to read and write from the terminal
#include <iostream>
#include <iomanip>
#include <termios.h>
#include <stdio.h>



// ROS API
#include <ros/ros.h>
#include <agv_ipleiria/AgvCommand.h>
#include <agv_ipleiria/AgvStatus.h>
#include <agv_ipleiria/RangeArray.h>
#include <agv_ipleiria/Agv_sensors.h>
#include <agv_ipleiria/Agv_main.h>

#include <cstdlib>



struct termios org_tios;

/**
 * Restore the terminal to its original settings
 */
void restoreTerminal()
{
    std::cout << "restoreTerminal called"<< std::endl;



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




int main(int argc, char **argv)
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

    // Output usage information
    std::cout << "\t___________________________________________\n"
              << "\t\t => AGV Main Service <=\n"
              << "\t\n"
              << "\t\t 1- Toggle ACTIVATE MOTOR POWER\t\n"
              << "\t\t 2- Toggle ENABLE MOTOR CONTROLLER\t\n"
              << "\t\t 3- Wheels Homing\t\n"
              << "\t\t 4- Turret Homing\t\n"
              << "\t\t Q- Quit\t\n"
              << "\t___________________________________________" << std::endl;





    ros::init(argc, argv, "agvmain_menu");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<agv_ipleiria::Agv_main>("AGVipleiria/Agv_main");
    agv_ipleiria::Agv_main srv;




    ros::Rate loop_rate(50);
    ros::Rate cycle(10.0); // Rate when no key is being pressed
    bool bKeyPressed; // Run the publish initially
    char cChar = ' ';
    int aux;



    while (ros::ok())
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

                case ' ':   // STOP
                    //                    i_lin_vel = 0;
                    //                    i_ang_vel = 0;

                    bKeyPressed = true;
                    break;



                case '1':   // ACTIVATE POWER

                    srv.request.Motor_power = 2;
                    aux = srv.response.Motor_power;
                    if (aux == 0)
                        srv.request.Motor_power = 1;
                    else
                    {
                        srv.request.Motor_power = 0;
                        srv.request.Motor_enable = 0;

                    }

                    bKeyPressed = true;

                    if (client.call(srv))
                    {
                        ROS_INFO("Motor Power State: %ld", (long int)srv.response.Motor_power);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service Agv_main");
                    }

                    break;


                case '2':   // MOTOR ENABLE


                    srv.request.Motor_enable = 2;
                    aux = srv.response.Motor_enable;

                    if (aux == 0)
                    {
                        aux = srv.response.Motor_power;
                        if (aux == 0)
                            ROS_INFO("Before enabling the controllers, press '1' to Activate Motor Power.");
                        else
                            srv.request.Motor_enable = 1;

                    }
                    else
                        srv.request.Motor_enable = 0;

                    bKeyPressed = true;

                    if (client.call(srv))
                    {
                        ROS_INFO("Motor Enable State: %ld", (long int)srv.response.Motor_enable);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service Agv_main");
                    }

                    break;



                case '3':   // WHEELS HOMING


                    aux = srv.response.Motor_power;

                    if (aux == 0)
                        ROS_INFO("Before homing the wheels, Activate Motor Power and enable the controllers.");
                    else
                    {

                        aux = srv.response.Motor_enable;

                        if (aux == 0)
                            ROS_INFO("Before homing the wheels, enable the controllers.");
                        else

                        {
                            srv.request.Homing = 1;
                        }

                    }




                    bKeyPressed = true;

                    if (client.call(srv))
                    {
                        ROS_INFO("Wheels Homing Procedure: %ld", (long int)srv.response.Homing);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service Agv_main");
                    }


                    break;

                case '4':   // TURRET HOMING


                    aux = srv.response.Motor_power;

                    if (aux == 0)
                        ROS_INFO("Before homing the turret, Activate Motor Power and enable the controllers.");
                    else
                    {

                        aux = srv.response.Motor_enable;

                        if (aux == 0)
                            ROS_INFO("Before homing the turret, enable the controllers.");
                        else
                        {
                            srv.request.Homing = 2;
                        }


                    }


                    bKeyPressed = true;

                    if (client.call(srv))
                    {
                        ROS_INFO("Turret Homing Procedure: %ld", (long int)srv.response.Homing-1);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service Agv_main");
                    }



                    break;

                }

            }

        }


        if (bKeyPressed)
        {
            if (cChar=='q' )
            {
                std::cout << "Key pressed:'" << cChar <<"' ---> Clear output values" << std::endl;
            }
            else
            {
                std::cout << "Key pressed:'" << cChar << std::endl;

            }

            if (cChar=='q') break; //Close program if (q)uit

            else
            {
                cycle.sleep(); // Limit the amount of messages when no key is pressed
            }
        }
    }




    restoreTerminal();

    std::cout << "Exiting..." << std::endl;
    return 1;


}



