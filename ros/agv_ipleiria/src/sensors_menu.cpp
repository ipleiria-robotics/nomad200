

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
              << "\t\t => AGV Sensors Service <=\n"
              << "\t\n"
              << "\t\t 0- SONAR SENSORS\t\n"
              << "\t\t 1- INFRARED SENSORES\t\n"
              << "\t\t 2- BUMPER CONTACTS\t\n"
              << "\t\t Q- Quit\t\n"
              << "\t___________________________________________" << std::endl;



////////////////////////////////////////////////////////////////
/////////
/////////
//// falta terminar este ficheiro para fazer os menus e submenus
///
///
///
/// ///////////////////////////////////////////////////////////////
///

    ros::init(argc, argv, "sensors_menu");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<agv_ipleiria::Agv_main>("AGVipleiria/Agv_sensors");
    agv_ipleiria::Agv_sensors srv;




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




                case '0':   // SONAR SENSORS
                        break;

                case '2':   // BUMPER CONTACTS

                    srv.request.radiation_type = 2;
                    srv.request.radiation_state = 2;
                    aux = srv.response.radiation_state;
                    if (aux == 0)
                        srv.request.radiation_state = 1;
                    else
                    {
                        srv.request.radiation_state= 0;

                    }

                    bKeyPressed = true;

                    if (client.call(srv))
                    {
                        ROS_INFO("Bumper Contacts State: %ld", (long int)srv.response.radiation_state);
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



