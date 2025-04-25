/*******************************************************************************
 @autor *******
 @brief node to read from and write to the motor encoders

*******************************************************************************/

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <iostream>
#include <thread>
#include <atomic>

#include "dynamixel_sdk.h"  // Uses dynamixel library

#define X_SERIES

// Control table address
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit
#define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit
#define BAUDRATE                    1000000

// Used data transfer protocol
#define PROTOCOL_VERSION  2.0

// Default id, is then adopted for the different motors
#define DXL_ID  1

// Use the actual port assigned 
#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b

// Zero positions for the used motors
#define ZERO_POSITION_2 0
#define ZERO_POSITION_3 2587
#define ZERO_POSITION_4 2060
#define ZERO_POSITION_5 3574
#define ZERO_POSITION_6 1049
#define ZERO_POSITION_7 990

#define ADDR_PRO_PROFILE_VELOCITY 108
#define ADDR_PRO_PROFILE_ACCELERATION 112
#define ADDR_PRO_CURRENT_LIMIT 38
#define ADDR_PRO_CONTROL_MODE 11
#define ADDR_PRO_GOAL_CURRENT 102

int getch() {
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void) 
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) 
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

// Array with all zero positions to iterate over
const int16_t ZERO_POSITION[] = {ZERO_POSITION_2, ZERO_POSITION_3, ZERO_POSITION_4, ZERO_POSITION_5, ZERO_POSITION_6, ZERO_POSITION_7};

// To be used in forked thread for waiting on keypress
std::atomic<bool> running(true);

// Outsourced funcitons for cleaner structure and reusability
void waitForKeyPress() 
{
    std::cin.get(); // Wait for a key press
    running = false; // Set running to false when a key is pressed
}

// Normalizing (shifting) the angle around zero so between -pi and pi
double normalize_angle(double angle)
{
    while (angle>M_PI)
    {
        angle -= 2*M_PI;
    }
    while (angle<-M_PI)
    {
        angle += 2*M_PI;
    }
    return angle;
}

// For pusblishing the motor configurations on the ros topic
void publish_joint_angles(dynamixel::PortHandler* portHandler,
                          dynamixel::PacketHandler* packetHandler,
                          ros::Publisher& pub,
                          sensor_msgs::JointState& joint_state_msg,
                          uint8_t& dxl_error)
{
    // Array to hold the positions of 6 joints
    uint16_t dxl_present_positions[6]; 
    for (int i = 0; i < 6; ++i) 
    {
        int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i + 2, ADDR_PRESENT_POSITION, &dxl_present_positions[i], &dxl_error);
    }

    // Update the timestamp for each iteration
    joint_state_msg.header.stamp = ros::Time::now();

    // Clear the previous positions and add the new ones
    joint_state_msg.position.clear(); // Clear previous positions
    
    // Converting the measured angle to rad, normalizing it and inserting it into the correct position to be published
    joint_state_msg.position.push_back(normalize_angle(static_cast<double>((dxl_present_positions[4] - ZERO_POSITION_6) % 4096) * (2 * M_PI / 4096)));
    joint_state_msg.position.push_back(normalize_angle(static_cast<double>((dxl_present_positions[2] - ZERO_POSITION_4) % 4096) * (2 * M_PI / 4096)));
    joint_state_msg.position.push_back(normalize_angle(static_cast<double>((dxl_present_positions[1] - ZERO_POSITION_3) % 4096) * (2 * M_PI / 4096)));
    joint_state_msg.position.push_back(normalize_angle(static_cast<double>((dxl_present_positions[5] - ZERO_POSITION_7) % 4096) * (2 * M_PI / 4096)));
    joint_state_msg.position.push_back(normalize_angle(static_cast<double>((dxl_present_positions[3] - ZERO_POSITION_5) % 4096) * (2 * M_PI / 4096)));
    joint_state_msg.position.push_back(normalize_angle(static_cast<double>((dxl_present_positions[0] - ZERO_POSITION_2) % 4096) * (2 * M_PI / 4096)));

    // publish the array with positions
    pub.publish(joint_state_msg);
}

int main(int argc, char **argv) 
{
    // Setup connection to ros node and encoders
    
    ros::init(argc, argv, "dynamixel_read_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("dynamixel_joint_states",10);
    // Bound by read and write speed of the registers
    ros::Rate loop_rate(10);
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

    // Different variable sizes for different used registers
    uint8_t dxl_error = 0; // DYNAMIXEL error
    #if defined(XL320)
    int16_t dxl_present_position = 0;  // XL-320 uses 2 byte Position data
    #else
    int32_t dxl_present_position = 0;  // Read 4 byte Position data
    #endif

    // Open port
    if (portHandler->openPort()) 
    {
        printf("Succeeded to open the port!\n");
    }
    else 
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) 
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else 
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }
    
    // create jont_state_msg to publish the joint state
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.push_back("joint1");
    joint_state_msg.name.push_back("joint2");
    joint_state_msg.name.push_back("joint3");
    joint_state_msg.name.push_back("joint4");
    joint_state_msg.name.push_back("joint5");
    joint_state_msg.name.push_back("joint6");
    
    // Start the alignment
    ROS_INFO("Press any key to start (Align master to zero).\n");
    if (getch() == ESC_ASCII_VALUE);
    
    // Activate position control
    for (int i = 0; i < 6; ++i) 
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_PRO_CONTROL_MODE,3, &dxl_error);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_PRO_CONTROL_MODE,3, &dxl_error);
    }
    
    // Set max velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_PRO_PROFILE_VELOCITY, 10000, &dxl_error);
    for (int i = 1; i < 6; ++i) 
    {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i+2, ADDR_PRO_PROFILE_VELOCITY, 50, &dxl_error);
    }
    
    // Set max acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_PRO_PROFILE_ACCELERATION, 100, &dxl_error);
    for (int i = 1; i < 6; ++i) 
    {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i+2, ADDR_PRO_PROFILE_ACCELERATION, 20, &dxl_error);
    }
    
    // Go to zero position
    for (int i = 0; i < 6; ++i) 
    {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i+2,ADDR_GOAL_POSITION , ZERO_POSITION[i], &dxl_error);
    }
    
    // Start reading from registers
    ROS_INFO("Press any key to contiue with recordig joint configurations.\n");
    if (getch() == ESC_ASCII_VALUE);
    
    std::thread keyPressThread(waitForKeyPress);
    
    ROS_INFO("Press any key to contiue with gravity compensation (Hold the arm).\n");
    
    // Loop until running is false, to wait for a keyboard input and read data meanwhile
    while (running) 
    { 
        for (int i = 0; i < 6; ++i) 
        {
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i+2,ADDR_GOAL_POSITION , ZERO_POSITION[i], &dxl_error);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Sleep for 10 ms
        publish_joint_angles(portHandler, packetHandler, pub, joint_state_msg, dxl_error);
    }

    keyPressThread.join(); // Wait for the key press thread to finish
    
    // activate position control
    for (int i = 0; i < 6; ++i)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_PRO_CONTROL_MODE,0, &dxl_error);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i+2, ADDR_PRO_CONTROL_MODE,0, &dxl_error);
    }
    
    // set current_limits
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 2, ADDR_PRO_CURRENT_LIMIT, 200, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 3, ADDR_PRO_CURRENT_LIMIT, 200, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 4, ADDR_PRO_CURRENT_LIMIT, 300, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 5, ADDR_PRO_CURRENT_LIMIT, 200, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 6, ADDR_PRO_CURRENT_LIMIT, 200, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, 7, ADDR_PRO_CURRENT_LIMIT, 200, &dxl_error);

    // start ros loop
    while (ros::ok())
    {
        ros::Time start_time = ros::Time::now();

        ros::spinOnce();

        // publish joint angles
        publish_joint_angles(portHandler, packetHandler, pub, joint_state_msg, dxl_error);

        ros::Duration duration = ros::Time::now() - start_time;
        ROS_INFO("Loop execution time: %f seconds", duration.toSec());
        loop_rate.sleep();
    }

    // Close port
    portHandler->closePort();
    return 0;
}
