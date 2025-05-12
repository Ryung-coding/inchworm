#ifndef DEFINE
#define DEFINE

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cstdlib>
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"

//_______________________________________
//
//          Model(ARM) values
//______________________________________
//
//     l       L         L       l      
//   <---> <-------> <-------> <--->
// 
//  {0}__{1}_______{2}_______{3}__{4}      
//   |            elbow            |     
//   d                             d      
//   |                             |       
//  ---                           --- 
//  base                          E.E.
//_______________________________________

// #define l 0.025 // [m] 
#define L 0.130 // [m] 
#define d 0.05 // [m]
#define FeedForward_theta1 0.07
#define FeedForward_theta2 0.02
#define FeedForward_theta3 -0.03
// System values
#define DEVICENAME "/dev/ttyUSB0"
#define loop_hz 100


// Dynamixel setting (Custom)
#define BAUDRATE 1000000 // 보드 레이트 1Mbps로 변경
#define BASE_MOTOR 1
#define XC330_DXL_ID 2
#define EE_MOTOR 3
  

// Common Dynamixel values
#define PPR 4096
#define PROTOCOL_VERSION 2.0
#define ADDR_SHUTDOWN 63
#define ADDR_TORQUE_ENABLE 64 
#define ADDR_GOAL_POSITION 116  
#define ADDR_PRESENT_POSITION 132  
#define ADDR_OPERATING_MODE 11  
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0


// control values
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;


int g_keyboard_command = 0;
int target_index = 0;
int grip_command = 0;
int keyboard_data = 0;

bool isManipulated = false;
bool manipulating_x = false;
bool manipulating_y = false;
bool manipulating_x_back = false;
bool manipulating_y_back = false;
bool grip_mode = false;
bool grip_mode_back = false;
bool localizing_forward = false;
bool localizing_backward = false;
bool BASEisOpend = false;
bool EEisOpend = false;

const double step_size = 0.002;
double free_step = 0.001;

double EE_Heanding_unit = 0.014753;
double move_size = 0.000;
double x_target = 0.0;
double y_target = 0.0;
double distance = 0.0;

double base_ref_pos = 0.0;
double elbow_ref_pos = 0.0;
double ee_ref_pos = 0.0;

enum KeyboardCommand
{
    STOP = 0,
    MOVE_Y_FORWARD = 1,
    MOVE_Y_BACKWARD = 2,
    MOVE_X_FORWARD = 3,
    MOVE_X_BACKWARD = 4,
    DECREASE_INTERVAL = 5,
    INCREASE_INTERVAL = 6,
    SWITCH_MODE = 7,
    GRIP_ON = 8,
    GRIP_OFF = 9
};

std::vector<std::pair<double, double>> getPoints(double move_size)
{
    move_size = move_size > 0.02 ? 0.02 : move_size;
    move_size = move_size < 0.00 ? 0.00 : move_size;
    return {
        {0.155, -0.015},
        {0.155 + move_size / 2, 0.02},
        {0.23 + move_size, 0.05},
        {0.23 + move_size, 0.0},
        {0.23 + move_size, -0.05},
        {0.155 + move_size / 2, -0.02},
        {0.155, -0.015}
    };
}

// std::vector<std::pair<double, double>> getPoints(double move_size)
// {
//     move_size = move_size > 0.02 ? 0.02 : move_size;
//     return {
//         {0.17, 0.0},
//         {0.19 + move_size / 2, 0.05},
//         {0.21 + move_size, 0.10},
//         {0.21 + move_size, 0.0},
//         {0.21 + move_size, -0.10},
//         {0.19 + move_size / 2, -0.05},
//         {0.17, 0.0}
//     };
// }

auto points = getPoints(move_size);
double x_ee = points[0].first;
double y_ee = points[0].second;


double base_now_pos = 0;
double elbow_now_pos = 0;
double ee_now_pos = 0;
rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr gripper_state_pub;

#endif // DEFINE
