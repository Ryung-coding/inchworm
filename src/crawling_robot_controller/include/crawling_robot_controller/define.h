#ifndef DEFINE
#define DEFINE

#include <cmath>
#include <cstdlib>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

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
#define FeedForward_theta1 0.00 //0.07
#define FeedForward_theta2 0.00 //0.02
#define FeedForward_theta3 0.00 // 0.03
// System values
#define DEVICENAME_Dynamicxel "/dev/ttyUSB1"
#define DEVICENAME_Arduino "/dev/ttyUSB0"
#define loop_hz 100


// Dynamixel setting (Custom)
#define BAUDRATE 1000000 // 보드 레이트 1Mbps로 변경
#define BASE_MOTOR 1
#define XC330_DXL_ID 2
#define EE_MOTOR 3
  

// Common Dynamixel values
#define PPR 4096
#define PROTOCOL_VERSION 2.0
#define ADDR_SHUTDOWN     63
#define OVERLOAD_BIT_MASK 0x10
#define ADDR_TORQUE_ENABLE 64 
#define ADDR_GOAL_POSITION 116  
#define ADDR_PRESENT_POSITION 132  
#define ADDR_OPERATING_MODE 11  
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define ADDR_POSITION_P_GAIN   84    // Position P Gain 주소
#define ADDR_POSITION_I_GAIN   82    // Position I Gain 주소
#define ADDR_POSITION_D_GAIN   80    // Position D Gain 주소
#define ADDR_VELOCITY_P_GAIN   78    // Velocity P Gain 주소
#define ADDR_VELOCITY_I_GAIN   76    // Velocity I Gain 주소

// control values
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
serial::Serial arduino_serial;

std::vector<std::pair<double, double>> getPoints(double move_size)
{
    move_size = move_size > 0.02 ? 0.02 : move_size;
    move_size = move_size < 0.00 ? 0.00 : move_size;
    return {
        {0.155, 0.00}, //-0.015 //0
        {0.155 + move_size / 2, 0.02}, // 1
        {0.23 + move_size, 0.03}, // 2
        {0.23 + move_size, 0.00}, // 3
        {0.23 + move_size, -0.03}, // 4
        {0.155 + move_size / 2, -0.02}, // 5
        {0.155, 0.00} // 6
    };
}

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

double base_now_pos = 0;
double elbow_now_pos = 0;
double ee_now_pos = 0;

auto points = getPoints(move_size);
double x_ee = points[0].first;
double y_ee = points[0].second;

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

void Manipulating_mode()
{
    switch (keyboard_data)
    {
        case STOP:
            manipulating_x = false;
            manipulating_y = false;
            manipulating_x_back = false;
            manipulating_y_back = false;
            grip_mode = false;
            grip_mode_back = false;
            break;
        case MOVE_Y_FORWARD:
            manipulating_y = true;
            manipulating_x = false;
            manipulating_x_back = false;
            manipulating_y_back = false;
            break;
        case MOVE_X_FORWARD:
            manipulating_x = true;
            manipulating_y = false;
            manipulating_x_back = false;
            manipulating_y_back = false;
            break;
        case MOVE_Y_BACKWARD:
            manipulating_y = false;
            manipulating_x = false;
            manipulating_x_back = false;
            manipulating_y_back = true;
            break;
        case MOVE_X_BACKWARD:
            manipulating_x = false;
            manipulating_y = false;
            manipulating_x_back = true;
            manipulating_y_back = false;
            break;
        case DECREASE_INTERVAL:
            EE_Heanding_unit -= 0.05;
            break;
        case INCREASE_INTERVAL:
            EE_Heanding_unit += 0.05;
            break;
        case SWITCH_MODE:
            isManipulated = false;
            manipulating_x = false;
            manipulating_y = false;
            manipulating_x_back = false;
            manipulating_y_back = false;
            break;
        case GRIP_ON:
            grip_mode = true;
            grip_mode_back = false;
            break;
        case GRIP_OFF:
            grip_mode = false;
            grip_mode_back = true;
            break;
    }
}

void Localizing_mode()
{
    switch (keyboard_data)
    {
        case STOP:
            localizing_forward = false;
            localizing_backward = false;
            break;
        case MOVE_X_FORWARD:
            localizing_forward = true;
            localizing_backward = false;
            break;
        case MOVE_X_BACKWARD:
            localizing_forward = false;
            localizing_backward = true;
            break;
        case DECREASE_INTERVAL:
            move_size += 0.01;
            points = getPoints(move_size);
            break;
        case INCREASE_INTERVAL:
            move_size -= 0.01;
            points = getPoints(move_size);
            break;
        case SWITCH_MODE:
            isManipulated = true;
            break;
    }
}

void path()
{
    if (target_index < points.size()) 
    {
        points = getPoints(move_size);
        x_target = points[target_index].first;
        y_target = points[target_index].second;
        distance = sqrt(pow(x_target - x_ee, 2) + pow(y_target - y_ee, 2));

        if (distance > step_size) 
        {
            x_ee += step_size * (x_target - x_ee) / distance;
            y_ee += step_size * (y_target - y_ee) / distance;
        } 
        else 
        {
            if(localizing_forward){
                target_index = (target_index + 1) % points.size();
            }
            else if(localizing_backward){
                target_index = target_index == 0 ? points.size() : target_index;
                target_index = (target_index - 1) % points.size();
            }

            // Update state variables
            if(localizing_forward){
                BASEisOpend = (target_index > 3 || target_index <1); //|| target_index < 1
                EEisOpend = (target_index <= 3 && target_index>=1); //&& target_index >= 1
            }
            else if(localizing_backward){
                BASEisOpend = target_index >= 3;
                EEisOpend = target_index < 3;
            }

            // Send serial command based on open states
            std::string command = (BASEisOpend ? "1" : "0") + std::string(",") + (EEisOpend ? "1" : "0") + "\n";
            arduino_serial.write(command);
            arduino_serial.flush();
        }
    }
}

static inline void setGains(
    uint8_t dxl_id,
    uint16_t pos_p, uint16_t pos_i, uint16_t pos_d,
    uint16_t vel_p, uint16_t vel_i
) {
    uint8_t dxl_error = 0;
    // Position PID
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_POSITION_P_GAIN, pos_p, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_POSITION_I_GAIN, pos_i, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_POSITION_D_GAIN, pos_d, &dxl_error);
    // Velocity PI
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_VELOCITY_P_GAIN, vel_p, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_VELOCITY_I_GAIN, vel_i, &dxl_error);
}

#endif // DEFINE