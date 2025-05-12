#ifndef CMD_FUNTION
#define CMD_FUNTION

#include "define.h"

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
    static int previous_index = -1;

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

            std_msgs::msg::Int8MultiArray gripper_state_msg;

            if(localizing_forward){
                BASEisOpend = (target_index > 3);
                EEisOpend = (target_index <= 3);
                gripper_state_msg.data.resize(2);
                gripper_state_msg.data[0] = BASEisOpend ? 1: 0;
                gripper_state_msg.data[1] = EEisOpend ? 1: 0;
            }
            else if(localizing_backward){
                BASEisOpend = target_index >= 3;
                EEisOpend = target_index < 3;
                gripper_state_msg.data.resize(2);
                gripper_state_msg.data[0] = BASEisOpend ? 1 : 0;
                gripper_state_msg.data[1] = EEisOpend ? 1 : 0;
            }

            gripper_state_pub->publish(gripper_state_msg);
            previous_index = target_index;
        }
    }
}

#endif // CMD_FUNTION
