#include "crawling_robot_controller/define.h"
#include "crawling_robot_controller/dynamixel_funtion.h"
#include "crawling_robot_controller/cmd_funtion.h"


void keyboard_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    keyboard_data = msg->data;
    if (isManipulated)
        Manipulating_mode();
    else
        Localizing_mode();
}

void IK_2dim(double x, double y)
{
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    double theta3 = (theta1 + theta2);

    base_ref_pos = M_PI + theta1 - M_PI / 2 ;
    elbow_ref_pos = M_PI + theta2 ;
    ee_ref_pos = M_PI + theta3 + M_PI / 2;
}

void IK_2dim_f(double x, double y)
{
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    double theta3 = (theta1 + theta2);

    base_ref_pos = M_PI + theta1 - M_PI / 2 + FeedForward_theta1;
    elbow_ref_pos = M_PI + theta2 + FeedForward_theta2;
    ee_ref_pos = M_PI + theta3 + M_PI / 2 + FeedForward_theta3;
}

void IK_2dim_b(double x, double y)
{
    double theta2 = -acos((x * x + y * y - 2 * L * L) / (2 * L * L));
    double theta1 = atan2(y, x) + asin(L * sin(-theta2) / sqrt(x * x + y * y));
    double theta3 = (theta1 + theta2);

    base_ref_pos = M_PI + theta1 - M_PI / 2 - FeedForward_theta1;
    elbow_ref_pos = M_PI + theta2 + FeedForward_theta2;
    ee_ref_pos = M_PI + theta3 + M_PI / 2 - FeedForward_theta3;
}


int main(int argc, char **argv)
{
    system(("sudo chmod 666 " + std::string(DEVICENAME)).c_str());

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("position_control");
    rclcpp::WallRate loop_rate(loop_hz);
    rclcpp::on_shutdown([]() { KILL_dynamixel(); });

    
    gripper_state_pub = node->create_publisher<std_msgs::msg::Int8MultiArray>("gripper_state", 10);
    auto keyboard_sub = node->create_subscription<std_msgs::msg::Int8>("keyboard", 10, keyboard_callback);

    CONNECT_dynamixel();
    SET_dynamixel();

    while (rclcpp::ok()) 
    {
        if (isManipulated) 
        {
            if (manipulating_x) x_ee += free_step;
            if (manipulating_y) y_ee += free_step;
            if (manipulating_x_back) x_ee -= free_step;
            if (manipulating_y_back) y_ee -= free_step;

            IK_2dim(x_ee, y_ee);
            BASE_position_control(base_ref_pos);
            ELBOW_position_control(elbow_ref_pos);
    
            ee_ref_pos = ee_ref_pos > 1.58 * M_PI ? 1.58 * M_PI: ee_ref_pos+EE_Heanding_unit;
            ee_ref_pos = ee_ref_pos < 0.42 * M_PI ? 0.42 * M_PI: ee_ref_pos+EE_Heanding_unit;
            EE_position_control(ee_ref_pos);

            std_msgs::msg::Int8MultiArray gripper_state_msg;
            gripper_state_msg.data.resize(1);

            if (grip_mode) grip_command = 3;
            else if (grip_mode_back) grip_command = 4;
            else grip_command = 0;

            gripper_state_msg.data[0] = grip_command;
            gripper_state_pub->publish(gripper_state_msg);
        }

        if(localizing_forward) 
        {
            path();
            IK_2dim_f(x_ee, y_ee);
            BASE_position_control(base_ref_pos);
            ELBOW_position_control(elbow_ref_pos);
            EE_position_control(ee_ref_pos);
        }

        if(localizing_backward) 
        {
            path();
            IK_2dim_b(x_ee, y_ee);
            BASE_position_control(base_ref_pos);
            ELBOW_position_control(elbow_ref_pos);
            EE_position_control(ee_ref_pos);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    KILL_dynamixel(); 
    return 0;
}
