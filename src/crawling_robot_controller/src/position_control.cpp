#include "crawling_robot_controller/define.h"
#include "crawling_robot_controller/dynamixel_funtion.h"

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

    base_ref_pos = M_PI + theta1 - M_PI / 2;
    elbow_ref_pos = M_PI + theta2;
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
    system(("sudo chmod 666 " + std::string(DEVICENAME_Dynamicxel)).c_str());
    system(("sudo chmod 666 " + std::string(DEVICENAME_Arduino)).c_str());

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("position_control");
    rclcpp::WallRate loop_rate(loop_hz);
    rclcpp::on_shutdown([]() { KILL_dynamixel(); });

    auto keyboard_sub = node->create_subscription<std_msgs::msg::Int8>("keyboard", 10, keyboard_callback);

    CONNECT_dynamixel();
    SET_dynamixel();

    arduino_serial.setPort(DEVICENAME_Arduino);
    arduino_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    arduino_serial.setTimeout(to);
    arduino_serial.open();

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
    
            ee_ref_pos = ee_ref_pos > 1.58 * M_PI ? 1.58 * M_PI: ee_ref_pos + EE_Heanding_unit;
            ee_ref_pos = ee_ref_pos < 0.42 * M_PI ? 0.42 * M_PI: ee_ref_pos + EE_Heanding_unit;
            EE_position_control(ee_ref_pos);
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

    arduino_serial.close();
    KILL_dynamixel(); 
    return 0;
}
