#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>

using std::placeholders::_1;
using namespace std;

class wheeltec_joy : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    
    // 速度参数
    double v_linear, v_angular;
    // 手柄映射参数
    int axis_linear, axis_angular, axis_strafe;

public:
    wheeltec_joy() : Node("wheeltec_joy")
    {
        RCLCPP_INFO(this->get_logger(), "Mecanum Wheel Joy Control Started");
        
        // 参数声明 - 麦轮专用控制映射
        this->declare_parameter<int>("axis_linear", 1);      // 左摇杆Y轴 - 前后控制
        this->declare_parameter<int>("axis_strafe", 0);      // 左摇杆X轴 - 横向平移控制  
        this->declare_parameter<int>("axis_angular", 2);     // 右摇杆X轴 - 旋转控制
        this->declare_parameter<double>("v_linear", 0.3);    // 线速度最大值
        this->declare_parameter<double>("v_angular", 1.0);   // 角速度最大值
        
        // 获取参数
        this->get_parameter("axis_linear", axis_linear);
        this->get_parameter("axis_strafe", axis_strafe);
        this->get_parameter("axis_angular", axis_angular);
        this->get_parameter("v_linear", v_linear);
        this->get_parameter("v_angular", v_angular);
        
        pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&wheeltec_joy::callback, this, _1));
    }

private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr Joy)
    {
        geometry_msgs::msg::Twist v;
        
        // 读取加速因子（右摇杆）
        //double acce_x = (Joy->axes[4] + 1.0) / 2.0;   // 右摇杆上下 - 线速度加速 (0-1范围)
        //double acce_z = (Joy->axes[3] + 1.0) / 2.0;   // 右摇杆左右 - 角速度加速 (0-1范围)
        
        // ========== 麦轮全向移动控制 ==========
        
    double forward = Joy->axes[1] * v_linear;   // 左摇杆Y轴
    double strafe = Joy->axes[0] * v_linear;     // 左摇杆X轴
    double rotate = Joy->axes[3] * v_angular;    // 右摇杆X轴
        
        // 设置速度命令
        v.linear.x = forward;   // 前后移动 (m/s)
        v.linear.y = strafe;    // 横向平移 (m/s)
        v.angular.z = rotate;   // 旋转 (rad/s)
        
        // 可选：添加死区处理，防止摇杆微小漂移
        if (fabs(v.linear.x) < 0.05) v.linear.x = 0;
        if (fabs(v.linear.y) < 0.05) v.linear.y = 0;
        if (fabs(v.angular.z) < 0.05) v.angular.z = 0;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Mecanum Control - X:%.2f, Y:%.2f, Z:%.2f", 
            v.linear.x, v.linear.y, v.angular.z);
        
        pub->publish(v);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    printf("Mecanum Wheel Joy Control Node Started\n");
    rclcpp::spin(std::make_shared<wheeltec_joy>());
    rclcpp::shutdown();
    return 0;
}