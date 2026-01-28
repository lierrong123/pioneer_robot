#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <iostream>

int main() {
    std::cout << "Testing ackermann_msgs..." << std::endl;
    
    // 创建一个消息实例
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp.sec = 123;
    msg.drive.speed = 1.0;
    
    std::cout << "Success! Speed: " << msg.drive.speed << std::endl;
    return 0;
}