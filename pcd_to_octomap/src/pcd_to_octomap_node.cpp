#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <string>

class PCDToOctomapNode : public rclcpp::Node {
public:
    PCDToOctomapNode() : Node("pcd_to_octomap_node") {
        // 声明参数
        this->declare_parameter<std::string>("pcd_file", "");
        this->declare_parameter<std::string>("output_file", "output.bt");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<std::string>("frame_id", "map");
        
        // 获取参数
        std::string pcd_file = this->get_parameter("pcd_file").as_string();
        std::string output_file = this->get_parameter("output_file").as_string();
        double resolution = this->get_parameter("resolution").as_double();
        std::string frame_id = this->get_parameter("frame_id").as_string();
        
        if (pcd_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "PCD file path is empty!");
            return;
        }
        
        // 转换PCD到Octomap
        if (convertPCDToOctomap(pcd_file, output_file, resolution)) {
            RCLCPP_INFO(this->get_logger(), "Successfully converted %s to %s", 
                       pcd_file.c_str(), output_file.c_str());
            
            // 发布八叉树消息（可选）
            octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>(
                "octomap", rclcpp::QoS(1).reliable());
            
            publishOctomap(output_file, frame_id);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert PCD to Octomap");
        }
    }

private:
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    
    bool convertPCDToOctomap(const std::string& pcd_file, 
                            const std::string& output_file, 
                            double resolution) {
        try {
            // 加载PCD文件
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file.c_str());
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "Loaded PCD with %lu points", cloud->size());
            
            // 创建八叉树
            std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(resolution);
            
            // 插入点云到八叉树
            for (const auto& point : cloud->points) {
                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    octree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
                }
            }
            
            // 更新内部占用信息
            octree->updateInnerOccupancy();
            
            // 保存八叉树
            octree->writeBinary(output_file);
            
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during conversion: %s", e.what());
            return false;
        }
    }
    
    void publishOctomap(const std::string& octomap_file, const std::string& frame_id) {
        // 加载八叉树文件
        std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(octomap_file);
        
        // 转换为ROS2消息
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = frame_id;
        octomap_msg.header.stamp = this->now();
        
        if (octomap_msgs::fullMapToMsg(*octree, octomap_msg)) {
            octomap_publisher_->publish(octomap_msg);
            RCLCPP_INFO(this->get_logger(), "Published octomap message");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to message");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDToOctomapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
