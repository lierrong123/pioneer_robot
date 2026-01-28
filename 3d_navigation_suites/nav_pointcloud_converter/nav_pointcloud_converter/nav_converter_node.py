#!/usr/bin/env python3
"""
导航专用点云转换节点 - 只过滤机器人自身点云
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class NavPointCloudConverter(Node):
    def __init__(self):
        super().__init__('nav_pointcloud_converter')
        
        # 声明参数
        self.declare_parameter('input_topic', '/unilidar/cloud')
        self.declare_parameter('output_topic', '/unilidar/filter_cloud')
        self.declare_parameter('debug_topic', '/unilidar/cloud_debug')
        self.declare_parameter('robot_self_topic', '/robot_self_points')
        
        # 激光雷达安装参数
        self.declare_parameter('lidar_position.x', 0.08484)
        self.declare_parameter('lidar_position.y', 0.0)
        self.declare_parameter('lidar_position.z', 0.289262)
        self.declare_parameter('lidar_orientation.pitch', -0.6978888)  # 40度向下倾斜
        
        # 机器人自身过滤参数 - 简单但有效的边界框
        # 对于0.5m×0.5m×0.5m的机器人
        self.declare_parameter('robot_length', 1)    # X方向长度，比实际稍大确保完全过滤
        self.declare_parameter('robot_width', 1)     # Y方向宽度
        self.declare_parameter('robot_height', 1)    # Z方向高度
        
        # 距离过滤参数
        self.declare_parameter('min_distance', 0.1)
        self.declare_parameter('max_distance', 30.0)
        
        # 获取参数
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        
        # 激光雷达安装参数
        self.lidar_pos = np.array([
            self.get_parameter('lidar_position.x').value,
            self.get_parameter('lidar_position.y').value,
            self.get_parameter('lidar_position.z').value
        ])
        self.lidar_pitch = self.get_parameter('lidar_orientation.pitch').value
        
        # 机器人尺寸参数（一半尺寸，用于边界检查）
        self.robot_half_size = np.array([
            self.get_parameter('robot_length').value / 2,
            self.get_parameter('robot_width').value / 2,
            self.get_parameter('robot_height').value / 2
        ])
        
        # 距离参数
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        
        # 计算补偿矩阵
        self.compensation_pitch = -self.lidar_pitch
        cp = math.cos(self.compensation_pitch)
        sp = math.sin(self.compensation_pitch)
        self.compensation_matrix = np.array([
            [cp,  0,  sp],
            [0,   1,   0],
            [-sp, 0,  cp]
        ])
        
        # 创建QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 订阅原始点云
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_profile
        )
        
        # 发布过滤后的点云
        self.publisher_filtered = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile
        )
        
        # 发布调试点云
        self.publisher_debug = self.create_publisher(
            PointCloud2,
            self.debug_topic,
            qos_profile
        )
        
        # 发布机器人自身点云
        self.publisher_robot_self = self.create_publisher(
            PointCloud2,
            self.get_parameter('robot_self_topic').value,
            qos_profile
        )
        
        self.get_logger().info(
            f'导航点云转换器已启动\n'
            f'  输入话题: {self.input_topic}\n'
            f'  输出话题: {self.output_topic}\n'
            f'  机器人尺寸: {self.robot_half_size * 2} m\n'
            f'  过滤距离: {self.min_distance} - {self.max_distance} m'
        )
        
        # 统计信息
        self.stats = {
            'total': 0,
            'filtered': 0,
            'robot_self': 0,
            'last_print': time.time(),
            'frames': 0
        }
    
    def transform_point_cloud(self, points):
        """将点云从雷达坐标系转换到base_link坐标系"""
        if points.shape[0] == 0:
            return points
        
        # 补偿雷达倾斜
        points_rotated = np.dot(points, self.compensation_matrix.T)
        
        # 加上雷达安装位置
        points_transformed = points_rotated + self.lidar_pos
        
        return points_transformed
    
    def filter_robot_points(self, points):
        if points.shape[0] == 0:
            return points, np.array([]), np.array([])
        
        # 1. 距离过滤
        distances = np.sqrt(np.sum(points**2, axis=1))
        distance_mask = (distances > self.min_distance) & (distances < self.max_distance)
        
        # 2. 机器人自身过滤
        in_x = np.abs(points[:, 0]) < self.robot_half_size[0]
        in_y = np.abs(points[:, 1]) < self.robot_half_size[1]
        in_z = np.abs(points[:, 2]) < self.robot_half_size[2]
        robot_mask = in_x & in_y & in_z
        
        # 3. 地面过滤（关键）
        # 假设地面高度在-0.1到0.1之间（根据你的机器人调整）
        ground_mask = (points[:, 2] > -0.35) & (points[:, 2] < 0.1)
        
        # 保留的点：在距离范围内、不在机器人内部、不是地面
        keep_mask = distance_mask & (~robot_mask) & (~ground_mask)
        
        # 分离机器人自身点云和地面点云
        robot_points = points[robot_mask]
        ground_points = points[ground_mask]
        filtered_points = points[keep_mask]
        
        return filtered_points, robot_points
    
    def create_pointcloud_msg(self, points, header, frame_id='base_link', intensity=100.0):
        """创建PointCloud2消息"""
        if len(points) == 0:
            return None
        
        # 定义点云字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 创建数据
        data = []
        for point in points:
            data.append([
                float(point[0]),
                float(point[1]),
                float(point[2]),
                float(intensity)
            ])
        
        # 创建消息
        msg = point_cloud2.create_cloud(header, fields, data)
        msg.header.frame_id = frame_id
        
        return msg
    
    def pointcloud_callback(self, msg):
        try:
            # 读取原始点云数据
            gen = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points_original = np.array([p for p in gen])
            
            if len(points_original) == 0:
                return
            
            # 转换到base_link坐标系
            points_transformed = self.transform_point_cloud(points_original)
            
            # 过滤机器人自身点云
            points_filtered, robot_points = self.filter_robot_points(points_transformed)
            
            # 更新统计信息
            self.stats['total'] += len(points_original)
            self.stats['filtered'] += len(points_filtered)
            self.stats['robot_self'] += len(robot_points)
            self.stats['frames'] += 1
            
            # ========== 发布过滤后的点云（给octomap_server） ==========
            if len(points_filtered) > 0:
                filtered_msg = self.create_pointcloud_msg(
                    points_filtered, msg.header, 'base_link', intensity=100.0
                )
                self.publisher_filtered.publish(filtered_msg)
            
            # ========== 发布调试点云（转换后的原始点云） ==========
            debug_msg = self.create_pointcloud_msg(
                points_transformed, msg.header, 'base_link', intensity=50.0
            )
            if debug_msg:
                self.publisher_debug.publish(debug_msg)
            
            # ========== 发布机器人自身点云（用于可视化检查） ==========
            if len(robot_points) > 0:
                robot_msg = self.create_pointcloud_msg(
                    robot_points, msg.header, 'base_link', intensity=255.0  # 红色高亮
                )
                self.publisher_robot_self.publish(robot_msg)
            
            # 每5秒打印一次统计信息
            current_time = time.time()
            if current_time - self.stats['last_print'] > 5.0:
                total = self.stats['total']
                filtered = self.stats['filtered']
                robot_self = self.stats['robot_self']
                frames = self.stats['frames']
                
                if total > 0:
                    self.get_logger().info(
                        f'统计（最近{frames}帧）：\n'
                        f'  总点数：{total}，平均每帧：{total/frames:.0f}\n'
                        f'  保留点数：{filtered} ({filtered/total*100:.1f}%)\n'
                        f'  机器人点数：{robot_self} ({robot_self/total*100:.1f}%)'
                    )
                
                # 重置统计
                self.stats['total'] = 0
                self.stats['filtered'] = 0
                self.stats['robot_self'] = 0
                self.stats['frames'] = 0
                self.stats['last_print'] = current_time
                
        except Exception as e:
            self.get_logger().error(f'处理点云时出错: {str(e)}', throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = NavPointCloudConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()