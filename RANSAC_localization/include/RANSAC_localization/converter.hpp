#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "config.hpp"

class Converter{
public:
  Converter(){};

  vector<config::LaserPoint> scan_to_vector(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Pose &pose, const double &odom_to_lidar_x, const double &odom_to_lidar_y){
    vector<config::LaserPoint> src_points;
    for(size_t i=0; i< msg->ranges.size(); ++i) {
      config::LaserPoint src_point;
      if(msg->ranges[i] > 14 || msg->ranges[i] < 0.5) continue;
      src_point.angle = msg->angle_min + msg->angle_increment * i - pose.yaw;
      src_point.dist = msg->ranges[i];
      src_point.x = src_point.dist * cos(src_point.angle) + pose.x + odom_to_lidar_x;
      src_point.y = -src_point.dist * sin(src_point.angle) + pose.y + odom_to_lidar_y;
      src_points.push_back(src_point);
    }
    return src_points;
  }

  sensor_msgs::msg::PointCloud2 vector_to_PC2(vector<config::LaserPoint> &points){
    vector<vector<float>> points_;
    vector<float> point;
    for(size_t i=0; i<points.size(); i++){
      point = {points[i].x, points[i].y, 0};
      points_.push_back(point);
    }

    // PointCloud2メッセージを作成
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "map";
    cloud_msg.height = 1;
    cloud_msg.width = points_.size();
    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[2].name = "z";

    for (int i = 0; i < 3; ++i){
      cloud_msg.fields[i].offset = i * 4;
      cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[i].count = 1;
    }

    cloud_msg.point_step = 4 * 3;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;

    // ポイントデータをPointCloud2メッセージに追加
    int j = 0;
    for (auto& point : points_){
      memcpy(&cloud_msg.data[j], &point[0], sizeof(float) * 3);
      j += cloud_msg.point_step;
    }

    return cloud_msg;
  }

private:
};
