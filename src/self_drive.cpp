#include "self_drive/self_drive.hpp" // 헤더 포함

SelfDrive::SelfDrive() : rclcpp::Node("self_drive_node") // 노드 이름 설정
{
  auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
  lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile, callback);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void SelfDrive::subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    float front = get_range_avg(scan, 0, 15);
    float f_left = get_range_avg(scan, 45, 15);
    float f_right = get_range_avg(scan, 315, 15);
    geometry_msgs::msg::Twist vel;

    if (front < STOP_DISTANCE) {
      vel.linear.x = 0.0;
      if (f_left > f_right) vel.angular.z = 1.0;
      else vel.angular.z = -1.0;
    }

    pose_pub_->publish(vel);
}

float SelfDrive::get_range_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int center_angle, int window)
{
  int size = scan->ranges.size();
  float sum = 0.0;
  int count = 0;
  for (int i = -window; i <= window; i++) {
    int idx = (center_angle + i + size) % size;
    float r = scan->ranges[idx];
    if (!std::isinf(r) && !std::isnan(r) && r > 0.01) {
      sum += r; count++;
    }
  }
  if (count == 0) return DEFAULT_RANGE;
  return sum / count;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}