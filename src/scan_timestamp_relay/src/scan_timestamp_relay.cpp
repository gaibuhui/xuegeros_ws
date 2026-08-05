#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanTimestampRelay : public rclcpp::Node
{
public:
  ScanTimestampRelay()
  : Node("scan_timestamp_relay")
  {
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
        auto corrected = *message;
        corrected.header.stamp = get_clock()->now();
        publisher_->publish(corrected);
      });
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_synced", rclcpp::SensorDataQoS());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanTimestampRelay>());
  rclcpp::shutdown();
  return 0;
}
