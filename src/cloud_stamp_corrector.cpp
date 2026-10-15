// 点群の stamp を、この PC で受信した時刻に直して配信し直す。
//
// 点群に stamp を付ける PC とこの PC の時計が同期していないときに使う
// （時刻同期済みの環境・Gazebo では起動しない）。
//
//   t = t_受信 − min_transport_delay
//
// 時計のずれを推定して直す mode "estimate" は 2026-10-15 に廃止した（時刻同期済みで使わないため）。
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CloudStampCorrector : public rclcpp::Node
{
public:
  explicit CloudStampCorrector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("cloud_stamp_corrector", options)
  {
    min_transport_delay_ns_ =
      static_cast<int64_t>(declare_parameter("min_transport_delay", 0.0) * 1e9);
    RCLCPP_INFO(
      get_logger(), "stamp: receipt time - min_transport_delay (%.3f s)",
      min_transport_delay_ns_ * 1e-9);

    // reliable は reliable / best effort どちらの subscriber とも接続できる
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    // best effort は reliable / best effort どちらの publisher とも接続できる
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&CloudStampCorrector::cloudReceived, this, std::placeholders::_1));
  }

private:
  void cloudReceived(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    const int64_t t_scan = now().nanoseconds() - min_transport_delay_ns_;
    msg->header.stamp = rclcpp::Time(t_scan, RCL_ROS_TIME);
    pub_->publish(std::move(msg));
  }

  int64_t min_transport_delay_ns_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudStampCorrector>());
  rclcpp::shutdown();
  return 0;
}
