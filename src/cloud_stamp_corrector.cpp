// 点群の stamp を、この PC の時計で使える時刻にして配信し直す（mode は localization_bringup の環境ファイルの stamp_mode がそのまま入る）。
//
//   mode "receipt"：t = t_受信 − min_transport_delay
//   mode "auto"   ：|t_受信 − t_stamp| が auto_threshold 以上なら受信した時刻（receipt と同じ）、
//                   それより小さければ stamp をそのまま使う（1 つずつ判断する）
// sim time で /clock をまだ受け取っていない間（受信した時刻が 0）は stamp のまま渡す。
// 時刻同期済み（stamp_mode: stamp）の環境・Gazebo では起動しない。
// 時計のずれを推定して直す mode "estimate" は 2026-10-15 に廃止した。
#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CloudStampCorrector : public rclcpp::Node
{
public:
  explicit CloudStampCorrector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("cloud_stamp_corrector", options)
  {
    mode_ = declare_parameter<std::string>("mode", "receipt");
    if (mode_ != "receipt" && mode_ != "auto") {
      RCLCPP_WARN(
        get_logger(), "Unknown mode \"%s\". Falling back to \"receipt\".", mode_.c_str());
      mode_ = "receipt";
    }
    min_transport_delay_ns_ = toNs(declare_parameter("min_transport_delay", 0.0));
    auto_threshold_ns_ = toNs(declare_parameter("auto_threshold", 0.1));
    RCLCPP_INFO(
      get_logger(), "mode: %s, min_transport_delay: %.3f s, auto_threshold: %.3f s",
      mode_.c_str(), min_transport_delay_ns_ * 1e-9, auto_threshold_ns_ * 1e-9);

    // reliable は reliable / best effort どちらの subscriber とも接続できる
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    // best effort は reliable / best effort どちらの publisher とも接続できる
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
      std::bind(&CloudStampCorrector::cloudReceived, this, std::placeholders::_1));
  }

private:
  static int64_t toNs(double sec) {return static_cast<int64_t>(sec * 1e9);}

  void cloudReceived(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    const int64_t t_receipt = now().nanoseconds();
    if (t_receipt == 0) {
      // sim time で /clock をまだ受け取っていない（受信した時刻が分からない）。stamp のまま渡す
      pub_->publish(std::move(msg));
      return;
    }
    const int64_t t_stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).nanoseconds();
    const int64_t diff = t_receipt - t_stamp;
    const int64_t abs_diff = diff < 0 ? -diff : diff;
    const bool use_receipt = mode_ == "receipt" || abs_diff >= auto_threshold_ns_;
    if (use_receipt) {
      msg->header.stamp = rclcpp::Time(t_receipt - min_transport_delay_ns_, RCL_ROS_TIME);
    }

    if (mode_ == "auto") {
      ++count_;
      count_receipt_ += use_receipt ? 1 : 0;
      max_abs_diff_ns_ = std::max(max_abs_diff_ns_, abs_diff);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "auto: receipt time used for %ld of %ld clouds (|receipt - stamp| latest %.1f ms, max %.1f ms)",
        static_cast<long>(count_receipt_), static_cast<long>(count_), diff * 1e-6,
        max_abs_diff_ns_ * 1e-6);
    }
    pub_->publish(std::move(msg));
  }

  std::string mode_;
  int64_t min_transport_delay_ns_;
  int64_t auto_threshold_ns_;
  int64_t count_{0};
  int64_t count_receipt_{0};
  int64_t max_abs_diff_ns_{0};

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
