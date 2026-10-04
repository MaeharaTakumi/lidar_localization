// 点群の stamp を、この PC の時計で表した取得時刻に直して配信し直す。
//
// 点群に stamp を付ける PC とこの PC の時計が同期していないときに使う
// （時刻同期済みの環境・Gazebo では起動しない）。
//
//   mode "estimate"：d_k = t_受信 − t_stamp（= 時計のずれ ＋ 通信遅れ）の直近 window 秒の最小値 d̂ を
//                    「時計のずれ ＋ 最小の通信遅れ」とみなし、t = t_stamp + d̂ − min_transport_delay とする。
//                    通信・executor の待ちによる揺らぎを除き、点群側で付いた取得間隔をそのまま保つ
//   mode "receipt" ：t = t_受信 − min_transport_delay
//
// 相手の時計が戻った（d_k が jump_threshold 以上大きい状態が jump_count 回続いた）ときは測り直す。
// 時刻は桁が大きい（時計が 1 年近くずれていることもある）ので整数のナノ秒で扱う。
#include <algorithm>
#include <cstdint>
#include <deque>
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
    mode_ = declare_parameter<std::string>("mode", "estimate");
    window_ns_ = toNs(declare_parameter("window", 10.0));
    min_transport_delay_ns_ = toNs(declare_parameter("min_transport_delay", 0.0));
    jump_threshold_ns_ = toNs(declare_parameter("jump_threshold", 0.5));
    jump_count_ = static_cast<int>(declare_parameter<int64_t>("jump_count", 5));
    if (mode_ != "estimate" && mode_ != "receipt") {
      RCLCPP_WARN(
        get_logger(), "Unknown mode \"%s\". Falling back to \"estimate\".", mode_.c_str());
      mode_ = "estimate";
    }
    RCLCPP_INFO(
      get_logger(), "mode: %s, window: %.1f s, min_transport_delay: %.3f s",
      mode_.c_str(), window_ns_ * 1e-9, min_transport_delay_ns_ * 1e-9);

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
    const int64_t t_stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).nanoseconds();

    int64_t t_scan = t_receipt;
    if (mode_ == "estimate") {
      updateOffset(t_receipt, t_receipt - t_stamp);
      // 受信より後に取得したことにはならない（測り直した直後の保険）
      t_scan = std::min(t_stamp + offset_ns_, t_receipt);
    }
    t_scan -= min_transport_delay_ns_;

    msg->header.stamp = rclcpp::Time(t_scan, RCL_ROS_TIME);
    pub_->publish(std::move(msg));
  }

  /// 直近 window 秒の d の最小値を offset_ns_ にする（単調キューで O(1)）
  void updateOffset(int64_t t, int64_t d)
  {
    if (!window_min_.empty() && d - window_min_.front().second > jump_threshold_ns_) {
      if (++jump_streak_ >= jump_count_) {
        RCLCPP_WARN(
          get_logger(), "Clock offset jumped by %.3f s. Re-estimating.",
          (d - window_min_.front().second) * 1e-9);
        window_min_.clear();
        jump_streak_ = 0;
      }
    } else {
      jump_streak_ = 0;
    }

    while (!window_min_.empty() && window_min_.back().second >= d) {
      window_min_.pop_back();
    }
    window_min_.emplace_back(t, d);
    while (window_min_.front().first < t - window_ns_) {
      window_min_.pop_front();
    }
    offset_ns_ = window_min_.front().second;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "clock offset (this PC - cloud stamp): %.6f s, latest delay above minimum: %.1f ms",
      offset_ns_ * 1e-9, (d - offset_ns_) * 1e-6);
  }

  std::string mode_;
  int64_t window_ns_;
  int64_t min_transport_delay_ns_;
  int64_t jump_threshold_ns_;
  int jump_count_;

  /// (受信時刻, d)。d が単調増加になるよう保ち、先頭が窓内の最小値
  std::deque<std::pair<int64_t, int64_t>> window_min_;
  int64_t offset_ns_{0};
  int jump_streak_{0};

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
