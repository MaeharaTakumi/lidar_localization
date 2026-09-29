#ifndef PCL_LOCALIZATION__EKF__EKF_PARAMS_HPP_
#define PCL_LOCALIZATION__EKF__EKF_PARAMS_HPP_

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "ekf/vehicle_ekf.hpp"
#include "ekf/vehicle_odom_ekf.hpp"

namespace pcl_localization
{

/// EKF 関係のパラメータ一式（src/ekf/config/ekf.yaml）
struct EkfConfig
{
  bool use_ekf{false};
  /// 閉ループ：NDT の初期値を EKF の推定 LiDAR 姿勢から作る
  bool use_init_guess{false};
  /// /pcl_pose・TF・/path を EKF の推定値にする
  bool use_for_pose_output{false};
  /// "ndt_only"（VehicleEkf）or "ndt_odom"（VehicleOdomEkf）
  std::string model{"ndt_only"};
  /// EKF に渡す時刻："wall"（now()）or "stamp"（メッセージ時刻）
  std::string time_source{"wall"};
  /// NDT の fitness_score が閾値を超えたら EKF を一切更新しない
  bool use_fitness_reject{false};
  double fitness_reject_threshold{1.0};
  /// 静的 TF base_link → LiDAR の子フレーム名
  std::string lidar_frame_id{"velodyne"};
  /// オドメトリ観測ノイズの出どころ："param"（R_odom）or "message"（twist.covariance）
  std::string odom_covariance_source{"param"};

  VehicleEkf::Params params;
  VehicleOdomEkf::OdomParams odom_params;
};

/// EKF のパラメータを宣言する（コンストラクタから呼ぶ）
void declareEkfParameters(rclcpp_lifecycle::LifecycleNode & node);
/// パラメータを読み込む。不正な値は WARN を出して既定値に戻す
EkfConfig loadEkfConfig(rclcpp_lifecycle::LifecycleNode & node);
/// 読み込んだ値を INFO で出す
void logEkfConfig(const rclcpp::Logger & logger, const EkfConfig & cfg);
/// model に応じた EKF を作る
std::unique_ptr<VehicleEkf> createEkf(const EkfConfig & cfg);

}  // namespace pcl_localization

#endif  // PCL_LOCALIZATION__EKF__EKF_PARAMS_HPP_
