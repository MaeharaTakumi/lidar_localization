#include "ekf/ekf_params.hpp"

#include <vector>

namespace pcl_localization
{

void declareEkfParameters(rclcpp_lifecycle::LifecycleNode & node)
{
  node.declare_parameter("use_ekf", false);
  node.declare_parameter("use_ekf_init_guess", false);
  node.declare_parameter("use_ekf_for_pose_output", false);
  node.declare_parameter("ekf_model", "ndt_only");
  node.declare_parameter("time_source", "wall");
  node.declare_parameter("lidar_offset", std::vector<double>{0.0, 0.0, 0.0});
  node.declare_parameter("lidar_rotation", std::vector<double>{0.0, 0.0, 0.0});
  node.declare_parameter("lidar_frame_id", "velodyne");
  node.declare_parameter(
    "Q", std::vector<double>{1.0e-3, 1.0e-3, 2.0e-3, 5.0e-3, 5.0e-3, 1.0e-3, 0.05, 0.2});
  node.declare_parameter(
    "R_ndt", std::vector<double>{1.0e-4, 1.0e-4, 1.0e-4, 1.0e-6, 1.0e-6, 1.0e-6});
  node.declare_parameter("max_predict_dt", 0.2);
  node.declare_parameter(
    "P_init", std::vector<double>{2.0e-4, 2.0e-4, 2.0e-4, 2.0e-6, 2.0e-6, 2.0e-6, 1.0, 1.0});
  node.declare_parameter("gate_horizontal", 11.34);
  node.declare_parameter("gate_1d", 6.63);
  node.declare_parameter("lockout_count", 10);
  node.declare_parameter("use_fitness_reject", false);
  node.declare_parameter("fitness_reject_threshold", 1.0);

  // オドメトリ観測（ekf_model: "ndt_odom" のみ）
  node.declare_parameter("odom_covariance_source", "param");
  node.declare_parameter("R_odom", std::vector<double>{2.5e-3, 5.0e-3});
  node.declare_parameter("gate_odom", 9.21);
  node.declare_parameter("odom_timeout_init", 0.2);
}

EkfConfig loadEkfConfig(rclcpp_lifecycle::LifecycleNode & node)
{
  const rclcpp::Logger logger = node.get_logger();
  EkfConfig cfg;

  // ベクトルパラメータ。要素数が合わなければ既定値のまま WARN を出す。
  const auto load = [&](const std::string & name, std::size_t n, std::vector<double> & out) {
      node.get_parameter(name, out);
      if (out.size() != n) {
        RCLCPP_WARN(
          logger, "Parameter \"%s\" must have %zu elements (got %zu). Using default.",
          name.c_str(), n, out.size());
        return false;
      }
      return true;
    };
  // 文字列パラメータ。候補外なら先頭の候補に戻す
  const auto choose = [&](const std::string & name, std::string & out,
      std::initializer_list<const char *> candidates) {
      node.get_parameter(name, out);
      for (const char * c : candidates) {
        if (out == c) {return;}
      }
      const char * fallback = *candidates.begin();
      RCLCPP_WARN(
        logger, "Unknown %s \"%s\". Falling back to \"%s\".", name.c_str(), out.c_str(), fallback);
      out = fallback;
    };

  node.get_parameter("use_ekf", cfg.use_ekf);
  node.get_parameter("use_ekf_init_guess", cfg.use_init_guess);
  node.get_parameter("use_ekf_for_pose_output", cfg.use_for_pose_output);
  choose("ekf_model", cfg.model, {"ndt_only", "ndt_odom"});
  choose("time_source", cfg.time_source, {"wall", "stamp"});
  node.get_parameter("use_fitness_reject", cfg.use_fitness_reject);
  node.get_parameter("fitness_reject_threshold", cfg.fitness_reject_threshold);
  node.get_parameter("lidar_frame_id", cfg.lidar_frame_id);
  choose("odom_covariance_source", cfg.odom_covariance_source, {"param", "message"});

  VehicleEkf::Params & prm = cfg.params;
  std::vector<double> lidar_offset, lidar_rotation, q, p_init, r_ndt, r_odom;
  if (load("lidar_offset", 3, lidar_offset)) {
    prm.o_x = lidar_offset[0];
    prm.o_y = lidar_offset[1];
    prm.o_z = lidar_offset[2];
  }
  if (load("lidar_rotation", 3, lidar_rotation)) {
    prm.roll_o = lidar_rotation[0];
    prm.pitch_o = lidar_rotation[1];
    prm.yaw_o = lidar_rotation[2];
  }
  // Q と P_init は状態と同じ並び [x, y, z, roll, pitch, yaw, v, omega] の 8 要素
  if (load("Q", 8, q)) {
    prm.q = Eigen::Map<const VehicleEkf::Vector8d>(q.data());
  }
  if (load("P_init", 8, p_init)) {
    prm.p_init = Eigen::Map<const VehicleEkf::Vector8d>(p_init.data());
  }
  if (load("R_ndt", 6, r_ndt)) {
    prm.r_ndt = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(r_ndt.data());
  }
  node.get_parameter("gate_horizontal", prm.gate_horizontal);
  node.get_parameter("gate_1d", prm.gate_1d);
  node.get_parameter("lockout_count", prm.lockout_count);
  node.get_parameter("max_predict_dt", prm.max_predict_dt);

  VehicleOdomEkf::OdomParams & oprm = cfg.odom_params;
  if (load("R_odom", 2, r_odom)) {
    oprm.r_odom = Eigen::Map<const Eigen::Vector2d>(r_odom.data());
  }
  node.get_parameter("gate_odom", oprm.gate_odom);
  node.get_parameter("odom_timeout_init", oprm.odom_timeout_init);

  return cfg;
}

void logEkfConfig(const rclcpp::Logger & logger, const EkfConfig & cfg)
{
  const VehicleEkf::Params & prm = cfg.params;
  RCLCPP_INFO(
    logger, "ekf lidar_offset: [%lf, %lf, %lf], lidar_rotation: [%lf, %lf, %lf]",
    prm.o_x, prm.o_y, prm.o_z, prm.roll_o, prm.pitch_o, prm.yaw_o);
  RCLCPP_INFO(
    logger, "ekf model: base_link 8-state EKF [x,y,z,roll,pitch,yaw,v,omega]%s",
    cfg.model == "ndt_odom" ? " + odom observation [v,omega]" : " (odom unused)");
  RCLCPP_INFO(
    logger, "ekf gate_horizontal: %lf, gate_1d: %lf, lockout_count: %d",
    prm.gate_horizontal, prm.gate_1d, prm.lockout_count);
  RCLCPP_INFO(
    logger, "ekf max_predict_dt: %lf, use_fitness_reject: %d (threshold %lf)",
    prm.max_predict_dt, cfg.use_fitness_reject, cfg.fitness_reject_threshold);
  RCLCPP_INFO(
    logger, "ekf Q(x,y,z,roll,pitch,yaw,v,omega): [%g, %g, %g, %g, %g, %g, %g, %g]",
    prm.q(0), prm.q(1), prm.q(2), prm.q(3), prm.q(4), prm.q(5), prm.q(6), prm.q(7));
  RCLCPP_INFO(
    logger, "ekf R_ndt: [%g, %g, %g, %g, %g, %g]",
    prm.r_ndt(0), prm.r_ndt(1), prm.r_ndt(2), prm.r_ndt(3), prm.r_ndt(4), prm.r_ndt(5));
  RCLCPP_INFO(
    logger, "ekf P_init(x,y,z,roll,pitch,yaw,v,omega): [%g, %g, %g, %g, %g, %g, %g, %g]",
    prm.p_init(0), prm.p_init(1), prm.p_init(2), prm.p_init(3), prm.p_init(4), prm.p_init(5),
    prm.p_init(6), prm.p_init(7));
  if (cfg.model == "ndt_odom") {
    const VehicleOdomEkf::OdomParams & oprm = cfg.odom_params;
    RCLCPP_INFO(
      logger, "ekf R_odom: [%g, %g] (source: %s), gate_odom: %lf, odom_timeout_init: %lf",
      oprm.r_odom(0), oprm.r_odom(1), cfg.odom_covariance_source.c_str(),
      oprm.gate_odom, oprm.odom_timeout_init);
  }
}

std::unique_ptr<VehicleEkf> createEkf(const EkfConfig & cfg)
{
  std::unique_ptr<VehicleEkf> ekf;
  if (cfg.model == "ndt_odom") {
    auto odom_ekf = std::make_unique<VehicleOdomEkf>();
    odom_ekf->setOdomParams(cfg.odom_params);
    ekf = std::move(odom_ekf);
  } else {
    ekf = std::make_unique<VehicleEkf>();
  }
  ekf->setParams(cfg.params);
  ekf->reset();
  return ekf;
}

}  // namespace pcl_localization
