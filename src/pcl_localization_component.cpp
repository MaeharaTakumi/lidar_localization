#include <pcl_localization/pcl_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pcl_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("enable_debug_topic", false);
  declare_parameter("ndt_num_threads", 0);

  // 点群の時刻の出どころ："receipt"（この PC の受信時刻）or "message"（ヘッダ stamp）
  declare_parameter("scan_time_source", "receipt");

  // /pcl_pose (→/current_pose) の基準："base_link"（車両中心）or "lidar"（LiDAR 中心）
  declare_parameter("pose_output_frame", "base_link");

  // RViz 専用 TF（/tf_rviz）を点群ヘッダの時刻で配信するか
  declare_parameter("publish_rviz_tf", true);

  // EKF（ekf_localizer）との連携
  declare_parameter("publish_tf", true);
  declare_parameter("use_tf_init_guess", false);
  declare_parameter("tf_init_guess_timeout", 0.05);
  declare_parameter("use_fitness_reject", false);
  declare_parameter("fitness_reject_threshold", 1.0);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();
  has_mount_ = false;

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();
  debug_pub_->on_activate();
  rviz_tf_pub_->on_activate();
  ndt_pose_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  debug_pub_->on_deactivate();
  rviz_tf_pub_->on_deactivate();
  ndt_pose_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  debug_pub_.reset();
  rviz_tf_pub_.reset();
  ndt_pose_pub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("enable_debug_topic", enable_debug_topic_);
  get_parameter("publish_tf", publish_tf_);
  get_parameter("use_tf_init_guess", use_tf_init_guess_);
  get_parameter("tf_init_guess_timeout", tf_init_guess_timeout_);
  get_parameter("use_fitness_reject", use_fitness_reject_);
  get_parameter("fitness_reject_threshold", fitness_reject_threshold_);
  get_parameter("pose_output_frame", pose_output_frame_);
  get_parameter("publish_rviz_tf", publish_rviz_tf_);
  if (pose_output_frame_ != "base_link" && pose_output_frame_ != "lidar") {
    RCLCPP_WARN(
      get_logger(), "Unknown pose_output_frame \"%s\". Falling back to \"base_link\".",
      pose_output_frame_.c_str());
    pose_output_frame_ = "base_link";
  }
  get_parameter("scan_time_source", scan_time_source_);
  if (scan_time_source_ != "receipt" && scan_time_source_ != "message") {
    RCLCPP_WARN(
      get_logger(), "Unknown scan_time_source \"%s\". Falling back to \"receipt\".",
      scan_time_source_.c_str());
    scan_time_source_ = "receipt";
  }

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(),"enable_debug_topic: %d", enable_debug_topic_);
  RCLCPP_INFO(get_logger(),"publish_tf: %d", publish_tf_);
  RCLCPP_INFO(get_logger(),"use_tf_init_guess: %d (timeout %lf s)", use_tf_init_guess_, tf_init_guess_timeout_);
  RCLCPP_INFO(get_logger(),"use_fitness_reject: %d (threshold %lf)", use_fitness_reject_, fitness_reject_threshold_);
  RCLCPP_INFO(get_logger(),"scan_time_source: %s", scan_time_source_.c_str());
  RCLCPP_INFO(get_logger(),"pose_output_frame: %s", pose_output_frame_.c_str());
  RCLCPP_INFO(get_logger(),"publish_rviz_tf: %d", publish_rviz_tf_);

}

bool PCLLocalization::lookupMount(const std::string & lidar_frame, Eigen::Affine3d & T_bl)
{
  if (!has_mount_) {
    try {
      const geometry_msgs::msg::TransformStamped tf =
        tfbuffer_.lookupTransform(base_frame_id_, lidar_frame, tf2::TimePointZero);
      mount_ = tf2::transformToEigen(tf);
      has_mount_ = true;
      RCLCPP_INFO(
        get_logger(), "mount %s -> %s (from TF): t=[%.3f, %.3f, %.3f]",
        base_frame_id_.c_str(), lidar_frame.c_str(),
        mount_.translation().x(), mount_.translation().y(), mount_.translation().z());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No TF %s -> %s (%s). TF, path and base-frame pose are not published.",
        base_frame_id_.c_str(), lidar_frame.c_str(), ex.what());
      return false;
    }
  }
  T_bl = mount_;
  return true;
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // QoS は tf2_ros::TransformListener の購読（KeepLast(100)・reliable）に合わせる
  rviz_tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>(
    "tf_rviz", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());

  debug_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "localization_debug",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // EKF が 1 つも落とさないよう reliable・深さ 10
  ndt_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ndt_pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  // KeepLast(1)：キューが飽和して古いスキャンを処理するのを避ける。
  // SensorDataQoS は best effort なので、深さだけ変えると publisher と接続できなくなる。
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  // EKF も initialpose でリセットされ、次の NDT 観測で初期化し直す。それまでの TF は古い姿勢なので、
  // しばらくは初期値に使わない（与えた姿勢・NDT の前回の解を使う）
  tf_init_guess_resume_time_ = now() + rclcpp::Duration::from_seconds(1.0);
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  cloudReceived(last_scan_ptr_);
  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!map_recieved_ || !initialpose_recieved_) {return;}

  // ---- 時刻 ----
  // 点群の stamp は別 PC（時刻同期なし）の時計なので、既定ではこの PC の受信時刻を使う。
  // scan_time は出力（pcl_pose / TF / path / ndt_pose）の stamp に使う（ndt_pose は EKF の観測時刻）。
  const rclcpp::Time t_receipt = now();
  const rclcpp::Time scan_time =
    (scan_time_source_ == "receipt") ? t_receipt : rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
  {
    const double offset = t_receipt.seconds() - rclcpp::Time(msg->header.stamp).seconds();
    if (std::abs(offset) > 1.0) {
      RCLCPP_WARN_ONCE(
        get_logger(), "Point cloud stamp differs from this PC's clock by %.3f s (%s).",
        offset, scan_time_source_ == "receipt" ?
        "using receipt time" : "scan_time_source is \"message\": outputs will be stamped wrong");
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (use_imu_) {
    lidar_undistortion_.adjustDistortion(cloud_ptr, scan_time.seconds());
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto & p : filtered_cloud_ptr->points) {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  Eigen::Matrix4f init_guess;
  bool init_guess_from_tf = false;
  if (use_tf_init_guess_ && scan_time > tf_init_guess_resume_time_) {
    // 閉ループ：EKF が配信する TF（map → base → LiDAR）を点群の取得時刻で引いて初期値にする。
    // 取得時刻は過去なので、TF は補間で求まる（EKF が届くまでの待ちは tf_init_guess_timeout まで）
    try {
      const geometry_msgs::msg::TransformStamped tf = tfbuffer_.lookupTransform(
        global_frame_id_, msg->header.frame_id, scan_time,
        rclcpp::Duration::from_seconds(tf_init_guess_timeout_));
      init_guess = tf2::transformToEigen(tf).matrix().cast<float>();
      init_guess_from_tf = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No TF for init_guess (%s). Using the last pose.",
        ex.what());
    }
  }
  if (!init_guess_from_tf) {
    Eigen::Affine3d affine;
    tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
    init_guess = affine.matrix().cast<float>();
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, init_guess);
  rclcpp::Time time_align_end = system_clock.now();

  bool has_converged = registration_->hasConverged();
  double fitness_score = registration_->getFitnessScore();
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }
  if (fitness_score > score_threshold_) {
    RCLCPP_WARN(get_logger(), "The fitness score is over %lf.", score_threshold_);
  }

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  // NDT の解を内部状態として保持する（次スキャンの init_guess 兼 EKF の観測）
  corrent_pose_with_cov_stamped_ptr_->header.stamp = scan_time;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(final_transformation(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(final_transformation(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(final_transformation(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;

  last_scan_ptr_ = msg;

  // ---- EKF へ：NDT の解（map → LiDAR、stamp は点群の取得時刻）を配信する ----
  // fitness による全体拒否：NDT 自体が破綻しているとみなし、EKF に渡さない
  if (use_fitness_reject_ && fitness_score > fitness_reject_threshold_) {
    RCLCPP_WARN(
      get_logger(), "NDT result rejected by fitness (%lf > %lf). /ndt_pose not published.",
      fitness_score, fitness_reject_threshold_);
  } else {
    ndt_pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
  }

  // ---- 出力（NDT の解）----
  // /pcl_pose は pose_output_frame で LiDAR 中心にできる。TF と /path は常に base_frame_id
  geometry_msgs::msg::PoseWithCovarianceStamped output_pose = *corrent_pose_with_cov_stamped_ptr_;
  if (pose_output_frame_ == "lidar") {
    pose_pub_->publish(output_pose);
  }
  // NDT の解（LiDAR 姿勢）を静的 TF の取付で base_frame_id に直す
  Eigen::Affine3d T_bl;
  const bool has_base_pose = lookupMount(msg->header.frame_id, T_bl);
  if (has_base_pose) {
    Eigen::Affine3d T_ml;
    tf2::fromMsg(output_pose.pose.pose, T_ml);
    output_pose.pose.pose = tf2::toMsg(T_ml * T_bl.inverse());
    if (pose_output_frame_ != "lidar") {
      pose_pub_->publish(output_pose);
    }
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = scan_time;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = base_frame_id_;
  transform_stamped.transform.translation.x = output_pose.pose.pose.position.x;
  transform_stamped.transform.translation.y = output_pose.pose.pose.position.y;
  transform_stamped.transform.translation.z = output_pose.pose.pose.position.z;
  transform_stamped.transform.rotation = output_pose.pose.pose.orientation;
  if (has_base_pose && publish_tf_) {
    broadcaster_.sendTransform(transform_stamped);
  }

  // RViz 用：同じ姿勢を点群ヘッダの時刻で /tf_rviz に配信する。点群の stamp は別 PC の時計で、
  // /tf（この PC の時刻）では RViz が /velodyne_points の変換を引けないため。
  // /tf と混ぜると時刻が 323 日ずれたデータが同じバッファに入るので、別トピックにしている
  if (has_base_pose && publish_rviz_tf_ && rviz_tf_pub_->is_activated()) {
    tf2_msgs::msg::TFMessage rviz_tf;
    rviz_tf.transforms.push_back(transform_stamped);
    rviz_tf.transforms.back().header.stamp = msg->header.stamp;
    rviz_tf_pub_->publish(rviz_tf);
  }

  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
  pose_stamped_ptr->header.stamp = scan_time;
  pose_stamped_ptr->header.frame_id = global_frame_id_;
  pose_stamped_ptr->pose = output_pose.pose.pose;
  if (has_base_pose) {
    path_ptr_->poses.push_back(*pose_stamped_ptr);
    path_pub_->publish(*path_ptr_);
  }

  if (enable_debug_ || enable_debug_topic_) {
    /* delta_angle check
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));

    if (enable_debug_) {
      std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
      std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() <<
        "[sec]" << std::endl;
      std::cout << "has converged: " << has_converged << std::endl;
      std::cout << "fitness score: " << fitness_score << std::endl;
      std::cout << "final transformation:" << std::endl;
      std::cout << final_transformation << std::endl;
      std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
      std::cout << "-----------------------------------------------------" << std::endl;
    }

    if (enable_debug_topic_) {
      std_msgs::msg::Float32MultiArray debug_msg;
      debug_msg.data.resize(5);
      debug_msg.data[0] = static_cast<float>(filtered_cloud_ptr->size());
      debug_msg.data[1] = static_cast<float>(time_align_end.seconds() - time_align_start.seconds());
      debug_msg.data[2] = static_cast<float>(has_converged);
      debug_msg.data[3] = static_cast<float>(fitness_score);
      debug_msg.data[4] = static_cast<float>(delta_angle * 180.0 / M_PI);
      debug_pub_->publish(debug_msg);
    }
  }
}
