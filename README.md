# pcl_localization_ros2
A ROS2 package of 3D LIDAR-based Localization using PCL.

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

## Requirements

- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2.git)

## IO
- input  
/velodyne_points  (sensor_msgs/PointCloud2)(best effort, depth 1)  
/map  (sensor_msgs/PointCloud2)(a map from a topic instead of `map_path`; reliable, transient local)  
/initialpose (geometry_msgs/PoseWithCovarianceStamped)(always subscribed; the LiDAR pose in `global_frame_id`. The initial pose of `set_initial_pose` is given in the same way at start-up)  
/imu  (sensor_msgs/Imu)(when `use_imu` is true)  
TF `global_frame_id` -> the cloud frame (LiDAR) at the scan time (when `use_tf_init_guess` is true; e.g. map -> base_link from [ekf_localizer](https://github.com/MaeharaTakumi/ekf_localizer) and the static mount)  
TF `base_frame_id` -> LiDAR (static, used to convert the NDT result to `base_frame_id`)  

- output  
/pcl_pose (geometry_msgs/PoseWithCovarianceStamped)(reliable, transient local; map -> LiDAR, or map -> `base_frame_id` when `pose_output_frame` is "base_link")  
/ndt_pose (geometry_msgs/PoseWithCovarianceStamped)(reliable, depth 10; map -> LiDAR, stamped with the scan time, covariance 0; input of ekf_localizer)  
/localization_debug (std_msgs/Float32MultiArray)(when `enable_debug_topic` is true; [points, align time s, converged, fitness, angle_diff deg])  
(/path (nav_msgs/Path) is commented out in the source since 2026-11-11: nothing subscribed to it, and the whole growing path was published for every scan)  
TF map -> `base_frame_id` (when `publish_tf` is true)  
/initial_map (sensor_msgs/PointCloud2)(when `use_pcd_map` is true; down sampled by `map_viz_leaf_size`)  

## params

The values below are the defaults in the code. This package has no parameter file (`param/localization.yaml` was removed on 2026-11-11);
the values for each environment are in `param/<env>/pcl_localization.yaml` of [localization_bringup](https://github.com/MaeharaTakumi/localization_bringup),
and parameters not written there take these defaults.

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT_OMP"|"NDT" or "GICP" of "NDT_OMP" of "GICP_OMP"|
|score_threshold|double|0.2|registration score threshold (a warning is printed when the fitness score exceeds it)|
|ndt_resolution|double|1.0|resolution size of voxels[m]|
|ndt_step_size|double|0.1|step_size maximum step length[m]|
|ndt_num_threads|int|0|threads using NDT_OMP(if `0` is set, maximum alloawble threads are used.)|
|transform_epsilon|double|0.01|transform epsilon to stop iteration in registration|
|voxel_leaf_size|double|0.1|down sample size of input cloud[m]|
|map_viz_leaf_size|double|0.1|down sample size of the map published for RViz[m] (the map for NDT is not down sampled)|
|scan_max_range|double|100.0|max range of input cloud[m]|
|scan_min_range|double|0.1|min range of input cloud[m]|
|scan_period|double|0.05|scan period of input cloud[sec] (not used)|
|use_pcd_map|bool|true|whether pcd_map is used or not|
|map_path|string|""|pcd_map file path|
|set_initial_pose|bool|true|whether or not to set the initial pose below|
|initial_pose_x|double|0.0|x-coordinate of the initial pose value[m] (LiDAR pose)|
|initial_pose_y|double|0.0|y-coordinate of the initial pose value[m]|
|initial_pose_z|double|0.0|z-coordinate of the initial pose value[m]|
|initial_pose_qx|double|0.0|Quaternion x of the initial pose value|
|initial_pose_qy|double|0.0|Quaternion y of the initial pose value|
|initial_pose_qz|double|0.0|Quaternion z of the initial pose value|
|initial_pose_qw|double|1.0|Quaternion w of the initial pose value|
|use_imu|bool|false|whether 9-axis imu is used or not for point cloud distortion correction|
|enable_debug|bool|false|whether debug is done or not|
|enable_debug_topic|bool|true|whether to publish /localization_debug ([points, align time, converged, fitness, angle_diff])|
|global_frame_id / odom_frame_id / base_frame_id|string|map / odom / base_link|frames (odom_frame_id is not used)|
|pose_output_frame|string|"lidar"|frame of /pcl_pose: "lidar" (the NDT result) or "base_link"|
|publish_tf|bool|true|whether to publish TF map -> base_link (set false when ekf_localizer publishes it)|
|use_tf_init_guess|bool|false|whether to take the initial guess from TF map -> LiDAR at the scan time|
|tf_init_guess_timeout|double|0.05|time to wait for the TF above[sec]|
|use_fitness_reject|bool|false|whether to drop results whose fitness score exceeds `fitness_reject_threshold` from /ndt_pose|
|fitness_reject_threshold|double|1.0|threshold for `use_fitness_reject`|
|stamp_mode|string|"stamp"|time used for the outputs (`/ndt_pose` stamp, TF initial guess): `stamp`: the cloud stamp, `receipt`: receipt time on this PC, `auto`: the receipt time only when \|receipt time − stamp\| ≥ `stamp_auto_threshold` (decided for each cloud). The stamp of the point cloud topic itself is not changed. Set in `param/<env>/pcl_localization.yaml` of localization_bringup (the odometry time is set by `odom_stamp_mode` of ekf_localizer)|
|stamp_auto_threshold|double|0.1|threshold for `auto` [sec]|

## demo

demo data(ROS1) by Tier IV  
https://data.tier4.jp/rosbag_details/?id=212  
To use ros1 rosbag , use [rosbags](https://pypi.org/project/rosbags/).  
The Velodyne VLP-16 was used in this data.

Before running, put `bin_tc-2017-10-15-ndmap.pcd` into your `map` directory and  
pass it with `map_path_arg:=<path>`. The launch also starts RViz with `rviz/localization2.rviz` (change it with `rviz_config:=<path>`)
and publishes the static TF base_link -> velodyne (x −0.17 m, z 1.2 m, yaw −0.017 rad) and base_link -> imu_link.
`localization2.rviz` shows `/current_pose` (red arrow: the NDT result as the LiDAR pose with `pose_output_frame: lidar`), the TF frames (TF display), `/velodyne_points` and `/initial_map`.
```
ros2 launch pcl_localization_ros2 pcl_localization.launch.py map_path_arg:=<path>/bin_tc-2017-10-15-ndmap.pcd
ros2 bag play tc_2017-10-15-15-34-02_free_download/
```

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)
