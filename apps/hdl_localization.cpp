#include <mutex>
#include <memory>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>


#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include "hdl_localization/msg/scan_matching_status.hpp"
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace hdl_localization {

class HdlLocalization : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalization(const rclcpp::NodeOptions & options) : Node("hdl_localization", options){
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    initialize_params();

    this->declare_parameter("robot_odom_frame_id", "robot_odom");
    this->declare_parameter("odom_child_frame_id", "base_link");
    this->declare_parameter("use_imu", true);
    this->declare_parameter("invert_acc", false);
    this->declare_parameter("invert_gyro", false);
    this->declare_parameter("use_global_localization", true);
    this->declare_parameter("enable_robot_odometry_prediction", false);
    this->declare_parameter("status_max_correspondence_dist", 0.5);
    this->declare_parameter("status_max_valid_point_dist", 25.0);

    robot_odom_frame_id = this->get_parameter("robot_odom_frame_id").as_string();
    odom_child_frame_id = this->get_parameter("odom_child_frame_id").as_string();

    use_imu = this->get_parameter("use_imu").as_bool();
    invert_acc = this->get_parameter("invert_acc").as_bool();
    invert_gyro = this->get_parameter("invert_gyro").as_bool();
    if (use_imu) {
      RCLCPP_INFO(this->get_logger(), "enable imu-based prediction");
      imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/gpsimu_driver/imu_data", 256, std::bind(&HdlLocalization::imu_callback, this, _1));
    }
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", rclcpp::SensorDataQoS(), std::bind(&HdlLocalization::points_callback, this, _1));
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    globalmap_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/globalmap", map_qos, std::bind(&HdlLocalization::globalmap_callback, this, _1));
    initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, std::bind(&HdlLocalization::initialpose_callback, this, _1));

    pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
    aligned_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
    status_pub = this->create_publisher<hdl_localization::msg::ScanMatchingStatus>("/status", 5);

    // global localization
    use_global_localization = this->get_parameter("use_global_localization").as_bool();
    if(use_global_localization) {
      RCLCPP_INFO_STREAM(this->get_logger(), "wait for global localization services");
      set_global_map_service = this->create_client<hdl_global_localization::srv::SetGlobalMap>("/hdl_global_localization/set_global_map");
      query_global_localization_service = this->create_client<hdl_global_localization::srv::QueryGlobalLocalization>("/hdl_global_localization/query");
      while (!set_global_map_service->wait_for_service(1s));
      while (!query_global_localization_service->wait_for_service(1s));

      relocalize_server = this->create_service<std_srvs::srv::Empty>("/relocalize", std::bind(&HdlLocalization::relocalize, this, _1, _2));
    }
  }
  virtual ~HdlLocalization() {
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    std::string reg_method = this->get_parameter("reg_method").as_string();
    std::string ndt_neighbor_search_method = this->get_parameter("ndt_neighbor_search_method").as_string();
    double ndt_neighbor_search_radius = this->get_parameter("ndt_neighbor_search_radius").as_double();
    double ndt_resolution = this->get_parameter("ndt_resolution").as_double();

    if(reg_method == "NDT_OMP") {
      RCLCPP_INFO(this->get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(this->get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(this->get_logger(), "invalid search method was given");
          RCLCPP_WARN(this->get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    } else if(reg_method.find("NDT_CUDA") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "NDT_CUDA is selected");
      std::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(new fast_gicp::NDTCuda<PointT, PointT>);
      ndt->setResolution(ndt_resolution);

      if(reg_method.find("D2D") != std::string::npos) {
        ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
      } else if (reg_method.find("P2D") != std::string::npos) {
        ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
      }

      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
      } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
        RCLCPP_INFO_STREAM(this->get_logger(), "search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
      } else {
        RCLCPP_WARN(this->get_logger(), "invalid search method was given");
      }
      return ndt;
    }

    RCLCPP_ERROR_STREAM(this->get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    // intialize scan matching method
    this->declare_parameter("reg_method", "NDT_OMP");
    this->declare_parameter("ndt_neighbor_search_method", "DIRECT7");
    this->declare_parameter("ndt_neighbor_search_radius", 2.0);
    this->declare_parameter("ndt_resolution", 1.0);
    this->declare_parameter("downsample_resolution", 0.1);
    this->declare_parameter("specify_init_pose", true);
    this->declare_parameter("init_pos_x", 0.0);
    this->declare_parameter("init_pos_y", 0.0);
    this->declare_parameter("init_pos_z", 0.0);
    this->declare_parameter("init_ori_w", 1.0);
    this->declare_parameter("init_ori_x", 0.0);
    this->declare_parameter("init_ori_y", 0.0);
    this->declare_parameter("init_ori_z", 0.0);
    this->declare_parameter("cool_time_duration", 0.5);

    double downsample_resolution = this->get_parameter("downsample_resolution").as_double();
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(this->get_logger(), "create registration method for localization");
    registration = create_registration();

    // global localization
    RCLCPP_INFO(this->get_logger(), "create registration method for fallback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    if(this->get_parameter("specify_init_pose").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(registration,
        Eigen::Vector3f(this->get_parameter("init_pos_x").as_double(), this->get_parameter("init_pos_y").as_double(), this->get_parameter("init_pos_z").as_double()),
        Eigen::Quaternionf(this->get_parameter("init_ori_w").as_double(), this->get_parameter("init_ori_x").as_double(), this->get_parameter("init_ori_y").as_double(), this->get_parameter("init_ori_z").as_double()),
        this->get_parameter("cool_time_duration").as_double()
      ));
    }
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::msg::Imu & imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::msg::PointCloud2 & points_msg) {
    if(!globalmap) {
      RCLCPP_ERROR(this->get_logger(), "globalmap has not been received!!");
      return;
    }

    const auto& stamp = rclcpp::Time(points_msg.header.stamp);
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(points_msg, *pcl_cloud);

    if(pcl_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    std::string tfError;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if(tf_buffer->canTransform(odom_child_frame_id, pcl_cloud->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.1), &tfError))
    {
        if(!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, *tf_buffer)) {
            RCLCPP_ERROR(this->get_logger(), "point cloud cannot be transformed into target frame!!");
            return;
        }
    }else
    {
        RCLCPP_ERROR(this->get_logger(), tfError.c_str());
        return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;

    if(relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      RCLCPP_ERROR(this->get_logger(), "waiting for initial pose input!!");
      return;
    }
    Eigen::Matrix4f before = pose_estimator->matrix();

    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        rclcpp::Time imu_stamp = rclcpp::Time(imu_iter->header.stamp);
        if(stamp < imu_stamp) {
          break;
        }
        const auto& acc = imu_iter->linear_acceleration;
        const auto& gyro = imu_iter->angular_velocity;
        double acc_sign = invert_acc ? -1.0 : 1.0;
        double gyro_sign = invert_gyro ? -1.0 : 1.0;
        pose_estimator->predict(imu_stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
    if(this->get_parameter("enable_robot_odometry_prediction").as_bool() && last_correction_time.nanoseconds() != 0) {
      geometry_msgs::msg::TransformStamped odom_delta;
      if(tf_buffer->canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration::from_seconds(0.1))) {
        odom_delta = tf_buffer->lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration::from_seconds(0.0));
      } else if(tf_buffer->canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, rclcpp::Time(0), robot_odom_frame_id, rclcpp::Duration::from_seconds(0.0))) {
        odom_delta = tf_buffer->lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, rclcpp::Time(0), robot_odom_frame_id, rclcpp::Duration::from_seconds(0.0));
      }

      if(rclcpp::Time(odom_delta.header.stamp).nanoseconds() == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // correct
    auto aligned = pose_estimator->correct(stamp, filtered);

    if(aligned_pub->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header.frame_id = "map";
      aligned_msg.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);
      aligned_pub->publish(aligned_msg);
    }

    if(status_pub->get_subscription_count()) {
      publish_scan_matching_status(points_msg.header, aligned);
    }

    publish_odometry(points_msg.header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::msg::PointCloud2 & points_msg) {
    RCLCPP_INFO(this->get_logger(), "globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    if(use_global_localization) {
      RCLCPP_INFO(this->get_logger(), "set globalmap for global localization!");
      auto request = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
      pcl::toROSMsg(*globalmap, request->global_map);

      while (!set_global_map_service->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }

      auto result = set_global_map_service->async_send_request(request, std::bind(&HdlLocalization::set_global_map_response_callback, this, _1));
    }
  }

  void set_global_map_response_callback(rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedFuture future) {
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready){
      RCLCPP_INFO(this->get_logger(), "done");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set global map");
    }
  }
  
  /**
   * @brief perform global localization to relocalize the sensor position
   * @param
   */
  void relocalize(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    if(last_scan == nullptr) {
      RCLCPP_INFO_STREAM(this->get_logger(), "no scan has been received");
      return;
    }

    relocalizing = true;
    delta_estimater->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

    auto srv = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, srv->cloud);
    srv->max_num_candidates = 1;

    while (!query_global_localization_service->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto resp = query_global_localization_service->async_send_request(srv, std::bind(&HdlLocalization::query_global_localization_callback, this, _1));
  }

  void query_global_localization_callback(rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedFuture future) {
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      if (future.get()->poses.empty()) {
        relocalizing = false;
        RCLCPP_INFO_STREAM(this->get_logger(), "global localization failed");
        return;
      }

      const auto& result = future.get()->poses[0];

      RCLCPP_INFO_STREAM(this->get_logger(), "--- Global localization result ---");
      RCLCPP_INFO_STREAM(this->get_logger(), "Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
      RCLCPP_INFO_STREAM(this->get_logger(), "Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
      RCLCPP_INFO_STREAM(this->get_logger(), "Error :" << future.get()->errors[0]);
      RCLCPP_INFO_STREAM(this->get_logger(), "Inlier:" << future.get()->inlier_fractions[0]);

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
      pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
      pose = pose * delta_estimater->estimated_delta();

      std::lock_guard<std::mutex> lock(pose_estimator_mutex);
      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        pose.translation(),
        Eigen::Quaternionf(pose.linear()),
        this->get_parameter("cool_time_duration").as_double()));

      relocalizing = false;
    } else {
      relocalizing = false;
      RCLCPP_INFO_STREAM(this->get_logger(), "global localization failed");
      return;
    }

  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg) {
    RCLCPP_INFO(this->get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg.pose.pose.position;
    const auto& q = pose_msg.pose.pose.orientation;
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            registration,
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            this->get_parameter("cool_time_duration").as_double())
    );
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    if(tf_buffer->canTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time(0))) {
      geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = odom_child_frame_id;
      map_wrt_frame.child_frame_id = "map";

      geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer->lookupTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::msg::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.transform = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = robot_odom_frame_id;

      tf_broadcaster->sendTransform(odom_trans);
    } else {
      geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = "map";
      odom_trans.child_frame_id = odom_child_frame_id;
      tf_broadcaster->sendTransform(odom_trans);
    }

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub->publish(odom);
  }

  /**
   * @brief publish scan matching status information
   */
  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    hdl_localization::msg::ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = 0.0;

    const double max_correspondence_dist = this->get_parameter("status_max_correspondence_dist").as_double();
    const double max_valid_point_dist = this->get_parameter("status_max_valid_point_dist").as_double();

    int num_inliers = 0;
    int num_valid_points = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      if (pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }
      num_valid_points++;

      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }

    status.matching_error /= num_inliers;
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    std::vector<double> errors(6, 0.0);

    if(pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }

    if(pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }

    if(pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub->publish(status);
  }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_pub;


  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // global localization
  bool use_global_localization;
  std::atomic_bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server;
  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_service;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_service;
};
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalization)
