#include <mutex>
#include <memory>
#include <iostream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/ScanMatchingStatus.h>

namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet() {}
  virtual ~HdlLocalizationNodelet() {
  }


  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    processing_time.resize(16);
    initialize_params();

    robot_odom_frame_id = private_nh.param<std::string>("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = private_nh.param<std::string>("odom_child_frame_id", "base_link");

    use_imu = private_nh.param<bool>("use_imu", true);
    invert_imu = private_nh.param<bool>("invert_imu", false);
    if(use_imu) {
      NODELET_INFO("enable imu-based prediction");
      imu_sub = mt_nh.subscribe("/gpsimu_driver/imu_data", 256, &HdlLocalizationNodelet::imu_callback, this);
    }
    points_sub = mt_nh.subscribe("/velodyne_points", 5, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/globalmap", 2, &HdlLocalizationNodelet::globalmap_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &HdlLocalizationNodelet::initialpose_callback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
    status_pub = nh.advertise<ScanMatchingStatus>("/status", 5, false);
  }

private:
  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    std::string reg_method = private_nh.param<std::string>("reg_method", "NDT_OMP");
    double voxel_resolution = private_nh.param<double>("voxel_resolution", 1.0);
    double voxel_search_radius = private_nh.param<double>("voxel_search_radius", 3.0);
    std::string neighbor_search_method = private_nh.param<std::string>("neighbor_search_method", "DIRECT7");
    std::string covariance_estimation_method = private_nh.param<std::string>("covariance_estimation_method", "GPU_RBF_KERNEL");

    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());


    if(reg_method == "NDT_OMP") {
      NODELET_INFO_STREAM("NDT_OMP is selected : resolution(" << voxel_resolution << ")");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(voxel_resolution);

      if(neighbor_search_method == "DIRECT1") {
        NODELET_INFO_STREAM("use DIRECT1 search method");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if(neighbor_search_method == "DIRECT7") {
        NODELET_INFO_STREAM("use DIRECT7 search method");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else if(neighbor_search_method == "KDTREE") {
        NODELET_INFO_STREAM("use KDTREE search method");
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      } else {
        NODELET_INFO_STREAM("unimplemented search method " << neighbor_search_method);
      }

      registration = ndt;
    } else if (reg_method == "FAST_VGICP") {
      NODELET_INFO_STREAM("FAST_VGICP is selected : resolution(" << voxel_resolution << ")");
      pcl::shared_ptr<fast_gicp::FastVGICP<PointT, PointT>> vgicp(new fast_gicp::FastVGICP<PointT, PointT>());
      vgicp->setResolution(voxel_resolution);

      if(neighbor_search_method == "DIRECT1") {
        NODELET_INFO_STREAM("use DIRECT1 search method");
        vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
      } else if(neighbor_search_method == "DIRECT7") {
        NODELET_INFO_STREAM("use DIRECT7 search method");
        vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
      } else {
        NODELET_INFO_STREAM("unimplemented search method " << neighbor_search_method);
      }

      registration = vgicp;
    }
    // #ifdef USE_VGICP_CUDA
    else if(reg_method == "FAST_VGICP_CUDA") {
      NODELET_INFO_STREAM("FAST_VGICP_CUDA is selected : resolution(" << voxel_resolution << ")");
      pcl::shared_ptr<fast_gicp::FastVGICPCuda<PointT, PointT>> vgicp(new fast_gicp::FastVGICPCuda<PointT, PointT>());
      vgicp->setResolution(voxel_resolution);

      if(neighbor_search_method == "DIRECT1") {
        NODELET_INFO_STREAM("use DIRECT1 search method");
        vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
      } else if(neighbor_search_method == "DIRECT7") {
        NODELET_INFO_STREAM("use DIRECT7 search method");
        vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
      } else if(neighbor_search_method == "DIRECT_RADIUS") {
        NODELET_INFO_STREAM("use DIRECT_RADIUS search method");
        vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, 3.0);
      } else {
        NODELET_INFO_STREAM("unimplemented search method " << neighbor_search_method);
      }

      if(covariance_estimation_method == "CPU_PARALLEL_KDTREE") {
        NODELET_INFO_STREAM("use CPU_PARALLEL_KDTREE covariance estimation");
        vgicp->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::CPU_PARALLEL_KDTREE);
      } else if(covariance_estimation_method == "GPU_BRUTEFORCE") {
        NODELET_INFO_STREAM("use CPU_PARALLEL_KDTREE covariance estimation");
        vgicp->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_BRUTEFORCE);
      } else if(covariance_estimation_method == "GPU_RBF_KERNEL") {
        NODELET_INFO_STREAM("use CPU_PARALLEL_KDTREE covariance estimation");
        vgicp->setNearestNeighborSearchMethod(fast_gicp::NearestNeighborMethod::GPU_RBF_KERNEL);
      }

      registration = vgicp;
    }
    // #endif

    // initialize pose estimator
    if(private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(registration,
        ros::Time::now(),
        Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0), private_nh.param<double>("init_ori_x", 0.0), private_nh.param<double>("init_ori_y", 0.0), private_nh.param<double>("init_ori_z", 0.0)),
        private_nh.param<double>("cool_time_duration", 0.5)
      ));
    }
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      NODELET_ERROR("waiting for initial pose input!!");
      return;
    }

    if(!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if(pcl_cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if(!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, this->tf_listener)) {
        NODELET_ERROR("point cloud cannot be transformed into target frame!!");
        return;
    }

    auto filtered = downsample(cloud);

    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(stamp < (*imu_iter)->header.stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double gyro_sign = invert_imu ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    ros::Time last_correction_time = pose_estimator->last_correction_time();
    if(private_nh.param<bool>("enable_robot_odometry_prediction", false) && !last_correction_time.isZero()) {
      tf::StampedTransform transform;
      if(tf_listener.waitForTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, last_correction_time, robot_odom_frame_id, ros::Duration(0))) {
        tf_listener.lookupTransform(cloud->header.frame_id, stamp, cloud->header.frame_id, last_correction_time, robot_odom_frame_id, transform);
      } else if(tf_listener.waitForTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, last_correction_time, robot_odom_frame_id, ros::Duration(0))) {
        tf_listener.lookupTransform(cloud->header.frame_id, ros::Time(0), cloud->header.frame_id, last_correction_time, robot_odom_frame_id, transform);
      }

      if(transform.stamp_.isZero()) {
        NODELET_WARN_STREAM("failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        const auto& origin = transform.getOrigin();
        const auto& rotation = transform.getRotation();

        Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
        delta.block<3, 3>(0, 0) = Eigen::Quaternionf(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
        delta.block<3, 1>(0, 3) = Eigen::Vector3f(origin.x(), origin.y(), origin.z());

        pose_estimator->predict_odom(delta);
      }
    }

    // correct
    auto t1 = ros::WallTime::now();
    auto aligned = pose_estimator->correct(stamp, filtered);
    auto t2 = ros::WallTime::now();

    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();
    // NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0 << "[msec]");

    if(aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }

    if(status_pub.getNumSubscribers()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);

    double downsample_resolution = private_nh.param<double>("globalmap_downsample_resolution", 0.5);
    if(downsample_resolution < 1e-3) {
      globalmap = cloud;
    } else {
      pcl::ApproximateVoxelGrid<PointT> voxelgrid;
      voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      voxelgrid.setInputCloud(cloud);

      globalmap.reset(new pcl::PointCloud<PointT>);
      voxelgrid.filter(*globalmap);
    }

    registration->setInputTarget(globalmap);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            registration,
            ros::Time::now(),
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            private_nh.param<double>("cool_time_duration", 0.5))
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
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", odom_child_frame_id);
    pose_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }

  /**
   * @brief publish scan matching status information
   */
  void publish_scan_matching_status(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for(int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();
    status.relative_pose = matrix2pose(registration->getFinalTransformation());

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    if(pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(matrix2pose(pose_estimator->wo_prediction_error().get()));
    }

    if(pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "imu";
      status.prediction_errors.push_back(matrix2pose(pose_estimator->imu_prediction_error().get()));
    }

    if(pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(matrix2pose(pose_estimator->odom_prediction_error().get()));
    }

    status_pub.publish(status);
  }

  /**
   * @brief convert an Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

  /**
   * @brief convert an Eigen::Matrix to geometry_msgs::Pose
   */
  geometry_msgs::Pose matrix2pose(const Eigen::Matrix4f& mat) {
    Eigen::Quaternionf quat(mat.block<3, 3>(0, 0));
    Eigen::Vector3f trans = mat.block<3, 1>(0, 3);

    geometry_msgs::Pose pose;
    pose.position.x = trans.x();
    pose.position.y = trans.y();
    pose.position.z = trans.z();

    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();

    return pose;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;

  bool use_imu;
  bool invert_imu;
  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher status_pub;
  tf::TransformBroadcaster pose_broadcaster;
  tf::TransformListener tf_listener;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // processing time buffer
  boost::circular_buffer<double> processing_time;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
