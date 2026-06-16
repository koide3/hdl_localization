#include <mutex>
#include <memory>
#include <iostream>
#include <chrono>


#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace hdl_localization {

class GlobalmapServer : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;


  GlobalmapServer(const rclcpp::NodeOptions & options) : Node("globalmap_server", options) {
    initialize_params();

    // publish globalmap with "latched" publisher
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    globalmap_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", map_qos);
    map_update_sub = this->create_subscription<std_msgs::msg::String>("/map_request/pcd", 10, std::bind(&GlobalmapServer::map_update_callback, this, _1));

    globalmap_pub_timer = this->create_wall_timer(1s, std::bind(&GlobalmapServer::pub_once_cb, this));
  }
  virtual ~GlobalmapServer() {
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    this->declare_parameter("globalmap_pcd", "");
    this->declare_parameter("convert_utm_to_local", true);
    this->declare_parameter("downsample_resolution", 0.1);

    std::string globalmap_pcd = this->get_parameter("globalmap_pcd").as_string();
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    std::ifstream utm_file(globalmap_pcd + ".utm");
    if (utm_file.is_open() && this->get_parameter("convert_utm_to_local").as_bool()) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for(auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Global map offset by UTM reference coordinates (x = "
                      << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
    }

    // downsample globalmap
    double downsample_resolution = this->get_parameter("downsample_resolution").as_double();
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    pcl::toROSMsg(*globalmap, globalmap_msg);
  }

  void pub_once_cb() {
    globalmap_pub->publish(globalmap_msg);
    globalmap_pub_timer->cancel();
  }

  void map_update_callback(const std_msgs::msg::String &msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received map request, map path : "<< msg.data);
    std::string globalmap_pcd = msg.data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = this->get_parameter("downsample_resolution").as_double();
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    pcl::toROSMsg(*globalmap, globalmap_msg);
    globalmap_pub->publish(globalmap_msg);
  }

private:
  // ROS2
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;

  rclcpp::TimerBase::SharedPtr globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;
  sensor_msgs::msg::PointCloud2 globalmap_msg;
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServer)
