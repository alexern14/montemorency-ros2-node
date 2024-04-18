#include <montemorency/montemorency_pcl.h>

#include <pcl/common/eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"

using std::placeholders::_1;

typedef pcl::PointXYZ PointT;

Montemorency_Trees::Montemorency_Trees(const rclcpp::NodeOptions &options)
    : Node("montemorency_trees", options) {

  declare_parameter<std::string>("topic_pointcloud_in", "/velodyne_points");
  declare_parameter<std::string>("topic_pointcloud_out",
                                 "/velodyne_points/trees");
  declare_parameter<std::string>("topic_pointcloud_filtered_out",
                                 "/velodyne_points/filtered");
  declare_parameter<std::string>("tree_count", "/velodyne_points/tree_count");

  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out =
      get_parameter("topic_pointcloud_out").as_string();
  param_tree_count = get_parameter("tree_count").as_string();
  param_topic_pointcloud_filtered_out =
      get_parameter("topic_pointcloud_filtered_out").as_string();

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      param_topic_pointcloud_out, 2);
  tree_count_publisher =
      this->create_publisher<std_msgs::msg::Int8>(param_tree_count, 2);
  publisherFiltered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      param_topic_pointcloud_filtered_out, 2);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      param_topic_pointcloud_in, 10,
      std::bind(&Montemorency_Trees::topic_callback, this, _1));

  RCLCPP_INFO(this->get_logger(),
              "\n"
              "Node:       montemorency_trees\n"
              "Subscribes: Pointcloud2 message: %s\n"
              "Publishes:  Pointcloud2 message: %s \n"
              "Details:    No filter applied in this example.\n"
              "Running...",
              param_topic_pointcloud_in.c_str(),
              param_topic_pointcloud_out.c_str());
}

void Montemorency_Trees::topic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  unsigned int num_points = msg->width;
  RCLCPP_INFO(this->get_logger(),
              "The number of points in the input pointcloud is %i", num_points);

  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered_out(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_plane_filtered_out(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered_filtered_out(
      new pcl::PCLPointCloud2());

  // ROS2 Pointcloud2 to PCL Pointcloud2

  pcl_conversions::toPCL(*msg, *cloud2);

  // Insert your pcl object here
  // -----------------------------------

  pcl::CropBox<PointT> cropBoxFilter(true);
  pcl::PassThrough<PointT> pass;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr crop_box(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

  // Create the filtering object: downsample the dataset using a leaf size of
  // 1cm
  pcl::VoxelGrid<PointT> vg;
  pcl::PointCloud<PointT>::Ptr cloud_filtered_again(
      new pcl::PointCloud<PointT>);

  pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);

  pcl::fromPCLPointCloud2(*cloud2, *cloud);

  // Cropbox filter
  cropBoxFilter.setInputCloud(cloud);
  Eigen::Vector4f min_pt(-10.0f, -10.0f, -10.0f, 10.0f);
  Eigen::Vector4f max_pt(10.0f, 10.0f, 10.0f, 10.0f);
  // Cropbox slightly bigger then bounding box of points
  cropBoxFilter.setMin(min_pt);
  cropBoxFilter.setMax(max_pt);
  // Indices
  std::vector<int> indices;
  cropBoxFilter.filter(indices);
  // Cloud
  cropBoxFilter.filter(*crop_box);

  pass.setInputCloud(crop_box);
  //   pass.setNegative(true);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.4, 0.6);
  pass.filter(*cloud_filtered);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int nr_points = (int)cloud_filtered_again->size();

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(0.05);
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(200);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  int j = 0;
  for (const auto &cluster : cluster_indices) {

    for (const auto &idx : cluster.indices) {
      cloud_cluster->push_back((*cloud_filtered)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->size() << " data points." << std::endl;
    j++;
  }

  pcl::toPCLPointCloud2(*cloud_cluster, *cloud_filtered_out);
  pcl::toPCLPointCloud2(*cloud_filtered, *cloud_filtered_filtered_out);

  //------------------------------------

  // PCL message to ROS2 message

  sensor_msgs::msg::PointCloud2 cloud_out;
  sensor_msgs::msg::PointCloud2 cloud_out_filtered;
  std_msgs::msg::Int8 tree_count_out;

  if (!cloud_cluster->points.empty()) {
    pcl_conversions::fromPCL(*cloud_filtered_out, cloud_out);

    cloud_out.header.frame_id = msg->header.frame_id;
    cloud_out.header.stamp = msg->header.stamp;

    // Publish to ROS2 network
    publisher_->publish(cloud_out);
  }

  tree_count_out.data = int(cluster_indices.size());

  pcl_conversions::fromPCL(*cloud_filtered_filtered_out, cloud_out_filtered);

  unsigned int num_points_out = cloud_out.width;
  RCLCPP_INFO(
      this->get_logger(),
      "The number of points in the montemorency_trees_out pointcloud is %i",
      num_points_out);

  cloud_out_filtered.header.frame_id = msg->header.frame_id;
  cloud_out_filtered.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisherFiltered_->publish(cloud_out_filtered);
  tree_count_publisher->publish(tree_count_out);
}

int main(int argc, char *argv[]) {
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Montemorency_Trees>());
  rclcpp::shutdown();
  return 0;
}