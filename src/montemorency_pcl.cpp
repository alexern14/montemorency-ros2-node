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
#include <std_msgs/msg/int8.hpp>

using std::placeholders::_1;

typedef pcl::PointXYZ PointT;

Montemorency_Trees::Montemorency_Trees(const rclcpp::NodeOptions &options)
    : Node("montemorency_trees", options) {

  declare_parameter<std::string>("topic_pointcloud_in", "/velodyne_points");
  declare_parameter<std::string>("topic_pointcloud_out",
                                 "/velodyne_points/trees");
  declare_parameter<std::string>("topic_pointcloud_filtered_out",
                                 "/velodyne_points/filtered");
  declare_parameter<std::string>("topic_pointcloud_filtered_out_plane",
                                 "/velodyne_points/plane");
  declare_parameter<std::string>("tree_count", "/velodyne_points/tree_count");

  param_topic_pointcloud_in = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out =
      get_parameter("topic_pointcloud_out").as_string();
  param_tree_count = get_parameter("tree_count").as_string();
  param_topic_pointcloud_filtered_out =
      get_parameter("topic_pointcloud_filtered_out").as_string();
  param_topic_pointcloud_filtered_out_plane =
      get_parameter("topic_pointcloud_filtered_out_plane").as_string();

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      param_topic_pointcloud_out, 2);
  tree_count_publisher =
      this->create_publisher<std_msgs::msg::Int8>(param_tree_count, 2);
  publisherPlane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      param_topic_pointcloud_filtered_out_plane, 2);
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

  //   cloud_filtered = cloud;

  pcl::CropBox<PointT> cropBoxFilter(true);
  pcl::PassThrough<PointT> pass;
  //   pcl::NormalEstimation<PointT, pcl::Normal> ne;
  //   pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  //   //   pcl::SACSegmentation<pcl::PointXYZ> seg;
  //   pcl::ExtractIndices<PointT> extract;
  //   pcl::ExtractIndices<pcl::Normal> extract_normals;
  //   pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr crop_box(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

  //   pcl::PointCloud<PointT>::Ptr cloud_filteredY(new
  //   pcl::PointCloud<PointT>); pcl::PointCloud<PointT>::Ptr
  //   cloud_filteredZ(new pcl::PointCloud<PointT>);
  //   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
  //       new pcl::PointCloud<pcl::Normal>);
  //   pcl::PointCloud<PointT>::Ptr cloud_filtered2(new
  //   pcl::PointCloud<PointT>); pcl::PointCloud<pcl::Normal>::Ptr
  //   cloud_normals2(
  //       new pcl::PointCloud<pcl::Normal>);
  //   pcl::ModelCoefficients::Ptr coefficients_plane(new
  //   pcl::ModelCoefficients),
  //       coefficients_cylinder(new pcl::ModelCoefficients);
  //   pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices),
  //       inliers_cylinder(new pcl::PointIndices);

  // Basic Cloud objects
  // pcl::PointCloud<PointT>::Ptr cloud        (new pcl::PointCloud<PointT>);
  //   pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
  //   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  //   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  //   pcl::PCDReader cloud_reader;

  //   // Normal Extraction Objects
  //   pcl::ModelCoefficients::Ptr cylinder_co(new pcl::ModelCoefficients);
  //   pcl::PointIndices::Ptr cylinder_in(new pcl::PointIndices);
  //   pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  //   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
  //       new pcl::PointCloud<pcl::Normal>);
  //   pcl::PointCloud<PointT>::Ptr cylinder_cloud(new
  //   pcl::PointCloud<PointT>()); pcl::PointCloud<PointT>::Ptr
  //   all_cylinders_cloud(
  //       new pcl::PointCloud<PointT>());

  //   // Normals computation objects
  //   pcl::NormalEstimation<PointT, pcl::Normal> normals_estimator;
  //   pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
  //   pcl::ExtractIndices<PointT> cylinder_indices_extractor;
  //   pcl::ExtractIndices<pcl::Normal> cylinder_indices_extractor_temp;

  // Create the filtering object: downsample the dataset using a leaf size of
  // 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_again(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZ>);

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

  //   // Build a passthrough filter to remove spurious NaNs and scene
  //   background pass.setInputCloud(crop_box); pass.setNegative(true);
  //   pass.setFilterFieldName("x");
  //   pass.setFilterLimits(-1.4, -0.8);
  //   pass.filter(*cloud_filtered);
  //   std::cerr << "PointCloud after filtering has: " << cloud_filtered->size()
  //             << " data points." << std::endl;

  pass.setInputCloud(crop_box);
  //   pass.setNegative(true);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.4, 0.6);
  pass.filter(*cloud_filtered);

  //   pass.setInputCloud(cloud_filteredY);
  // //   pass.setNegative(true);
  //   pass.setFilterFieldName("z");
  //   pass.setFilterLimits(-2.0, 2.0);
  //   pass.filter(*cloud_filteredZ);

  // ********************************************

  //   vg.setInputCloud(cloud_filtered);
  //   vg.setLeafSize(0.01f, 0.01f, 0.01f);
  //   vg.filter(*cloud_filtered_again);
  //   std::cout << "PointCloud after filtering has: "
  //             << cloud_filtered_again->size() << " data points." <<
  //             std::endl; //*

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);

  int nr_points = (int)cloud_filtered_again->size();
  //   while (cloud_filtered_again->size() > 0.3 * nr_points) {
  //     // Segment the largest planar component from the remaining cloud
  //     seg.setInputCloud(cloud_filtered_again);
  //     seg.segment(*inliers, *coefficients);
  //     if (inliers->indices.size() == 0) {
  //       std::cout << "Could not estimate a planar model for the given
  //       dataset."
  //                 << std::endl;
  //       break;
  //     }

  //     // Extract the planar inliers from the input cloud
  //     pcl::ExtractIndices<pcl::PointXYZ> extract;
  //     extract.setInputCloud(cloud_filtered_again);
  //     extract.setIndices(inliers);
  //     extract.setNegative(false);

  //     // Get the points associated with the planar surface
  //     extract.filter(*cloud_plane);
  //     std::cout << "PointCloud representing the planar component: "
  //               << cloud_plane->size() << " data points." << std::endl;

  //     // Remove the planar inliers, extract the rest
  //     extract.setNegative(true);
  //     extract.filter(*cloud_f);
  //     *cloud_filtered_again = *cloud_f;
  //   }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
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
    // std::stringstream ss;
    // ss << std::setw(4) << std::setfill('0') << j;
    // writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd",
    //                             *cloud_cluster, false); //*
    j++;
  }

  // ***********************************************

  //   // Voxel filter applying
  //   pcl::VoxelGrid<PointT> voxel_filter;
  //   voxel_filter.setInputCloud(cloud_filtered);
  //   voxel_filter.setLeafSize(0.1, 0.1, 0.1);
  //   voxel_filter.filter(*voxel_cloud);

  //   // Performing estimation of normals
  //   normals_estimator.setSearchMethod(tree);
  //   normals_estimator.setInputCloud(voxel_cloud);
  //   normals_estimator.setKSearch(30);
  //   normals_estimator.compute(*cloud_normals);

  //   // Parameters for segmentation
  //   cylinder_segmentor.setOptimizeCoefficients(true);
  //   cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
  //   cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
  //   cylinder_segmentor.setNormalDistanceWeight(0.1);
  //   cylinder_segmentor.setMaxIterations(100);
  //   cylinder_segmentor.setDistanceThreshold(0.05);
  //   cylinder_segmentor.setRadiusLimits(0.075, 0.2);
  //   int l = 0;

  //   while (true) {

  //     // Appplying segmentation
  //     cylinder_segmentor.setInputCloud(voxel_cloud);
  //     cylinder_segmentor.setInputNormals(cloud_normals);
  //     cylinder_segmentor.segment(*cylinder_in, *cylinder_co);

  //     // extracting indices
  //     cylinder_indices_extractor.setInputCloud(voxel_cloud);
  //     cylinder_indices_extractor.setIndices(cylinder_in);
  //     cylinder_indices_extractor.setNegative(false);
  //     cylinder_indices_extractor.filter(*cylinder_cloud);

  //     if (!cylinder_cloud->points.empty()) {
  //       std::stringstream ss;
  //       ss << "ex_cylinder_" << l << ".pcd";
  //       std::cout << "Cloud Contains " << cylinder_cloud->points.size()
  //                 << std::endl;
  //       if (cylinder_cloud->points.size() > 20) {
  //         // cloud_saver(ss.str(), path, cylinder_cloud);
  //         l++;
  //         *all_cylinders_cloud += *cylinder_cloud;
  //       }

  //       cylinder_indices_extractor.setNegative(true);
  //       cylinder_indices_extractor.filter(*voxel_cloud);

  //       // processing normals
  //       cylinder_indices_extractor_temp.setInputCloud(cloud_normals);
  //       cylinder_indices_extractor_temp.setIndices(cylinder_in);
  //       cylinder_indices_extractor_temp.setNegative(true);
  //       cylinder_indices_extractor_temp.filter(*cloud_normals);

  //     } else {
  //       break;
  //     }
  //   }

  // *********************************************

  //   // Estimate point normals
  //   ne.setSearchMethod(tree);
  //   ne.setInputCloud(cloud_filtered);
  //   ne.setKSearch(50);
  //   ne.compute(*cloud_normals);

  //   // Create the segmentation object for the planar model and set all the
  //   // parameters
  //   seg.setOptimizeCoefficients(true);
  //   seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  //   seg.setNormalDistanceWeight(0.1);
  //   seg.setMethodType(pcl::SAC_RANSAC);
  //   seg.setMaxIterations(100);
  //   seg.setDistanceThreshold(0.03);
  //   seg.setInputCloud(cloud_filtered);
  //   seg.setInputNormals(cloud_normals);
  //   // Obtain the plane inliers and coefficients
  //   seg.segment(*inliers_plane, *coefficients_plane);
  //   std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  //   // Extract the planar inliers from the input cloud
  //   extract.setInputCloud(cloud_filtered);
  //   extract.setIndices(inliers_plane);
  //   extract.setNegative(false);

  //   pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
  //   extract.filter(*cloud_plane);

  //   // Remove the planar inliers, extract the rest
  //   extract.setNegative(true);
  //   extract.filter(*cloud_filtered2);
  //   extract_normals.setNegative(true);
  //   extract_normals.setInputCloud(cloud_normals);
  //   extract_normals.setIndices(inliers_plane);
  //   extract_normals.filter(*cloud_normals2);

  //   // Create the segmentation object for cylinder segmentation and set all
  //   the
  //   // parameters
  //   seg.setOptimizeCoefficients(true);
  //   seg.setModelType(pcl::SACMODEL_CYLINDER);
  //   seg.setMethodType(pcl::SAC_RANSAC);
  //   seg.setNormalDistanceWeight(0.1);
  //   seg.setMaxIterations(10000);
  //   seg.setDistanceThreshold(0.05);
  //   seg.setRadiusLimits(0, 0.1);
  //   seg.setInputCloud(cloud_filtered2);
  //   seg.setInputNormals(cloud_normals2);

  //   // Obtain the cylinder inliers and coefficients
  //   seg.segment(*inliers_cylinder, *coefficients_cylinder);

  //   // Write the cylinder inliers to disk
  //   extract.setInputCloud(cloud_filtered2);
  //   extract.setIndices(inliers_cylinder);
  //   extract.setNegative(false);
  //   pcl::PointCloud<PointT>::Ptr cloud_cylinder(new
  //   pcl::PointCloud<PointT>()); extract.filter(*cloud_cylinder); if
  //   (cloud_cylinder->points.empty()) {
  //     std::cerr << "Can't find the cylindrical component." << std::endl;

  //     return;
  //   }

  //   std::cerr << "PointCloud representing the cylindrical component: "
  //             << cloud_cylinder->size() << " data points." << std::endl;
  //   //   writer.write("table_scene_mug_stereo_textured_cylinder.pcd",
  //   //   *cloud_cylinder,
  //   //                false);

  // *******************************************************************************

  //   pcl::toPCLPointCloud2(*cloud_plane, *cloud_plane_filtered_out);
  pcl::toPCLPointCloud2(*cloud_cluster, *cloud_filtered_out);
  pcl::toPCLPointCloud2(*cloud_filtered, *cloud_filtered_filtered_out);

  //------------------------------------

  // PCL message to ROS2 message

  sensor_msgs::msg::PointCloud2 cloud_out;
  sensor_msgs::msg::PointCloud2 cloud_plane_out;
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

  tree_count_publisher->publish(tree_count_out);

  //   pcl_conversions::fromPCL(*cloud_plane_filtered_out, cloud_plane_out);
  //   pcl_conversions::fromPCL(*cloud_filtered_out, cloud_out);
  pcl_conversions::fromPCL(*cloud_filtered_filtered_out, cloud_out_filtered);

  unsigned int num_points_out = cloud_out.width;
  RCLCPP_INFO(
      this->get_logger(),
      "The number of points in the montemorency_trees_out pointcloud is %i",
      num_points_out);

  //   cloud_out.header.frame_id = msg->header.frame_id;
  //   cloud_out.header.stamp = msg->header.stamp;

  //   // Publish to ROS2 network
  //   publisher_->publish(cloud_out);

  //   cloud_plane_out.header.frame_id = msg->header.frame_id;
  //   cloud_plane_out.header.stamp = msg->header.stamp;

  //   // Publish to ROS2 network
  //   publisherPlane_->publish(cloud_plane_out);

  cloud_out_filtered.header.frame_id = msg->header.frame_id;
  cloud_out_filtered.header.stamp = msg->header.stamp;

  // Publish to ROS2 network
  publisherFiltered_->publish(cloud_out_filtered);
}

int main(int argc, char *argv[]) {
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Montemorency_Trees>());
  rclcpp::shutdown();
  return 0;
}