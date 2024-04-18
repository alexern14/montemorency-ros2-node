#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/int8.hpp>

/**
 * @brief
 *
 */
class Montemorency_Trees : public rclcpp::Node {
public:
  /**
   * @brief A constructor for Montemorency_Trees class
   * @param options Additional options to control creation of the node.
   */
  explicit Montemorency_Trees(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Montemorency_Trees object
   *
   */
  ~Montemorency_Trees(){};

protected:
  /**
   * @brief Use a no filter of pcl library
   *
   * @param msg Pointcloud2 message received from the ros2 node
   */
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ROS2 subscriber and related topic name
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::string param_topic_pointcloud_in;

  // ROS2 publisher and related topic name
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string param_topic_pointcloud_out;

  // ROS2 publisher and related topic name
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr tree_count_publisher;
  std::string param_tree_count;

  // ROS2 publisher and related topic name
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisherFiltered_;
  std::string param_topic_pointcloud_filtered_out;
};