
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg)
{
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = std::to_string(frame_id);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  bool show_camera = false;
  size_t depth = rmw_qos_profile_default.depth;
  double freq = 60.0;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  std::string topic("image");
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto node = rclcpp::Node::make_shared("cam2image");
  rclcpp::Logger node_logger = node->get_logger();
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy,
      depth));
  qos.reliability(reliability_policy);

  RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
  auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, qos);
  bool is_flipped = false;
  auto callback =
    [&is_flipped, &node_logger](const std_msgs::msg::Bool::SharedPtr msg) -> void
    {
      is_flipped = msg->data;
      RCLCPP_INFO(node_logger, "Set flip mode to: %s", is_flipped ? "on" : "off");
    };
  auto sub = node->create_subscription<std_msgs::msg::Bool>(
    "flip_image", rclcpp::SensorDataQoS(), callback);
  rclcpp::WallRate loop_rate(freq);

  cv::VideoCapture cap;
  cap.open(2);
  cv::Mat frame;
  cv::Mat flipped_frame;

  size_t i = 1;

  while (rclcpp::ok()) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;
    cap >> frame;
    if (!frame.empty()) {
      if (!is_flipped) {
        convert_frame_to_message(frame, i, *msg);
      } else {
        cv::flip(frame, flipped_frame, 1);
        convert_frame_to_message(flipped_frame, i, *msg);
      }
      if (show_camera) {
        cv::Mat cvframe = frame;
        cv::imshow("cam2image", cvframe);
        cv::waitKey(1);
      }
      RCLCPP_INFO(node_logger, "Publishing image #%zd", i);
      pub->publish(std::move(msg));
      ++i;
    } else {
      RCLCPP_INFO(node_logger, "Finished");
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // rclcpp::shutdown();

  return 0;
}