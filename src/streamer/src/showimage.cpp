#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

void show_image(
  const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
  std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

  if (show_camera) {
    cv::Mat frame(
      msg->height, msg->width, encoding2mat_type(msg->encoding),
      const_cast<unsigned char *>(msg->data.data()), msg->step);

    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    cv::Mat cvframe = frame;
    cv::imshow("showimage", cvframe);
    cv::waitKey(1);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  bool show_camera = true;
  std::string topic("image");
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (show_camera) {
    cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
    cv::waitKey(1);
  }
  auto node = rclcpp::Node::make_shared("showimage");
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy,
      depth
  ));
  qos.reliability(reliability_policy);

  auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      show_image(msg, show_camera, node->get_logger());
    };
  std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic, qos, callback);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}