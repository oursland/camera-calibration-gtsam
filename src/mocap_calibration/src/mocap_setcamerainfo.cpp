#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mocap_msgs/srv/calibrate.hpp>

#include <rcl_interfaces/msg/parameter.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

#include <apriltag_ros/apriltag_detector.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>

#include <unistd.h>

using namespace std::chrono_literals;

std::string matrix_to_string(std::array<double, 9> arr) {
  std::string str = "\n[ ";
  int i = 1;
  for(auto v : arr) {
    str.append(std::to_string(v) + ", ");
    if(i % 3 == 0) {
      str.append("]\n[ ");
    }
    i++;
  }
  return str;
}

std::string matrix_to_string(std::array<double, 12> arr) {
  std::string str = "\n[ ";
  int i = 1;
  for(auto v : arr) {
    str.append(std::to_string(v) + ", ");
    if(i % 4 == 0) {
      str.append("]\n[ ");
    }
    i++;
  }
  return str;
}

int main(int argc, char** argv)
{
  rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: %s [service] [camera_info_file.yaml]", argv[0]);
      exit(EXIT_FAILURE);
  }

  auto camera_info_service = argv[1];
  auto camera_info_file = argv[2];

  auto node = rclcpp::Node::make_shared("set_camera_info");

  auto camera_info_manager = new camera_info_manager::CameraInfoManager(node.get());
  if(camera_info_manager->loadCameraInfo(std::string("file://") + camera_info_file)) {
    auto camera_info = camera_info_manager->getCameraInfo();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  width: %d", camera_info.width);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  height: %d", camera_info.height);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  distortion_model: %s", camera_info.distortion_model.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  k: %s", matrix_to_string(camera_info.k).c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  r: %s", matrix_to_string(camera_info.r).c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  p: %s", matrix_to_string(camera_info.p).c_str());
  } else {
    exit(EXIT_FAILURE);
  }

  auto client = node->create_client<sensor_msgs::srv::SetCameraInfo>(camera_info_service);
  auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
  request->camera_info = camera_info_manager->getCameraInfo();

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->status_message.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service %s", camera_info_service);
  }

  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
