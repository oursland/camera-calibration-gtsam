cmake_minimum_required(VERSION 3.8)
project(mocap_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
set(GTSAM_LIBS gtsam)

ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS "3.2.0")
  message(FATAL "Minimum OpenCV version is 3.2.0 (found version ${OpenCV_VERSION})")
endif()

ament_auto_add_library(mocap_intrinsic_calibration SHARED
  src/mocap_intrinsic_calibration.cpp
)
target_compile_definitions(mocap_intrinsic_calibration
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(mocap_intrinsic_calibration
  PLUGIN "mocap::IntrinsicCalibration"
  EXECUTABLE mocap_intrinsic_calibration_node
)
ament_target_dependencies(mocap_intrinsic_calibration
    "apriltag_ros"
    "apriltag_msgs"
    "camera_info_manager"
    "image_transport"
    "rclcpp"
    "rcutils"
    "std_msgs"
)
target_include_directories(mocap_intrinsic_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(mocap_intrinsic_calibration
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBS}
)
target_compile_features(mocap_intrinsic_calibration PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_auto_add_library(mocap_extrinsic_calibration SHARED
  src/mocap_extrinsic_calibration.cpp
)
target_compile_definitions(mocap_extrinsic_calibration
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_node(mocap_extrinsic_calibration
  PLUGIN "mocap::ExtrinsicCalibration"
  EXECUTABLE mocap_extrinsic_calibration_node
)
ament_target_dependencies(mocap_extrinsic_calibration
    "apriltag_ros"
    "apriltag_msgs"
    "camera_info_manager"
    "image_transport"
    "rclcpp"
    "rcutils"
    "std_msgs"
)
target_include_directories(mocap_extrinsic_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(mocap_extrinsic_calibration
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBS}
)
target_compile_features(mocap_extrinsic_calibration PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_auto_add_executable(mocap_setcamerainfo
  src/mocap_setcamerainfo.cpp
)
ament_target_dependencies(mocap_setcamerainfo
    "apriltag_ros"
    "apriltag_msgs"
    "camera_info_manager"
    "rclcpp"
    "rcutils"
    "std_msgs"
)
target_include_directories(mocap_setcamerainfo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mocap_setcamerainfo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
