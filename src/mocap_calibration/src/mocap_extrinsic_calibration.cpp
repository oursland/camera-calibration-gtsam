#include <cv_bridge/cv_bridge.h>
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

#include <apriltag_ros/apriltag_detector.h>

#include <opencv2/core/ocl.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/GeneralSFMFactor.h>

#include <atomic>
#include <cstdio>
#include <filesystem>

#include <unistd.h>

namespace mocap
{

class ExtrinsicCalibration : public rclcpp::Node
{
public:
  explicit ExtrinsicCalibration(rclcpp::NodeOptions const& options)
    : rclcpp::Node{ "extrinsic_calibration", options }
    , tag_count_(0)
    , frame_count_(0)
    , calibrate_camera_(false)
    , pose_count_(0)
  {
    const int detector_type = declare_parameter("detector_type", 0);                    // enum { MIT, UMICH };
    const std::string tag_family = declare_parameter("tag_family", "36h11");            // "36h11", "25h9", "16h5"
    const int black_border = declare_parameter("black_border", 2);

    target_tag_cols_ = declare_parameter("tag_cols", 3);
    target_tag_rows_ = declare_parameter("tag_rows", 4);
    target_tag_size_ = declare_parameter("tag_size", 0.172);
    target_tag_spacing_ratio_ = declare_parameter("tag_spacing", 0.1);

    apriltag_ros::TagFamily tagFamily;
    if (tag_family == "36h11") {
      tagFamily = apriltag_ros::TagFamily::tf36h11;
    } else if (tag_family == "25h9") {
      tagFamily = apriltag_ros::TagFamily::tf25h9;
    } else if (tag_family == "16h5") {
      tagFamily = apriltag_ros::TagFamily::tf16h5;
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid tag family: " << tag_family);
      throw std::invalid_argument("invalid tag family!");
    }

    detector_ = apriltag_ros::ApriltagDetector::Create((apriltag_ros::DetectorType)detector_type, tagFamily);
    detector_->set_black_border(black_border);

    const auto num_tags = target_tag_rows_ * target_tag_cols_;
    target_tag_corner_positions_.resize(num_tags);
    RCLCPP_INFO(get_logger(), "AprilTag positions:");
    for(auto i = 0; i < num_tags; ++i) {
      target_tag_corner_positions_[i] = get_tag_corner_positions(i);
      RCLCPP_DEBUG(
        get_logger(),
        "  ID: %d (%f, %f), (%f, %f), (%f, %f), (%f, %f)",
        i,
        target_tag_corner_positions_[i][0][0], target_tag_corner_positions_[i][0][1],
        target_tag_corner_positions_[i][1][0], target_tag_corner_positions_[i][1][1],
        target_tag_corner_positions_[i][2][0], target_tag_corner_positions_[i][2][1],
        target_tag_corner_positions_[i][3][0], target_tag_corner_positions_[i][3][1]);
    }

    // Add a prior on pose x1.
    pose_ = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,-M_PI/2), gtsam::Point3(0, 0, 0));
    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6)
        << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.1))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll, pitch, yaw
    graph_.addPrior(gtsam::Symbol('x', 0), pose_, pose_noise);

    auto hosts = std::vector<std::string>{"/cam1", "/cam2", "/cam3", "/cam4", "/cam5", "/cam6"};
    for(size_t i = 0; i < hosts.size(); ++i) {
      auto host = hosts[i];

      RCLCPP_INFO(get_logger(), "Subscribing to: %s", (host + "/image_rect").c_str());

      image_mono_sub_.push_back(image_transport::create_subscription(
        this,
        host + "/image_rect",
        std::bind(
          &ExtrinsicCalibration::image_cb, this, i, std::placeholders::_1),
        "raw"));

      image_apriltag_pub_.push_back(image_transport::create_publisher(
        this,
        host + "/image_apriltags"));
    }

    calibrate_srv_ = create_service<mocap_msgs::srv::Calibrate>("calibrate", std::bind(&mocap::ExtrinsicCalibration::calibrate, this, std::placeholders::_1, std::placeholders::_2));
  }

  void image_cb(const size_t i, const sensor_msgs::msg::Image::ConstSharedPtr & image_msg) {
    cv_bridge::CvImageConstPtr const cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat const img = cv_ptr->image;

    // detect tags
    auto img_apriltags = detector_->Detect(img);
    const unsigned int num_tags = target_tag_rows_ * target_tag_cols_;
    if(img_apriltags.size() >= 4) {
      pose_count_++;

      for (auto const &tag : img_apriltags) {
        const size_t id = tag.id;
        if ((size_t)id >= num_tags) {
          RCLCPP_ERROR_STREAM(
            get_logger(),
            "tag with invalid id found: "
            << id << " (check your calibration target!)");
          continue;
        }

        RCLCPP_DEBUG(get_logger(),
          "Frame: %d, Pose %d: Tag: %d Corners: (%f, %f), (%f, %f), (%f, %f), (%f, %f)",
          frame_count_,
          pose_count_,
          tag.id,
          tag.corners[0].x, tag.corners[0].y,
          tag.corners[1].x, tag.corners[1].y,
          tag.corners[2].x, tag.corners[2].y,
          tag.corners[3].x, tag.corners[3].y);

        for (int k = 0; k < 4; ++k) {
          const auto &ip = tag.corners[k];

          gtsam::Point2 measurement{ ip.x, ip.y };
          auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
          graph_.emplace_shared<gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye> >(
            measurement,
            measurementNoise,
            gtsam::Symbol('x', pose_count_),
            gtsam::Symbol('l', id*4 + k),
            gtsam::Symbol('K', 0));
        }
        tag_count_++;
      }
    }

    if (frame_count_ % 10 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "frames: %4d, poses: %d, total # tags found: %d",
        frame_count_,
        pose_count_,
        tag_count_);
    }

    // publish debug images
    if (image_apriltag_pub_[i].getNumSubscribers()) {
      cv::Mat color;
      cv::cvtColor(img, color, CV_GRAY2BGR);
      apriltag_ros::DrawApriltags(color, img_apriltags);
      cv_bridge::CvImage cv_img(image_msg->header, sensor_msgs::image_encodings::BGR8, color);
      image_apriltag_pub_[i].publish(cv_img.toImageMsg());
    }

    frame_count_++;
  }

private:
  std::array<gtsam::Point3, 4> get_tag_corner_positions(unsigned int tag_id) const {
    unsigned int const tag_col = tag_id % target_tag_cols_;
    unsigned int const tag_row = tag_id / target_tag_cols_;

    std::array<gtsam::Point3, 4> corners;
    corners[0][0] = tag_col * target_tag_size_ * (1 + target_tag_spacing_ratio_);
    corners[0][1] = tag_row * target_tag_size_ * (1 + target_tag_spacing_ratio_);
    corners[0][2] = 0;
    corners[1][0] = corners[0][0] + target_tag_size_;
    corners[1][1] = corners[0][1];
    corners[1][2] = 0;
    corners[2][0] = corners[0][0] + target_tag_size_;
    corners[2][1] = corners[0][1] + target_tag_size_;
    corners[2][2] = 0;
    corners[3][0] = corners[0][0];
    corners[3][1] = corners[0][1] + target_tag_size_;
    corners[3][2] = 0;

    return corners;
  }

  void calibrate(
    const std::shared_ptr<mocap_msgs::srv::Calibrate::Request> request,
    std::shared_ptr<mocap_msgs::srv::Calibrate::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Calibrating!");
    calibrate_camera_ = true;

    if(calibrate_camera_) {
      // camera_model: pinhole
      // Extrinsics: [530.766, 532.796, 295.858, 266.729]
      // distortion_model: equidistant
      // distortion_coeffs: [-0.0865375, 0.0379966, -0.142482, 0.188395]
      // resolution: [640, 480]

      // Cal3_S2(fx, fy, s, u0, v0)
      // gtsam::Cal3_S2 K(530.766, 532.796, 0.0, 295.858, 266.729);

      // Cal3Fisheye(fx, fy, s, u0,v0, k1, k2, k3, k4, tol = 1e-5)
      K_ = gtsam::Cal3Fisheye(
        530.766, 532.796, 0.0, 295.858, 266.729,
        -0.0865375, 0.0379966, -0.142482, 0.188395);

      // Add a prior on the position of the first landmark.
      auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
      graph_.addPrior(gtsam::Symbol('l', 0), target_tag_corner_positions_[0][0], pointNoise);  // add directly to graph

      // Add a prior on the calibration.
      auto calNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(9) << 500, 500, 0.1, 100, 100, 1.0, 1.0, 1.0, 1.0).finished());
      graph_.addPrior(gtsam::Symbol('K', 0), K_, calNoise);

      // Create the initial estimate to the solution
      // now including an estimate on the camera calibration parameters
      gtsam::Values initialEstimate;
      initialEstimate.insert(gtsam::Symbol('K', 0), K_);
      for (unsigned int i = 0; i < pose_count_; ++i) {
        initialEstimate.insert(gtsam::Symbol('x', i), pose_);
      }
      for (size_t j = 0; j < target_tag_corner_positions_.size(); ++j) {
        for (auto k = 0; k < 4; ++k) {
          auto pos = target_tag_corner_positions_[j][k];
          initialEstimate.insert<gtsam::Point3>(gtsam::Symbol('l', j*4 + k), pos);
        }
      }

      RCLCPP_INFO(get_logger(), "Number of factors: %zu", graph_.nrFactors());
      auto dot = graph_.dot();
      std::ofstream fout("graph.dot");
      fout << dot;
      fout.close();

      /* Optimize the graph and print results */
      gtsam::Values result = gtsam::DoglegOptimizer(graph_, initialEstimate).optimize();
      result.print("Final results:\n");

      calibrate_camera_ = false;
    }

    response->success = true;
    response->message = "success";
  }

  int target_tag_cols_;
  int target_tag_rows_;
  float target_tag_size_;
  float target_tag_spacing_ratio_;
  std::vector<std::array<gtsam::Point3, 4>> target_tag_corner_positions_;

  int tag_count_;
  unsigned int frame_count_;
  std::atomic_bool calibrate_camera_;

  apriltag_ros::ApriltagDetector::Ptr detector_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Cal3Fisheye K_;
  gtsam::Pose3 pose_;
  unsigned int pose_count_;

  std::vector<image_transport::Subscriber> image_mono_sub_;
  std::vector<image_transport::Publisher> image_apriltag_pub_;

  rclcpp::Service<mocap_msgs::srv::Calibrate>::SharedPtr calibrate_srv_;
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mocap::ExtrinsicCalibration>(rclcpp::NodeOptions{});

  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  exit(EXIT_SUCCESS);
}

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mocap::ExtrinsicCalibration)
