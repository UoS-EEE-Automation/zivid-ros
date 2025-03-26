#include <Zivid/Calibration/Detector.h>
#include <Zivid/Camera.h>
#include <Zivid/Experimental/Calibration.h>

#include <cstdint>
#include <numeric>
#include <sstream>
#include <zivid_camera/capture_settings_controller.hpp>
#include <zivid_camera/detector_controller.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{

DetectorController::DetectorController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
  CaptureSettingsController<Zivid::Settings> & settings_controller)
: node_{zivid_camera_node}, camera_{camera}, settings_controller_{settings_controller}
{
  using namespace std::placeholders;

  detect_calibration_board_ = node_.create_service<zivid_interfaces::srv::DetectCalibrationBoard>(
    "detect_calibration_board",
    std::bind(&DetectorController::detectCalibrationBoard, this, _1, _2, _3));
  detect_markers_ = node_.create_service<zivid_interfaces::srv::DetectMarkers>(
    "detect_markers", std::bind(&DetectorController::detectMarkers, this, _1, _2, _3));
}

DetectorController::~DetectorController() = default;

void DetectorController::detectCalibrationBoard(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::DetectCalibrationBoard::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::DetectCalibrationBoard::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      auto detection = Zivid::Calibration::detectCalibrationBoard(camera_);
      response->detection_result = toZividMsgDetectionResult(detection);
      if (
        response->detection_result.status !=
        zivid_interfaces::msg::DetectionResultCalibrationBoard::STATUS_OK) {
        response->success = false;
        response->message =
          "Failed to detect calibration board. " + response->detection_result.status_description;
      }
    },
    response, node_.get_logger(), "detectCalibrationBoard");
}

void DetectorController::detectMarkers(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::DetectMarkers::Request> request,
  std::shared_ptr<zivid_interfaces::srv::DetectMarkers::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      const auto settings = settings_controller_.currentSettings();
      RCLCPP_INFO(
        node_.get_logger(), "Capturing with %zd acquisition(s)", settings.acquisitions().size());
      RCLCPP_DEBUG_STREAM(node_.get_logger(), settings);
      const auto frame = camera_.capture(settings);
      const auto detection = Zivid::Calibration::detectMarkers(
        frame, request->marker_ids,
        Zivid::Calibration::MarkerDictionary::fromString(request->marker_dictionary));
      response->detection_result = toZividMsgDetectionResult(detection);
      response->success = detection.valid();
      if (!response->success) {
        response->message = "Failed to detect any fiducial markers.";
      }
    },
    response, node_.get_logger(), "detectMarkers");
}

}  // namespace zivid_camera
