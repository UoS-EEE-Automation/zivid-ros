#include <Zivid/Calibration/Detector.h>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/Calibration/InfieldCorrection.h>

#include <cstdint>
#include <numeric>
#include <sstream>
#include <zivid_camera/infield_correction_controller.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{
namespace
{
std::string infieldCorrectionDistancesToString(
  const std::vector<Zivid::Experimental::Calibration::InfieldCorrectionInput> & dataset)
{
  auto zValues = std::vector<float>(dataset.size());
  std::transform(dataset.begin(), dataset.end(), zValues.begin(), [](const auto & input) {
    return input.detectionResult().centroid().z;
  });
  std::sort(zValues.begin(), zValues.end());

  std::stringstream ss;
  ss << "Measured positions: [";
  for (size_t i = 0; i < zValues.size(); ++i) {
    ss << std::setprecision(1) << std::fixed << zValues.at(i) << "mm";
    if (i < zValues.size() - 1) {
      ss << ", ";
    }
  }
  ss << "].";

  if (!zValues.empty()) {
    ss << " Range covered: " << zValues.back() - zValues.front() << "mm.";
  }

  return ss.str();
}

struct InfieldCorrectionStatistics
{
  size_t numberOfCaptures;
  float currentTruenessError;
  float averageTruenessError;
  float maximumTruenessError;
};

InfieldCorrectionStatistics calculateInfieldCorrectionStatistics(
  const std::vector<Zivid::Experimental::Calibration::InfieldCorrectionInput> & dataset)
{
  if (dataset.empty()) {
    throw std::runtime_error("Empty dataset");
  }
  const auto dimensionTruenessErrors = [&] {
    std::vector<float> result;
    std::transform(
      dataset.begin(), dataset.end(), std::back_inserter(result), [](const auto & input) {
        return Zivid::Experimental::Calibration::verifyCamera(input).localDimensionTrueness();
      });
    return result;
  }();
  const float currentTruenessError = dimensionTruenessErrors.back();
  const float averageTruenessError =
    std::accumulate(cbegin(dimensionTruenessErrors), cend(dimensionTruenessErrors), 0.0F) /
    static_cast<float>(size(dimensionTruenessErrors));
  const float maximumTruenessError =
    *std::max_element(dimensionTruenessErrors.begin(), dimensionTruenessErrors.end());
  return InfieldCorrectionStatistics{
    dataset.size(), currentTruenessError, averageTruenessError, maximumTruenessError};
}

std::string infieldCorrectionEstimateToString(
  const InfieldCorrectionStatistics & statistics,
  const Zivid::Experimental::Calibration::AccuracyEstimate & accuracyEstimate)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Number of captures: " << statistics.numberOfCaptures << "\n";
  ss << "Camera metrics:";
  ss << "\n  Current trueness error: " << 100.0F * statistics.currentTruenessError << "%.";
  if (statistics.numberOfCaptures > 1) {
    ss << "\n  Average trueness error: " << 100.0F * statistics.averageTruenessError << "%.";
    ss << "\n  Maximum trueness error: " << 100.0F * statistics.maximumTruenessError << "%.";
  }
  ss << "\n";
  ss << "Expected post-correction metrics:";
  ss << "\n  Dimension trueness error: " << 100.0F * accuracyEstimate.dimensionAccuracy()
     << "% or less.";
  ss << "\n  Optimized workspace (depth): " << static_cast<int>(std::round(accuracyEstimate.zMin()))
     << " - " << static_cast<int>(std::round(accuracyEstimate.zMax())) << " mm.";
  return ss.str();
}
}  // namespace

InfieldCorrectionController::InfieldCorrectionController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera)
: node_{zivid_camera_node}, camera_{camera}
{
  using namespace std::placeholders;

  infield_correction_state_ = std::make_unique<InfieldCorrectionState>();
  infield_correction_read_ = node_.create_service<zivid_interfaces::srv::InfieldCorrectionRead>(
    "infield_correction/read",
    std::bind(&InfieldCorrectionController::infieldCorrectionRead, this, _1, _2, _3));
  infield_correction_reset_ = node_.create_service<std_srvs::srv::Trigger>(
    "infield_correction/reset",
    std::bind(&InfieldCorrectionController::infieldCorrectionReset, this, _1, _2, _3));
  infield_correction_verify_ = node_.create_service<zivid_interfaces::srv::InfieldCorrectionVerify>(
    "infield_correction/verify",
    std::bind(&InfieldCorrectionController::infieldCorrectionVerify, this, _1, _2, _3));
  infield_correction_remove_last_capture_ = node_.create_service<std_srvs::srv::Trigger>(
    "infield_correction/remove_last_capture",
    std::bind(&InfieldCorrectionController::infieldCorrectionRemoveLastCapture, this, _1, _2, _3));
  infield_correction_start_ = node_.create_service<std_srvs::srv::Trigger>(
    "infield_correction/start",
    std::bind(&InfieldCorrectionController::infieldCorrectionStart, this, _1, _2, _3));
  infield_correction_capture_ =
    node_.create_service<zivid_interfaces::srv::InfieldCorrectionCapture>(
      "infield_correction/capture",
      std::bind(&InfieldCorrectionController::infieldCorrectionCapture, this, _1, _2, _3));
  infield_correction_compute_ =
    node_.create_service<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute",
      std::bind(&InfieldCorrectionController::infieldCorrectionCompute, this, _1, _2, _3));
  infield_correction_compute_and_write_ =
    node_.create_service<zivid_interfaces::srv::InfieldCorrectionCompute>(
      "infield_correction/compute_and_write",
      std::bind(&InfieldCorrectionController::infieldCorrectionComputeAndWrite, this, _1, _2, _3));
}

InfieldCorrectionController::~InfieldCorrectionController() = default;

void InfieldCorrectionController::infieldCorrectionRead(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionRead::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionRead::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      if (Zivid::Experimental::Calibration::hasCameraCorrection(camera_)) {
        const auto timestamp = Zivid::Experimental::Calibration::cameraCorrectionTimestamp(camera_);
        const auto time = std::chrono::system_clock::to_time_t(timestamp);
        std::stringstream ss;
        ss << "Timestamp of the current camera correction: "
           << std::put_time(std::gmtime(&time), "%FT%TZ");
        response->message = ss.str();
        response->has_camera_correction = true;
        response->camera_correction_timestamp.sec = static_cast<int32_t>(time);
        response->camera_correction_timestamp.nanosec = 0;
      } else {
        response->has_camera_correction = false;
        response->message = "This camera has no in-field correction written to it.";
      }
    },
    response, node_.get_logger(), "InfieldCorrectionRead");
}

void InfieldCorrectionController::infieldCorrectionReset(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() { Zivid::Experimental::Calibration::resetCameraCorrection(camera_); }, response,
    node_.get_logger(), "InfieldCorrectionReset");
}

void InfieldCorrectionController::infieldCorrectionVerify(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionVerify::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionVerify::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "Capturing calibration object");
      const auto detectionResult = Zivid::Experimental::Calibration::detectFeaturePoints(camera_);
      const auto input = Zivid::Experimental::Calibration::InfieldCorrectionInput{detectionResult};
      if (!input.valid()) {
        response->message = "Invalid capture! Feedback: " + input.statusDescription();
        response->success = false;
        return;
      }
      RCLCPP_DEBUG_STREAM(node_.get_logger(), "Successful measurement, starting verification");
      const auto verification = Zivid::Experimental::Calibration::verifyCamera(input);
      const auto centroid = detectionResult.centroid();
      std::stringstream ss;
      ss << "Successful measurement at " << detectionResult.centroid()
         << ". Estimated dimension trueness error at measured position: " << std::setprecision(2)
         << std::fixed << 100.0F * verification.localDimensionTrueness() << "%";
      response->local_dimension_trueness = verification.localDimensionTrueness();
      response->position.x = static_cast<double>(centroid.x);
      response->position.y = static_cast<double>(centroid.y);
      response->position.z = static_cast<double>(centroid.z);
      response->message = ss.str();
    },
    response, node_.get_logger(), "InfieldCorrectionVerify");
}

void InfieldCorrectionController::infieldCorrectionRemoveLastCapture(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      ensureStarted();
      if (infield_correction_state_->dataset.empty()) {
        throw std::runtime_error("Infield correction dataset is empty");
      }
      infield_correction_state_->dataset.pop_back();
    },
    response, node_.get_logger(), "InfieldCorrectionRemoveLastCapture");
}

void InfieldCorrectionController::infieldCorrectionStart(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      *infield_correction_state_ = {};
      infield_correction_state_->state = InfieldCorrectionState::State::Started;
    },
    response, node_.get_logger(), "InfieldCorrectionStart");
}

void InfieldCorrectionController::infieldCorrectionCapture(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCapture::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCapture::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      auto & dataset = infield_correction_state_->dataset;
      response->number_of_captures = safeCast<int>(dataset.size());
      ensureStarted();
      const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(camera_);
      const auto input = Zivid::Experimental::Calibration::InfieldCorrectionInput{detectionResult};

      using Response = zivid_interfaces::srv::InfieldCorrectionCapture::Response;
      using Zivid::Experimental::Calibration::InfieldCorrectionDetectionStatus;
      switch (input.status()) {
        case InfieldCorrectionDetectionStatus::ok:
          response->status = Response::STATUS_OK;
          break;
        case InfieldCorrectionDetectionStatus::detectionFailed:
          response->status = Response::STATUS_DETECTION_FAILED;
          break;
        case InfieldCorrectionDetectionStatus::invalidCaptureMethod:
          response->status = Response::STATUS_INVALID_CAPTURE_METHOD;
          break;
        case InfieldCorrectionDetectionStatus::invalidAlignment:
          response->status = Response::STATUS_INVALID_ALIGNMENT;
          break;
        default:
          throw std::runtime_error(
            "Unhandled status value: " + std::to_string(static_cast<int>(input.status())));
      }

      if (input.valid()) {
        dataset.push_back(input);
        response->number_of_captures = safeCast<int>(dataset.size());
        response->message = "Valid detection. Collected " + std::to_string(dataset.size()) +
                            " valid measurement(s) so far. " +
                            infieldCorrectionDistancesToString(dataset);

      } else {
        response->success = false;
        response->message = "Invalid detection. This measurement will not be used. Feedback: " +
                            input.statusDescription();
      }
    },
    response, node_.get_logger(), "InfieldCorrectionCapture");
}

void InfieldCorrectionController::infieldCorrectionCompute(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      const auto & dataset = infield_correction_state_->dataset;
      // Set started & number of captures before computing, so that it is reported regardless of any exceptions.
      response->infield_correction_started =
        (infield_correction_state_->state == InfieldCorrectionState::State::Started);
      response->number_of_captures = safeCast<int>(dataset.size());
      ensureStarted();
      const auto correction = Zivid::Experimental::Calibration::computeCameraCorrection(dataset);
      const auto accuracyEstimate = correction.accuracyEstimate();
      const auto statistics = calculateInfieldCorrectionStatistics(dataset);
      response->current_trueness_error = statistics.currentTruenessError;
      response->average_trueness_error = statistics.averageTruenessError;
      response->maximum_trueness_error = statistics.maximumTruenessError;
      response->z_min = accuracyEstimate.zMin();
      response->z_max = accuracyEstimate.zMax();
      response->dimension_accuracy = accuracyEstimate.dimensionAccuracy();
      response->message = "Camera correction computed successfully.\n" +
                          infieldCorrectionEstimateToString(statistics, accuracyEstimate);
    },
    response, node_.get_logger(), "InfieldCorrectionCompute");
}

void InfieldCorrectionController::infieldCorrectionComputeAndWrite(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Request> /*request*/,
  std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      auto & dataset = infield_correction_state_->dataset;
      response->infield_correction_started =
        (infield_correction_state_->state == InfieldCorrectionState::State::Started);
      response->number_of_captures = safeCast<int>(dataset.size());
      ensureStarted();
      const auto correction = Zivid::Experimental::Calibration::computeCameraCorrection(dataset);
      RCLCPP_DEBUG(node_.get_logger(), "Writing correction to camera");
      Zivid::Experimental::Calibration::writeCameraCorrection(camera_, correction);
      const auto accuracyEstimate = correction.accuracyEstimate();
      const auto statistics = calculateInfieldCorrectionStatistics(dataset);
      response->current_trueness_error = statistics.currentTruenessError;
      response->average_trueness_error = statistics.averageTruenessError;
      response->maximum_trueness_error = statistics.maximumTruenessError;
      response->z_min = accuracyEstimate.zMin();
      response->z_max = accuracyEstimate.zMax();
      response->dimension_accuracy = accuracyEstimate.dimensionAccuracy();
      response->message = "Camera correction successfully written to camera.\n" +
                          infieldCorrectionEstimateToString(statistics, accuracyEstimate);
      // Clear the infield correction session after a successful write, the captures cannot be used again.
      *infield_correction_state_ = {};
      infield_correction_state_->state = InfieldCorrectionState::State::WriteCompleted;
    },
    response, node_.get_logger(), "InfieldCorrectionComputeAndWrite");
}

void InfieldCorrectionController::ensureStarted() const
{
  switch (infield_correction_state_->state) {
    case InfieldCorrectionState::State::Uninitialized:
      throw std::runtime_error(
        "Infield correction not started. Please call the '" +
        std::string{infield_correction_start_->get_service_name()} + "' service first.");
      break;
    case InfieldCorrectionState::State::Started:
      break;
    case InfieldCorrectionState::State::WriteCompleted:
      throw std::runtime_error(
        "A new infield correction has been written to the camera. The infield correction session "
        "needs to be restarted before proceeding. This can be done by calling the '" +
        std::string{infield_correction_start_->get_service_name()} + "' service");
    default:
      throw std::runtime_error(
        "Internal error. Unhandled infield correction state: " +
        std::to_string(static_cast<int>(infield_correction_state_->state)));
  }
}

}  // namespace zivid_camera
