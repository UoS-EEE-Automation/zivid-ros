#include <Zivid/Calibration/Detector.h>
#include <Zivid/Experimental/Calibration/HandEyeLowDOF.h>

#include <cstdint>
#include <numeric>
#include <sstream>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <zivid_camera/capture_settings_controller.hpp>
#include <zivid_camera/hand_eye_calibration_controller.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{
namespace
{
std::filesystem::path ensureEmptyDirectoryOrThrow(const std::string & inPath)
{
  const std::filesystem::path path{inPath};
  if (!path.is_absolute()) {
    throw std::runtime_error{"Expected an absolute path but got: " + inPath};
  }

  if (!std::filesystem::is_directory(path)) {
    if (!std::filesystem::create_directory(path)) {
      throw std::runtime_error{"Could not create directory: " + inPath};
    }
  }

  if (!std::filesystem::is_empty(path)) {
    throw std::runtime_error{"Provided directory is not empty: " + inPath};
  }

  return path;
}

void saveHandEyeCaptureInWorkingDirectory(
  const std::filesystem::path & working_directory, int capture_handle, const Zivid::Frame & frame,
  const Zivid::Calibration::Pose & robot_pose,
  const std::optional<Zivid::Calibration::Pose> & camera_pose)
{
  const auto capture_handle_string = std::to_string(capture_handle);
  const std::filesystem::path frame_path =
    working_directory / ("calibration_object_pose_" + capture_handle_string + ".zdf");
  const std::filesystem::path robot_pose_path =
    working_directory / ("robot_pose_" + capture_handle_string + ".yaml");
  const std::filesystem::path camera_pose_path =
    working_directory / ("checkerboard_pose_in_camera_frame_" + capture_handle_string + ".yaml");

  for (const std::filesystem::path * path_ptr :
       {&frame_path, &robot_pose_path, camera_pose.has_value() ? &camera_pose_path : nullptr}) {
    if (path_ptr) {
      if (std::filesystem::exists(*path_ptr)) {
        throw std::runtime_error{"Cannot save to file, file already exists: " + path_ptr->string()};
      }
    }
  }

  frame.save(frame_path);
  robot_pose.toMatrix().save(robot_pose_path);
  if (camera_pose.has_value()) {
    camera_pose->toMatrix().save(camera_pose_path);
  }
}

HandEyeCalibrationObjects getHandEyeCalibrationObjects(
  const zivid_interfaces::msg::HandEyeCalibrationObjects & calibration_objects)
{
  using MsgHandEyeCalibrationObjects = zivid_interfaces::msg::HandEyeCalibrationObjects;

  switch (calibration_objects.type) {
    case MsgHandEyeCalibrationObjects::CALIBRATION_BOARD:
      return CalibrationBoard{};
    case MsgHandEyeCalibrationObjects::FIDUCIAL_MARKERS:
      return FiducialMarkers{
        Zivid::Calibration::MarkerDictionary::fromString(calibration_objects.marker_dictionary),
        calibration_objects.marker_ids,
      };
    default:
      throw std::runtime_error{
        "Invalid calibration object type: " + std::to_string(calibration_objects.type)};
  }
}

using HandEyeDetectionResult = std::variant<
  Zivid::Calibration::DetectionResult,                 // Valid calibration board detection
  Zivid::Calibration::DetectionResultFiducialMarkers,  // Valid fiducial markers detection
  std::string                                          // Error message
  >;

HandEyeDetectionResult detectHandEyeCalibrationObject(
  const HandEyeCalibrationObjects & calibration_object, const Zivid::Frame frame)
{
  return std::visit(
    Overloaded{
      [&](const CalibrationBoard &) -> HandEyeDetectionResult {
        const auto result = Zivid::Calibration::detectCalibrationBoard(frame);
        if (!result.valid()) {
          return std::string{"Could not detect calibration board: " + result.statusDescription()};
        }
        return result;
      },
      [&](const FiducialMarkers & fiducial_markers) -> HandEyeDetectionResult {
        const auto result = Zivid::Calibration::detectMarkers(
          frame, fiducial_markers.ids, fiducial_markers.dictionary);
        if (!result.valid()) {
          return std::string{"Could not detect fiducial markers: " + result.toString()};
        }
        return result;
      },
    },
    calibration_object);
}

std::vector<Zivid::Calibration::HandEyeInput> loadHandEyeWorkspace(
  const std::filesystem::path & working_directory,
  const HandEyeCalibrationObjects & calibration_objects)
{
  std::vector<Zivid::Calibration::HandEyeInput> result;

  if (!std::filesystem::is_directory(working_directory)) {
    throw std::runtime_error("Path is not a directory: " + working_directory.string());
  }

  for (int index = 1; true; ++index) {
    const std::string index_string = std::to_string(index);
    const auto frame_path =
      working_directory / ("calibration_object_pose_" + index_string + ".zdf");
    const auto robot_pose_path = working_directory / ("robot_pose_" + index_string + ".yaml");
    if (!std::filesystem::is_regular_file(frame_path)) {
      break;
    }
    if (!std::filesystem::is_regular_file(robot_pose_path)) {
      throw std::runtime_error("Missing robot pose file: " + robot_pose_path.string());
    }

    const Zivid::Frame frame{frame_path.string()};
    const Zivid::Calibration::Pose robot_pose{Zivid::Matrix4x4{robot_pose_path.string()}};

    const auto detection_result = detectHandEyeCalibrationObject(calibration_objects, frame);
    std::visit(
      Overloaded{
        [&](const Zivid::Calibration::DetectionResult & valid_detection) {
          result.emplace_back(robot_pose, valid_detection);
        },
        [&](const Zivid::Calibration::DetectionResultFiducialMarkers & valid_detection) {
          result.emplace_back(robot_pose, valid_detection);
        },
        [&](const std::string & error_message) {
          throw std::runtime_error{
            "Could not detect calibration object in frame \"" + frame_path.string() + "\". " +
            error_message};
        }},

      detection_result);
  }

  if (result.empty()) {
    throw std::runtime_error(
      "No hand-eye calibration input found in directory: " + working_directory.string());
  }

  return result;
}

std::optional<Zivid::Experimental::Calibration::HandEyeLowDOF::FixedPlacementOfCalibrationObjects>
fixedObjectsRequestToZivid(
  const zivid_interfaces::msg::FixedPlacementOfCalibrationObjects & fixed_objects)
{
  using MsgFixedPlacementObjects = zivid_interfaces::msg::FixedPlacementOfCalibrationObjects;
  using namespace Zivid::Experimental::Calibration::HandEyeLowDOF;

  switch (fixed_objects.type) {
    case MsgFixedPlacementObjects::NONE: {
      return std::nullopt;
    }

    case MsgFixedPlacementObjects::CALIBRATION_BOARD: {
      const auto & calibration_board = fixed_objects.calibration_board;
      switch (calibration_board.representation) {
        case zivid_interfaces::msg::FixedPlacementOfCalibrationBoard::POSE: {
          const auto board_pose = toZividPose(calibration_board.pose);
          return FixedPlacementOfCalibrationObjects{FixedPlacementOfCalibrationBoard{board_pose}};
        } break;
        case zivid_interfaces::msg::FixedPlacementOfCalibrationBoard::POSITION: {
          return FixedPlacementOfCalibrationObjects{
            FixedPlacementOfCalibrationBoard{toZividPoint(calibration_board.position)}};
        } break;
        default:
          throw std::runtime_error{
            "Unexpected hand-eye calibration fixed calibration board representation: " +
            std::to_string(calibration_board.representation)};
      }
    } break;

    case MsgFixedPlacementObjects::FIDUCIAL_MARKERS: {
      const auto marker_dictionary =
        Zivid::Calibration::MarkerDictionary::fromString(fixed_objects.marker_dictionary);

      std::vector<FixedPlacementOfFiducialMarker> fiducial_markers;
      for (const auto & m : fixed_objects.markers) {
        fiducial_markers.emplace_back(
          FixedPlacementOfFiducialMarker{m.marker_id, toZividPoint(m.position)});
      }
      return FixedPlacementOfCalibrationObjects{
        FixedPlacementOfFiducialMarkers{marker_dictionary, fiducial_markers}};
    } break;

    default:
      throw std::runtime_error{
        "Invalid fixed calibration object type: " + std::to_string(fixed_objects.type)};
  }

  throw std::runtime_error{
    "Internal error: Unexpected calibrationObjectsRequestToZivid fall through"};
}
}  // namespace

HandEyeCalibrationController::HandEyeCalibrationController(
  rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
  CaptureSettingsController<Zivid::Settings> & settings_controller)
: node_{zivid_camera_node}, camera_{camera}, settings_controller_{settings_controller}
{
  using namespace std::placeholders;

  state_ = std::make_unique<HandEyeCalibrationState>();
  start_service_ = node_.create_service<zivid_interfaces::srv::HandEyeCalibrationStart>(
    "hand_eye_calibration/start",
    std::bind(&HandEyeCalibrationController::startServiceHandler, this, _1, _2, _3));
  load_service_ = node_.create_service<zivid_interfaces::srv::HandEyeCalibrationLoad>(
    "hand_eye_calibration/load",
    std::bind(&HandEyeCalibrationController::loadServiceHandler, this, _1, _2, _3));
  capture_service_ = node_.create_service<zivid_interfaces::srv::HandEyeCalibrationCapture>(
    "hand_eye_calibration/capture",
    std::bind(&HandEyeCalibrationController::captureServiceHandler, this, _1, _2, _3));
  calibrate_service_ = node_.create_service<zivid_interfaces::srv::HandEyeCalibrationCalibrate>(
    "hand_eye_calibration/calibrate",
    std::bind(&HandEyeCalibrationController::calibrateServiceHandler, this, _1, _2, _3));
}

HandEyeCalibrationController::~HandEyeCalibrationController() = default;

void HandEyeCalibrationController::startServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationStart::Request> request,
  std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationStart::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      *state_ = {};

      if (!request->working_directory.empty()) {
        const auto directory = ensureEmptyDirectoryOrThrow(request->working_directory);
        state_->working_directory = directory;
      }

      state_->calibration_objects = getHandEyeCalibrationObjects(request->calibration_objects);

      state_->state = HandEyeCalibrationState::State::CaptureAndCalibrate;
      response->success = true;
    },
    response, node_.get_logger(), "HandEyeCalibrationStart");
}

void HandEyeCalibrationController::loadServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationLoad::Request> request,
  std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationLoad::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      *state_ = {};

      if (request->working_directory.empty()) {
        throw std::runtime_error{"Provided path to working directory is empty"};
      }

      const std::filesystem::path path{request->working_directory};
      if (!std::filesystem::is_directory(path)) {
        throw std::runtime_error{
          "Provided path to working directory is not a directory: " + request->working_directory};
      }

      state_->calibration_objects = getHandEyeCalibrationObjects(request->calibration_objects);

      state_->input = loadHandEyeWorkspace(
        state_->working_directory.value(), state_->calibration_objects.value());

      state_->state = HandEyeCalibrationState::State::ReadWorkspace;
      response->success = true;
    },
    response, node_.get_logger(), "HandEyeCalibrationLoad");
}

void HandEyeCalibrationController::captureServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCapture::Request> request,
  std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCapture::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      switch (state_->state) {
        case HandEyeCalibrationState::State::Uninitialized:
          throw std::runtime_error{
            "Hand-eye calibration is not started, capturing is not allowed."};
        case HandEyeCalibrationState::State::CaptureAndCalibrate:
          // Proceed with capture
          break;
        case HandEyeCalibrationState::State::ReadWorkspace:
          throw std::runtime_error{
            "Hand-eye calibration is currently in workspace read-only state, capturing is not "
            "allowed."};
        default:
          throw std::runtime_error{
            "Internal error. Unhandled hand-eye calibration state: " +
            std::to_string(static_cast<int>(state_->state))};
      }

      const auto settings = settings_controller_.currentSettings();
      RCLCPP_INFO(
        node_.get_logger(), "Capturing with %zd acquisition(s)", settings.acquisitions().size());
      RCLCPP_DEBUG_STREAM(node_.get_logger(), settings);
      const auto frame = camera_.capture(settings);

      const auto robot_pose = toZividPose(request->robot_pose);
      const int capture_handle = safeCast<int>(state_->input.size());
      const auto & working_directory = state_->working_directory;

      const auto detection_result =
        detectHandEyeCalibrationObject(state_->calibration_objects.value(), frame);

      std::visit(
        Overloaded{
          [&](const Zivid::Calibration::DetectionResult & valid_detection) {
            if (working_directory.has_value()) {
              saveHandEyeCaptureInWorkingDirectory(
                working_directory.value(), capture_handle, frame, robot_pose,
                valid_detection.pose());
            }
            state_->input.emplace_back(robot_pose, valid_detection);
          },
          [&](const Zivid::Calibration::DetectionResultFiducialMarkers & valid_detection) {
            if (working_directory.has_value()) {
              saveHandEyeCaptureInWorkingDirectory(
                working_directory.value(), capture_handle, frame, robot_pose, std::nullopt);
            }
            state_->input.emplace_back(robot_pose, valid_detection);
          },
          [&](const std::string & error_message) { throw std::runtime_error{error_message}; }},

        detection_result);

      response->capture_handle = capture_handle;
      response->success = true;
    },
    response, node_.get_logger(), "HandEyeCalibrationCapture");
}

void HandEyeCalibrationController::calibrateServiceHandler(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request> request,
  std::shared_ptr<zivid_interfaces::srv::HandEyeCalibrationCalibrate::Response> response)
{
  RCLCPP_INFO_STREAM(node_.get_logger(), __func__);

  runFunctionAndCatchExceptions(
    [&]() {
      switch (state_->state) {
        case HandEyeCalibrationState::State::Uninitialized:
          throw std::runtime_error{
            "Hand-eye calibration is not started, calibration is not allowed."};
        case HandEyeCalibrationState::State::CaptureAndCalibrate:
          // Proceed with calibration
          break;
        case HandEyeCalibrationState::State::ReadWorkspace:
          // Proceed with calibration
        default:
          throw std::runtime_error{
            "Internal error. Unhandled hand-eye calibration state: " +
            std::to_string(static_cast<int>(state_->state))};
      }

      std::vector<Zivid::Calibration::HandEyeInput> selected_input;
      if (request->capture_handles.empty()) {
        selected_input = state_->input;
      } else {
        for (const int capture_handle : request->capture_handles) {
          const size_t input_index = static_cast<size_t>(capture_handle);
          if (input_index >= state_->input.size()) {
            throw std::runtime_error("Invalid capture handle: " + std::to_string(capture_handle));
          }
          selected_input.push_back(state_->input.at(input_index));
        }
      }

      const auto fixed_objects = fixedObjectsRequestToZivid(request->fixed_objects);

      const Zivid::Calibration::HandEyeOutput result = [&] {
        switch (request->configuration) {
          case zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_TO_HAND: {
            if (fixed_objects.has_value()) {
              return Zivid::Experimental::Calibration::calibrateEyeToHandLowDOF(
                selected_input, fixed_objects.value());
            }
            return Zivid::Calibration::calibrateEyeToHand(selected_input);
          }
          case zivid_interfaces::srv::HandEyeCalibrationCalibrate::Request::EYE_IN_HAND: {
            if (fixed_objects.has_value()) {
              return Zivid::Experimental::Calibration::calibrateEyeInHandLowDOF(
                selected_input, fixed_objects.value());
            }
            return Zivid::Calibration::calibrateEyeInHand(selected_input);
          }
          default:
            throw std::runtime_error("Invalid calibration type requested.");
        }
      }();

      if (!result.valid()) {
        throw std::runtime_error{"Hand-eye calibration failed: " + result.toString()};
      }

      const auto transform = result.transform();
      if (
        state_->state == HandEyeCalibrationState::State::CaptureAndCalibrate &&
        state_->working_directory.has_value()) {
        transform.save(state_->working_directory.value() / "hand_eye_transform.yaml");
      }

      response->set__transform(toGeometryMsgTransform(transform));

      response->residuals.reserve(result.residuals().size());
      for (const auto & residual : result.residuals()) {
        auto & response_residual = response->residuals.emplace_back();
        response_residual.translation =
          static_cast<float>(zividLengthToRos(residual.translation()));
        response_residual.rotation = residual.rotation();
      }

      response->success = true;
      response->message = "Hand-eye calibration completed. " + result.toString();
    },
    response, node_.get_logger(), "HandEyeCalibrationCalibrate");
}

}  // namespace zivid_camera
