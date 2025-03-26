// Copyright 2025 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <test_zivid_camera.hpp>
#include <zivid_interfaces/srv/detect_calibration_board.hpp>
#include <zivid_interfaces/srv/detect_markers.hpp>

namespace
{
void verifyNonDefault(const geometry_msgs::msg::Quaternion & quaternion)
{
  ASSERT_NE(quaternion.x, 0);
  ASSERT_NE(quaternion.y, 0);
  ASSERT_NE(quaternion.z, 0);
  ASSERT_NE(quaternion.w, 1);
}

void verifyNonDefault(const geometry_msgs::msg::Point & point)
{
  ASSERT_NE(point.x, 0);
  ASSERT_NE(point.y, 0);
  ASSERT_NE(point.z, 0);
}

void verifyNonDefault(const geometry_msgs::msg::Pose & pose)
{
  verifyNonDefault(pose.position);
  verifyNonDefault(pose.orientation);
}
}  // namespace

class TestDetectorWithCalibrationBoard : public ZividNodeTest
{
protected:
  TestDetectorWithCalibrationBoard(NodeReusePolicy reusePolicy = NodeReusePolicy::AllowReuse)
  : ZividNodeTest{FileCameraMode::CalibrationBoard, reusePolicy}
  {
  }
};

TEST_F(ZividNodeTest, testDetectorDetectCalibrationBoardFailMissing)
{
  auto calibration_board =
    doEmptySrvRequest<zivid_interfaces::srv::DetectCalibrationBoard>("detect_calibration_board");
  verifyTriggerResponseError(calibration_board);
  static_assert(std::is_same_v<
                decltype(calibration_board->detection_result),
                zivid_interfaces::msg::DetectionResultCalibrationBoard>);
  ASSERT_EQ(
    calibration_board->detection_result.status,
    zivid_interfaces::msg::DetectionResultCalibrationBoard::
      STATUS_NO_VALID_FIDUCIAL_MARKER_DETECTED);
  ASSERT_FALSE(calibration_board->detection_result.status_description.empty());
  ASSERT_TRUE(calibration_board->detection_result.feature_points.empty());
  ASSERT_TRUE(calibration_board->detection_result.feature_points_2d.empty());
  ASSERT_EQ(calibration_board->detection_result.feature_points_width, 0);
  ASSERT_EQ(calibration_board->detection_result.feature_points_height, 0);
}

TEST_F(ZividNodeTest, testDetectorDetectMarkersFailParameters)
{
  setSingleDefaultAcquisitionSettingsUsingYml();
  auto response = doEmptySrvRequest<zivid_interfaces::srv::DetectMarkers>("detect_markers");
  verifyTriggerResponseError(response);
  ASSERT_PRED2(
    containsSubstring, response->message,
    "Invalid dictionary name \"\". Expected one of the following values:");
  ASSERT_TRUE(response->detection_result.detected_markers.empty());
}

TEST_F(ZividNodeTest, testDetectorDetectMarkersFailMissing)
{
  setSingleDefaultAcquisitionSettingsUsingYml();
  using Request = zivid_interfaces::srv::DetectMarkers::Request;
  auto request = std::make_shared<Request>();
  request->marker_dictionary = "aruco4x4_50";
  request->marker_ids = {1, 2, 3};
  auto response = doSrvRequest<zivid_interfaces::srv::DetectMarkers>("detect_markers", request);
  verifyTriggerResponseError(response);
  ASSERT_TRUE(response->detection_result.detected_markers.empty());
}

TEST_F(TestDetectorWithCalibrationBoard, testDetectorDetectCalibrationBoard)
{
  auto response =
    doEmptySrvRequest<zivid_interfaces::srv::DetectCalibrationBoard>("detect_calibration_board");
  verifyTriggerResponseSuccess(response);

  const auto & detection = response->detection_result;
  ASSERT_EQ(detection.status, zivid_interfaces::msg::DetectionResultCalibrationBoard::STATUS_OK);

  // Considering a Zivid calibration board (ZVDA-CB01)
  ASSERT_EQ(detection.feature_points.size(), 30);
  ASSERT_EQ(detection.feature_points_2d.size(), 30);
  ASSERT_EQ(detection.feature_points_width, 6);
  ASSERT_EQ(detection.feature_points_height, 5);

  verifyNonDefault(detection.centroid);
  verifyNonDefault(detection.pose);
}

TEST_F(TestDetectorWithCalibrationBoard, testDetectorDetectMarkers)
{
  setSingleDefaultAcquisitionSettingsUsingYml();

  const std::vector<int> marker_ids = {1, 2, 3};
  using Request = zivid_interfaces::srv::DetectMarkers::Request;
  auto request = std::make_shared<Request>();
  request->marker_dictionary = "aruco4x4_50";
  request->marker_ids = marker_ids;
  auto response = doSrvRequest<zivid_interfaces::srv::DetectMarkers>("detect_markers", request);
  verifyTriggerResponseSuccess(response);

  ASSERT_EQ(response->detection_result.allowed_marker_ids, marker_ids);
  ASSERT_EQ(response->detection_result.detected_markers.size(), 1);

  const auto & marker = response->detection_result.detected_markers.at(0);
  ASSERT_EQ(marker.id, 1);
  for (const auto & corner_in_camera_coordinates : marker.corners_in_camera_coordinates) {
    verifyNonDefault(corner_in_camera_coordinates);
  }
  for (const auto & corner_in_pixel_coordinates : marker.corners_in_pixel_coordinates) {
    ASSERT_NE(corner_in_pixel_coordinates.x, 0);
    ASSERT_NE(corner_in_pixel_coordinates.y, 0);
    ASSERT_EQ(corner_in_pixel_coordinates.z, 0);
  }
  verifyNonDefault(marker.pose);
}
