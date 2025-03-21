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

#include <ctime>
#include <std_srvs/srv/trigger.hpp>
#include <test_zivid_camera.hpp>
#include <zivid_interfaces/srv/infield_correction_capture.hpp>
#include <zivid_interfaces/srv/infield_correction_compute.hpp>
#include <zivid_interfaces/srv/infield_correction_read.hpp>
#include <zivid_interfaces/srv/infield_correction_verify.hpp>

class TestWithCalibrationBoard : public ZividNodeTest
{
protected:
  TestWithCalibrationBoard(NodeReusePolicy reusePolicy = NodeReusePolicy::AllowReuse)
  : ZividNodeTest{FileCameraMode::CalibrationBoard, reusePolicy}
  {
  }

  static void verifyInfieldComputeSuccess(
    const std::shared_ptr<zivid_interfaces::srv::InfieldCorrectionCompute::Response> & response,
    int number_of_captures, bool compute_and_write = false)
  {
    const std::string operation_message =
      (compute_and_write ? "successfully written to camera" : "computed successfully");
    ASSERT_TRUE(response->success);
    ASSERT_PRED2(
      containsSubstring, response->message,
      "Camera correction " + operation_message +
        ".\nNumber of captures: " + std::to_string(number_of_captures) + "\nCamera metrics:\n");
    ASSERT_EQ(response->infield_correction_started, true);
    ASSERT_EQ(response->number_of_captures, number_of_captures);
    ASSERT_GT(response->average_trueness_error, 0.0);
    ASSERT_LT(response->average_trueness_error, 1.0);
    ASSERT_EQ(response->average_trueness_error, response->current_trueness_error);
    ASSERT_EQ(response->average_trueness_error, response->maximum_trueness_error);
    ASSERT_GT(response->dimension_accuracy, 0.0);
    ASSERT_LT(response->dimension_accuracy, 1.0);
    ASSERT_GT(response->z_min, 0.0);
    ASSERT_GT(response->z_max, response->z_min);
  }
};

class TestWithCalibrationBoardFreshNode : public TestWithCalibrationBoard
{
protected:
  TestWithCalibrationBoardFreshNode() : TestWithCalibrationBoard{NodeReusePolicy::RestartNode} {}

  static void verifyYearsOfTimeSinceEpoch(const builtin_interfaces::msg::Time & time)
  {
    const auto seconds_since_epoch = static_cast<std::time_t>(time.sec);
    std::tm * tm_info = std::localtime(&seconds_since_epoch);
    const int year = tm_info->tm_year + 1900;
    ASSERT_GE(year, 2025);
    ASSERT_LE(year, 2125);  // This needs to be updated in 100 years.
  }
};

TEST_F(TestWithFileCamera, testInfieldCorrectionCaptureFailed)
{
  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_FALSE(capture->success);
  ASSERT_EQ(
    capture->status,
    zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_DETECTION_FAILED);
  ASSERT_EQ(capture->number_of_captures, 0);
  ASSERT_EQ(
    capture->message,
    "Invalid detection. This measurement will not be used. Feedback: Failed to detect calibration "
    "object. Detection failed because no fiducial markers corresponding to a Zivid calibration "
    "board were found. Ensure the calibration board and its fiducial marker are both fully "
    "visible.");
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionCaptureOk)
{
  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);
  ASSERT_EQ(capture->status, zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_OK);
  ASSERT_EQ(capture->number_of_captures, 1);
}

TEST_F(TestWithCalibrationBoard, testInfieldCorrectionStart)
{
  auto response = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(response);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionStartRequiredBeforeCapture)
{
  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_FALSE(capture->success);
  ASSERT_EQ(
    capture->status, zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_UNKNOWN);
  ASSERT_EQ(capture->number_of_captures, 0);
  ASSERT_EQ(
    capture->message,
    "Infield correction not started. Please call the infield correction start service before "
    "capturing.");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);
  ASSERT_EQ(capture->status, zivid_interfaces::srv::InfieldCorrectionCapture::Response::STATUS_OK);
  ASSERT_EQ(capture->number_of_captures, 1);
  ASSERT_PRED2(
    containsSubstring, capture->message,
    "Valid detection. Collected 1 valid measurement(s) so far. Measured positions:");
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionRestart)
{
  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);

  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_EQ(compute->number_of_captures, 1);

  start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_EQ(compute->number_of_captures, 0);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionCompute)
{
  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_FALSE(compute->success);
  ASSERT_EQ(compute->infield_correction_started, false);
  ASSERT_EQ(compute->number_of_captures, 0);
  ASSERT_EQ(
    compute->message, "Cannot compute in-field correction with empty data set. { dataset: {  } }");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  verifyTriggerResponseSuccess(start);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_FALSE(compute->success);
  ASSERT_EQ(compute->infield_correction_started, true);
  ASSERT_EQ(compute->number_of_captures, 0);
  ASSERT_EQ(
    compute->message, "Cannot compute in-field correction with empty data set. { dataset: {  } }");

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  verifyInfieldComputeSuccess(compute, 1);

  capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  verifyInfieldComputeSuccess(compute, 2);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionReadWriteResetCycle)
{
  auto read =
    doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionRead>("infield_correction/read");
  ASSERT_TRUE(read->success);
  ASSERT_FALSE(read->has_camera_correction);
  ASSERT_EQ(read->camera_correction_timestamp.sec, 0);
  ASSERT_EQ(read->camera_correction_timestamp.nanosec, 0);
  ASSERT_EQ(read->message, "This camera has no in-field correction written to it.");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  ASSERT_TRUE(start->success);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);

  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute_and_write");
  verifyInfieldComputeSuccess(compute, 1, true);

  read = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionRead>("infield_correction/read");
  ASSERT_TRUE(read->success);
  ASSERT_TRUE(read->has_camera_correction);
  ASSERT_GT(read->camera_correction_timestamp.sec, 0);
  ASSERT_EQ(read->camera_correction_timestamp.nanosec, 0);
  verifyYearsOfTimeSinceEpoch(read->camera_correction_timestamp);
  ASSERT_PRED2(containsSubstring, read->message, "Timestamp of the current camera correction: ");

  auto reset = doStdSrvsTriggerRequest("infield_correction/reset");
  ASSERT_TRUE(reset->success);
  ASSERT_EQ(reset->message, "");

  read = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionRead>("infield_correction/read");
  ASSERT_TRUE(read->success);
  ASSERT_FALSE(read->has_camera_correction);
  ASSERT_EQ(read->camera_correction_timestamp.sec, 0);
  ASSERT_EQ(read->camera_correction_timestamp.nanosec, 0);
  ASSERT_EQ(read->message, "This camera has no in-field correction written to it.");
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionRemoveLastCapture)
{
  auto remove_last_capture = doStdSrvsTriggerRequest("infield_correction/remove_last_capture");
  ASSERT_FALSE(remove_last_capture->success);
  ASSERT_EQ(remove_last_capture->message, "Infield correction dataset is empty");

  auto start = doStdSrvsTriggerRequest("infield_correction/start");
  ASSERT_TRUE(start->success);

  auto capture = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  ASSERT_TRUE(capture->success);

  auto compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  verifyInfieldComputeSuccess(compute, 1);

  remove_last_capture = doStdSrvsTriggerRequest("infield_correction/remove_last_capture");
  ASSERT_TRUE(remove_last_capture->success);
  ASSERT_EQ(remove_last_capture->message, "");

  compute = doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  ASSERT_FALSE(compute->success);
  ASSERT_EQ(compute->infield_correction_started, true);
  ASSERT_EQ(compute->number_of_captures, 0);
}

TEST_F(TestWithCalibrationBoardFreshNode, testInfieldCorrectionVerify)
{
  auto response =
    doEmptySrvRequest<zivid_interfaces::srv::InfieldCorrectionVerify>("infield_correction/verify");
  ASSERT_TRUE(response->success);
  ASSERT_PRED2(containsSubstring, response->message, "Successful measurement at");
  ASSERT_PRED2(
    containsSubstring, response->message,
    "Estimated dimension trueness error at measured position: ");
  ASSERT_NE(response->position.x, 0.0);
  ASSERT_NE(response->position.y, 0.0);
  ASSERT_NE(response->position.z, 0.0);
  ASSERT_GT(response->local_dimension_trueness, 0.0);
}
