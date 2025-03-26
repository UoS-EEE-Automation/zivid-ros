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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <zivid_interfaces/srv/detect_calibration_board.hpp>
#include <zivid_interfaces/srv/detect_markers.hpp>

namespace Zivid
{
class Camera;
}

namespace zivid_camera
{
template <typename SettingsType>
class CaptureSettingsController;

class DetectorController
{
public:
  DetectorController(
    rclcpp::Node & zivid_camera_node, Zivid::Camera & camera,
    CaptureSettingsController<Zivid::Settings> & settings_controller);
  ~DetectorController();

private:
  void detectCalibrationBoard(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::DetectCalibrationBoard::Request> request,
    std::shared_ptr<zivid_interfaces::srv::DetectCalibrationBoard::Response> response);
  void detectMarkers(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<zivid_interfaces::srv::DetectMarkers::Request> request,
    std::shared_ptr<zivid_interfaces::srv::DetectMarkers::Response> response);

  rclcpp::Node & node_;
  Zivid::Camera & camera_;
  CaptureSettingsController<Zivid::Settings> & settings_controller_;

  rclcpp::Service<zivid_interfaces::srv::DetectCalibrationBoard>::SharedPtr
    detect_calibration_board_;
  rclcpp::Service<zivid_interfaces::srv::DetectMarkers>::SharedPtr detect_markers_;
};
}  // namespace zivid_camera
