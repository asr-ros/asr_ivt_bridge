/**

Copyright (c) 2016, Hutmacher Robin, Kleinert Daniel, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef IVT_BRIDGE_IVT_CALIBRATION_H
#define IVT_BRIDGE_IVT_CALIBRATION_H

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>
#include <Calibration/Calibration.h>
#include <Calibration/StereoCalibration.h>
#include <stdexcept>
#include <opencv2/core/core.hpp>

namespace ivt_bridge {

  /**
   * IVT IvtCalibration class. Conversion between representation of calibration parameters for mono camera as ROS message and IVT datastructure.
   *
   * @author Pascal Meissner, Daniel Kleinert.
   * @version See SVN.
   */

class IvtCalibration {
public:

	/**
	* \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
	*/
	bool fromCameraInfo(const sensor_msgs::CameraInfo& msg);

	/**
	* \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
	*/
	bool fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

	/**
	* \brief Get the IVT Calibration.
	*
	* \param forRectifiedImages If true the calibration will be for rectified images of ros::image_proc, otherwise for a (colored) raw image
	*/
	boost::shared_ptr<CCalibration> getCalibration(bool forRectifiedImages=false) const;


private:
	image_geometry::PinholeCameraModel cam_model;
};

  /**
   * IVT IvtStereoCalibration class. Conversion between representation of calibration parameters for stereo cameras as ROS message and IVT datastructure.
   *
   * @author Pascal Meissner, Daniel Kleinert.
   * @version See SVN.
   */

class IvtStereoCalibration {
public:

	/**
	* \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
	*/
	bool fromCameraInfo(const sensor_msgs::CameraInfo& left, const sensor_msgs::CameraInfo& right);

	/**
	* \brief Set the camera parameters from the sensor_msgs/CameraInfo messages.
	*/
	bool fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& left, const sensor_msgs::CameraInfoConstPtr& right);

	/**
	* \brief Get the IVT StereoCalibration.
	*
	* \param forRectifiedImages If true the calibration will be for rectified images of ros::image_proc, otherwise for a (colored) raw image
	*/
	boost::shared_ptr<CStereoCalibration> getStereoCalibration(bool forRectifiedImages=false) const;
    
private:
	image_geometry::StereoCameraModel cam_model;
};

} // namespace ivt_bridge


#endif
