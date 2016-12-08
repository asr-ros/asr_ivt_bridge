/**

Copyright (c) 2016, Hutmacher Robin, Kleinert Daniel, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ivt_bridge/ivt_calibration.h>
#include <boost/make_shared.hpp>

#include <iostream>

namespace ivt_bridge {

cv::Mat_<double> matx34ToMat_(const cv::Matx<double, 3,4> &matx) {
	cv::Mat_<double>M(3,4);
	for (int i = 0; i < M.rows; ++i) {
		for (int j = 0; j < M.cols; ++j) {
			M(i,j) = matx(i,j);
		}
	}
	return M;
}

cv::Mat_<double> matx33ToMat_(const cv::Matx<double, 3,3> &matx) {
	cv::Mat_<double>M(3,3);
	for (int i = 0; i < M.rows; ++i) {
		for (int j = 0; j < M.cols; ++j) {
			M(i,j) = matx(i,j);
		}
	}
	return M;
}

cv::Mat_<double> matx35ToMat_(const cv::Matx<double, 3,5> &matx) {
	cv::Mat_<double>M(3,5);
	for (int i = 0; i < M.rows; ++i) {
		for (int j = 0; j < M.cols; ++j) {
			M(i,j) = matx(i,j);
		}
	}
	return M;
}


boost::shared_ptr<CCalibration> calibrationImpl(const image_geometry::PinholeCameraModel& cam_model, bool forRectifiedImages){
	boost::shared_ptr<CCalibration> calibration = boost::make_shared<CCalibration>();

	Mat3d ivtR; Vec3d ivtt;
	// Set with + height
	calibration->SetCameraParameters(0, 0, 0, 0, 0, 0, 0, 0, ivtR, ivtt, cam_model.rawRoi().width, cam_model.rawRoi().height);

	cv::Mat_<double> projectionMatrix = matx34ToMat_(cam_model.projectionMatrix());
	cv::Mat_<double> intrinsicMatrix = matx33ToMat_(cam_model.intrinsicMatrix());
	cv::Mat_<double> R = matx33ToMat_(cam_model.rotationMatrix());
	cv::Mat_<double> t;
	const cv::Mat_<double> distortion;

	if (forRectifiedImages) {
		cv::decomposeProjectionMatrix(projectionMatrix, intrinsicMatrix, R, t);
		// IVT calibrates in mm while ROS does it in m. (-1) determined empirical.
		t = t * -1000;
		calibration->SetDistortion(0, 0, 0, 0);
	} else {
		//intrinsicMatrix = cam_model.intrinsicMatrix();
		// Rec = P[0,3]^(-1) * H * K (src:Learning OpenCV Book) => H = P[0,3] * Rec * K^(-1)
		const cv::Mat_<double> H = projectionMatrix.colRange(0,3) * R * intrinsicMatrix.inv();
		// P^(*) = H * P = H * K * [R|t] (src:  Image processing, Analysis, and Machine vision) => [R|t] = K^(-1) * H^(-1) * P^(*)
		const cv::Mat_<double> Rt = intrinsicMatrix.inv() * H.inv() * projectionMatrix;
		R = Rt.colRange(0,3);
		// IVT calibrates in mm while ROS does it in m.
		t = Rt.col(3) * 1000;
		cv::Mat_<double> distortion = cam_model.distortionCoeffs();
		calibration->SetDistortion(distortion(0,0), distortion(0,1), distortion(0,2), distortion(0,3));
	}

	calibration->SetIntrinsicBase(intrinsicMatrix(0,2), intrinsicMatrix(1,2), intrinsicMatrix(0,0), intrinsicMatrix(1,1));

	Math3d::SetMat(ivtR,
			R(0,0), R(0,1), R(0,2),
			R(1,0), R(1,1), R(1,2),
			R(2,0), R(2,1), R(2,2));
	calibration->SetRotation(ivtR);

	Math3d::SetVec(ivtt, t(0,0), t(1,0), t(2,0));
	calibration->SetTranslation(ivtt);

	return calibration;

}

bool IvtCalibration::fromCameraInfo(const sensor_msgs::CameraInfo& msg){
	return cam_model.fromCameraInfo(msg);
}

bool IvtCalibration::fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg){
	return cam_model.fromCameraInfo(msg);
}

boost::shared_ptr<CCalibration> IvtCalibration::getCalibration(bool forRectifiedImages) const{
	return calibrationImpl(cam_model, forRectifiedImages);
}

bool IvtStereoCalibration::fromCameraInfo(const sensor_msgs::CameraInfo& left, const sensor_msgs::CameraInfo& right){
	return cam_model.fromCameraInfo(left, right);
}

bool IvtStereoCalibration::fromCameraInfo(const sensor_msgs::CameraInfoConstPtr& left, const sensor_msgs::CameraInfoConstPtr& right){
	return cam_model.fromCameraInfo(left, right);
}

boost::shared_ptr<CStereoCalibration> IvtStereoCalibration::getStereoCalibration(bool forRectifiedImages) const{
	boost::shared_ptr<CStereoCalibration> stereoCalibration = boost::make_shared<CStereoCalibration>();
	stereoCalibration->SetSingleCalibrations(*calibrationImpl(cam_model.left(), forRectifiedImages), *calibrationImpl(cam_model.right(), forRectifiedImages), true);

	Mat3d Hl, Hr;
	cv::Mat_<double> projectionMatrixLeft = matx34ToMat_(cam_model.left().projectionMatrix());
	cv::Mat_<double> projectionMatrixRight = matx34ToMat_(cam_model.right().projectionMatrix());
	cv::Mat_<double> intrinsicMatrixLeft = matx33ToMat_(cam_model.left().intrinsicMatrix());
	cv::Mat_<double> intrinsicMatrixRight = matx33ToMat_(cam_model.right().intrinsicMatrix());
	cv::Mat_<double> RLeft = matx33ToMat_(cam_model.left().rotationMatrix());
	cv::Mat_<double> RRight = matx33ToMat_(cam_model.right().rotationMatrix());

	if (forRectifiedImages) {
		// use identity
		Math3d::SetMat(Hl, 1,0,0,0,1,0,0,0,1);
		Math3d::SetMat(Hr, 1,0,0,0,1,0,0,0,1);
	} else {
		// ivt saves H inverted
		// Rec = P[0,3]^(-1) * H * K (src:Learning OpenCV Book) => H = P[0,3] * Rec * K^(-1)
		const cv::Mat_<double> rectificationMatrixLeft = (projectionMatrixLeft.colRange(0,3) * RLeft * intrinsicMatrixLeft.inv()).inv();
		Math3d::SetMat(Hl,
				rectificationMatrixLeft(0,0), rectificationMatrixLeft(0,1), rectificationMatrixLeft(0,2),
				rectificationMatrixLeft(1,0), rectificationMatrixLeft(1,1), rectificationMatrixLeft(1,2),
				rectificationMatrixLeft(2,0), rectificationMatrixLeft(2,1), rectificationMatrixLeft(2,2));

		const cv::Mat_<double> rectificationMatrixRight = (projectionMatrixRight.colRange(0,3) * RRight * intrinsicMatrixRight.inv()).inv();
		Math3d::SetMat(Hr,
				rectificationMatrixRight(0,0), rectificationMatrixRight(0,1), rectificationMatrixRight(0,2),
				rectificationMatrixRight(1,0), rectificationMatrixRight(1,1), rectificationMatrixRight(1,2),
				rectificationMatrixRight(2,0), rectificationMatrixRight(2,1), rectificationMatrixRight(2,2));
	}

	stereoCalibration->rectificationHomographyLeft = Hl;
	stereoCalibration->rectificationHomographyRight = Hr;

	return stereoCalibration;
}


} //namespace ivt_bridge
