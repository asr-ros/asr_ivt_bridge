/**

Copyright (c) 2016, Hutmacher Robin, Kleinert Daniel, Meißner Pascal
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef IVT_BRIDGE_IVT_IMAGE_H
#define IVT_BRIDGE_IVT_IMAGE_H

#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <Image/ByteImage.h>
#include <stdexcept>

namespace ivt_bridge {

class Exception: public std::runtime_error {
public:
	Exception(const std::string& description) :
		std::runtime_error(description) {
	}
};

class IvtImage;

typedef boost::shared_ptr<IvtImage> IvtImagePtr;
typedef boost::shared_ptr<IvtImage const> IvtImageConstPtr;

/**
 * \brief Image message class that is interoperable with sensor_msgs/Image but uses a CByteImage representation for the image data.
 *
 * @author Pascal Meissner, Daniel Kleinert.
 * @version See SVN.
 */
class IvtImage {
public:
	std_msgs::Header header; //!< ROS header
	std::string encoding; //!< Image encoding ("mono8", "bgr8", etc.)
	CByteImage *image; //!< Image data for use with Ivt

	~IvtImage();

	/**
	 * \brief Convert this message to a ROS sensor_msgs::Image message.
	 *
	 * The returned sensor_msgs::Image message contains a copy of the image data.
	 */
	sensor_msgs::ImagePtr toImageMsg() const;

	/**
	 * \brief Copy the message data to a ROS sensor_msgs::Image message.
	 *
	 * This overload is intended mainly for aggregate messages such as stereo_msgs::DisparityImage,
	 * which contains a sensor_msgs::Image as a data member.
	 */
	void toImageMsg(sensor_msgs::Image& ros_image) const;

protected:
	boost::shared_ptr<void const> tracked_object_; // for sharing ownership, hold on to image message so it doesn't get released

	/// @cond DOXYGEN_IGNORE
	friend IvtImageConstPtr toIvtShare(const sensor_msgs::Image& source, const boost::shared_ptr<void const>& tracked_object, const std::string& encoding);
	/// @endcond
};

/**
 * \brief Convert a sensor_msgs::Image message to an Ivt-compatible CByteImage, copying the
 * image data.
 *
 * \param source   A shared_ptr to a sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "rgb8"
 *
 * If \a encoding is the empty string (the default), the returned IvtImage has the same encoding
 * as \a source.
 */
IvtImagePtr toIvtCopy(const sensor_msgs::ImageConstPtr& source, const std::string& encoding = std::string());

/**
 * \brief Convert a sensor_msgs::Image message to an Ivt-compatible CByteImage, copying the
 * image data.
 *
 * \param source   A sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "rgb8"
 *
 * If \a encoding is the empty string (the default), the returned IvtImage has the same encoding
 * as \a source.
 */
IvtImagePtr toIvtCopy(const sensor_msgs::Image& source, const std::string& encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::Image message to an Ivt-compatible CByteImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned IvtImage will share
 * the image data with \a source without copying it. The returned IvtImage cannot be modified, as that
 * could modify the \a source data.
 *
 * \param source   A shared_ptr to a sensor_msgs::Image message
 * \param encoding The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "rgb8"
 *
 */
IvtImageConstPtr toIvtShare(const sensor_msgs::ImageConstPtr& source, const std::string& encoding = std::string());

/**
 * \brief Convert an immutable sensor_msgs::Image message to an Ivt-compatible CByteImage, sharing
 * the image data if possible.
 *
 * If the source encoding and desired encoding are the same, the returned IvtImage will share
 * the image data with \a source without copying it. The returned IvtImage cannot be modified, as that
 * could modify the \a source data.
 *
 * This overload is useful when you have a shared_ptr to a message that contains a
 * sensor_msgs::Image, and wish to share ownership with the containing message.
 *
 * \param source         The sensor_msgs::Image message
 * \param tracked_object A shared_ptr to an object owning the sensor_msgs::Image
 * \param encoding       The desired encoding of the image data, one of the following strings:
 *    - \c "mono8"
 *    - \c "rgb8"
 *
 */
IvtImageConstPtr toIvtShare(const sensor_msgs::Image& source, const boost::shared_ptr<void const>& tracked_object, const std::string& encoding = std::string());

/**
 * \brief Convert a IvtImage to another encoding.
 */
//IvtImagePtr cvtColor(const IvtImageConstPtr& source, const std::string& encoding);

/**
 * \brief Get the OpenCV type enum corresponding to the encoding.
 *
 * For example, "bgr8" -> CV_8UC3.
 */
int getCvType(const std::string& encoding);

/**
 * \brief Get the CByteImage type enum corresponding to the encoding.
 *
 * For example, rgb8 -> eRGB24.
 */
CByteImage::ImageType getIvtType(const std::string& encoding);

} // namespace ivt_bridge


#endif
