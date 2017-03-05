/**
 * @file KinectSensorCommon.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_SENSOR_COMMON_H__
#define __KINECT_SENSOR_COMMON_H__

#include "System/Portage.h"
#include "System/Event.h"

class KinectSensorCommon
{
public:
	enum ImgType {
		KS_UNK,			// Unkown
		KS_RGBA,		// RGBA
		KS_YUV2,		// YUV2
		KS_UINT16_12,	// UINT12, generally depth data from the Kinect 1.x version
		KS_UINT16		// UINT16, depth data from the Kinect 2 version
	};

protected:
	int GatheredSources;

	Omiscid::Event DeviceFullyStarted;
};

#endif // __KINECT_SENSOR_COMMON_H__