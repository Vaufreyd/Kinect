/**
 * @file KinectRGBStream.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_RGB_STREAM_H__
#define __KINECT_RGB_STREAM_H__

#ifdef KINECT_2

#include "KinectSensor-v2.h"

namespace MobileRGBD { namespace Kinect2 {

class KinectRGBStream: public Omiscid::Thread, public KinectExtraRecorder
{
public:
	KinectRGBStream (KinectSensor& KinectSensorCaller) : KinectExtraRecorder(KinectSensorCaller)
	{
	}

	virtual void FUNCTION_CALL_TYPE Run();
};

}} // namespace MobileRGBD::Kinect2

#endif

#endif //__KINECT_RGB_STREAM_H__