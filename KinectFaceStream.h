/**
 * @file KinectFaceStream.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_FACE_STREAM_H__
#define __KINECT_FACE_STREAM_H__

#ifdef KINECT_2

#include "KinectSensor-v2.h"

namespace MobileRGBD { namespace Kinect2 {

class KinectFaceStream : public Omiscid::Thread, public KinectExtraRecorder
{
public:
	KinectFaceStream(KinectSensor& KinectSensorCaller); 
	virtual ~KinectFaceStream() {}

	virtual void FUNCTION_CALL_TYPE Run();

	void Exchange(KinectBodies& CurrentBodies);


protected:
	UINT64	BodyTrackingIds[BODY_COUNT];
	bool	StartedSearch[BODY_COUNT];
};

}} // namespace MobileRGBD::Kinect2

#endif

#endif //__KINECT_FACE_STREAM_H__