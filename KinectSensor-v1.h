/**
 * @file KinectSensor-v1.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_SENSOR_V1_H__
#define __KINECT_SENSOR_V1_H__

#ifndef KINECT_2

#include <System/Thread.h>

#include "RecordingManagement.h"
#include "VideoKinectRecording.h"

#include <fstream>
#include <iostream>
#include <NuiApi.h>
#include <string>
#include <sstream>
#include <sys/timeb.h>
#include <time.h>
#include <vector>

namespace MobileRGBD { namespace Kinect1 {

#ifdef LIVE_RENDERING

	#include "../Drawing/DrawCameraView.h"
	#include "../Drawing/DrawSkeleton.h"

#endif // LIVE_RENDERING


class KinectSensorCommon
{
protected:
	int GatheredSources;

	Omiscid::Event DeviceFullyStarted;
};

class KinectSensor : public Omiscid::Thread, public RecordingManagement, public KinectSensorCommon
{
public:
	// standard constructor/destructor
    KinectSensor();
    ~KinectSensor();

	virtual void FUNCTION_CALL_TYPE Run();

	virtual void ProcessColorFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessDepthFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessBodyFrame(NUI_SKELETON_DATA* CurrentBodies, int NumBodies, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}

	// Face and audio will be implemented later
	// virtual void ProcessFaceFrame(KinectFace * CurrentFace, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	// virtual void ProcessAudioFrame(float * CurrentAudio, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}

	// Set properties needed to be added
   	bool Init( int DesiredSources );

	// Release the kinect device
	void Release();

	// Specific Kinet v1.x function

	// Move kinect tilt angle, only v1.x
	bool SetKinectAngle( LONG RequestedKinectAngle );	

	// Ask for near mode
	bool SetNearMode(bool NewMode)
	{
		if ( m_bNuiInitialized == true || IsRunning() == true )
		{
			fprintf( stderr, "Could not change NearMode after starting the device\n");
			return false;
		}
		NearMode = NewMode;
		return true;
	}

	// Ask for seated mode
	bool SetSeatedMode(bool NewMode)
	{
		if ( m_bNuiInitialized == true || IsRunning() == true )
		{
			fprintf( stderr, "Could not change NearMode after starting the device\n");
			return false;
		}
		SeatedMode = NewMode;
		return true;
	}

protected:

	bool NearMode;		// Near mode
	bool SeatedMode;	// Seated mode, i.e. upper body

	bool GetRawFrame( struct timeb& lTimestamp, HANDLE CurrentStream, VideoKinectRecording& RecContext, int NumberOfBytesPerPixels );

	// Avants for streams
	enum { DepthFrameEvent, VideoFrameEvent, SkeletonFrameEvent, AudioEvent, NumberOfEvents };
	HANDLE      FrameEvents[NumberOfEvents];

    HANDLE      m_pDepthStreamHandle;
    HANDLE      m_pVideoStreamHandle;

    bool        m_bNuiInitialized;

	NUI_SKELETON_FRAME SkeletonFrame;
    void ProcessSkeletonFrame(struct timeb& lTimestamp, HANDLE CurrentStream, VideoKinectRecording& RecContext );
	void SaveSkeletonsInfo(NUI_SKELETON_DATA * _skeletonData , int _nbSkel, struct timeb& lTimestamp);
};

}} // namespace MOBILERGBD::Kinect1

#endif // KINECT_2

#endif // __KINECT_SENSOR_V1_H__



