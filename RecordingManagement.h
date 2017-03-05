/**
 * @file RecordingManagement.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __RECORDING_MANAGEMENT_FACTORY_H__
#define __RECORDING_MANAGEMENT_FACTORY_H__

#include "RecordingContextFactory.h"
#include <System/Mutex.h>
#include <System/SimpleList.h>

#ifdef KINECT_2
enum ToCompleteKinectFrameSourceTypes
{
	FrameSourceTypes_Face = 0x80,
	FrameSourceTypes_All = 0xff
};
#else
enum Kinect2StyleSourceType
    {
        FrameSourceTypes_None	= 0,
        FrameSourceTypes_Color	= 0x1,
        FrameSourceTypes_Infrared	= 0x2,				// Will generate an error on Kinectv1, not available
        FrameSourceTypes_LongExposureInfrared	= 0x4,  // Will generate an error on Kinectv1, not available
        FrameSourceTypes_Depth	= 0x8,
        FrameSourceTypes_BodyIndex	= 0x10,				// Will generate an warning on Kinectv1, body index is included in Depth
        FrameSourceTypes_Body	= 0x20,
        FrameSourceTypes_Audio	= 0x40,
		FrameSourceTypes_Face = 0x80
    } ;
#endif

enum ImgType {
	KS_UNK,			// Unkown
	KS_RGBA,		// RGBA, Kinect v1.x version
	KS_YUV2,		// YUV2, Kinect 2 version
	KS_UINT16_12,	// UINT12, generally depth data from the Kinect 1.x version
	KS_UINT16		// UINT16, depth data from the Kinect 2 version
};

class RecordingManagement
{
public:
	RecordingManagement()
	{
		IsRecording = false;
	};

	virtual ~RecordingManagement() {};
	
	double GetCurrentTime()
	{
		struct timeb lTimestamp;
		ftime(&lTimestamp);
		return((double)lTimestamp.time + (lTimestamp.millitm / 1000.0));
	}
	
	bool StartRecording(const char* sessionFolder);
	void StopRecording();

	void SaveTimestamp( FILE* f, const struct timeb& lTimestamp, unsigned int numFrame, TIMESPAN FrameTime, char * SuppInfo /* = nullptr */ );
	void SaveDataAndIncreaseInputNumber( KinectRecording& RecordContext, const struct timeb& lTimestamp, char * SuppInfo /* = nullptr */ );

// protected:
	Omiscid::ReentrantMutex ProtectAccess;

	bool IsRecording;

	Omiscid::SimpleList<KinectRecording *> RecContexts;
	KinectRecording* AddRecContext(const Omiscid::SimpleString& Prefix, const Omiscid::SimpleString& _FrameType, int TypeOfContext);
	void ClearRecContexts();
};



#endif // __RECORDING_MANAGEMENT_FACTORY_H__
