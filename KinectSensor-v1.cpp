/**
 * @file KinectSensor-v1.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef KINECT_2

#include "KinectSensor-v1.h"

#ifdef DO_FADO_FACE_TRACKING
	#include <FaceTrackLib.h>
#endif

#include <tchar.h>
#include <math.h>
#include <iostream>
#define BUFSIZE MAX_PATH

using namespace std;

namespace MobileRGBD { namespace Kinect1 {

KinectSensor::KinectSensor()
{
	// Create all events
	for( auto i = 0; i < NumberOfEvents; i++ )
	{
		FrameEvents[i] = CreateEvent(NULL, TRUE, FALSE, NULL);
	}

	m_pDepthStreamHandle = NULL;
	m_pVideoStreamHandle = NULL;
	m_bNuiInitialized = false;

	GatheredSources = FrameSourceTypes_None;
	NearMode = false;
	SeatedMode = false;
}

KinectSensor::~KinectSensor()
{
	Release();
}


bool KinectSensor::SetKinectAngle( LONG RequestedKinectAngle )
{
	if ( RequestedKinectAngle < NUI_CAMERA_ELEVATION_MINIMUM || RequestedKinectAngle > NUI_CAMERA_ELEVATION_MAXIMUM )
	{
		return false;
	}

	// Put Kinect tilt position (elevation) at RequestedKinectAngle° if needed
	// Set CameraAngle to impossible value
	LONG CameraAngle = (NUI_CAMERA_ELEVATION_MINIMUM-1);

	// retrieve camera angle
	NuiCameraElevationGetAngle( &CameraAngle );

	// If camera angle is not suitable for us, move it to the right position
	if ( CameraAngle != RequestedKinectAngle )
	{
		NuiCameraElevationSetAngle( RequestedKinectAngle );
		Omiscid::Thread::Sleep( 1000 );
	}

	return true;
}

bool KinectSensor::Init( int DesiredSources )
{
	HRESULT hr = NOERROR;

	GatheredSources = DesiredSources;

	if ( GatheredSources & (FrameSourceTypes_Infrared|FrameSourceTypes_LongExposureInfrared) )
	{
		fprintf( stderr, "Kinect v1.X does not support Infrared Streams\n");
		return false;
	}

	Release(); // Deal with double initializations.

	const NUI_IMAGE_TYPE colorType = NUI_IMAGE_TYPE_COLOR;
	const NUI_IMAGE_TYPE depthType = NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX;


	// KINECT INIT
#ifdef PROCESS_AUDIO
	hr = NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_AUDIO );
#else
	hr = NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_COLOR );
#endif 

	if (FAILED(hr))
	{
		return false;
	}
	m_bNuiInitialized = true;

	// Depth event init
	hr = NuiImageStreamOpen( depthType, NUI_IMAGE_RESOLUTION_640x480, (NearMode)? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0, 2, FrameEvents[DepthFrameEvent], &m_pDepthStreamHandle );

	if ( FAILED(hr) )
	{
		return false;
	}

	// Video event init
	hr = NuiImageStreamOpen( colorType, NUI_IMAGE_RESOLUTION_640x480, 0, 2, FrameEvents[VideoFrameEvent], &m_pVideoStreamHandle ); 

	if (FAILED(hr))
	{
		return false;
	}

	// Skeleton event init
	DWORD dwSkeletonFlags = NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE;
	if ( SeatedMode == true )
	{
		dwSkeletonFlags |= NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
	}
	hr = NuiSkeletonTrackingEnable( FrameEvents[SkeletonFrameEvent], dwSkeletonFlags );
	if (FAILED(hr))
	{
		return false;
	}

	// Start pump data
	DeviceFullyStarted.Reset();
	if ( StartThread() == false   )
	{
		return false;
	}

	// Wait 20s for the thread to fully start its jobs
	if ( DeviceFullyStarted.Wait(20000) == false )
	{
		fprintf( stderr, "Initialisation of the device failed\n" );
		return false;
	}

	return true;
}

void KinectSensor::Release()
{
	// Stop the Nui processing thread
	StopThread();

	StopRecording();

	// Destroy handle all events
	for( auto i = 0; i < NumberOfEvents; i++ )
	{
		if ( FrameEvents[i] != INVALID_HANDLE_VALUE )
		{
			CloseHandle( FrameEvents[i] );
			FrameEvents[i] = NULL;
		}
	}
	
	if ( m_bNuiInitialized == true )
	{
		NuiShutdown();
	}
	m_bNuiInitialized = false;
}

void FUNCTION_CALL_TYPE KinectSensor::Run()
{

	// Define all RecContext to this 
	// KinectRecording* ColorRecContext = nullptr;
	KinectRecording* ColorRecContext = nullptr;
	KinectRecording* DepthRecContext = nullptr;
	KinectRecording* BodyRecContext = nullptr;
	KinectRecording* FaceRecContext = nullptr;

	if ( GatheredSources & FrameSourceTypes_BodyIndex ) { fprintf( stderr, "BodyIndex (aka player detection) is enbedded in the Depth map"); }

	// Activate Mandatory Recording Contexts
	if ( GatheredSources & FrameSourceTypes_Color ) { ColorRecContext = reinterpret_cast<VideoKinectRecording*>(AddRecContext( "video", "RGBA", RecordingContextFactory::VideoRecContext)); }
	if ( GatheredSources & FrameSourceTypes_Depth ) { DepthRecContext = AddRecContext( "depth", "UINT16", RecordingContextFactory::VideoRecContext);  }
	if ( GatheredSources & FrameSourceTypes_Body )
	{
		BodyRecContext = AddRecContext( "skeleton", "KinectBody", RecordingContextFactory::VideoRecContext);
		if ( GatheredSources & FrameSourceTypes_Face )
		{
			// Create face context, no concurrency between thread, KFT.StartThread did not occur yet
			FaceRecContext = reinterpret_cast<VideoKinectRecording*>(AddRecContext( "face", "KinectFace", RecordingContextFactory::VideoRecContext));
		}
	}

	// Here everyting is initialized
	DeviceFullyStarted.Signal();

	//Timestamp
	struct timeb lTimestamp;

	// forever
	while(StopPending() != true)
	{
		// Wait for an event to be signaled
		WaitForMultipleObjects( NumberOfEvents, FrameEvents, FALSE, 20);

		// Get time of this event (from Client side)
		ftime(&lTimestamp);

		// Process signal events
		if (WAIT_OBJECT_0 == WaitForSingleObject(FrameEvents[DepthFrameEvent], 0))
		{
			VideoKinectRecording& rcs = *reinterpret_cast<VideoKinectRecording*>(DepthRecContext);
			if ( GetRawFrame(lTimestamp, m_pDepthStreamHandle, rcs, 2 /* Bytes per pixels */ ) )
			{
				ProcessDepthFrame(rcs.BufferData, rcs.FrameLengthInPixels, rcs.Width, rcs.Height, KS_UINT16_12, rcs.InputNumber, lTimestamp, rcs.LastFrameTime);
			}
		}

		// VIDEO
		if ( WAIT_OBJECT_0 == WaitForSingleObject(FrameEvents[VideoFrameEvent], 0) )
		{
			VideoKinectRecording& rcs = *reinterpret_cast<VideoKinectRecording*>(ColorRecContext);
			if ( GetRawFrame(lTimestamp, m_pVideoStreamHandle, rcs, 4 /* Bytes per pixels */ ) )
			{
				ProcessColorFrame(rcs.BufferData, rcs.FrameLengthInPixels, rcs.Width, rcs.Height, KS_RGBA, rcs.InputNumber, lTimestamp, rcs.LastFrameTime);
			}	


		}

		//SKELETON
		if ( WAIT_OBJECT_0 == WaitForSingleObject(FrameEvents[SkeletonFrameEvent], 0) )
		{
			ProcessSkeletonFrame(lTimestamp, m_pDepthStreamHandle, *reinterpret_cast<VideoKinectRecording*>(ColorRecContext) );

#ifdef LIVE_RENDERING
			for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
			{
				if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
				{
					DrawSkel.DrawElement( ImgDepth, &SkeletonFrame.SkeletonData[i] );
				}
			}
#endif // LIVE_RENDERING
		}
	}

	return;
}

bool KinectSensor::GetRawFrame( struct timeb& lTimestamp, HANDLE CurrentStream, VideoKinectRecording& RecContext, int NumberOfBytesPerPixels )
{
	if ( RecContext.InitDescription == true)
	{
		DWORD width = 0;
		DWORD height = 0;
		NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);
				
		RecContext.StartTime = (double)lTimestamp.time + (((double)lTimestamp.millitm)/1000.0);
		RecContext.Width = width;
		RecContext.Height = height;
		RecContext.BytesPerPixel = NumberOfBytesPerPixels;
		RecContext.BufferSize = width*height*RecContext.BytesPerPixel;
		RecContext.FrameLengthInPixels = width*height*RecContext.BytesPerPixel;
		RecContext.InitDescription = false;
	}

	const NUI_IMAGE_FRAME* pImageFrame = NULL;

	if ( FAILED(NuiImageStreamGetNextFrame(CurrentStream, 0, &pImageFrame)) ) return false;

	// Get the internal timestamp
	RecContext.LastFrameTime = pImageFrame->liTimeStamp.QuadPart;

	NUI_LOCKED_RECT LockedRect;
	pImageFrame->pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	if ( LockedRect.Pitch != 0 )
	{  
		// Copy depth frame to face tracking
		memcpy( RecContext.BufferData, PBYTE(LockedRect.pBits), RecContext.FrameLengthInPixels );
	}
	else
	{
		fprintf( stderr, "Buffer length of received texture is bogus\r\n" );
		NuiImageStreamReleaseFrame(CurrentStream, pImageFrame);
		return false;
	}

	// Release current frame
	NuiImageStreamReleaseFrame(CurrentStream, pImageFrame);

	// Save data if mandatory
	if ( IsRecording == true )
	{
		// Save data
		SaveDataAndIncreaseInputNumber( RecContext, lTimestamp, nullptr );
	}

	return true;
}

void KinectSensor::ProcessSkeletonFrame( struct timeb& lTimestamp, HANDLE CurrentStream, VideoKinectRecording& RecContext )
{
	HRESULT hr = NuiSkeletonGetNextFrame(0, &SkeletonFrame);
	if(FAILED(hr))
	{
		return;
	}

	// Count tracked skeletons
	unsigned int NumberOfSkeletons = 0;
	for( auto i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
	{
		if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
		{
			NumberOfSkeletons++;	
		}
	}

	if ( NumberOfSkeletons != 0 && IsRecording == true )
	{
		SaveSkeletonsInfo( SkeletonFrame.SkeletonData, NumberOfSkeletons, lTimestamp );
	}


	if ( NumberOfSkeletons != 0 )
	{
		 ProcessBodyFrame(SkeletonFrame.SkeletonData, NumberOfSkeletons, 0, lTimestamp, 0);
	}
	else
	{
		 ProcessBodyFrame(nullptr, 0, 0, lTimestamp, 0);
	}
}

void KinectSensor::SaveSkeletonsInfo( NUI_SKELETON_DATA * _skeletonData, int _nbSkel, struct timeb& lTimestamp )
{
	if ( IsRecording == false )
	{
		return;
	}

	// To do
}

}} // MobileRGBD::Kinect1

#endif // #ifndef KINECT_2
