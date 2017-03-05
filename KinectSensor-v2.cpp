#ifdef KINECT_2

#include "System/ElapsedTime.h"

#include "KinectSensor-v2.h"
#include "RecordingContextFactory.h"
#include "KinectFaceStream.h"
#include "KinectRGBStream.h"
#include "KinectAudioStream.h"

// #define RESIZE_KINECT_RGB 2

#ifdef RESIZE_KINECT_RGB
#include "KinectImageConverter.h"
#endif

using namespace MobileRGBD;
using namespace MobileRGBD::Kinect2;

namespace MobileRGBD { namespace Kinect2 {

KinectSensor::KinectSensor()
{
	pSensor = nullptr; 
	GatheredSources = FrameSourceTypes::FrameSourceTypes_None;

	IsRecording = false;

	FloorEstimationProcess = NoEstimation;
	AngleWithFloor = 0;
	NumberOfIdenticalFloorEStimation = 0;
}

KinectSensor::~KinectSensor()
{
	StopThread();

	Join(5000);

	StopRecording();

	while( RecContexts.IsNotEmpty() )
	{
		delete RecContexts.ExtractFirst();
	}
}

void FUNCTION_CALL_TYPE KinectSensor::Run()
{
	SetThreadPriority( GetCurrentThread(), THREAD_PRIORITY_HIGHEST );

	WAITABLE_HANDLE hMultiSource = NULL;

	struct timeb lTimestamp;

	KinectAudioStream AudioStream(*this);
	KinectFaceStream FaceStream(*this);
	KinectRGBStream RGBStream(*this);

	//// Initialize the Kinect and get the depth reader
	//IDepthFrameSource* pDepthFrameSource = NULL;
	if ( FAILED(pSensor->Open()) )
	{
		fprintf(stderr, "Could not get open Kinect\n");
		InitStepDone.Data = false;
		InitStepDone.Signal();
		return;
	}

	// "Empty" Fake Kinect recording, help us to call StartRecording without any sources
	KinectRecording * pEmptyKinectRecording = RecordingContextFactory::CreateContextRecording( "","", RecordingContextFactory::DummyRecContext );

	// Define all RecContext to this 
	// KinectRecording* ColorRecContext = nullptr;
	KinectRecording* DepthRecContext = nullptr;
	KinectRecording* InfraredRecContext = nullptr;
	KinectRecording* LongExposureInfraredRecContext = nullptr;
	KinectRecording* BodyIndexRecContext = nullptr;
	KinectRecording* BodyRecContext = nullptr;
	bool BodyWasSentPreviously = false;

#ifdef GET_DEPTH_MAX_VALUES
	IDepthFrameSource  * pDep;
	pSensor->get_DepthFrameSource(&pDep);
	UINT16 MinVal, MaxVal;
	pDep->get_DepthMaxReliableDistance(&MaxVal);
	pDep->get_DepthMinReliableDistance(&MinVal);
#endif

		
	// If we want Face tracking, we need to enable Body tracking
	if ( GatheredSources & FrameSourceTypes_Face )
	{
		// add body tracking
		GatheredSources |= FrameSourceTypes_Body;
	}
	
	// RGB stream is managed by an external recorder
	if ( GatheredSources & FrameSourceTypes_Color )
	{
		RGBStream.SetRecContext( reinterpret_cast<RawKinectRecording*>(AddRecContext( "video", "YUY2", RecordingContextFactory::VideoRecContext)) );
		RGBStream.StartThread();
	}

	if ( GatheredSources & FrameSourceTypes_Depth ) {  DepthRecContext = AddRecContext( "depth", "UINT16", RecordingContextFactory::VideoRecContext);  }
	if ( GatheredSources & FrameSourceTypes_Infrared ) { InfraredRecContext = AddRecContext( "infrared", "UINT16", RecordingContextFactory::VideoRecContext); }
	if ( GatheredSources & FrameSourceTypes_LongExposureInfrared ) { LongExposureInfraredRecContext = AddRecContext( "longexp_infrared", "UINT16", RecordingContextFactory::VideoRecContext); }
	if ( GatheredSources & FrameSourceTypes_BodyIndex ) { BodyIndexRecContext = AddRecContext( "body_index", "UINT8", RecordingContextFactory::VideoRecContext); }
	if ( GatheredSources & FrameSourceTypes_Body )
	{
		BodyRecContext = AddRecContext( "skeleton", "KinectBody", RecordingContextFactory::VideoRecContext);

		// Face stream is managed by an external recorder
		if ( GatheredSources & FrameSourceTypes_Face )
		{
			// Create face context, no concurrency between thread, FaceStream.StartThread did not occur yet
			FaceStream.SetRecContext( reinterpret_cast<RawKinectRecording*>(AddRecContext( "face", "KinectFace", RecordingContextFactory::VideoRecContext)) );

			// Open Face reader thread
			FaceStream.StartThread();
		}
	}

	// Audio stream is managed by an external recorder
	if ( GatheredSources & FrameSourceTypes_Audio )
	{
		AudioStream.SetRecContext( reinterpret_cast<RawKinectRecording*>(AddRecContext( "audio", "AudioSample", RecordingContextFactory::VideoRecContext)) );
				
		// Open audio reader thread
		AudioStream.StartThread();
	}

	IMultiSourceFrameReader * pFrameReader = (IMultiSourceFrameReader*)nullptr;
	// Compute recorded source (we can not user IMultiSourceFrameReader for Audio and Face, remove them from the recording panel if needed)
	int GatheredSourcesExceptAudio = GatheredSources & ~(FrameSourceTypes_Audio | FrameSourceTypes_Face | FrameSourceTypes_Color );
	pSensor->OpenMultiSourceFrameReader(GatheredSourcesExceptAudio, &pFrameReader);

	if ( pFrameReader == (IMultiSourceFrameReader*)nullptr )
	{
		fprintf(stderr, "Unable to open Multisource reader\n");
		pSensor->Close();
		InitStepDone.Data = false;
		InitStepDone.Signal();
		return;
	}

	// Allocate kinect bodies manager
	KinectBodies * CurrentBodies = nullptr;
	if ( GatheredSources & FrameSourceTypes_Body )
	{
		CurrentBodies = new KinectBodies((unsigned char*)BodyRecContext->BufferData);
	}

	// Init done sucessfully, set bool data to true and signal it
	InitStepDone.Data = true;
	InitStepDone.Signal();
	
	// Start loop
	pFrameReader->SubscribeMultiSourceFrameArrived(&hMultiSource);
	while( StopPending() == false )
	{
		IMultiSourceFrameArrivedEventArgs* pArgs = nullptr;
		// TRACE(L"IR Frame Event Signaled.");

		HRESULT hr = pFrameReader->GetMultiSourceFrameArrivedEventData(hMultiSource, &pArgs);
	
		if (SUCCEEDED(hr))
		{
			// fprintf( stderr, "." );

			// We lock access to RecordContexts
			Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

			// Get received timestamp
			ftime(&lTimestamp);

			IMultiSourceFrameReference * pRef = nullptr;
			if (FAILED(pArgs->get_FrameReference(&pRef)))
			{
				SafeRelease(pArgs);
				continue;
			}
			SafeRelease(pArgs);

			IMultiSourceFrame * pFrame = nullptr;
			if (FAILED(pRef->AcquireFrame(&pFrame)))
			{
				SafeRelease(pRef);
				continue;
			}
			SafeRelease(pRef);

			// Get Depth frame
			if ( GatheredSources & FrameSourceTypes_Depth )
			{
				GetFrameReferenceAndFrameBuffer( pFrame, &IMultiSourceFrame::get_DepthFrameReference, &IDepthFrame::AccessUnderlyingBuffer, reinterpret_cast<RawKinectRecording&>(*DepthRecContext), "Depth" );
			}

			// Get Infrared frame
			if ( GatheredSources & FrameSourceTypes_Infrared )
			{
				GetFrameReferenceAndFrameBuffer( pFrame, &IMultiSourceFrame::get_InfraredFrameReference, &IInfraredFrame::AccessUnderlyingBuffer, reinterpret_cast<RawKinectRecording&>(*InfraredRecContext), "Infrared" );
			}

			// Get long exposure infrared frame
			if ( GatheredSources & FrameSourceTypes_LongExposureInfrared )
			{
				 GetFrameReferenceAndFrameBuffer( pFrame, &IMultiSourceFrame::get_LongExposureInfraredFrameReference, &ILongExposureInfraredFrame::AccessUnderlyingBuffer, reinterpret_cast<RawKinectRecording&>(*LongExposureInfraredRecContext), "Long Exposure Infrared" );
			}

			// Get body index frame
			if ( GatheredSources & FrameSourceTypes_BodyIndex )
			{
				GetFrameReferenceAndFrameBuffer( pFrame, &IMultiSourceFrame::get_BodyIndexFrameReference, &IBodyIndexFrame::AccessUnderlyingBuffer, reinterpret_cast<RawKinectRecording&>(*BodyIndexRecContext), "BodyIndex" );
			}

			// Get body data
			if ( GatheredSources & FrameSourceTypes_Body )
			{
				GetBodyFrameReferenceAndFrameBuffer( pFrame, *BodyRecContext, *CurrentBodies, "Body" );

				// If we have bodies, ok, let's search for a face
				if ( GatheredSources & FrameSourceTypes_Face ) // && CurrentBodies->ActualNbBody != 0 )
				{
					// Exchange body id with face trackers
					FaceStream.Exchange(*CurrentBodies);
				}
			}

			// Release multisource frame
			SafeRelease(pFrame);

			// Here we have all data, if needed
			double CurrentTimeAsDouble = (double)lTimestamp.time + (((double)lTimestamp.millitm)/1000.0);
			if ( IsRecording == true )
			{
				for( RecContexts.First(); RecContexts.NotAtEnd(); RecContexts.Next() )
				{
					KinectRecording * pRec = RecContexts.GetCurrent();
					if( pRec->InputNumber == 0 )
					{
						pRec->StartTime = CurrentTimeAsDouble;
					}
				}
			}

			if ( GatheredSources & FrameSourceTypes_Depth && DepthRecContext->LastFrameTime != 0 )
			{				
				RawKinectRecording& rcs =reinterpret_cast<RawKinectRecording&>(*DepthRecContext);

				// Save data
				SaveDataAndIncreaseInputNumber( rcs, lTimestamp, nullptr );
												
				// Call Process function if any
				ProcessDepthFrame(rcs.BufferData, rcs.BufferSize, rcs.Width, rcs.Height, KS_UINT16, rcs.InputNumber, lTimestamp, rcs.LastFrameTime );
			}

			if ( GatheredSources & FrameSourceTypes_Infrared && InfraredRecContext->LastFrameTime != 0 )
			{				
				RawKinectRecording& rcs = reinterpret_cast<RawKinectRecording&>(*InfraredRecContext);

				// Save data
				SaveDataAndIncreaseInputNumber(rcs, lTimestamp, nullptr );

				// Call Process function if any
				ProcessInfaredFrame(rcs.BufferData, rcs.BufferSize, rcs.Width, rcs.Height, KS_UINT16, rcs.InputNumber, lTimestamp, rcs.LastFrameTime );
			}

			if( GatheredSources & FrameSourceTypes_LongExposureInfrared && LongExposureInfraredRecContext->LastFrameTime != 0 )
			{
				RawKinectRecording& rcs = reinterpret_cast<RawKinectRecording&>(*LongExposureInfraredRecContext);

				SaveDataAndIncreaseInputNumber(rcs, lTimestamp, nullptr );

				// Call Process function if any
				ProcessLongExposureInfaredFrame( rcs.BufferData, rcs.BufferSize, rcs.Width, rcs.Height, KS_UINT16, rcs.InputNumber, lTimestamp, rcs.LastFrameTime );
			}

			if( GatheredSources & FrameSourceTypes_BodyIndex && BodyIndexRecContext->LastFrameTime != 0 )
			{
				RawKinectRecording& rcs = reinterpret_cast<RawKinectRecording&>(*BodyIndexRecContext);

				// Save data
				SaveDataAndIncreaseInputNumber(rcs, lTimestamp, nullptr );

				// Call Process function if any
				ProcessBodyIndexFrame( rcs.BufferData, rcs.BufferSize, rcs.Width, rcs.Height, KS_UINT16, rcs.InputNumber, lTimestamp, rcs.LastFrameTime );
			}

			if( GatheredSources & FrameSourceTypes_Body && BodyRecContext->LastFrameTime != 0 )
			{
				RawKinectRecording& rcs = reinterpret_cast<RawKinectRecording&>(*BodyRecContext);

				// Create quick a string containing number of bodies (even if always 6, can change later, hopefully to strictly less than 10)
				char TmpNb[2];
				TmpNb[0] = (char)(CurrentBodies->ActualNbBody+48);	// Create ascii version of the number
				TmpNb[1] = '\0';

				// Save Data
				SaveDataAndIncreaseInputNumber( rcs, lTimestamp, TmpNb );

				// Call Process function if any
				if ( CurrentBodies->ActualNbBody != 0 )
				{
					// Call only if we had bodies
					ProcessBodyFrame( *CurrentBodies, rcs.InputNumber, lTimestamp, rcs.LastFrameTime );

					// Remember we sent body
					BodyWasSentPreviously = true;

					// Input number was increased by 1 in SaveDataAndIncreaseInputNumber, increase it by the actual number of body-1
					rcs.InputNumber += CurrentBodies->ActualNbBody - 1;
				}
				else
				{
					if ( BodyWasSentPreviously == true )
					{
						// Call the call back saying there is no more body
						ProcessBodyFrame( *CurrentBodies, -1, lTimestamp, 0 );
					}

					// Remember we did sen no body (or empty one)
					BodyWasSentPreviously = false;

					// Input number was increased by 1 in SaveDataAndIncreaseInputNumber, if we do not have any body, let decrease it
					rcs.InputNumber--;
				}
			}


			if ( IsRecording == true )
			{
				// Compute time between frames
				for( RecContexts.First(); RecContexts.NotAtEnd(); RecContexts.Next() )
				{
					KinectRecording * pRec = RecContexts.GetCurrent();
					if ( pRec->IsActive == false )
					{
						continue;
					}

					double TotalRecordingTime = CurrentTimeAsDouble - pRec->StartTime;
					if ( TotalRecordingTime == 0.0 || pRec->InputNumber == 0 )
					{
						pRec->FrameRate = 0;
					}
					else
					{
						pRec->FrameRate = (float)pRec->InputNumber/(float)TotalRecordingTime;
					}
				}
			}

			// Shall we draw something?
			if ( ShowFlags != SHOW_KINECT_NONE)
			{
				if ( ShowFlags & SHOW_KINECT_DEPTH )
				{
					RawKinectRecording * pRec = reinterpret_cast<RawKinectRecording*>(DepthRecContext);
					DrawDepthFrame( (unsigned char*)pRec->BufferData,  pRec->Width, pRec->Height );
				}

				if ( ShowFlags & SHOW_KINECT_IR )
				{
					RawKinectRecording * pRec = reinterpret_cast<RawKinectRecording*>(InfraredRecContext);
					DrawInfraredFrame( (unsigned char*)pRec->BufferData,  pRec->Width, pRec->Height );
				}

				cv::waitKey(1);
			}
		}
	}

	// Unsubscribe from Kinect events
	pFrameReader->UnsubscribeMultiSourceFrameArrived(hMultiSource);
	pFrameReader->Release();
	pSensor->Close();

	// Ok, here stop the worker Threads
	AudioStream.StopThread();	// Audio
	FaceStream.StopThread();	// Face
	RGBStream.StopThread();	// Stop RGB 

	// Stop recording
	StopRecording();

	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);
	ClearRecContexts();
}

bool KinectSensor::Init( int DesiredSources )
{
	if ( IsRunning() )
	{
		if ( GatheredSources != DesiredSources )
		{
			fprintf( stderr, "Kinect already started with another data sources.\n" );
			return false;
		}
		// already started with same sources
		return true;
	}

	if ( DesiredSources == FrameSourceTypes_None )
	{
		fprintf( stderr, "You must select data sources while calling Init.\n" );
		return false;
	}

	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

	// Set Gathered Sources
	GatheredSources = DesiredSources;

	// Get default Kinect2 sensor (for the moment)
	HRESULT	hr = GetDefaultKinectSensor(&pSensor);
	if ( FAILED(hr) )
	{
		return false;
	}

	fprintf(stderr, "Default Kinect Retreived - HR: %d\n", hr);

	// Start pump thread(s)
	if ( StartThread() == true )
	{
		// Wait for Init phase to be done within 5 seconds in the other thread
		// Note, if the thread failed, the event will also be signals
		ProtectAccess_SL.Unlock();

		InitStepDone.Wait(5000);
		return InitStepDone.Data;
	}

	return false;
}

}} // namespaceMobileRGBD::Kinect2

#endif // KINECT_2