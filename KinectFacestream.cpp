/**
 * @file KinectFaceStream.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "KinectFaceStream.h"

using namespace MobileRGBD::Kinect2;

KinectFaceStream::KinectFaceStream(KinectSensor& KinectSensorCaller) : KinectExtraRecorder(KinectSensorCaller)
{
	for( auto i = 0; i < BODY_COUNT; i++ )
	{
		BodyTrackingIds[i]	= 0;
		StartedSearch[i]	= false;
	}
}

void KinectFaceStream::Exchange(KinectBodies& CurrentBodies)
{
	Omiscid::SmartLocker SL_Protection(Protection);

	bool FaceTrackerActuallySet[BODY_COUNT] = { false };

	unsigned int iBody;
	for( iBody = 0; iBody < CurrentBodies.ActualNbBody; iBody++ )
	{
		// Get original index to let the frame tracker to ba associated with the same index
		int BodyOriginalIndex = CurrentBodies.InitialBodyIndex[iBody];

		// Ok, we have a body tracking id or already start search with it, if not done, initialise search
		if ( BodyTrackingIds[BodyOriginalIndex] != *CurrentBodies.BodiesInformation[iBody].trackingId )
		{
			BodyTrackingIds[BodyOriginalIndex] = *CurrentBodies.BodiesInformation[iBody].trackingId;
			// StartedSearch[iBody] = true will be set in Thread::Run
		}

		FaceTrackerActuallySet[BodyOriginalIndex] = true;
	}

	int NbTrackedBody = 0;
	for( iBody = 0; iBody < BODY_COUNT; iBody++ )
	{
		if ( FaceTrackerActuallySet[iBody] == false )
		{
			StartedSearch[iBody] = false;
			BodyTrackingIds[iBody] = 0;
		} 
		else
		{
			NbTrackedBody++;
		}
	}
}

/* virtual */ void FUNCTION_CALL_TYPE KinectFaceStream::Run()
{
	if ( RecContext == nullptr )
	{
		fprintf( stderr, "No buffer available for faces\n" );
		return;
	}

	IKinectSensor * pSensor = Caller.pSensor;
	if ( pSensor == (IKinectSensor *)nullptr )
	{
		fprintf( stderr, "No kinect sensor available for faces\n" );
		return;
	}

	HRESULT	hr = S_OK;
	struct timeb lTimestamp = { 0 };

	// Face sources and readers
	IFaceFrameReader *	   pFaceFrameReaders[BODY_COUNT] = { nullptr };
	IFaceFrameSource*	pFaceFrameSources[BODY_COUNT] = { nullptr };

	// Open Face reader
	DWORD FaceFrameFeatures = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
			| FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInInfraredSpace
			| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
			| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
			| FaceFrameFeatures::FaceFrameFeatures_Happy
			| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
			| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
			| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
			| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
			| FaceFrameFeatures::FaceFrameFeatures_LookingAway
			| FaceFrameFeatures::FaceFrameFeatures_Glasses
			| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;	// Everything

	WAITABLE_HANDLE FramesWaiters[BODY_COUNT];
	for (int i = 0; i < BODY_COUNT; i++)
    {
		FramesWaiters[i] = NULL;

        // create the face frame source by specifying the required face frame features
        HRESULT hr = CreateFaceFrameSource(pSensor, 0, FaceFrameFeatures, &pFaceFrameSources[i]);
        if ( SUCCEEDED(hr) )
        {
            // open the corresponding reader
            hr = pFaceFrameSources[i]->OpenReader(&pFaceFrameReaders[i]);

			if ( SUCCEEDED(hr) )
			{
				if ( FAILED(pFaceFrameReaders[i]->SubscribeFrameArrived(&FramesWaiters[i])) )
				{
				}
			}
        }		
    }

	// Allocated memory
	unsigned char * lBuffer = (unsigned char*)RecContext->BufferData;

	KinectFaces KF((unsigned char*)RecContext->BufferData);

	Omiscid::SmartLocker SL_Protection(Protection, false );

	UINT64 CurrentSearchFace[BODY_COUNT] = { 0 };
	bool LastFrameContainedFaces = false;

	while( !StopPending() )
	{
		// No face at this point
		KF.ActualNbFaces = 0;

		// Set tracking id to face tracker
		SL_Protection.Lock();
		for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
		{
			CurrentSearchFace[iFace] = BodyTrackingIds[iFace];
			if ( CurrentSearchFace[iFace] != 0 && StartedSearch[iFace] == false )
			{
				StartedSearch[iFace] = true;
				pFaceFrameSources[iFace]->put_TrackingId(BodyTrackingIds[iFace]);
			}
		}
		SL_Protection.Unlock();

		// Go though faces
		// int NBSearchesFaces = 0;

		for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
		{
			// Ok, we have a body tracking id, if not skip
			if ( StartedSearch[iFace] == false )
			{
				continue;
			}

			// NBSearchesFaces++;

			// Check if we have data from this face tracker
			IFaceFrameArrivedEventArgs* pArgs = nullptr;
			if ( FAILED(hr = pFaceFrameReaders[iFace]->GetFrameArrivedEventData(FramesWaiters[iFace], &pArgs)) )
			{
				continue;
			}

			// we do not use this, but release it
			SafeRelease(pArgs);

			// retrieve the latest face frame from this reader
			IFaceFrame* pFaceFrame = nullptr;

			// Omiscid::PerfElapsedTime ET;
			hr = pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);
			// fprintf( stderr, "%f\n", ET.GetInSeconds() );

			if ( FAILED(hr) || pFaceFrame == nullptr )
			{
				continue;
			}

			// Get timestamp
			ftime(&lTimestamp);
					
			BOOLEAN bFaceTracked = FALSE;
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);

			if ( bFaceTracked == TRUE )
			{
			#ifdef SET_ASSOCIATED_BODY_TACKING_ID
				KF.FacesInformation[KF.ActualNbFaces].Init(pFaceFrame, BodyTrackingIds[iFace] );
			#else
				KF.FacesInformation[KF.ActualNbFaces].Init(pFaceFrame);
			#endif
				
				KF.ActualNbFaces++;
				pFaceFrame->get_RelativeTime(&(RecContext->LastFrameTime));
			}
			else
			{
				// no face is tracked, set trackingId
				if ( BodyTrackingIds[iFace] != 0 )
				{
					pFaceFrameSources[iFace]->put_TrackingId(BodyTrackingIds[iFace]);
				}
			}

			SafeRelease(pFaceFrame);
		}

		// fprintf( stderr, "%d (%d searched)\n", KF.ActualNbFaces, NBSearchesFaces );

		if ( KF.ActualNbFaces == 0 )
		{
			if ( LastFrameContainedFaces == true )
			{
				// Caller.ProcessFaceFrame( KF, -1, lTimestamp, 0 );
			}
			LastFrameContainedFaces = false;

			Omiscid::Thread::Sleep(5);
			continue;
		}

		if ( KF.ActualNbFaces != 0 && RecContext != nullptr )
		{
			// To make source code looks like others
			RawKinectRecording& rcs = *RecContext;
			
			// Create quick a string containing number of bodies (even if always 6, can change later, hopefully to strictly less than 10)
			char TmpNb[2];
			TmpNb[0] = (char)(KF.ActualNbFaces+48);	// Create ascii version of the number
			TmpNb[1] = '\0';

			// Save Data, first set buffer size (in term of number of faces)
			rcs.BufferSize = KF.ActualNbFaces*KinectFace::FaceSize;
			rcs.SaveDataAndIncreaseInputNumber( lTimestamp, TmpNb );

			// Call only if we had faces
			LastFrameContainedFaces = true;
			Caller.ProcessFaceFrame( KF, rcs.InputNumber, lTimestamp, rcs.LastFrameTime );

			// Input number was increased by 1 in SaveDataAndIncreaseInputNumber, increase it by the actual number of faces-1
			rcs.InputNumber += KF.ActualNbFaces - 1;
		}
	}
}