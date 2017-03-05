/**
 * @file KinectAudioStream.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "KinectAudioStream.h"

using namespace MobileRGBD::Kinect2;

/* virtual */ void FUNCTION_CALL_TYPE KinectAudioStream::Run()
{
	if ( RecContext == nullptr )
	{
		fprintf( stderr, "No buffer available for audio\n" );
		return;
	}

	WAITABLE_HANDLE hAudioSource = NULL;
	timeb lTimestamp;

	IKinectSensor * pSensor = Caller.pSensor;
	if ( pSensor == nullptr )
	{
		fprintf( stderr, "Unable to find Kinect sensor for audio\n" );
		return;
	}

	// Open Audio
	IAudioSource * pAudioSource = nullptr;
	pSensor->get_AudioSource(&pAudioSource);

	IAudioBeamFrameReader *pAudioReader = nullptr;
	pAudioSource->OpenReader(&pAudioReader);

	pAudioReader->SubscribeFrameArrived(&hAudioSource);

	unsigned char * InitBuffer = (unsigned char*)RecContext->BufferData;

	while(!StopPending())
	{
		IAudioBeamFrameArrivedEventArgs* pArgs = nullptr;
		
		if ( SUCCEEDED(pAudioReader->GetFrameArrivedEventData(hAudioSource, &pArgs)) )
		{
			// we do not use them
			SafeRelease(pArgs);

			// Get timestamp
			ftime(&lTimestamp);

			// get data
			IAudioBeamFrameList * pAudioFrameList = nullptr;
			if ( FAILED(pAudioReader->AcquireLatestBeamFrames(&pAudioFrameList)) ) { continue; }

			// Only one beam for the moment, SDK v2.0.1409
			UINT NbBeams = 1;	// pAudioFrameList->get_BeamCount(&NbBeams);

			bool NbValidBeam = 0;
					
			for( UINT NumBeam = 0; NumBeam < NbBeams; NumBeam++ )	// As only one beam now, compiler will simplify this, but the code will remind for further SDK
			{
				IAudioBeamFrame * pAudioBeamFrame = nullptr;
				UINT SubFrameCount = 0;
											
				unsigned char * InBuffer = InitBuffer;
				RecContext->BufferSize = 0;

				if ( SUCCEEDED(pAudioFrameList->OpenAudioBeamFrame(NbBeams-1, &pAudioBeamFrame)) && 
						SUCCEEDED(pAudioBeamFrame->get_SubFrameCount(&SubFrameCount)) )
				{

					for( UINT i = 0; i < SubFrameCount; i++ )
					{
						IAudioBeamSubFrame* pAudioBeamSubFrame = NULL;
								
						if ( SUCCEEDED( pAudioBeamFrame->GetSubFrame(i, &pAudioBeamSubFrame) ) )
						{
									
							UINT BufferSize = 0;
							BYTE * BufferAddress = nullptr;

							if ( SUCCEEDED(pAudioBeamSubFrame->AccessUnderlyingBuffer(&BufferSize,&BufferAddress)) )
							{
								memcpy( InBuffer, BufferAddress, BufferSize );
								InBuffer += BufferSize;
								RecContext->BufferSize += BufferSize; 
							}
						}
			
						SafeRelease(pAudioBeamSubFrame);
					}

					NbValidBeam++;
				}

				SafeRelease(pAudioBeamFrame);
			}

			SafeRelease(pAudioFrameList);

			// Save audio data if any
			if ( RecContext != nullptr && RecContext->BufferSize != 0 )
			{
				// To make source code looks like others
				RawKinectRecording& rcs = *RecContext;

				// Create quick a string containing number of samples
				char TmpNb[10];
				sprintf( TmpNb, "%d", rcs.BufferSize/sizeof(float) );

				// Save Data, first set buffer size (in term of number of faces)
				rcs.SaveDataAndIncreaseInputNumber( lTimestamp, TmpNb );

				// Ok, here call the virtual function from the parent thread, from now, one beam, one call
				// Caller.ProcessAudio();
			}
		}
	}
}
