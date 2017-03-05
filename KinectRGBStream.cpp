/**
 * @file KinectRGBStream.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "KinectRGBStream.h"

using namespace MobileRGBD::Kinect2;

/* virtual */ void FUNCTION_CALL_TYPE KinectRGBStream::Run()
{
	if ( RecContext == nullptr )
	{
		fprintf( stderr, "No buffer available for RGB\n" );
		return;
	}

	IKinectSensor * m_pKinectSensor = Caller.pSensor; 
		
	// Initialize the Kinect and get the color reader
	IColorFrameSource* pColorFrameSource = nullptr;

	if (FAILED(m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource)) || pColorFrameSource == nullptr )
	{
		SafeRelease(m_pKinectSensor);
		return;
	}

	IColorFrameReader * m_pColorFrameReader = nullptr;
	if (FAILED( pColorFrameSource->OpenReader(&m_pColorFrameReader)) || m_pColorFrameReader == nullptr )
	{
		SafeRelease(m_pKinectSensor);
		SafeRelease(pColorFrameSource);
		SafeRelease(m_pColorFrameReader);
		return;
	}

	SafeRelease(pColorFrameSource);
		
	WAITABLE_HANDLE Wait = NULL;
	m_pColorFrameReader->SubscribeFrameArrived(&Wait);

#ifdef RESIZE_KINECT_RGB
	KinectImageConverter ImgConvert;
#endif

	while( !StopPending() )
	{
		IColorFrameArrivedEventArgs* pArgs = nullptr;
		if ( FAILED(m_pColorFrameReader->GetFrameArrivedEventData(Wait, &pArgs)) )
		{
			// Thread::Sleep(5);
			continue;
		}

		SafeRelease(pArgs);

		IColorFrame * pColorFrame = nullptr;
		if ( FAILED(m_pColorFrameReader->AcquireLatestFrame(&pColorFrame)) || pColorFrame == nullptr )
		{
			// Thread::Sleep(5);
			continue;
		}

		// GetFrame;
		BYTE * FrameData = nullptr;
		if ( RecContext->InitDescription == true )
		{
			Caller.GetFrameDescription(pColorFrame, *RecContext);
			RecContext->InitDescription = false;
		}
				
		// return buffer size if wrong in august version of sdk, used computed one
		UINT BufferSize = 0;
		if ( FAILED( pColorFrame->AccessRawUnderlyingBuffer(&BufferSize, &FrameData)) || FrameData == nullptr )
		{
			fprintf(stderr, "Could not get %s buffer\n", "RGB" );
			return;
		}
		else
		{
#ifdef RESIZE_KINECT_RGB
			if ( ImgConvert.ConvertYVY2ToBRG( FrameData, (unsigned char*)RecContext->BufferData, 1920, 1080, RESIZE_KINECT_RGB ) == false )
			{
				fprintf(stderr, "Could not convert %s buffer\n", "RGB" );
				return;
			}
			RecContext->BufferSize = (1920*1080/RESIZE_KINECT_RGB)*3;
			RecContext->Width = 1920/RESIZE_KINECT_RGB;
			RecContext->Height = 1080/RESIZE_KINECT_RGB;
			RecContext->BytesPerPixel = 3;
			RecContext->FrameType = "BGR";
#else
			memcpy(RecContext->BufferData, FrameData, RecContext->BufferSize );
#endif
		}

		// Get Frame timestamp if possible
		RecContext->LastFrameTime = 0;
		pColorFrame->get_RelativeTime(&(RecContext->LastFrameTime));

		SafeRelease( pColorFrame );
			
		timeb lTimestamp;
		ftime(&lTimestamp);

		// Save data
		RecContext->SaveDataAndIncreaseInputNumber( lTimestamp, nullptr );

		// Call Process function if any
		Caller.ProcessColorFrame(RecContext->BufferData, RecContext->BufferSize, RecContext->Width, RecContext->Height, KS_YUV2, RecContext->InputNumber, lTimestamp, RecContext->LastFrameTime );
	}

	m_pColorFrameReader->UnsubscribeFrameArrived(Wait);
	SafeRelease( m_pColorFrameReader );
	m_pKinectSensor->Close();
	SafeRelease(m_pKinectSensor);
}
