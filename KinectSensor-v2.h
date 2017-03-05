#pragma once

#if defined WIN32 || defined WIN54

#ifdef KINECT_2

#include <System/Thread.h>
#include <System/TypedMemoryBuffer.h>

#include <string>
#include <sstream>
#include <time.h>
#include <vector>

#ifdef KINECT_LIVE
#include <Kinect.h>
#include <Kinect.Face.h>
#endif
#include <memory>

// #define ACTIVATE_KINECT_DRAWING

#include "KinectBasics.h"
#include "KinectDraw.h"
#include "KinectBody.h"
#include "KinectFace.h"

#include "RecordingManagement.h"
#include "KinectSensorCommon.h"
#include "DepthCameraIntrinsics.h"

class DepthCameraIntrinsics;

namespace MobileRGBD { namespace Kinect2 {

template<typename DATA_TYPE> 
class DataEvent : public Omiscid::Event
{
public:
	DataEvent() {}

	virtual ~DataEvent() {}

	DATA_TYPE Data;
};

class KinectSensor : public Omiscid::Thread, public KinectDraw, public RecordingManagement
{
public:
	KinectSensor();

	virtual ~KinectSensor();

	// Callback functions
	virtual void ProcessColorFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessDepthFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessInfaredFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessLongExposureInfaredFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessBodyIndexFrame(void * Buffer, unsigned int BufferSize, int Width, int Height, ImgType FrameType, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessBodyFrame(MobileRGBD::Kinect2::KinectBodies& CurrentBodies, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessFaceFrame(MobileRGBD::Kinect2::KinectFaces& CurrentFaces, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}
	virtual void ProcessAudioFrame(float * CurrentAudio, int NumFrame, const struct timeb& FrameTimestamp, TIMESPAN InternalFrameTime) {}

	// Init start with source selection
	bool Init(int DesiredSources);

protected:
	friend class KinectAudioStream;
	friend class KinectRGBStream;
	friend class KinectFaceStream;

	int GatheredSources;
	IKinectSensor * pSensor;

	virtual void FUNCTION_CALL_TYPE Run();


	// Estimate floor Angle using Kinect SDK
	enum { NoEstimation, OnlineFloorEstimation, EstimateFloorUntilStable };
	int FloorEstimationProcess;
	Vector4 FloorClipPlane;
	float AngleWithFloor;
	int NumberOfIdenticalFloorEStimation;
	Omiscid::Event FloorPlaneEstimated;


	DataEvent<bool> InitStepDone;

	// Utility functions
	bool IsDepthFrameToCameraSpaceLookupTableFilled;
	Omiscid::TypedMemoryBuffer<PointF> DepthFrameToCameraSpaceLookupTable;
	bool GetDepthFrameToCameraSpaceTable()
	{
		if ( pSensor == nullptr )
		{
			return false;
		}

		ICoordinateMapper * pMapper;
		if ( FAILED(pSensor->get_CoordinateMapper(&pMapper)) )
		{
				fprintf( stderr, "Unable to retrieve CoordinateMapper\n" );
				return false;
		}

		UINT32 NumberOfElementInTable = 0;
		PointF *tableEntries = nullptr;
		if ( FAILED(pMapper->GetDepthFrameToCameraSpaceTable(&NumberOfElementInTable, &tableEntries)) )
		{
			return false;
		}
		// Set space to number of depth pixels
		DepthFrameToCameraSpaceLookupTable.SetNewNumberOfElementsInBuffer( MobileRGBD::Kinect2::DepthWidth * MobileRGBD::Kinect2::DepthHeight );

		// Copy data
		memcpy( (PointF*)DepthFrameToCameraSpaceLookupTable, tableEntries, NumberOfElementInTable*sizeof(PointF) );

		// Free memory provided by Kinect
		CoTaskMemFree(tableEntries);

		return true;
	}

	DepthCameraIntrinsics  DepthCameraInts;
	bool GetDepthCameraIntrinsics()
	{
		if ( pSensor == nullptr )
		{
			return false;
		}

		ICoordinateMapper * pMapper;
		if ( FAILED(pSensor->get_CoordinateMapper(&pMapper)) )
		{
				fprintf( stderr, "Unable to retrieve CoordinateMapper\n" );
				return false;
		}
		pMapper->GetDepthCameraIntrinsics(&DepthCameraInts.IntrinsicsParameters);
		return true;
	}

	// Templating for code generation !
	template<class Interface>
	inline void GetFrameDescription(Interface * pInterfaceToFrame, RawKinectRecording& RecordContext)
	{
		IFrameDescription * pFrameDesc = nullptr;
		if (FAILED(pInterfaceToFrame->get_FrameDescription(&pFrameDesc)))
		{
			fprintf(stderr, "Could not get frame description\n");
			return;
		}
		pFrameDesc->get_Height(&RecordContext.Height);
		pFrameDesc->get_Width(&RecordContext.Width);
		pFrameDesc->get_HorizontalFieldOfView(&RecordContext.HorizontalFieldOfView);
		pFrameDesc->get_VerticalFieldOfView(&RecordContext.VerticalFieldOfView);
		pFrameDesc->get_DiagonalFieldOfView(&RecordContext.DiagonalFieldOfView);
		// For serialization reasons
		unsigned int pVal;
		pFrameDesc->get_LengthInPixels(&pVal);
		RecordContext.FrameLengthInPixels = pVal;
		pFrameDesc->get_BytesPerPixel(&pVal);
		RecordContext.BytesPerPixel = pVal;

		RecordContext.BufferSize = RecordContext.FrameLengthInPixels * RecordContext.BytesPerPixel;
	}

	template<class Interface, typename BufferType>
	inline void GetFrame(Interface * pInterfaceToFrame,
		// HRESULT (STDMETHODCALLTYPE Interface::*RetrieveFrameBuffer)( _Out_  UINT, _Out_writes_all_(capacity)BufferType *),
		HRESULT(STDMETHODCALLTYPE Interface::*RetrieveFrameBuffer)(UINT*, BufferType **),
		RawKinectRecording& RecordContext, const char * DataTypeName)
	{
		BufferType * FrameData = nullptr;
		if (RecordContext.InitDescription == true)
		{
			GetFrameDescription(pInterfaceToFrame, RecordContext);
			RecordContext.InitDescription = false;
		}

		// return buffer size if wrong in august version of sdk, used computed one
		UINT BufferSize = 0;
		if (FAILED(((pInterfaceToFrame)->*(RetrieveFrameBuffer))(&BufferSize, &FrameData)) || FrameData == nullptr)
		{
			fprintf(stderr, "Could not get %s buffer\n", DataTypeName);
			return;
		}
		else
		{
			// fprintf(stderr, "get %s \n", DataTypeName );
			// char * BufferData = new char[1024*1024]
			memcpy(RecordContext.BufferData, FrameData, RecordContext.BufferSize);
		}

		// Get Frame timestamp if possible
		RecordContext.LastFrameTime = 0;
		pInterfaceToFrame->get_RelativeTime(&RecordContext.LastFrameTime);
	}

	inline void GetBodyFrame(IBodyFrame * pInterfaceToFrame, KinectRecording& RecordContext, MobileRGBD::Kinect2::KinectBodies& Bodies, const char * DataTypeName)
	{
		// if we are tracking floor clip plane and floor angle
		if (FloorEstimationProcess != NoEstimation)
		{
			pInterfaceToFrame->get_FloorClipPlane(&FloorClipPlane);

			float tmpf = ceilf(atan2f(FloorClipPlane.z, FloorClipPlane.y) * 180.0f / 3.14f);
			if (tmpf == AngleWithFloor)
			{
				NumberOfIdenticalFloorEStimation++;
			}
			else
			{
				// Get the new value
				AngleWithFloor = tmpf;
				NumberOfIdenticalFloorEStimation = 0;
			}

			if (FloorEstimationProcess == EstimateFloorUntilStable && NumberOfIdenticalFloorEStimation > 5 * 30)
			{
				// Stop processing
				FloorEstimationProcess = NoEstimation;
				NumberOfIdenticalFloorEStimation = 0;

				// Signal that we found something
				FloorPlaneEstimated.Signal();
			}

		}

		// TODO : free previous IBody
		for( int iBody = 0; iBody < BODY_COUNT; iBody++ )
		{
			MobileRGBD::Kinect2::SafeRelease(Bodies.ppBodies[iBody]);
		}
		// memset(Bodies.ppBodies, 0, sizeof(IBody*)*BODY_COUNT);

		if (FAILED(pInterfaceToFrame->GetAndRefreshBodyData(_countof(Bodies.ppBodies), Bodies.ppBodies)))
		{
			// Thus, We do not have bodies
			Bodies.ActualNbBody = 0;
			fprintf(stderr, "Could not get %s buffer\n", DataTypeName);
			return;
		}

		// Get Frame timestamp if possible
		pInterfaceToFrame->get_RelativeTime(&RecordContext.LastFrameTime);

		RecordContext.BufferSize = Bodies.GatherBodiesInformationAndReturnSizeOfBodyBuffer();
	}

	inline void GetBodyFrameReferenceAndFrameBuffer(IMultiSourceFrame * pFrame, KinectRecording& RecordContext,
		MobileRGBD::Kinect2::KinectBodies& Bodies, const char * DataTypeName)
	{
		IBodyFrame * pLocalFrame = nullptr;
		IBodyFrameReference * pLocalRef = nullptr;
		RecordContext.LastFrameTime = 0;
		if (SUCCEEDED(pFrame->get_BodyFrameReference(&pLocalRef)))
		{
			pLocalRef->AcquireFrame(&pLocalFrame);
			MobileRGBD::Kinect2::SafeRelease(pLocalRef);

			// retrieve Data
			if (pLocalFrame != nullptr)
			{
				GetBodyFrame(pLocalFrame, RecordContext, Bodies, DataTypeName);
				MobileRGBD::Kinect2::SafeRelease(pLocalFrame);
			}
		}
	}

	inline void GetAudioFrameReferenceAndFrameBuffer(IMultiSourceFrame * pFrame, RawKinectRecording& RecordContext,
		MobileRGBD::Kinect2::KinectBodies& Bodies, const char * DataTypeName)
	{
		IBodyFrame * pLocalFrame = nullptr;
		IBodyFrameReference * pLocalRef = nullptr;
		RecordContext.LastFrameTime = 0;
		if (SUCCEEDED(pFrame->get_BodyFrameReference(&pLocalRef)))
		{
			pLocalRef->AcquireFrame(&pLocalFrame);
			MobileRGBD::Kinect2::SafeRelease(pLocalRef);

			// retrieve Data
			if (pLocalFrame != nullptr)
			{
				GetBodyFrame(pLocalFrame, RecordContext, Bodies, DataTypeName);
				MobileRGBD::Kinect2::SafeRelease(pLocalFrame);
			}
		}
	}

	template<class FrameReferenceClass, class Interface, typename BufferType>
	inline void GetFrameReferenceAndFrameBuffer(IMultiSourceFrame * pFrame,
		HRESULT(STDMETHODCALLTYPE IMultiSourceFrame::*GetFrameReference)(_COM_Outptr_  FrameReferenceClass **),
		// HRESULT (STDMETHODCALLTYPE Interface::*RetrieveFrameBuffer)( _Out_  UINT, _Out_writes_all_(capacity)BufferType *),
		HRESULT(STDMETHODCALLTYPE Interface::*RetrieveFrameBuffer)(UINT*, BufferType **),
		RawKinectRecording& RecordContext, const char * DataTypeName)
	{
		Interface * pLocalFrame = nullptr;
		FrameReferenceClass * pLocalRef = nullptr;
		RecordContext.LastFrameTime = 0;
		if (SUCCEEDED(((pFrame)->*(GetFrameReference))(&pLocalRef)))
		{
			pLocalRef->AcquireFrame(&pLocalFrame);
			MobileRGBD::Kinect2::SafeRelease(pLocalRef);

			// retrieve Data
			if (pLocalFrame != nullptr)
			{
				GetFrame(pLocalFrame, RetrieveFrameBuffer, RecordContext, DataTypeName);
				MobileRGBD::Kinect2::SafeRelease(pLocalFrame);
			}
			else
			{
				// fprintf(stderr, "Could not get reference class for %s\n", DataTypeName );
				return;
			}
		}
	}

};

// Class to multithread recording (maximum multisources and performance)
class KinectExtraRecorder
{
public:
	KinectExtraRecorder(KinectSensor& KinectSensorCaller) : Caller(KinectSensorCaller)
	{
	}

	virtual ~KinectExtraRecorder() {}

	void SetRecContext(RawKinectRecording* Ptr)
	{
		Omiscid::SmartLocker SL_Protection(Protection);

		RecContext = Ptr;
	}

	Omiscid::Mutex Protection;

protected:
	RawKinectRecording* RecContext	= nullptr;
	KinectSensor& Caller;
};

}} // namespace MOBILERGBD::Kinect2 

#endif // ifdef KINECT_2

#endif // WIN32 ou WIN64