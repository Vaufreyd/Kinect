/**
 * @file KinectFace.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_FACE_H__
#define __KINECT_FACE_H__

#ifdef KINECT_2

#include "KinectBasics.h"
#include "KinectDataAsMemoryBuffer.h"

#ifdef ACTIVATE_KINECT_DRAWING

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#endif // ACTIVATE_KINECT_DRAWING

namespace MobileRGBD { namespace Kinect2 {

/**
 * @class KinectFace KinectFace.cpp KinectFace.h
 * @brief Class to manage face data from Kinect. All data are gathered in a unique buffer.
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 */
class KinectFace : public KinectDataAsMemoryBuffer
{
public:
	/** @brief constructor.
	 */
	KinectFace()
	{
			DWORD * FaceFrameFeatures = (DWORD *)nullptr;
			UINT64 * TrackingId = (UINT64 *)nullptr;							// Gets the tracking ID.
			Vector4 * RotationQuaternion = (Vector4 *)nullptr;						// FaceQuaternion
			RectI * BoundingBoxInColorSpace  = (RectI *)nullptr;					// BoundingBox
			RectI * BoundingBoxInInfraredSpace = (RectI *)nullptr;					// BoundingBox
			PointF * FacePointsInColorSpace = (PointF *)nullptr;					
			PointF * FacePointsInInfraredSpace = (PointF *)nullptr;
			DetectionResult * FaceProperties = (DetectionResult *)nullptr;
	};

	/** @brief constructor. Will set the data in memory.
	 *
	 * @param Buffer [in] Buffer to use for KinectFace data
	 */
	KinectFace(unsigned char *Buffer)
	{
		KinectFace();
		Set(Buffer);
	}

	/** @brief Virtual destructor, always.
	 */
	virtual ~KinectFace() {};

	/** @brief Set will put all reference to memory buffer for KinectBody data
	 *
	 * @param Buffer [in] Buffer to use for KinectFace data
	 */
	virtual void Set(unsigned char *Buffer)
	{
		unsigned char * InitBuffer = Buffer;

		// Engaged = (DetectionResult*)Buffer; Buffer += sizeof(DetectionResult);
		SetAddressAndIncreaseBuffer(TrackingId, Buffer);
		SetAddressAndIncreaseBuffer(RotationQuaternion, Buffer);
		SetAddressAndIncreaseBuffer(BoundingBoxInColorSpace, Buffer);
		SetAddressAndIncreaseBuffer(BoundingBoxInInfraredSpace, Buffer);
		SetAddressAndIncreaseBuffer(FacePointsInColorSpace, Buffer, FacePointType::FacePointType_Count);
		SetAddressAndIncreaseBuffer(FacePointsInInfraredSpace, Buffer, FacePointType::FacePointType_Count);
		SetAddressAndIncreaseBuffer(FaceProperties, Buffer, FaceProperty::FaceProperty_Count);

		FaceSize = int(Buffer - InitBuffer);
	}

#ifdef KINECT_LIVE
	/** @brief Fill KinectBody buffer with data coming from the Kinect2. Exists only if KINECT_LIVE is defined. 
	 *
	 * @param pFaceFrame [in] Pointer to the Kinect2 structure.
	 */
	bool Init(IFaceFrame* pFaceFrame)
	{
		if ( pFaceFrame == nullptr )
		{
			return false;
		}

		IFaceFrameResult* pFaceFrameResult = nullptr;
		HRESULT hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

		// need to verify if pFaceFrameResult contains data before trying to access it
        if (FAILED(hr) || pFaceFrameResult == nullptr)
        {
			return false;
		}

		hr = pFaceFrame->get_TrackingId(TrackingId);
		hr = pFaceFrameResult->get_FaceRotationQuaternion(RotationQuaternion);
		hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(BoundingBoxInColorSpace);
		hr = pFaceFrameResult->get_FaceBoundingBoxInInfraredSpace(BoundingBoxInInfraredSpace);
		hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, FacePointsInColorSpace);
		hr = pFaceFrameResult->GetFacePointsInInfraredSpace(FacePointType::FacePointType_Count, FacePointsInInfraredSpace);
		hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, FaceProperties);

		pFaceFrameResult->Release();

		return true;
	}
#endif

#ifdef ACTIVATE_KINECT_DRAWING
	/** @brief Draw one face in image.
	 *
	 * @param WhereToDraw [in] cv::Mat object to draw in.
	 * @param DrawInDepth [in] Do we draw in Depth frame or in RGB frame?
	 */
	void Draw(cv::Mat WhereToDraw, bool DrawInDepth = true )
	{
		// Compute actual scaling factor
		if ( DrawInDepth == true )
		{
			float scale_x = (float)WhereToDraw.cols/(float)DepthWidth;
			float scale_y = (float)WhereToDraw.rows/(float)DepthHeight;
		
			cv::Rect FaceBB((int)(scale_x*(float)BoundingBoxInInfraredSpace->Left), (int)(scale_y*(float)BoundingBoxInInfraredSpace->Top),
			(int)(scale_x*(float)(BoundingBoxInInfraredSpace->Right-BoundingBoxInInfraredSpace->Left)),
			(int)(scale_y*(float)(BoundingBoxInInfraredSpace->Bottom-BoundingBoxInInfraredSpace->Top)) );
			cv::rectangle( WhereToDraw, FaceBB, cv::Scalar(0,0,0) );
				
			for( int i = 0; i < FacePointType_Count; i++ )
			{
				cv::circle( WhereToDraw, cv::Point((int)(scale_x*FacePointsInInfraredSpace[i].X), (int)(scale_y*FacePointsInInfraredSpace[i].Y)), 2, cv::Scalar(0,0,255), -1 );
			}
		}
		else
		{
			float scale_x = (float)WhereToDraw.cols/(float)CamWidth;
			float scale_y = (float)WhereToDraw.rows/(float)CamHeight;

			cv::Rect FaceBB((int)(scale_x*(float)BoundingBoxInColorSpace->Left), (int)(scale_y*(float)BoundingBoxInColorSpace->Top),
			(int)(scale_x*(float)(BoundingBoxInColorSpace->Right-BoundingBoxInColorSpace->Left)),
			(int)(scale_y*(float)(BoundingBoxInColorSpace->Bottom-BoundingBoxInColorSpace->Top)) );
			cv::rectangle( WhereToDraw, FaceBB, cv::Scalar(0,0,0) );
				
			for( int i = 0; i < FacePointType_Count; i++ )
			{
				cv::circle( WhereToDraw, cv::Point((int)(scale_x*FacePointsInColorSpace[i].X), (int)(scale_y*FacePointsInColorSpace[i].Y)), 2, cv::Scalar(0,0,255), -1 );
			}
		}

	}

#endif // ACTIVATE_KINECT_DRAWING

	static int FaceSize;					/*!< @brief Size of all KinectFace buffer. To compute it, one can call Set with nullptr */

	UINT64 * TrackingId;					/*!< @brief Gets the tracking ID. */
	Vector4 * RotationQuaternion;			/*!< @brief FaceQuaternion */
    RectI * BoundingBoxInColorSpace;		/*!< @brief BoundingBox in color image */
	RectI * BoundingBoxInInfraredSpace;		/*!< @brief BoundingBox in Depth image */
    PointF * FacePointsInColorSpace;		/*!< @brief Face point in Color image */
	PointF * FacePointsInInfraredSpace;		/*!< @brief Face point in Depth image */
	DetectionResult * FaceProperties;		/*!< @brief Face properties */
};

/**
 * @class KinectFaces KinectBody.cpp KinectBody.h
 * @brief Class to handle several KinectFace. All data are gathered in a unique buffer.
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 */
class KinectFaces
{
public:
	/** @brief constructor. Will set the data in memory for up to BODY_COUNT KinectFace instances.
	 *
	 * @param Buffer [in] Buffer to use for KinectBody data
	 */
	KinectFaces(unsigned char * ExtBuffer = nullptr)
	{
		// Compute size of a body/face in bytes => result in KinectBody::BodySize KinectFace::FaceSize static values
		FacesInformation[0].Set(nullptr);

		// Allocate memory
		if ( ExtBuffer == (unsigned char*)nullptr )
		{
			AllocatedBuffer.SetNewBufferSize(BODY_COUNT*KinectFace::FaceSize);
			FaceData = (unsigned char*)AllocatedBuffer;
		}
		else
		{
			FaceData = ExtBuffer;
		}

		// Initialize memory for all faces
		ActualNbFaces = 0;
		unsigned char * lBuffer = FaceData;
		for( int i = 0; i < BODY_COUNT; i++ )
		{
			FacesInformation[i].Set(lBuffer);
			lBuffer += KinectFace::FaceSize;
			FaceIsTracked[i] = false;
		}
	}

	/** @brief Virtual destructor, always.
	 */
	virtual ~KinectFaces()
	{
	}

	bool FaceIsTracked[BODY_COUNT];				/*!< @brief Store tracked state of faces. */

	KinectFace FacesInformation[BODY_COUNT];	/*!< @brief Up to BODY_COUNT KinectFace can be handled. */
	unsigned int ActualNbFaces;					/*!< @brief Actual number of faces in the buffer. */
	unsigned char * FaceData;					/*!< @brief Store current faces information in one memory block for save/load operation. */

protected:
	Omiscid::MemoryBuffer AllocatedBuffer;		/*!< @brief Memory manager for BodyData. */
};

}} // namespace MobileRGBD::Kinect2

#endif // KINECT_2

#endif // __KINECT_FACE_H__