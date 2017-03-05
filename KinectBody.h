/**
 * @file KinectBody.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_BODY_H__
#define __KINECT_BODY_H__

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
 * @class KinectBody KinectBody.cpp KinectBody.h
 * @brief Class to manage body data from Kinect. All data are gathered in a unique buffer.
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 */
class KinectBody : public KinectDataAsMemoryBuffer
{
public:

	/** @brief constructor.
	 */
	KinectBody()
	{

#if (defined WIN32 || defined WIN64) && defined KINECT_LIVE
		if ( CoordinateMapper == (ICoordinateMapper*)nullptr )
		{
			IKinectSensor * pKinectSensor = nullptr;
			if ( FAILED(GetDefaultKinectSensor(&pKinectSensor)) )
			{
				fprintf( stderr, "Unable to find a Kinect Sensor\n");
			}

			if ( FAILED(pKinectSensor->get_CoordinateMapper(&CoordinateMapper)) )
			{
				fprintf( stderr, "Unable to retrieve CoordinateMapper\n" );
			}
		}
#endif

		// Not set in memory
		Engaged = (DetectionResult*)nullptr;
		HandsTrackingConfidence = (TrackingConfidence *)nullptr;
		HandsState = (HandState *)nullptr;
		IsRestricted = (unsigned char*)nullptr;
		trackingId = (UINT64 *)nullptr;
		Joints = (Joint*)nullptr;
		JointOrientations = (JointOrientation*)nullptr;
		JointsInColorSpace = (PixelPosition*)nullptr;
		JointsInDepthSpace = (PixelPosition*)nullptr;
	}

	/** @brief constructor. Will set the data in memory.
	 *
	 * @param Buffer [in] Buffer to use for KinectBody data
	 */
	KinectBody(unsigned char *Buffer) : KinectBody()
	{
		Set(Buffer);
	}

	/** @brief Virtual destructor, always.
	 */
	virtual ~KinectBody() {}

	/** @brief Set will put all reference to memory buffer for KinectBody data
	 *
	 * @param Buffer [in] Buffer to use for KinectBody data
	 */
	virtual void Set(unsigned char *Buffer)
	{
		unsigned char * InitBuffer = Buffer;

		// Engaged = (DetectionResult*)Buffer; Buffer += sizeof(DetectionResult);
		SetAddressAndIncreaseBuffer(Engaged, Buffer);
		SetAddressAndIncreaseBuffer(HandsTrackingConfidence, Buffer, NumberOfHands);
		SetAddressAndIncreaseBuffer(HandsState, Buffer, NumberOfHands);
		SetAddressAndIncreaseBuffer(Lean, Buffer);
		SetAddressAndIncreaseBuffer(LeanTrackingState, Buffer);
		SetAddressAndIncreaseBuffer(trackingId, Buffer );
		SetAddressAndIncreaseBuffer(IsRestricted, Buffer);
		SetAddressAndIncreaseBuffer(Joints, Buffer,JointType_Count);
		SetAddressAndIncreaseBuffer(JointOrientations, Buffer, JointType_Count);

		// fprintf( stderr, "BodySize=%d", Buffer - InitBuffer );

		SetAddressAndIncreaseBuffer(JointsInColorSpace, Buffer, JointType_Count);
		SetAddressAndIncreaseBuffer(JointsInDepthSpace, Buffer, JointType_Count);

		BodySize = int(Buffer - InitBuffer);
	}

	static int BodySize;								/*!< @brief Size of all KinectBody buffer. To compute it, one can call Set with nullptr */

	// ClippedEdges Gets the clipped edges.
	DetectionResult* Engaged;							/*!< Gets the level of user engagement. */

	enum { HandLeft, HandRight, NumberOfHands };
	TrackingConfidence * HandsTrackingConfidence;		/*!< Gets the tracking confidence for the hands. */
	HandState * HandsState;								/*!< Gets the hand states. */

	PointF * Lean;										/*!< Gets the amount a body is leaning, which is a number between - 1 (leaning left or back) and 1 (leaning right or front) */
	tTrackingState * LeanTrackingState;					/*!< Gets the lean tracking state, which indicates if the body is tracked. */

	UINT64 * trackingId;								/*!< Gets the tracking ID. */
	unsigned char * IsRestricted;						/*!< Retrieves a boolean value that indicates if the body is restricted from a full range of motion. */

	// For later usage
	// DetectionResult Activities[Activity_Count];		// Gets the activity detection results from IBody.
	// DetectionResult Appearances[Appearance_Count];	// Gets the appearance.
	// DetectionResult Expression;						//  Expression_Count == 1
	// GetExpressionDetectionResults					// Gets the dictionary of expressions.

	Joint  * Joints;									/*!< Gets the dictionary of joints. */
	JointOrientation * JointOrientations;				/*!< Gets the dictionary of joint orientations. */

	PixelPosition * JointsInColorSpace;					/*!< Position of Joints in Color image (RGB) */
	PixelPosition * JointsInDepthSpace;					/*!< Position of Joints in Depth image */


#if (defined WIN32 || defined WIN64) && defined KINECT_LIVE
	static ICoordinateMapper * CoordinateMapper;		/*!< Coordinate Mapper. Exists only if KINECT_LIVE is defined. */

	/** @brief Compute Skeleton Point in RGB and Depth. Exists only if KINECT_LIVE is defined.
	 */
	void ComputeJointsInRGBAndDepth()
	{
		if ( CoordinateMapper == nullptr )
		{
			memset(JointsInDepthSpace, 0, sizeof(PixelPosition)*JointType_Count);
			memset(JointsInColorSpace, 0, sizeof(PixelPosition)*JointType_Count);
			return;
		}

	/*	CameraIntrinsics CamI;
		CoordinateMapper->GetDepthCameraIntrinsics()*/

		// Only used when recording, not when reading, thus only on Windows Application
		for( int i = 0; i < JointType_Count; i++ )
		{
			CoordinateMapper->MapCameraPointToDepthSpace(Joints[i].Position, (DepthSpacePoint*)&JointsInDepthSpace[i] );
			CoordinateMapper->MapCameraPointToColorSpace(Joints[i].Position, (ColorSpacePoint*)&JointsInColorSpace[i] );
		}
	}
#else
	void ComputeJointsInRGBAndDepth() {}
#endif	// (defined WIN32 || defined WIN64) && defined KINECT_LIVE

#ifdef KINECT_LIVE
	/** @brief Fill KinectBody buffer with data coming from the Kinect2. Exists only if KINECT_LIVE is defined. 
	 *
	 * @param pBody [in] Pointer to the Kinect2 structure.
	 */
	void Init(IBody* pBody)
	{
		BOOLEAN TmpBool = FALSE;

		pBody->get_Engaged(Engaged);

		pBody->get_HandLeftConfidence(&HandsTrackingConfidence[HandLeft]);
		pBody->get_HandLeftState(&HandsState[HandLeft]);
		pBody->get_HandRightConfidence(&HandsTrackingConfidence[HandRight]);
		pBody->get_HandRightState(&HandsState[HandRight]);

		pBody->get_Lean(Lean);
		pBody->get_LeanTrackingState(LeanTrackingState);

		pBody->get_TrackingId(trackingId);
		// fprintf( stderr, "Body %lu\n", *trackingId );

		pBody->GetJoints(JointType_Count, Joints);
		pBody->GetJointOrientations(JointType_Count, JointOrientations);

		pBody->get_IsRestricted(&TmpBool);
		SetValueFromBollean(*IsRestricted, TmpBool);

		// Compute Skeleton Point in RGB and Depth
		ComputeJointsInRGBAndDepth();
	}
#endif // KINECT_LIVE

#ifdef ACTIVATE_KINECT_DRAWING
	/** @brief Draw one bone in image. According to the state of joints, color of bones changes (green if tracked, red if inferred).
	 *
	 * @param frame [in] cv::Mat object to draw in.
	 * @param joint0 [in] starting joint number.
	 * @param joint1 [in] ending joint number.
	 * @param DrawInDepth [in] Do we draw in Depth frame or in RGB frame?
	 * @param scale_x [in] Scale factor in x. Default value=1.0f;
	 * @param scale_y [in] Scale factor in y. Default value=1.0f;
	 */
	void DrawBone(cv::Mat& frame, int joint0, int joint1, bool DrawInDepth, float scale_x = 1.0f, float scale_y = 1.0f )
	{
		int state0 = Joints[joint0].TrackingState;
		int state1 = Joints[joint1].TrackingState;

		PixelPosition * Pixels;
		// Did we draw body in depth or in RGB ?
		if ( DrawInDepth == true )
		{
			Pixels = JointsInDepthSpace;
			// Are joints within image ?
			if ( Pixels[joint0].x < 0 || Pixels[joint0].x > DepthWidth || Pixels[joint0].y < 0 || Pixels[joint0].y > DepthHeight ||
				 Pixels[joint1].x < 0 || Pixels[joint1].x > DepthWidth || Pixels[joint1].y < 0 || Pixels[joint1].y > DepthHeight )
			{
				return;
			}
		}
		else
		{
			Pixels = JointsInColorSpace;
			// Are joints within image ?
			if ( Pixels[joint0].x < 0 || Pixels[joint0].x > CamWidth || Pixels[joint0].y < 0 || Pixels[joint0].y > CamHeight ||
				 Pixels[joint1].x < 0 || Pixels[joint1].x > CamWidth || Pixels[joint1].y < 0 || Pixels[joint1].y > CamHeight )
			{
				return;
			}
		}

		// If we can't find either of these joints, exit
		if ( state0 == TrackingState_NotTracked || state1 == TrackingState_NotTracked )
		{
			return;
		}

		// Don't draw if both points are inferred
		if (state0 == TrackingState_Inferred && state1 == TrackingState_Inferred )
		{
			return;
		}

		// We assume all drawn bones are inferred unless BOTH joints are tracked
		if ( state0 == TrackingState_Tracked && state1 == TrackingState_Tracked )
		{
			line(frame,
				cv::Point((int)(Pixels[joint0].x*scale_x), (int)(Pixels[joint0].y*scale_y)),
				cv::Point((int)(Pixels[joint1].x*scale_x), (int)(Pixels[joint1].y*scale_y)),
				cvScalar(0,255,0),
				2,
				8,
				0);
		}
		else
		{
			line(frame,
				cv::Point((int)(Pixels[joint0].x*scale_x), (int)(Pixels[joint0].y*scale_y)),
				cv::Point((int)(Pixels[joint1].x*scale_x), (int)(Pixels[joint1].y*scale_y)),
				cvScalar(0,0,255),
				1,
				8,
				0);
		}

	}

	/** @brief Draw on body.
	 *
	 * @param WhereToDraw [in] cv::Mat object to draw in.
	 * @param DrawInDepth [in] Do we draw in Depth frame or in RGB frame?
	 */
	void Draw(cv::Mat WhereToDraw, bool DrawInDepth = true )
	{
		float scale_x = 1.0f;
		float scale_y = 1.0f;

		// Compute actual scaling factor
		if ( DrawInDepth == true )
		{
			scale_x = (float)WhereToDraw.cols/(float)DepthWidth;
			scale_y = (float)WhereToDraw.rows/(float)DepthHeight;
		}
		else
		{
			scale_x = (float)WhereToDraw.cols/(float)CamWidth;
			scale_y = (float)WhereToDraw.rows/(float)CamHeight;
		}

		// Torso
		DrawBone(WhereToDraw,JointType_Head, JointType_Neck, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_Neck, JointType_SpineShoulder, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_SpineShoulder, JointType_SpineMid, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_SpineMid, JointType_SpineBase, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_SpineShoulder, JointType_ShoulderRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_SpineShoulder, JointType_ShoulderLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_SpineBase, JointType_HipRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_SpineBase, JointType_HipLeft, DrawInDepth, scale_x, scale_y );
    
		// Right Arm    
		DrawBone(WhereToDraw,JointType_ShoulderRight, JointType_ElbowRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_ElbowRight, JointType_WristRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_WristRight, JointType_HandRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_HandRight, JointType_HandTipRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_WristRight, JointType_ThumbRight, DrawInDepth, scale_x, scale_y );

		// Left Arm
		DrawBone(WhereToDraw,JointType_ShoulderLeft, JointType_ElbowLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_ElbowLeft, JointType_WristLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_WristLeft, JointType_HandLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_HandLeft, JointType_HandTipLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_WristLeft, JointType_ThumbLeft, DrawInDepth, scale_x, scale_y );

		// Right Leg
		DrawBone(WhereToDraw,JointType_HipRight, JointType_KneeRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_KneeRight, JointType_AnkleRight, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_AnkleRight, JointType_FootRight, DrawInDepth, scale_x, scale_y );

		// Left Leg
		DrawBone(WhereToDraw,JointType_HipLeft, JointType_KneeLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_KneeLeft, JointType_AnkleLeft, DrawInDepth, scale_x, scale_y );
		DrawBone(WhereToDraw,JointType_AnkleLeft, JointType_FootLeft, DrawInDepth, scale_x, scale_y );
	}

#endif // ACTIVATE_KINECT_DRAWING

};

/**
 * @class KinectBodies KinectBody.cpp KinectBody.h
 * @brief Class to handle several KinectBody. All data are gathered in a unique buffer.
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 */
class KinectBodies
{
public:
	/** @brief constructor. Will set the data in memory for up to BODY_COUNT KinectBody instances.
	 *
	 * @param Buffer [in] Buffer to use for KinectBody data
	 */
	KinectBodies(unsigned char * ExtBuffer = nullptr)
	{
		// Compute size of a body/face in bytes => result in KinectBody::BodySize KinectFace::FaceSize static values
		BodiesInformation[0].Set(nullptr);

		// Allocate memory
		if ( ExtBuffer == (unsigned char*)nullptr )
		{
			AllocatedBuffer.SetNewBufferSize(BODY_COUNT*KinectBody::BodySize);
			BodyData = (unsigned char*)AllocatedBuffer;
		}
		else
		{
			BodyData = ExtBuffer;
		}
		
		// Structure will be stored in 1 memory block, initialize this buffer
		ActualNbBody = 0;
		unsigned char * lBuffer = BodyData;
		for( int i = 0; i < BODY_COUNT; i++ )
		{
			BodiesInformation[i].Set(lBuffer);
			lBuffer += KinectBody::BodySize;
			BodyIsPresent[i] = false;
#ifdef KINECT_LIVE			
			ppBodies[i] = nullptr;
#endif
		}
	}

	/** @brief Virtual destructor, always.
	 */
	virtual ~KinectBodies()
	{
#ifdef KINECT_LIVE		
		for( int i = 0; i < BODY_COUNT; i++ )
		{
			SafeRelease(ppBodies[i]);
		}
#endif
	}

	
	/** @brief Copy operator
	 */
	KinectBodies& operator=(KinectBodies& Other)
	{
		ActualNbBody = Other.ActualNbBody;

		// Copy ppBodies is useless, it has been released
		// BodyIsPresent
		memcpy( BodyIsPresent, Other.BodyIsPresent, sizeof(BodyIsPresent) );
		// Initial body index
		memcpy( InitialBodyIndex, Other.InitialBodyIndex, sizeof(InitialBodyIndex) );

		if ( ActualNbBody == 0 )
		{
				return Other;
		}

		// copy data to data
		memcpy( BodyData, Other.BodyData, AllocatedBuffer.GetLength() );

		return Other;
	}

#ifdef KINECT_LIVE
	/** @brief Fill all KinectBody information available in one atomic buffer.
	 *
	 * @return size of the data buffer (maybe less than size of the fuill  buffer).
	 */
	int GatherBodiesInformationAndReturnSizeOfBodyBuffer()
	{
		// First, we do not know how many valid bodies we have
		ActualNbBody = 0;

		// Fill BodyInformation
		for (unsigned int i = 0; i < BODY_COUNT; ++i)
		{
			IBody* pBody = ppBodies[i];
			if ( pBody != (IBody*)nullptr )
			{
				BOOLEAN IsTracked = FALSE;
				pBody->get_IsTracked(&IsTracked);
				if ( IsTracked == FALSE )
				{
					BodyIsPresent[i] = false;
					SafeRelease(ppBodies[i]);
					continue;
				}

				BodyIsPresent[i] = true;
				BodiesInformation[ActualNbBody].Init(pBody);
				InitialBodyIndex[ActualNbBody] = i;
				
				ActualNbBody++;
			}

			SafeRelease(ppBodies[i]);
		}

		return (ActualNbBody * KinectBody::BodySize);
	}

	
	IBody* ppBodies[BODY_COUNT];				/*!< @brief Store Body from Kinect, maybe sparse, i.e. 2 skeletons but 1st and 5th. */
#endif 

	bool BodyIsPresent[BODY_COUNT];				/*!< @brief Store presence of body data state. */
	int InitialBodyIndex[BODY_COUNT];			/*!< @brief Store index in original Kinect data. */

	KinectBody BodiesInformation[BODY_COUNT];	/*!< @brief Up to BODY_COUNT KinectBody can be handled. */
	unsigned int ActualNbBody;					/*!< @brief Actual number of body in the buffer. */
	unsigned char * BodyData;					/*!< @brief Store current bodies information in one memory block for save/load operation. */

protected:
	Omiscid::MemoryBuffer AllocatedBuffer;		/*!< @brief Memory manager for BodyData. */
};

}}	// namespace MobileRGBD::Kinect2

#endif // KINECT_2

#endif // __KINECT_BODY_H__