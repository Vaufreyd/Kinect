/**
 * @file KinectBasics.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_BASICS_H__
#define __KINECT_BASICS_H__

#ifdef KINECT_1

namespace MobileRGBD { namespace Kinect1 {

#ifndef OMISCID_ON_WINDOWS
	typedef INT64 TIMESPAN;
#else
	typedef long long int TIMESPAN;
#endif

static const int CamWidth = 640;
static const int CamHeight = 480;
static const int CamBytesPerPixel = 4;
static const int DepthWidth = 640;
static const int DepthHeight = 480;
static const int DepthBytesPerPixel = 2;

}} // namespace MobileRGBD::Kinect1

#endif // KINECT_1

// If we compile for Kinect2 support
#ifdef KINECT_2

#include <cstdio>

// Do we used the Kinect in live (for recording for instance) ?
#ifdef KINECT_LIVE

#define _WINSOCKAPI_   /* Prevent inclusion of winsock.h in windows.h */
#include <Kinect.h>
#include <Kinect.Face.h>

namespace MobileRGBD { namespace Kinect2 {

// Safe release for interfaces, from Microsoft SDK examples
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if ( pInterfaceToRelease != nullptr )
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

// For Linux compatibility
#if defined WIN32 || defined WIN64
typedef TrackingState tTrackingState;
#endif // OMISCID_ON_WINDOWS


}} // namespace MobileRGBD::Kinect2

#else

namespace MobileRGBD { namespace Kinect2 {

// Here we did no use the Kinect in live (not recording or Linux/Mac OSX)
// Define everything we need to work with data
#ifndef OMISCID_ON_WINDOWS

// type definition for linux/MacOS
typedef unsigned char BOOLEAN;
#define FALSE               0
#define TRUE                1

typedef int INT32;
typedef long int INT64;
typedef unsigned long int    UINT64;

// From Kinect.h
typedef INT64 TIMESPAN;
typedef INT64 DWORD;

#endif // OMISCID_ON_WINDOWS

const int BODY_COUNT = 6;

typedef enum 
{
    FrameSourceTypes_None = 0x00, FrameSourceTypes_Color = 0x01, FrameSourceTypes_Infrared = 0x02, FrameSourceTypes_LongExposureInfrared = 0x04,
    FrameSourceTypes_Depth = 0x08, FrameSourceTypes_BodyIndex = 0x10, FrameSourceTypes_Body = 0x20, FrameSourceTypes_Audio = 0x40
} FrameSourceTypes;

typedef enum 
{
    HandState_Unknown = 0, HandState_NotTracked, HandState_Open, HandState_Closed, HandState_Lasso
} HandState;

typedef enum 
{
        Expression_Neutral = 0, Expression_Happy,
        Expression_Count
} Expression;

typedef enum 
{
    DetectionResult_Unknown = 0, DetectionResult_No, DetectionResult_Maybe, DetectionResult_Yes
} DetectionResult;

typedef enum 
{
    TrackingConfidence_Low = 0, TrackingConfidence_High
} TrackingConfidence;

typedef enum 
{
    Activity_EyeLeftClosed = 0, Activity_EyeRightClosed, Activity_MouthOpen, Activity_MouthMoved, Activity_LookingAway,
    Activity_Count
} Activity;

typedef enum 
{
    Appearance_WearingGlasses = 0,
    Appearance_Count
} Appearance;

typedef enum 
{
	JointType_SpineBase = 0, JointType_SpineMid, JointType_Neck, JointType_Head, JointType_ShoulderLeft, JointType_ElbowLeft,
	JointType_WristLeft, JointType_HandLeft, JointType_ShoulderRight, JointType_ElbowRight, JointType_WristRight,
	JointType_HandRight, JointType_HipLeft, JointType_KneeLeft, JointType_AnkleLeft, JointType_FootLeft, JointType_HipRight,
	JointType_KneeRight, JointType_AnkleRight, JointType_FootRight, JointType_SpineShoulder, JointType_HandTipLeft,
	JointType_ThumbLeft, JointType_HandTipRight, JointType_ThumbRight,
	JointType_Count
} tJointType;

typedef enum 
{
    TrackingState_NotTracked = 0, TrackingState_Inferred, TrackingState_Tracked
} tTrackingState;


typedef struct
{
	float x;
	float y;
	float z;
	float w;
} Vector4;

typedef struct
{
	float X;
	float Y;
} PointF;


typedef struct
{
	float X;
	float Y;
	float Width;
	float Height;
} RectF;

typedef struct
{
	float X;
	float Y;
} ColorSpacePoint;

typedef struct
{
	float X;
	float Y;
} DepthSpacePoint;

typedef struct
{
	float X;
	float Y;
	float Z;
} CameraSpacePoint;

typedef struct
{
	tJointType JointType;
	CameraSpacePoint Position;
	tTrackingState TrackingState;
} Joint;

typedef struct
{
	tJointType JointType;
	Vector4 Orientation;
} JointOrientation;

typedef struct
{
	float FocalLengthX; float FocalLengthY;
	float PrincipalPointX; float PrincipalPointY;
	float RadialDistortionSecondOrder; float RadialDistortionFourthOrder; float RadialDistortionSixthOrder;
} CameraIntrinsics;

typedef enum 
{
	FacePointType_None = -1, FacePointType_EyeLeft, FacePointType_EyeRight, FacePointType_Nose,
	FacePointType_MouthCornerLeft, FacePointType_MouthCornerRight, FacePointType_Count
} FacePointType;

typedef enum 
{
	FaceFrameFeatures_None = 0x0000, FaceFrameFeatures_BoundingBoxInInfraredSpace = 0x0001,
	FaceFrameFeatures_PointsInInfraredSpace = 0x0002, FaceFrameFeatures_BoundingBoxInColorSpace = 0x0004,
	FaceFrameFeatures_PointsInColorSpace = 0x0008, FaceFrameFeatures_RotationOrientation = 0x0010,
	FaceFrameFeatures_Happy = 0x0020, FaceFrameFeatures_RightEyeClosed = 0x0040,
	FaceFrameFeatures_LeftEyeClosed = 0x0080, FaceFrameFeatures_MouthOpen = 0x0100,
	FaceFrameFeatures_MouthMoved = 0x0200, FaceFrameFeatures_LookingAway = 0x0400,
	FaceFrameFeatures_Glasses = 0x0800, FaceFrameFeatures_FaceEngagement = 0x1000
} FaceFrameFeatures;

typedef enum 
{
	FaceProperty_Happy = 0, FaceProperty_Engaged, FaceProperty_WearingGlasses, FaceProperty_LeftEyeClosed,
	FaceProperty_RightEyeClosed, FaceProperty_MouthOpen, FaceProperty_MouthMoved, FaceProperty_LookingAway,
	FaceProperty_Count
} FaceProperty;

typedef struct
{
	INT32 Left;
	INT32 Top;
	INT32 Right;
	INT32 Bottom;
} 	RectI;

}} // namespace MobileRGBD::Kinect2


#endif // KINECT_LIVE

namespace MobileRGBD { namespace Kinect2 {

static const int CamWidth = 1920;
static const int CamHeight = 1080;
static const int CamBytesPerPixel = 2;		// 32 bits for 2 pixels, 2 bytes per pixel
static const int DepthWidth = 512;
static const int DepthHeight = 424;
static const int DepthBytesPerPixel = 2;
static const int InfraredWidth = DepthWidth;
static const int InfraredHeight = DepthHeight;
static const int InfraredBytesPerPixel = 2;

typedef struct
{
	float x;
	float y;
} PixelPosition;

}} // namespace MobileRGBD::Kinect2

#endif // KINECT_2

#endif // __KINECT_BASICS_H__