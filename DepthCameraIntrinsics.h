/**
 * @file DepthCameraIntrinsics.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __DEPTH_CAMERA_INTRINSICS_H__
#define __DEPTH_CAMERA_INTRINSICS_H__

#ifdef KINECT_2

#include "KinectBasics.h"

// Add Serialization facilities
#include <Messaging/Serializable.h>

class DepthCameraIntrinsics : public Omiscid::Serializable
{
public:
	DepthCameraIntrinsics() :
		FocalLengthX(IntrinsicsParameters.FocalLengthX),
		FocalLengthY(IntrinsicsParameters.FocalLengthY),
		PrincipalPointX(IntrinsicsParameters.PrincipalPointX),
		PrincipalPointY(IntrinsicsParameters.PrincipalPointY),
		RadialDistortionSecondOrder(IntrinsicsParameters.RadialDistortionSecondOrder),
		RadialDistortionFourthOrder(IntrinsicsParameters.RadialDistortionFourthOrder),
		RadialDistortionSixthOrder(IntrinsicsParameters.RadialDistortionSixthOrder)
	{};

	virtual ~DepthCameraIntrinsics() {};

	// To (Un)serialize data, declare JSON mapping 
	virtual void DeclareSerializeMapping()
	{
		AddToSerialization( "FocalLengthX", IntrinsicsParameters.FocalLengthX );
		AddToSerialization( "FocalLengthY", IntrinsicsParameters.FocalLengthY );
		AddToSerialization( "PrincipalPointX", IntrinsicsParameters.PrincipalPointX );
		AddToSerialization( "PrincipalPointY", IntrinsicsParameters.PrincipalPointY );
		AddToSerialization( "RadialDistortionSecondOrder", IntrinsicsParameters.RadialDistortionSecondOrder );
		AddToSerialization( "RadialDistortionFourthOrder", IntrinsicsParameters.RadialDistortionFourthOrder );
		AddToSerialization( "RadialDistortionSixthOrder", IntrinsicsParameters.RadialDistortionSixthOrder );
	}

	// Add references to intrinsics values
	float &FocalLengthX;
    float &FocalLengthY;
    float &PrincipalPointX;
    float &PrincipalPointY;
    float &RadialDistortionSecondOrder;
    float &RadialDistortionFourthOrder;
    float &RadialDistortionSixthOrder;

public:
	// Internal value from the Kinect
	CameraIntrinsics IntrinsicsParameters;
};

#endif // KINECT_2

#endif // __DEPTH_CAMERA_INTRINSICS_H__
