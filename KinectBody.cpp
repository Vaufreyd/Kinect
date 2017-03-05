/**
 * @file KinectBody.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "KinectBody.h"

using namespace MobileRGBD::Kinect2;

/* static */ int KinectBody::BodySize = 0;						/*!< @brief Size of all KinectBody buffer. To compute it, one can call Set with nullptr */

#if (defined WIN32 || defined WIN64) && defined KINECT_LIVE
	ICoordinateMapper * KinectBody::CoordinateMapper = nullptr;	/*!< Coordinate Mapper. Exists only if KINECT_LIVE is defined. */
#endif