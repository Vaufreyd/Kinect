/**
 * @file KinectFace.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "KinectFace.h"

using namespace MobileRGBD::Kinect2;

/* static */ int KinectFace::FaceSize = 0;		/*!< @brief Size of all KinectFace buffer. To compute it, one can call Set with nullptr */