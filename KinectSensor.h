/**
 * @file KinectSensor.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_SENSOR_H__
#define __KINECT_SENSOR_H__

#ifdef KINECT_2
#include "KinectSensor-v2.h"
#else
#include "KinectSensor-v1.h"
#endif

#include <string>
#include <sstream>
#include <sys/timeb.h>
#include <time.h>
#include <vector>

#endif //__KINECT_SENSOR_H__