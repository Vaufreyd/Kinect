/**
 * @file KinectDraw.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_DRAW_H__
#define __KINECT_DRAW_H__

#ifdef ACTIVATE_KINECT_DRAWING

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#endif // ACTIVATE_KINECT_DRAWING

#include <cmath>

class KinectDraw
{
public:
	KinectDraw();
	virtual ~KinectDraw();

	enum { NumberOfItems = 10 };
	static const char * WindowNames[NumberOfItems];	// Must put a constant value...

	// Viewing management
	short int RGBScaleRatio;
	enum {
		SHOW_KINECT_NONE = 0,
		SHOW_KINECT_RGB = 1,				
		SHOW_KINECT_DEPTH = 2,				
		SHOW_KINECT_IR = 4,					
		SHOW_KINECT_LIR = 8,				
		SHOW_KINECT_BODY = 16,				
		SHOW_KINECT_BODYINDEX = 32,			
		SHOW_KINECT_FACE = 64,				
		SHOW_KINECT_ALL = 0xffff };

	void Show(short int ShowChoices);
	void Hide(short int ShowChoices);

	inline int GetNum( int ToDraw )
	{
		return (int)(log((float)ToDraw)/log(2.0f));
	}

	void DrawRGBFrame(unsigned char *buffer_in, int rawFile_width, int rawFile_height );
	void DrawDepthFrame(unsigned char *buffer_in, int rawFile_width, int rawFile_height);
	void DrawInfraredFrame(unsigned char *buffer_in, int rawFile_width, int rawFile_height);

protected:
	short int ShowFlags;

	void Draw16bitsFrame(int NumOfWindowName, unsigned char *buffer_in, int rawFile_width, int rawFile_height);
};

#endif // __KINECT_DRAW_H__
