/**
 * @file KinectDraw.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "KinectDraw.h"

#ifdef KINECT_2

#include "KinectImageConverter.h"

#ifdef ACTIVATE_KINECT_DRAWING
	#include <Drawing/DrawCameraView.h>
	#include <Drawing/DrawDepthView.h>
#endif

#endif

using namespace MobileRGBD;
using namespace MobileRGBD::Kinect2;


const char * KinectDraw::WindowNames[NumberOfItems] =
{
	"RGB",				// SHOW_KINECT_RGB = 1				=> 0		SHOW_KINECT_NONE = 0,
	"Depth",			// SHOW_KINECT_DEPTH = 2			=> 1
	"Infrared",			// SHOW_KINECT_IR = 4				=> 2
	"LE Infrared",		// SHOW_KINECT_LIR = 8				=> 3
	"Skeleton",			// SHOW_KINECT_BODY = 16			=> 4
	"Body index",		// SHOW_KINECT_BODYINDEX			=> 5
	"Face",				// SHOW_KINECT_FACE	= 64			=> 6
	(char*)nullptr,
	(char*)nullptr,
	(char*)nullptr
};


KinectDraw::KinectDraw()
{
	RGBScaleRatio = 4;	// Default
	ShowFlags = SHOW_KINECT_NONE;
}


KinectDraw::~KinectDraw()
{
}


// Viewing management
void KinectDraw::Show(short int ShowChoices)
{
	fprintf( stderr, "Live drawing functions will be implemented later due to issues with Opencv windows and multithreading.");
	ShowFlags |= ShowChoices;
}

void KinectDraw::Hide(short int ShowChoices)
{
	ShowFlags &= ~ShowChoices;

	/*int NumOfWindowName = GetNum(ShowChoices);
	cvDestroyWindow(WindowNames[NumOfWindowName]);
	cv::waitKey(1);*/
}

void KinectDraw::DrawRGBFrame(unsigned char *buffer_in, int rawFile_width, int rawFile_height)
{
#ifdef MULTITHREADING_ISSUES_WITH_OPENCV_ARE_SOLVED

#ifdef ACTIVATE_KINECT_DRAWING
	int NumOfWindowName = GetNum(SHOW_KINECT_RGB);

#ifdef KINECT_2
	cv::Mat MatInit(rawFile_height/RGBScaleRatio, rawFile_width/RGBScaleRatio, CV_8UC3);
	DrawCameraView::Draw(MatInit, buffer_in, RGBScaleRatio  );
	imshow(WindowNames[NumOfWindowName], MatInit);
#else //KINECT 1
	cv::Mat MatInit(rawFile_height/RGBScaleRatio, rawFile_width/RGBScaleRatio, CV_8UC3, buffer_in);
	cv::imshow(WindowNames[NumOfWindowName], MatInit);
#endif // KINECT_2
#endif // ACTIVATE_KINECT_DRAWING

#endif
}

void KinectDraw::Draw16bitsFrame(int NumOfWindowName, unsigned char *buffer_in, int rawFile_width, int rawFile_height)
{
#ifdef MULTITHREADING_ISSUES_WITH_OPENCV_ARE_SOLVED

#ifdef ACTIVATE_KINECT_DRAWING
	cv::Mat MatInit(rawFile_height,rawFile_width, CV_16UC1, buffer_in);
	cv::Mat MatForConversion(rawFile_height, rawFile_width, CV_8UC3);
	MatInit.convertTo(MatForConversion, CV_8UC3, 1.0 / 16.0, 0.0);

	imshow(WindowNames[NumOfWindowName], MatForConversion);
#endif // ACTIVATE_KINECT_DRAWING
	
#endif
}

void KinectDraw::DrawDepthFrame(unsigned char *buffer_in, int rawFile_width, int rawFile_height)
{
#ifdef MULTITHREADING_ISSUES_WITH_OPENCV_ARE_SOLVED

#ifdef ACTIVATE_KINECT_DRAWING
	int NumOfWindowName = GetNum(SHOW_KINECT_DEPTH);
#ifdef KINECT_2
	cv::Mat MatInit(DepthHeight,DepthWidth, CV_16UC1);
	DrawDepthView::Draw( MatInit, buffer_in );
	imshow(WindowNames[NumOfWindowName], MatInit);
#endif // KINECT_2

#endif // ACTIVATE_KINECT_DRAWING

#endif
}

void KinectDraw::DrawInfraredFrame(unsigned char *buffer_in, int rawFile_width, int rawFile_height)
{
#ifdef MULTITHREADING_ISSUES_WITH_OPENCV_ARE_SOLVED

#ifdef ACTIVATE_KINECT_DRAWING

#ifdef KINECT_2
	int NumOfWindowName = GetNum(SHOW_KINECT_IR);

	Draw16bitsFrame( NumOfWindowName, buffer_in, rawFile_width, rawFile_height);
#endif // KINECT_2

#endif // ACTIVATE_KINECT_DRAWING

#endif
}
