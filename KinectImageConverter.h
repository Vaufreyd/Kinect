/**
 * @file KinectImageConverter.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @author Emeric Grange, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_IMAGE_CONVERTER_H__
#define __KINECT_IMAGE_CONVERTER_H__

#include <System/MemoryBuffer.h>

/**
 * @class DrawBodyIndexView DrawBodyIndexView.cpp DrawBodyIndexView.h
 * @brief Class to convert YVY2 from Kinect2 to BGR buffer
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @author Emeric Grange, Inria
 */
class KinectImageConverter
{
public:
	/** @brief constructor.
	 */
	KinectImageConverter() {}

	/** @brief Virtual destructor, always.
	 */
	virtual ~KinectImageConverter() {}

	/** @brief Conversion function in internal buffer. A buffer will be allocated an return for conversion.
	 *
	 * @param buffer_in [in] buffer to the YVY2 data from the Kinect2.
	 * @param rawFile_width [in] original width of the data.
	 * @param rawFile_height [in] original height of the data.
	 * @param resizefactor [in] resize image to a smaller one (by parsing less data from buffer_in). resizefactor must be a power of 2.
	 * @return Pointer to an internal buffer used for conversion. nullptr if there was a problem.
	 */
	unsigned char * ConvertYVY2ToBRG(unsigned char *buffer_in, int rawFile_width, int rawFile_height, int resizefactor  = 1 );

	/** @brief Conversion function in internal buffer. A buffer will be allocated an return for conversion.
	 *
	 * @param buffer_in [in] buffer to the YVY2 data from the Kinect2.
	 * @param buffer_out [in,out] buffer where the BGR data will be stored.
	 * @param rawFile_width [in] original width of the data.
	 * @param rawFile_height [in] original height of the data.
	 * @param resizefactor [in] resize image to a smaller one (by parsing less data from buffer_in). resizefactor must be a power of 2.
	 * @return true if conversion was done.
	 */
	bool ConvertYVY2ToBRG(unsigned char *buffer_in, unsigned char *buffer_out, int rawFile_width, int rawFile_height, int resizefactor = 1 );

protected:
	Omiscid::MemoryBuffer InternalBuffer;
};

#endif // __KINECT_IMAGE_CONVERTER_H__
