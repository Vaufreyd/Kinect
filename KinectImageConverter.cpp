/**
 * @file KinectImageConverter.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @author Emeric Grange, Inria
 * @copyright All right reserved.
 */

#include "KinectImageConverter.h"

#include <algorithm>

/** @brief Clip BGR value in [0:255]
 */
inline int clip(const int Val)
{
	// return std::min(std::max(Val, 0), 255);
	if ( Val < 0 )
	{
		return 0;
	}
	if (Val > 255)
	{
		return 255;
	}
	return Val;
	// return std::min(Val, 0);
}

#define IndexB 0	/* @brief index of Blue data, could be changed to do RGB instead of BGR */
#define IndexG 1	/* @brief index of Green data, could be changed to do RGB instead of BGR */
#define IndexR 2	/* @brief index of Red data, could be changed to do RGB instead of BGR */

/** @brief Conversion function in internal buffer. A buffer will be allocated an return for conversion.
	*
	* @param buffer_in [in] buffer to the YVY2 data from the Kinect2.
	* @param buffer_out [in,out] buffer where the BGR data will be stored.
	* @param rawFile_width [in] original width of the data.
	* @param rawFile_height [in] original height of the data.
	* @param resizefactor [in] resize image to a smaller one (by parsing less data from buffer_in). resizefactor must be a power of 2.
	* @return true if conversion was done.
	*/
bool KinectImageConverter::ConvertYVY2ToBRG(unsigned char *buffer_in, unsigned char *ConvertedBuffer, int rawFile_width, int rawFile_height, int resizefactor /* = 1 */ )
{
	if ( buffer_in == nullptr || rawFile_width == 0 || rawFile_height == 0 )
	{
		return false;
	}

	if ( resizefactor == 1 )
	{
		for ( int i = 0; i < rawFile_height; i++ )
		{
			unsigned char * buffer_out = &ConvertedBuffer[i*rawFile_width * 3];

			// Conversion
			for (int j = 0; j < rawFile_width / 2; j++)
			{
				// Read 32 bit = represents 2 pixels
				int y0 = buffer_in[0];
				int u0 = buffer_in[1];
				int y1 = buffer_in[2];
				int v0 = buffer_in[3];
				buffer_in += 4;

				//std::cout << "Pixel = [y0: " << y0 << " u0: " << u0 << " y1: " << y1 << " v0: " << v0 << std::endl;

				int c = y0 - 16;
				int d = u0 - 128;
				int e = v0 - 128;

				buffer_out[IndexR] = clip(((298 * c) + (409 * e) + 128) >> 8);				// R
				buffer_out[IndexG] = clip(((298 * c) - (100 * d) - (208 * e) + 128) >> 8);	// G
				buffer_out[IndexB] = clip(((298 * c) + (516 * d) + 128) >> 8);				// B
				buffer_out += 3;

				// only the luma change for the second pixel
				c = y1 - 16;

				// RGB 2
				buffer_out[IndexR] = clip((298 * c + 409 * e + 128) >> 8);				// R
				buffer_out[IndexG] = clip((298 * c - 100 * d - 208 * e + 128) >> 8);		// G
				buffer_out[IndexB] = clip((298 * c + 516 * d + 128) >> 8);				// B
				buffer_out += 3;
			}
		}
	}
	else
	{
		int SkipFactor= 4 * resizefactor/2;

		for ( int i = 0; i < rawFile_height/resizefactor; i++ )
		{
			unsigned char * buffer_out = &ConvertedBuffer[i*(rawFile_width / resizefactor) * 3];
			unsigned char * l_buffer_in = &buffer_in[i*rawFile_width * 2 * resizefactor];

			// Conversion
			for (int j = 0; j < rawFile_width / resizefactor; j++)
			{
				// Read 32 bit = represents 2 pixels
				int y0 = l_buffer_in[0];
				int u0 = l_buffer_in[1];
				int y1 = l_buffer_in[2];
				int v0 = l_buffer_in[3];
				l_buffer_in += SkipFactor;

				//std::cout << "Pixel = [y0: " << y0 << " u0: " << u0 << " y1: " << y1 << " v0: " << v0 << std::endl;

				int c = y0 - 16;
				int d = u0 - 128;
				int e = v0 - 128;

				buffer_out[IndexR] = clip(((298 * c) + (409 * e) + 128) >> 8);				// R
				buffer_out[IndexG] = clip(((298 * c) - (100 * d) - (208 * e) + 128) >> 8);	// G
				buffer_out[IndexB] = clip(((298 * c) + (516 * d) + 128) >> 8);				// B
				buffer_out += 3;

				// only the luma change for the second pixel
				c = y1 - 16;
			}
		}
	}

	return true;
}

/** @brief Conversion function in internal buffer. A buffer will be allocated an return for conversion.
	*
	* @param buffer_in [in] buffer to the YVY2 data from the Kinect2.
	* @param rawFile_width [in] original width of the data.
	* @param rawFile_height [in] original height of the data.
	* @param resizefactor [in] resize image to a smaller one (by parsing less data from buffer_in). resizefactor must be a power of 2.
	* @return Pointer to an internal buffer used for conversion. nullptr if there was a problem.
	*/
unsigned char * KinectImageConverter::ConvertYVY2ToBRG(unsigned char *buffer_in, int rawFile_width, int rawFile_height, int resizefactor /* = 1 */)
{
	if ( buffer_in == nullptr || rawFile_width == 0 || rawFile_height == 0 ) 
	{
		return (unsigned char*)nullptr;
	}

	try
	{
		 InternalBuffer.SetNewBufferSize((rawFile_width*rawFile_height*3)/resizefactor);
		 unsigned char * ConvertedBuffer = InternalBuffer;
		 if ( ConvertYVY2ToBRG(buffer_in, ConvertedBuffer, rawFile_width, rawFile_height, resizefactor ) == true )
		 {
			 return ConvertedBuffer;
		 }
		 // Will return null after exception catching
	}
	catch(Omiscid::SimpleException&)
	{
	}

	return (unsigned char*)nullptr;
}