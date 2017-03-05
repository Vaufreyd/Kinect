/**
 * @file KinectDataAsMemoryBuffer.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __KINECT_DATA_AS_MEMORY_BUFFER_H__
#define __KINECT_DATA_AS_MEMORY_BUFFER_H__

#ifdef KINECT_2

#include "KinectBasics.h"
#include <System/MemoryBuffer.h>

namespace MobileRGBD { namespace Kinect2 {

/**
 * @class KinectDataAsMemoryBuffer KinectDataAsMemoryBuffer.cpp KinectDataAsMemoryBuffer.h
 * @brief Manage buffer for Kinect data.
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 */
class KinectDataAsMemoryBuffer
{
public:
	/** @brief constructor.
	 */
	KinectDataAsMemoryBuffer() {};

	/** @brief Virtual destructor, always.
	 */
	virtual ~KinectDataAsMemoryBuffer() {};

	/** @brief Abstract Set function. Set will put all reference to memory buffer for KinectBody data
	 *
	 * @param Buffer [in] Buffer to use for data
	 */
	virtual void Set(unsigned char *Buffer) = 0;

protected:
	/** @brief Store in an unsigned char value a BOOLEAN value from the Kinect
	 *
	 * @param ToSet [in] Where to store the value
	 * @param Val [in] Value to store
	 */
	static inline void SetValueFromBollean(unsigned char& ToSet, BOOLEAN Val)
	{
		if ( Val == TRUE )
		{
			ToSet = 1;
		}
		else
		{
			ToSet = 0;
		}
	}

	/** @brief Store in a bool value a BOOLEAN value from the Kinect
	 *
	 * @param ToSet [in] Where to store the value
	 * @param Val [in] Value to store
	 */
	static inline void SetValueFromBollean(bool& ToSet, BOOLEAN Val)
	{
		if ( Val == TRUE )
		{
			ToSet = true;
		}
		else
		{
			ToSet = false;
		}
	}

	/** @brief Set adress of data at Buffer. Increase Buffer by sizeof(TYPE)*NumberOfItems.
	 *
	 * @param TypePointer [in] Type of data
	 * @param Buffer [in,out] Starting buffer for the data
	 * @param NumberOfItems [in] Number of element to store (default=1, more than 1 for arrays).
	 */
	template<typename TYPE>
	void SetAddressAndIncreaseBuffer(TYPE*& TypePointer, unsigned char*&Buffer, int NumberOfItems = 1)
	{
		TypePointer = (TYPE*)Buffer;
		int SizeOfEment = sizeof(TYPE)*NumberOfItems;
		Buffer += SizeOfEment;
	}
};

}} // namespace MobileRGBD::Kinect2

#endif // KINECT_2

#endif // __KINECT_DATA_AS_MEMORY_BUFFER_H__