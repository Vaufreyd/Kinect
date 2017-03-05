/**
 * @file RecordingContextFactory.h
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef __RECORDING_CONTEXT_FACTORY_H__
#define __RECORDING_CONTEXT_FACTORY_H__

#include "KinectRecording.h"
#include "RawKinectRecording.h"

class RecordingContextFactory
{
public:
	enum ContextType { DummyRecContext, VideoRecContext, BodyReccontext, FaceRecContext, AudioRecContext };

	static KinectRecording * CreateContextRecording(const Omiscid::SimpleString& Prefix, const Omiscid::SimpleString& _FrameType, ContextType TypeOfContext)
	{
		switch(TypeOfContext)
		{
			case DummyRecContext:
				return &EmptyRecContext;

			case VideoRecContext:
			{
				RawKinectRecording * pTmp = new RawKinectRecording();
				if ( pTmp != nullptr )
				{
					pTmp->Init( Prefix, _FrameType );
					pTmp->AllocateBuffer( 5*1024*1024 + 2 ); // Max image size, + 2 for padding if necessary
					// memset( pTmp->BufferData, 0, 10*1024*1024 + 2 );
				}
				return pTmp;
			}

			default:
				fprintf( stderr, "Unkown ContextType in RecordingContextFactory::CreateContextRecording\n" );
				return nullptr;
		}
	}

	static KinectRecording& GetEmptyRecContext()
	{
		return EmptyRecContext;
	}

protected:
	static KinectRecording EmptyRecContext;
};

#endif // __RECORDING_CONTEXT_FACTORY_H__
