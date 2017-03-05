/**
 * @file RecordingManagement.cpp
 * @ingroup Kinect
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#include "RecordingManagement.h"

bool RecordingManagement::StartRecording( const char* SessionFolder )
{
	// If we are recording, do nothing
	if ( IsRecording == true )
	{
		return false;
	}

	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

	bool Success = true;
	for( RecContexts.First(); RecContexts.NotAtEnd(); RecContexts.Next() )
	{
		KinectRecording * pRec = RecContexts.GetCurrent();
		if ( pRec->IsActive )
		{
			if ( pRec->StartRecording( SessionFolder ) == false )
			{
				// One sub recorder does not start, it is likely that the other
				// won't start also, anyway, recording process is partial, stop it
				Success = false;
				break;
			}
		}
	}

	// Check is we succeeded in all sub start recording
	if ( Success == false )
	{
		// Unlock mutex
		ProtectAccess_SL.Unlock();

		// Stop all started recordings
		StopRecording();
		return false;
	}

	// everything went fine
	IsRecording = true;

	return true;
}

void RecordingManagement::StopRecording()
{
	if ( IsRecording == false )
	{
		return;
	}

	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

	double CurrentTime = GetCurrentTime();

	// Call all record context to stop
	for( RecContexts.First(); RecContexts.NotAtEnd(); RecContexts.Next() )
	{
		KinectRecording * pRec = RecContexts.GetCurrent();
		if ( pRec->IsActive )
		{
			pRec->StopRecording( CurrentTime );
		}
	}

	IsRecording = false;
}

KinectRecording* RecordingManagement::AddRecContext(const Omiscid::SimpleString& Prefix, const Omiscid::SimpleString& _FrameType, int TypeOfContext )
{
	 KinectRecording* pKR = RecordingContextFactory::CreateContextRecording( Prefix, _FrameType, RecordingContextFactory::VideoRecContext);
	 RecContexts.AddTail(pKR);
	 return pKR;
}

void RecordingManagement::ClearRecContexts()
{
	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

	while( RecContexts.IsNotEmpty() == true )
	{
		KinectRecording* pKR = RecContexts.ExtractFirst();
		if ( pKR != (KinectRecording*)nullptr )
		{
			delete pKR;
		}
	}
}

void RecordingManagement::SaveTimestamp( FILE* f, const struct timeb& lTimestamp, unsigned int numFrame, TIMESPAN FrameTime, char * SuppInfo /* = nullptr */ )
{
	if ( IsRecording == false )
	{
		return;
	}

	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

	KinectRecording::SaveTimestamp( f, lTimestamp, numFrame, FrameTime, SuppInfo );
}

void RecordingManagement::SaveDataAndIncreaseInputNumber( KinectRecording& RecordContext, const struct timeb& lTimestamp, char * SuppInfo /* = nullptr */ )
{
	Omiscid::SmartLocker ProtectAccess_SL(ProtectAccess);

	RecordContext.SaveDataAndIncreaseInputNumber( lTimestamp, SuppInfo );
}
