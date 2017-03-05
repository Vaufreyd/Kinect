#ifndef __KINECT_RECORDING_H__
#define __KINECT_RECORDING_H__

#include <Messaging/Serializable.h>
#include <System/Mutex.h>
#include <System/LockManagement.h>

#include "KinectBasics.h"

#include <sys/timeb.h>


class KinectRecording : public Omiscid::Serializable
{
public:
	FILE* TimestampedFile;

	bool RawRecording;
	FILE * RawFile;
	// Keep trace of BufferSize in RawMode
	unsigned int BufferSize;

	bool InitDescription;
	FILE * DescriptionFile;
		
	double StartTime;
	unsigned int InputNumber;
	TIMESPAN LastFrameTime;
	void * BufferData;
	float FrameRate;

	Omiscid::SimpleString FilePrefix;
	bool IsActive;

	Omiscid::Mutex InternalProtection;

	static void CloseAndSetNull(FILE*& fd)
	{
		if ((fd) != (FILE*)NULL)
		{
			fclose(fd);
			fd = (FILE*)NULL;
		}
	}

	KinectRecording()
	{
		Init( "" );
	}

	KinectRecording(const Omiscid::SimpleString& Prefix)
	{
		Init( Prefix );
	}

	virtual ~KinectRecording()
	{
		StopRecording(0.0);
		
		if ( BufferData != nullptr )
		{
			delete BufferData;
		}
	}
		
	void Init(const Omiscid::SimpleString& Prefix, bool doRawRecord = false )
	{
		Omiscid::SmartLocker InternalProtection_SL(InternalProtection);

		// For standard recording
		FilePrefix		= Prefix;
		TimestampedFile = (FILE*)nullptr;
		InputNumber		= 0;
		StartTime		= 0.0;
		FrameRate		= 0.0;

		LastFrameTime = 0;
		BufferData = nullptr;

		// Raw recording
		RawRecording	= doRawRecord;
		RawFile = (FILE*)nullptr;
		BufferSize = 0;

		InitDescription = true;
		DescriptionFile = (FILE*)nullptr;

		// Set if I am active
		IsActive = FilePrefix != "";
	}

	virtual void DeclareSerializeMapping()
	{
		// TODO : change it for skeleton, better description
		AddToSerialization("EventFrameRate", FrameRate);
	}

	void AllocateBuffer(size_t SizeOfBuffer)
	{
		if ( SizeOfBuffer != 0 )
		{
			BufferData = (void*)new char[SizeOfBuffer];
		}
	}

	virtual bool StartRecording( const char* SessionFolder )
	{
		Omiscid::SmartLocker InternalProtection_SL(InternalProtection);

		Omiscid::SimpleString str;
		Omiscid::SimpleString str2;

		InputNumber = 0;
		InitDescription = true;

		StartTime = 0;
		LastFrameTime = 0;

		// Create session folder
		if ( CreateDirectory(SessionFolder, NULL) == FALSE && GetLastError() != ERROR_ALREADY_EXISTS)
		{
			fprintf( stderr, "Could not create '%s' folder\n", SessionFolder );
			return false;
		}

		// Prefix
		str = SessionFolder;
		str += "/" + FilePrefix;

		// Create folder if not already done
		if ( CreateDirectory(str.GetStr(), NULL) == FALSE && GetLastError() != ERROR_ALREADY_EXISTS)
		{
			fprintf( stderr, "Could not create '%s' folder\n", str.GetStr() );
			return false;
		}

		// Set root name
		str += "/" + FilePrefix;

		// Create standard timestamp file
		// str equals for instance "depth/depth."
		str2 = str;
		str2 += ".timestamp";

		TimestampedFile = fopen(str2.GetStr(), "wb");
		if ( TimestampedFile == nullptr )
		{
			fprintf( stderr, "Could not create '%s' timestamp file\n", str2.GetStr() );
			return false;
		}

	// Create Raw and Description file if mandatory
		if ( RawRecording == true )
		{
			str2 = str + ".raw";
			RawFile = fopen(str2.GetStr(), "wb");
			if ( RawFile == nullptr )
			{
				fprintf( stderr, "Could not create '%s' raw file\n", str2.GetStr() );
				CloseAndSetNull(TimestampedFile);
				return false;
			}

			str2 = str + ".desc";
			DescriptionFile = fopen(str2.GetStr(), "wb");
			if ( DescriptionFile == nullptr )
			{
				fprintf( stderr, "Could not create '%s' desc file\n", str2.GetStr() );
				CloseAndSetNull(TimestampedFile);
				CloseAndSetNull(RawFile);
				return false;
			}
		}

		return true;
	}

	virtual void StopRecording(double CurrentTime)
	{
		Omiscid::SmartLocker InternalProtection_SL(InternalProtection);

		if ( RawRecording == true )
		{
			CloseAndSetNull(DescriptionFile);
			CloseAndSetNull(RawFile);
		}
		CloseAndSetNull(TimestampedFile);
	}

	void SaveDataAndIncreaseInputNumber( const struct timeb& lTimestamp, char * SuppInfo /* = nullptr */ )
	{
		Omiscid::SmartLocker InternalProtection_SL(InternalProtection);

		if ( RawFile != (FILE*)nullptr )
		{
			if ( BufferSize != 0 )
			{
				while (fwrite(BufferData, BufferSize, 1, RawFile) != 1) {}
			}
			SaveTimestamp(TimestampedFile, lTimestamp, InputNumber, LastFrameTime, SuppInfo);
		}

		// We had an input
		InputNumber++;
	}


	// Static utility function
	static void SaveTimestamp(FILE* f, const struct timeb& lTimestamp, unsigned int numFrame, TIMESPAN FrameTime, char * SuppInfo /* = nullptr */ )
	{
		if ( f == (FILE*)nullptr)
		{
			return;
		}

		if ( SuppInfo != (char *)nullptr )
		{
			fprintf(f, "%d.%03d %u, %s, %I64d\n", (int)lTimestamp.time, (int)lTimestamp.millitm, numFrame, SuppInfo, FrameTime);
		}
		else
		{
			fprintf(f, "%d.%03d %u, %I64d\n", (int)lTimestamp.time, (int)lTimestamp.millitm, numFrame, FrameTime);
		}
	}
};

#endif // __KINECT_RECORDING_H__
