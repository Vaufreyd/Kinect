
#ifndef __RAW_KINECT_RECORDING_H__
#define __RAW_KINECT_RECORDING_H__


#include "KinectRecording.h"

class RawKinectRecording : public KinectRecording
{
public:
	Omiscid::SimpleString FrameType;

	// Frame and acquisition process description
	int Width;
	int Height;
	float HorizontalFieldOfView;
	float VerticalFieldOfView;
	float DiagonalFieldOfView;
	int FrameLengthInPixels;
	int BytesPerPixel;

	virtual void DeclareSerializeMapping()
	{
		// TODO : change it for skeleton, audio and face for better description

		// First local description
		AddToSerialization("FrameType", FrameType);
		AddToSerialization("Width", Width);
		AddToSerialization("Height", Height);
		AddToSerialization("HorizontalFieldOfView", HorizontalFieldOfView);
		AddToSerialization("VerticalFieldOfView", VerticalFieldOfView);
		AddToSerialization("DiagonalFieldOfView", DiagonalFieldOfView);
		AddToSerialization("FrameLengthInPixels", FrameLengthInPixels);
		AddToSerialization("BytesPerPixel", BytesPerPixel);

		// next standard description for KinectRecordings, i.e. FrameRate
		KinectRecording::DeclareSerializeMapping();
	}

	RawKinectRecording()
	{
		FrameType = Omiscid::SimpleString::EmptyString;
		Width = 0;
		Height = 0;
		HorizontalFieldOfView = 0.0;
		VerticalFieldOfView =0.0;
		DiagonalFieldOfView = 0.0;
		FrameLengthInPixels = 0;
		BytesPerPixel = 0;
		
		BufferSize = 0;
		FrameRate = 0.0f;
	}

	void Init(const Omiscid::SimpleString& Prefix, const Omiscid::SimpleString& _FrameType)
	{
		// Call for raw recording
		KinectRecording::Init( Prefix, true );

		FrameType = _FrameType;
		Width = 0;
		Height = 0;
		HorizontalFieldOfView = 0.0;
		VerticalFieldOfView =0.0;
		DiagonalFieldOfView = 0.0;
		FrameLengthInPixels = 0;
		BytesPerPixel = 0;
		
		BufferSize = 0;
		FrameRate = 0.0f;
	}

	virtual ~RawKinectRecording()
	{
	}

	// Nothing more than in KinectRecording::StartRecording
	// virtual bool StartRecording(const char* SessionFolder, double CurrentTime )

	// Compute frame rate, save description and call KinectRecording::StopRecording
	virtual void StopRecording( double CurrentTime )
	{
		if ( DescriptionFile != nullptr )
		{
			// Compute actual frame rate for this recording
			FrameRate = (float)InputNumber/(float)(CurrentTime-StartTime);
			
			// Create description, save it and close description file in JSON
			Omiscid::SimpleString Description = Omiscid::StructuredMessage(Serialize());
			fprintf(DescriptionFile, "%s\n", Description.GetStr());
		}
		
		KinectRecording::StopRecording( CurrentTime );
	}
};



#endif // __RAW_KINECT_RECORDING_H__
