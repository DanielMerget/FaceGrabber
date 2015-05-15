#include "stdafx.h"
#include "KinectImageWriterThread.h"
#undef max
#undef min


KinectImageWriterThread::KinectImageWriterThread()
{
}


KinectImageWriterThread::~KinectImageWriterThread()
{
}


void KinectImageWriterThread::writeImagesToFile(IRecordingConfigurationPtr recordingConfiguration)
{
	//collect important writing information
	auto recordingFileFormat = recordingConfiguration->getRecordFileFormat();
	auto filePath = recordingConfiguration->getFullRecordingPathString();
	auto fileName = recordingConfiguration->getFileNameString();

	//select the correct Extension
	std::string fileFormatExtension;
	switch (recordingFileFormat)
	{
	case PPM_BINARY:
		fileFormatExtension = ".ppm";
		break;
	case PGM_BINARY:
		fileFormatExtension = ".pgm";
		break;
	case PNG:
		fileFormatExtension = ".png";
		break;
	case RECORD_FILE_FORMAT_COUNT:
		break;
	default:
		break;
	}

	while (true){
		//pull data
		ImageMeasurement measurement;
		measurement.image = nullptr;
		measurement.index = 0;
		bool isDone = m_source->pullData(measurement);
		if (!measurement.image){
			OutputDebugString(L"writer finished cloud null");
			return;
		}

		wchar_t buffer[256];
		wsprintfW(buffer, L"%d", recordingFileFormat);
		OutputDebugString(buffer);

		//write data to constructed path
		std::stringstream outputFileWithPath;
		outputFileWithPath << filePath << "\\" << fileName << "_" << measurement.index << fileFormatExtension;
		if (recordingFileFormat == PPM_BINARY)
		{
			cv::Mat converted;
			cv::cvtColor(*measurement.image, converted, CV_BGRA2BGR);
			cv::imwrite(outputFileWithPath.str(), converted);
		}
		else
		{
			cv::imwrite(outputFileWithPath.str(), *measurement.image);
		}

		if (isDone){
			OutputDebugString(L"writer finished isDone");
			return;
		}

	}
}


void KinectImageWriterThread::setKinectImageOutputWriter(ImageMeasurementSource* source)
{
	m_source = source;
}
