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


void KinectImageWriterThread::writeImagesToFile(IImageRecordingConfigurationPtr recordingConfiguration)
{
	//collect important writing information
	auto recordingFileFormat = recordingConfiguration->getRecordFileFormat();
	auto filePath = recordingConfiguration->getFullRecordingPathString();
	auto fileName = recordingConfiguration->getFileNameString();

	//select the correct Extension
	std::string fileFormatExtension;
	switch (recordingFileFormat)
	{
	case PNM_BINARY:
		fileFormatExtension = ".pnm";
		break;
	case PNG:
		fileFormatExtension = ".png";
		break;
	case IMAGE_RECORD_FILE_FORMAT_COUNT:
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

		if (recordingFileFormat == PNM_BINARY){
			if (measurement.image->type() == CV_8UC4){
				//drop alpha channel
				cv::Mat imageCopy;
				imageCopy = measurement.image->clone();
				cv::cvtColor(imageCopy, *measurement.image, CV_BGRA2BGR);
				fileFormatExtension = ".ppm";
			}
			else if(measurement.image->type() == CV_16U){
				fileFormatExtension = ".pgm";
			}
		}

		//write data to constructed path
		std::stringstream outputFileWithPath;
		outputFileWithPath << filePath << "\\" << fileName << "_" << std::setfill('0') << std::setw(6) << measurement.index << fileFormatExtension;

		cv::imwrite(outputFileWithPath.str(), *measurement.image);

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
