#include "stdafx.h"
#include "KinectFileWriterThread.h"
#undef max
#undef min
#include <pcl/io/file_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


template KinectFileWriterThread < pcl::PointXYZRGB >;
template KinectFileWriterThread < pcl::PointXYZ >;



template < class PointCloudType >
KinectFileWriterThread< PointCloudType >::KinectFileWriterThread()
{
}

template < class PointCloudType >
KinectFileWriterThread< PointCloudType >::~KinectFileWriterThread()
{
}

template < typename PointCloudType >
void KinectFileWriterThread<PointCloudType>::writeCloudsToFile(IRecordingConfigurationPtr recordingConfiguration)
{
	//collect important writing information
	auto recordingFileFormat =	recordingConfiguration->getRecordFileFormat();
	auto filePath =				recordingConfiguration->getFullRecordingPathString();
	auto fileName =				recordingConfiguration->getFileNameString();

	//select the correct filewriter
	std::shared_ptr<pcl::FileWriter> writer;
	std::string fileFormatExtension;
	bool isBinary = false;
	switch (recordingFileFormat)
	{
	case PLY:
		writer = std::shared_ptr<pcl::PLYWriter> (new pcl::PLYWriter());
		fileFormatExtension = ".ply";
		isBinary = false;
		break;
	case PLY_BINARY:
		writer = std::shared_ptr<pcl::PLYWriter>(new pcl::PLYWriter());
		fileFormatExtension = ".ply";
		isBinary = true;
		break;
	case PCD:
		writer = std::shared_ptr<pcl::PCDWriter>(new pcl::PCDWriter());
		fileFormatExtension = ".pcd";
		isBinary = false;
		break;
	case PCD_BINARY:
		writer = std::shared_ptr<pcl::PCDWriter>(new pcl::PCDWriter());
		fileFormatExtension = ".pcd";
		isBinary = true;
		break;
	case RECORD_FILE_FORMAT_COUNT:
		break;
	default:
		break;
	}
	
	while(true){
		//pull data
		PointCloudMeasurement< PointCloudType > measurement;
		measurement.cloud = nullptr;
		measurement.index = 0;
		bool isDone = m_source->pullData(measurement);
		if (!measurement.cloud){
			OutputDebugString(L"writer finished cloud null");
			return;
		}
		//write data to constructed path
		std::stringstream outputFileWithPath;
		outputFileWithPath << filePath << "\\" << fileName << measurement.index << fileFormatExtension;

		writer->write(outputFileWithPath.str(), *measurement.cloud, isBinary);

		if (isDone){
			OutputDebugString(L"writer finished isDone");
			return;
		}

	} 
}

template < typename PointCloudType >
void KinectFileWriterThread<PointCloudType>::setKinectCloudOutputWriter(CloudMeasurementSource<PointCloudType>* source)
{
	m_source = source;
}
