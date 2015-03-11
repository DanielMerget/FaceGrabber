#pragma once
#include "SingleProducerBuffer.h"
#include <memory>
#include "RecordingConfiguration.h"
#include "CloudMeasurementSource.h"


template < class PointCloudType >
class KinectFileWriterThread
{
public:
	KinectFileWriterThread();
	~KinectFileWriterThread();

	//void writeCloudToFile(int index, RecordingFileFormat recordingFileFormat, std::string filePath, std::string fileName);
	void writeCloudToFile(int index, RecordingConfigurationPtr recordingConfiguration);
	//void setBuffer(std::shared_ptr<SingleProducerBuffer<std::shared_ptr<PointCloudMeasurement<PointCloudType>>>> m_buffer);

	void setKinectCloudOutputWriter(CloudMeasurementSource<PointCloudType>* source);

	CloudMeasurementSource<PointCloudType>* m_source;
};

