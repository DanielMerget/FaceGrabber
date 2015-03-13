#pragma once
#include <string>
#include <vector>
#include <memory>
enum RecordingFileFormat{
	PLY,
	PLY_BINARY,
	PCD,
	PCD_BINARY,
	RECORD_FILE_FORMAT_COUNT
};

enum RecordCloudType{
	HDFace,
	FaceRaw,
	FullDepthRaw,
	RECORD_CLOUD_TYPE_COUNT
};

class IRecordingConfiguration
{
public:
	virtual RecordingFileFormat getRecordFileFormat() = 0;
	virtual std::string getFullRecordingPathString() = 0;
	virtual std::string getFileNameString() = 0;
	virtual bool isRecordingDurationUnLimited() = 0;
	virtual int getMaxNumberOfFrames() = 0;
};

typedef std::shared_ptr<IRecordingConfiguration> IRecordingConfigurationPtr;
typedef std::vector<IRecordingConfigurationPtr> SharedIRecordingConfigurations;
