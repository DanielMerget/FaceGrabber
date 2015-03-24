#pragma once
#include "stdafx.h"

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

/**
 * \class	IRecordingConfiguration
 *
 * \brief	The recording configuration interface provides all information required by the
 * 			KinectCloudOutputWriter to record point clouds to the hard disc.
 */

class IRecordingConfiguration
{
public:

	/**
	 * \fn	virtual RecordingFileFormat IRecordingConfiguration::getRecordFileFormat() = 0;
	 *
	 * \brief	Gets record file format.
	 *
	 * \return	The record file format.
	 */
	virtual RecordingFileFormat getRecordFileFormat() = 0;

	/**
	 * \fn	virtual std::string IRecordingConfiguration::getFullRecordingPathString() = 0;
	 *
	 * \brief	Gets full recording path.
	 *
	 * \return	The full recording path.
	 */
	virtual std::string getFullRecordingPathString() = 0;

	/**
	 * \fn	virtual std::string IRecordingConfiguration::getFileNameString() = 0;
	 *
	 * \brief	Gets file name.
	 *
	 * \return	The file name.
	 */
	virtual std::string getFileNameString() = 0;

	/**
	 * \fn	virtual bool IRecordingConfiguration::isRecordingDurationUnLimited() = 0;
	 *
	 * \brief	Query if this object is recording duration unlimited. If it is limited
	 * 			getMaxNumberOfFrames provides the amount of frames to record. 
	 *
	 * \return	true if recording duration un limited, false if not.
	 */
	virtual bool isRecordingDurationUnLimited() = 0;

	/**
	 * \fn	virtual int IRecordingConfiguration::getMaxNumberOfFrames() = 0;
	 *
	 * \brief	Gets maximum number of frames to record.
	 *
	 * \return	The maximum number of frames to record.
	 */
	virtual int getMaxNumberOfFrames() = 0;

	/**
	 * \fn	virtual int IRecordingConfiguration::getThreadCountToStart() = 0;
	 *
	 * \brief	Gets thread amount of threads to start for recording.
	 *
	 * \return	The number of threads to start.
	 */
	virtual int getThreadCountToStart() = 0;
};

typedef std::shared_ptr<IRecordingConfiguration> IRecordingConfigurationPtr;
typedef std::vector<IRecordingConfigurationPtr> SharedIRecordingConfigurations;
