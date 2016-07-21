#pragma once
#include "stdafx.h"

#include <string>
#include <vector>
#include <memory>

enum ImageRecordingFileFormat{
	PNG,
	PNM_BINARY,
	IMAGE_RECORD_FILE_FORMAT_COUNT
};

enum StringFileRecordingFileFormat{
	TXT,
	STRING_FILE_RECORD_FILE_FORMAT_COUNT
};

enum ImageRecordType{
	KinectColorRaw,
	KinectDepthRaw,
	KinectAlignedDepthRaw,
	KinectInfrared,
	KinectAlignedInfrared,
	IMAGE_RECORD_TYPE_COUNT
};

enum StringFileRecordType{
	FiveKeyPoints,
	STRING_FILE_RECORD_TYPE_COUNT
};

/**
* \class	IImageRecordingConfiguration
*
* \brief	The recording configuration interface provides all information required by the
* 			KinectRawOutputWriter to record point clouds to the hard disc.
*/

class IImageRecordingConfiguration
{
public:

	/**
	* \fn	virtual ImageRecordingFileFormat IRecordingConfiguration::getRecordFileFormat() = 0;
	*
	* \brief	Gets record file format.
	*
	* \return	The record file format.
	*/
	virtual ImageRecordingFileFormat getRecordFileFormat() = 0;

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

typedef std::shared_ptr<IImageRecordingConfiguration> IImageRecordingConfigurationPtr;
typedef std::vector<IImageRecordingConfigurationPtr> SharedIImageRecordingConfigurations;
