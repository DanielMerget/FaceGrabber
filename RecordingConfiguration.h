#pragma once
#include "stdafx.h"

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <atlstr.h>
#include <regex>

#include "IRecordingConfiguration.h"

/**
 * \class	RecordingConfiguration
 *
 * \brief	The Recording Configuration stores all information neccessary to the KinectCloudFileWriter to save
 * 			ply and pcd files to the directory. It allows to specify the directory for the files and automatically
 * 			searches for the files. It constructs the directory path to save the point clouds in the following way:
 *			//outputfolder/timestamp/cloudtype/filename.fileformat.
 */

class RecordingConfiguration : public IRecordingConfiguration{
#define UNLIMITED_FRAMES -1;
#define FRAMES_NOT_SET 0;

public:
	RecordingConfiguration();

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format);
	 *
	 * \brief	Constructor.
	 *
	 * \param	cloudType	Type of the cloud to record..
	 * \param	format   	Describes the format to use.
	 */

	RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format);

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordingConfiguration& recordingConfiguration);
	 *
	 * \brief	Copy constructor.
	 *
	 * \param [in]	recordingConfiguration	The recording configuration.
	 */

	RecordingConfiguration(RecordingConfiguration& recordingConfiguration);

	
	static CString getFileFormatAsString(RecordingFileFormat fileFormat);


	bool isRecordConfigurationValid();

	CString getFilePathCString();

	std::string getFilePath();

	bool isEnabled();

	RecordCloudType getRecordCloudType();

	RecordingFileFormat getRecordFileFormat();

	CString getFileNameCString();

	bool isRecordingDurationUnLimited();

	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	int getMaxNumberOfFrames();

	std::string getFileNameString();

	LPTSTR getFileName();

	void setFileName(LPTSTR fileName);
	void setFilePath(std::string filePath);

	void setFilePath(LPTSTR filePath);

	void setEnabled(bool enabled);

	void setFileFormat(RecordingFileFormat fileFormat);

	void setTimeStampFolderName(CString folderName);

	CString getTimeStampFolderName();

	CString getFullRecordingPath();

	std::string getFullRecordingPathString();
	
	static CString getSubFolderNameForCloudType(RecordCloudType cloudType);

	static CString getFullRecordingPathForCloudType(RecordCloudType cloudType, CString outputFolder, CString timeStampFolderName);

	int getThreadCountToStart();

	void setThreadCountToStart(int threadsCount);

	boost::signal<void(RecordCloudType, bool)> recordConfigurationStatusChanged;
	boost::signal<void(RecordCloudType)>	recordPathOrFileNameChanged;
private:
	CString getDefauldFileName();

	void setDefaultFileName();

	std::vector<std::string> m_foundCloudFiles;

	//outputfolder/timestamp/cloudtype/filename.fileformat

	CString					m_outputFolder;
	CString					m_timeStampFolderName;
	CString					m_fileName;
	RecordCloudType			m_cloudType;
	RecordingFileFormat		m_fileFormat;
	int						m_maxNumberOfFrames;
	bool					m_enabled;
	int						m_threadsCount;
};
typedef std::shared_ptr<RecordingConfiguration> RecordingConfigurationPtr;
typedef std::vector<RecordingConfigurationPtr> SharedRecordingConfiguration;
