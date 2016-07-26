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
#include "IImageRecordingConfiguration.h"
/**
 * \class	RecordingConfiguration
 *
 * \brief	The Recording Configuration stores all information neccessary to the KinectCloudFileWriter to save
 * 			ply and pcd files to the directory. It allows to specify the directory for the files and automatically
 * 			searches for the files. It constructs the directory path to save the point clouds in the following way:
 *			//outputfolder/timestamp/cloudtype/filename.fileformat.
 */

class StringFileRecordingConfiguration {
#define UNLIMITED_FRAMES -1;
#define FRAMES_NOT_SET 0;

public:
	StringFileRecordingConfiguration();

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordCloudType cloudType, RecordingFileFormat format);
	 *
	 * \brief	Constructor.
	 *
	 * \param	cloudType	Type of the cloud to record..
	 * \param	format   	Describes the format to use.
	 */

	StringFileRecordingConfiguration(StringFileRecordType RecordType, StringFileRecordingFileFormat format);

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordingConfiguration& recordingConfiguration);
	 *
	 * \brief	Copy constructor.
	 *
	 * \param [in]	recordingConfiguration	The recording configuration.
	 */

	StringFileRecordingConfiguration(StringFileRecordingConfiguration& recordingConfiguration);

	
	static CString getFileFormatAsString(StringFileRecordingFileFormat fileFormat);

	

	/**
	 * \fn	bool RecordingConfiguration::isRecordConfigurationValid();
	 *
	 * \brief	Query if record configuration is valid: Valid only if we have a 
	 * 			valid path, name and number of frames to captures are set.
	 *
	 * \return	true if record configuration valid, false if not.
	 */
	bool isRecordConfigurationValid();

	/**
	 * \fn	CString RecordingConfiguration::getFilePathCString();
	 *
	 * \brief	Gets file path as c-string. (for WinApi)
	 *
	 * \return	The file path c string.
	 */
	CString getFilePathCString();

	/**
	 * \fn	std::string RecordingConfiguration::getFilePath();
	 *
	 * \brief	Gets file path as std::string (for pcl)
	 *
	 * \return	The file path.
	 */
	std::string getFilePath();

	/**
	 * \fn	bool RecordingConfiguration::isEnabled();
	 *
	 * \brief	Query if Recording is enabled.
	 *
	 * \return	true if enabled, false if not.
	 */
	bool isEnabled();

	/**
	 * \fn	RecordCloudType RecordingConfiguration::getRecordCloudType();
	 *
	 * \brief	Gets cloud type to record
	 *
	 * \return	The record cloud type.
	 */
	StringFileRecordType getRecordType();

	/**
	 * \fn	RecordingFileFormat RecordingConfiguration::getRecordFileFormat();
	 *
	 * \brief	Gets fileformat to record
	 *
	 * \return	The record file format.
	 */
	StringFileRecordingFileFormat getRecordFileFormat();



	/**
	 * \fn	CString RecordingConfiguration::getFileNameCString();
	 *
	 * \brief	Gets file name  as c-string. (for WinApi)
	 * 			e.g. HD_Face_Cloud_*.ply
	 *
	 * \return	The file name c string.
	 */
	CString getFileNameCString();

	/**
	 * \fn	bool RecordingConfiguration::isRecordingDurationUnLimited();
	 *
	 * \brief	Query if recording duration is unlimited.
	 *
	 * \return	true if recording duration un limited, false if number of frames to record
	 * 			was set.
	 */
	bool isRecordingDurationUnLimited();

	/**
	 * \fn	void RecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames);
	 *
	 * \brief	Sets maximum number of frames to record.
	 *
	 * \param	newMaxNumberOfFrames	The new maximum number of frames.
	 */
	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	/**
	 * \fn	int RecordingConfiguration::getMaxNumberOfFrames();
	 *
	 * \brief	Gets the maximum number of frames to record. 
	 * 			IF: UNLIMITED_FRAMES: recording is not limited
	 *
	 * \return	The maximum number of frames.
	 */
	int getMaxNumberOfFrames();

	/**
	 * \fn	std::string RecordingConfiguration::getFileNameString();
	 *
	 * \brief	Gets file name string.
	 *
	 * \return	The file name string.
	 */
	std::string getFileNameString();

	/**
	 * \fn	void RecordingConfiguration::setFileName(CString fileName);
	 *
	 * \brief	Sets file name.
	 *
	 * \param	fileName	Filename of the file.
	 */
	void setFileName(CString fileName);

	/**
	 * \fn	void RecordingConfiguration::setFilePath(CString filePath);
	 *
	 * \brief	Sets file path.
	 *
	 * \param	filePath	Full pathname of the file.
	 */
	void setFilePath(CString filePath);

	/**
	 * \fn	void RecordingConfiguration::setEnabled(bool enabled);
	 *
	 * \brief	Sets an enabled.
	 *
	 * \param	enabled	true to enable, false to disable.
	 */
	void setEnabled(bool enabled);

	/**
	 * \fn	void RecordingConfiguration::setFileFormat(RecordingFileFormat fileFormat);
	 *
	 * \brief	Sets file format.
	 *
	 * \param	fileFormat	The file format in which to record.
	 */
	void setFileFormat(StringFileRecordingFileFormat fileFormat);



	/**
	 * \fn	void RecordingConfiguration::setTimeStampFolderName(CString folderName);
	 *
	 * \brief	Sets time stamp folder name.
	 *
	 * \param	folderName	Name of the time stamp folder
	 */
	void setTimeStampFolderName(CString folderName);

	/**
	 * \fn	CString RecordingConfiguration::getTimeStampFolderName();
	 *
	 * \brief	Gets time stamp folder name.
	 *
	 * \return	The time stamp folder name.
	 */
	CString getTimeStampFolderName();

	/**
	 * \fn	CString RecordingConfiguration::getFullRecordingPath();
	 *
	 * \brief	Gets full recording path.
	 *
	 * \return	The full recording path.
	 */
	CString getFullRecordingPath();

	/**
	 * \fn	std::string RecordingConfiguration::getFullRecordingPathString();
	 *
	 * \brief	Gets full recording path as std::string.
	 *
	 * \return	The full recording path string.
	 */
	std::string getFullRecordingPathString();

	/**
	 * \fn	static CString RecordingConfiguration::getSubFolderNameForCloudType(RecordCloudType cloudType);
	 *
	 * \brief	Gets sub folder name for cloud type.
	 *
	 * \param	cloudType	Type of the cloud.
	 *
	 * \return	The sub folder name for cloud type.
	 */
	static CString getSubFolderNameForCloudType(StringFileRecordType cloudType);

	/**
	 * \fn	static CString RecordingConfiguration::getFullRecordingPathForRecordType(RecordCloudType cloudType, CString outputFolder, CString timeStampFolderName);
	 *
	 * \brief	Gets full recording path for the given parameters.
	 *
	 * \param	cloudType		   	Type of the cloud.
	 * \param	outputFolder	   	Path of the output folder.
	 * \param	timeStampFolderName	the time stamp.
	 *
	 * \return	The full recording path for the cloud recording parameters.
	 */
	static CString getFullRecordingPathForRecordType(StringFileRecordType cloudType, CString outputFolder, CString timeStampFolderName,KinectVersionType kinectVersion);

	/**
	 * \fn	int RecordingConfiguration::getThreadCountToStart();
	 *
	 * \brief	Gets amount of threads to start.
	 *
	 * \return	amount of threads to start
	 */
	int getThreadCountToStart();

	/**
	 * \fn	void RecordingConfiguration::setThreadCountToStart(int threadsCount);
	 *
	 * \brief	Sets amount of threads to start.
	 *
	 * \param	threadsCount	Number of threads.
	 */
	void setThreadCountToStart(int threadsCount);

	boost::signal<void(StringFileRecordType, bool)> recordConfigurationStatusChanged;
	boost::signal<void(StringFileRecordType)>	recordPathOrFileNameChanged;

	void setKinectVersion(KinectVersionType	kinectVersion);
private:

	/**
	 * \fn	CString RecordingConfiguration::getDefaultFileName();
	 *
	 * \brief	Gets default file name of the set cloudtype
	 *
	 * \return	The default file name.
	 */
	CString getDefaultFileName();

	/**
	 * \fn	void RecordingConfiguration::setDefaultFileName();
	 *
	 * \brief	Sets default file name for the set cloudtype.
	 */
	void setDefaultFileName();

	/** \brief	outputfolder/timestamp/cloudtype/filename.fileformat. */
	CString					m_outputFolder;

	/** \brief	Pathname of the time stamp folder. */
	CString					m_timeStampFolderName;

	/** \brief	name of the fileprefix. */
	CString					m_fileName;
	
	/** \brief	Type of the cloud to record. */
	StringFileRecordType			m_recordType;

	/** \brief	The file format. */
	StringFileRecordingFileFormat		m_fileFormat;

	

	/** \brief	The maximum number of frames to record. can be UNLIMITED_FRAMES (-1). */
	int						m_maxNumberOfFrames;

	/** \brief	true to enable recording, false to disable. */
	bool					m_enabled;

	/** \brief	Number of threads to start for recording. */
	int						m_threadsCount;
	bool					m_points_enabled;
	bool					m_angles_enabled;

	KinectVersionType		m_kinectVersion;
};
typedef std::shared_ptr<StringFileRecordingConfiguration> StringFileRecordingConfigurationPtr;
typedef std::vector<StringFileRecordingConfigurationPtr> SharedStringFileRecordingConfiguration;
