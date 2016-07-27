#pragma once
#include "stdafx.h"

#include <string>
#include "stdafx.h"
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <atlstr.h>
#include <regex>

#include "IImageRecordingConfiguration.h"

/**
* \class	ImageRecordingConfiguration
*
* \brief	The Recording Configuration stores all information neccessary to the KinectRawFileWriter to save
* 			image files to the directory. It allows to specify the directory for the files and automatically
* 			searches for the files. It constructs the directory path to save the images in the following way:
*			//outputfolder/timestamp/imagetype/filename.fileformat.
*/

class ImageRecordingConfiguration : public IImageRecordingConfiguration{
#define UNLIMITED_FRAMES -1;
#define FRAMES_NOT_SET 0;

public:
	ImageRecordingConfiguration();

	/**
	* \fn	ImageRecordingConfiguration::ImageRecordingConfiguration(ImageRecordType imageType, ImageRecordingFileFormat format);
	*
	* \brief	Constructor.
	*
	* \param	imageType	Type of the image to record.
	* \param	format   	Describes the format to use.
	*/

	ImageRecordingConfiguration(ImageRecordType imageType, ImageRecordingFileFormat format);

	//ImageRecordingConfiguration(StringFileRecordType imageType, StringFileRecordingFileFormat format);

	/**
	* \fn	ImageRecordingConfiguration::ImageRecordingConfiguration(ImageRecordingConfiguration& ImageRecordingConfiguration);
	*
	* \brief	Copy constructor.
	*
	* \param [in]	ImageRecordingConfiguration	The recording configuration.
	*/

	ImageRecordingConfiguration(ImageRecordingConfiguration& ImageRecordingConfiguration);


	static CString getFileFormatAsString(ImageRecordingFileFormat fileFormat);

	/**
	* \fn	bool ImageRecordingConfiguration::isRecordConfigurationValid();
	*
	* \brief	Query if record configuration is valid: Valid only if we have a
	* 			valid path, name and number of frames to captures are set.
	*
	* \return	true if record configuration valid, false if not.
	*/
	bool isRecordConfigurationValid();

	/**
	* \fn	CString ImageRecordingConfiguration::getFilePathCString();
	*
	* \brief	Gets file path as c-string. (for WinApi)
	*
	* \return	The file path c string.
	*/
	CString getFilePathCString();

	/**
	* \fn	std::string ImageRecordingConfiguration::getFilePath();
	*
	* \brief	Gets file path as std::string (for pcl)
	*
	* \return	The file path.
	*/
	std::string getFilePath();

	/**
	* \fn	bool ImageRecordingConfiguration::isEnabled();
	*
	* \brief	Query if Recording is enabled.
	*
	* \return	true if enabled, false if not.
	*/
	bool isEnabled();

	/**
	* \fn	ImageRecordType ImageRecordingConfiguration::getImageRecordType();
	*
	* \brief	Gets Image type to record
	*
	* \return	The record Image type.
	*/
	ImageRecordType getImageRecordType();

	/**
	* \fn	ImageRecordingFileFormat ImageRecordingConfiguration::getRecordFileFormat();
	*
	* \brief	Gets fileformat to record
	*
	* \return	The record file format.
	*/
	ImageRecordingFileFormat getRecordFileFormat();



	/**
	* \fn	CString ImageRecordingConfiguration::getFileNameCString();
	*
	* \brief	Gets file name  as c-string. (for WinApi)
	*
	* \return	The file name c string.
	*/
	CString getFileNameCString();

	/**
	* \fn	bool ImageRecordingConfiguration::isRecordingDurationUnLimited();
	*
	* \brief	Query if recording duration is unlimited.
	*
	* \return	true if recording duration un limited, false if number of frames to record
	* 			was set.
	*/
	bool isRecordingDurationUnLimited();

	/**
	* \fn	void ImageRecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames);
	*
	* \brief	Sets maximum number of frames to record.
	*
	* \param	newMaxNumberOfFrames	The new maximum number of frames.
	*/
	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	/**
	* \fn	int ImageRecordingConfiguration::getMaxNumberOfFrames();
	*
	* \brief	Gets the maximum number of frames to record.
	* 			IF: UNLIMITED_FRAMES: recording is not limited
	*
	* \return	The maximum number of frames.
	*/
	int getMaxNumberOfFrames();

	/**
	* \fn	std::string ImageRecordingConfiguration::getFileNameString();
	*
	* \brief	Gets file name string.
	*
	* \return	The file name string.
	*/
	std::string getFileNameString();

	/**
	* \fn	void ImageRecordingConfiguration::setFileName(CString fileName);
	*
	* \brief	Sets file name.
	*
	* \param	fileName	Filename of the file.
	*/
	void setFileName(CString fileName);

	/**
	* \fn	void ImageRecordingConfiguration::setFilePath(CString filePath);
	*
	* \brief	Sets file path.
	*
	* \param	filePath	Full pathname of the file.
	*/
	void setFilePath(CString filePath);

	/**
	* \fn	void ImageRecordingConfiguration::setEnabled(bool enabled);
	*
	* \brief	Sets an enabled.
	*
	* \param	enabled	true to enable, false to disable.
	*/
	void setEnabled(bool enabled);

	/**
	* \fn	void ImageRecordingConfiguration::setFileFormat(ImageRecordingFileFormat fileFormat);
	*
	* \brief	Sets file format.
	*
	* \param	fileFormat	The file format in which to record.
	*/
	void setFileFormat(ImageRecordingFileFormat fileFormat);

	/**
	* \fn	void ImageRecordingConfiguration::setTimeStampFolderName(CString folderName);
	*
	* \brief	Sets time stamp folder name.
	*
	* \param	folderName	Name of the time stamp folder
	*/
	void setTimeStampFolderName(CString folderName);

	/**
	* \fn	CString ImageRecordingConfiguration::getTimeStampFolderName();
	*
	* \brief	Gets time stamp folder name.
	*
	* \return	The time stamp folder name.
	*/
	CString getTimeStampFolderName();

	/**
	* \fn	CString ImageRecordingConfiguration::getFullRecordingPath();
	*
	* \brief	Gets full recording path.
	*
	* \return	The full recording path.
	*/
	CString getFullRecordingPath();

	/**
	* \fn	std::string ImageRecordingConfiguration::getFullRecordingPathString();
	*
	* \brief	Gets full recording path as std::string.
	*
	* \return	The full recording path string.
	*/
	std::string getFullRecordingPathString();

	/**
	* \fn	static CString ImageRecordingConfiguration::getSubFolderNameForImageType(ImageRecordType imageType);
	*
	* \brief	Gets sub folder name for image type.
	*
	* \param	imageType	Type of the image.
	*
	* \return	The sub folder name for image type.
	*/
	static CString getSubFolderNameForImageType(ImageRecordType imageType);

	/**
	* \fn	static CString ImageRecordingConfiguration::getFullRecordingPathForImageType(ImageRecordType imageType, CString outputFolder, CString timeStampFolderName);
	*
	* \brief	Gets full recording path for the given parameters.
	*
	* \param	imageType		   	Type of the image.
	* \param	outputFolder	   	Path of the output folder.
	* \param	timeStampFolderName	the time stamp.
	*
	* \return	The full recording path for the image recording parameters.
	*/
	static CString getFullRecordingPathForImageType(ImageRecordType imageType, CString outputFolder, CString timeStampFolderName, KinectVersionType kinectVersion); //static 

	/**
	* \fn	int ImageRecordingConfiguration::getThreadCountToStart();
	*
	* \brief	Gets amount of threads to start.
	*
	* \return	amount of threads to start
	*/
	int getThreadCountToStart();

	/**
	* \fn	void ImageRecordingConfiguration::setThreadCountToStart(int threadsCount);
	*
	* \brief	Sets amount of threads to start.
	*
	* \param	threadsCount	Number of threads.
	*/
	void setThreadCountToStart(int threadsCount);

	boost::signal<void(ImageRecordType, bool)> recordConfigurationStatusChanged;

	//boost::signal<void(StringFileRecordType, bool)> recordConfigurationStatusChanged;

	boost::signal<void(ImageRecordType)>	recordPathOrFileNameChanged;

	void setKinectVersion(KinectVersionType	kinectVersion);

private:

	/**
	* \fn	CString ImageRecordingConfiguration::getDefaultFileName();
	*
	* \brief	Gets default file name of the set Imagetype
	*
	* \return	The default file name.
	*/
	CString getDefaultFileName();

	/**
	* \fn	void ImageRecordingConfiguration::setDefaultFileName();
	*
	* \brief	Sets default file name for the set Imagetype.
	*/
	void setDefaultFileName();

	/** \brief	outputfolder/timestamp/Imagetype/filename.fileformat. */
	CString					m_outputFolder;

	/** \brief	Pathname of the time stamp folder. */
	CString					m_timeStampFolderName;

	/** \brief	name of the fileprefix. */
	CString					m_fileName;

	/** \brief	Type of the image to record. */
	ImageRecordType			m_imageType;

	KinectVersionType		m_kinectVersion;

	StringFileRecordType			m_stringFileType;

	/** \brief	The file format. */
	ImageRecordingFileFormat		m_fileFormat;

	StringFileRecordingFileFormat		m_stringFileFormat;

	/** \brief	The maximum number of frames to record. can be UNLIMITED_FRAMES (-1). */
	int						m_maxNumberOfFrames;

	/** \brief	true to enable recording, false to disable. */
	bool					m_enabled;

	/** \brief	Number of threads to start for recording. */
	int						m_threadsCount;

	
};
typedef std::shared_ptr<ImageRecordingConfiguration> ImageRecordingConfigurationPtr;
typedef std::vector<ImageRecordingConfigurationPtr> SharedImageRecordingConfiguration;
