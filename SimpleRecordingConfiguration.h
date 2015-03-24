#pragma once
#include "stdafx.h"

#include "IRecordingConfiguration.h"
#include <memory>

/**
 * \class	SimpleRecordingConfiguration
 *
 * \brief	A simple recording configuration which allows to specify
 * 			only the required attributes manually. In contrast to 
 * 			RecordingConfiguration no concatenation of outputfolder,
 * 			timestamps etc. is done here.
 */

class SimpleRecordingConfiguration : public IRecordingConfiguration
{
public:

	/**
	 * \fn	SimpleRecordingConfiguration::SimpleRecordingConfiguration();
	 *
	 * \brief	Default constructor.
	 */
	SimpleRecordingConfiguration();

	/**
	 * \fn	SimpleRecordingConfiguration::~SimpleRecordingConfiguration();
	 *
	 * \brief	Destructor.
	 */
	~SimpleRecordingConfiguration();

	/**
	 * \fn	RecordingFileFormat SimpleRecordingConfiguration::getRecordFileFormat();
	 *
	 * \brief	Gets record file format.
	 *
	 * \return	The record file format.
	 */
	RecordingFileFormat getRecordFileFormat();

	/**
	 * \fn	std::string SimpleRecordingConfiguration::getFullRecordingPathString();
	 *
	 * \brief	Gets full recording path string.
	 *
	 * \return	The full recording path string.
	 */
	std::string getFullRecordingPathString();

	/**
	 * \fn	std::string SimpleRecordingConfiguration::getFileNameString();
	 *
	 * \brief	Gets file name prefix as string.
	 *
	 * \return	The file name string.
	 */
	std::string getFileNameString();

	/**
	 * \fn	void SimpleRecordingConfiguration::setRecordFileFormat(RecordingFileFormat fileformat);
	 *
	 * \brief	Sets record file format.
	 *
	 * \param	fileformat	The fileformat.
	 */
	void setRecordFileFormat(RecordingFileFormat fileformat);

	/**
	 * \fn	void SimpleRecordingConfiguration::setFullRecordingPathString(std::string fullRecordinGapth);
	 *
	 * \brief	Sets full recording path string.
	 *
	 * \param	fullRecordinGapth	The full recording gapth.
	 */
	void setFullRecordingPathString(std::string fullRecordingGapth);

	/**
	 * \fn	void SimpleRecordingConfiguration::setFileNameString(std::string theFilePrefix);
	 *
	 * \brief	Sets file name given as string.
	 *
	 * \param	theFilePrefix	the file prefix.
	 */
	void setFileNameString(std::string theFilePrefix);

	/**
	 * \fn	bool SimpleRecordingConfiguration::isRecordConfigurationValid();
	 *
	 * \brief	Query if the SimpleRecordingConfiguration is record configuration valid
	 * 			meaning all necessary values are set.
	 *
	 * \return	true if record configuration valid, false if not.
	 */
	bool isRecordConfigurationValid();

	/**
	 * \fn	bool SimpleRecordingConfiguration::isRecordingDurationUnLimited();
	 *
	 * \brief	Query if recording duration unlimited.
	 *
	 * \return	true if recording duration un limited, false if not.
	 */
	bool isRecordingDurationUnLimited();

	/**
	 * \fn	int SimpleRecordingConfiguration::getMaxNumberOfFrames();
	 *
	 * \brief	Gets maximum number of frames to record.
	 *
	 * \return	The maximum number of frames.
	 */
	int getMaxNumberOfFrames();

	/**
	 * \fn	void SimpleRecordingConfiguration::setMaxNumberOfFrames(int newMaxNumberOfFrames);
	 *
	 * \brief	Sets maximum number of frames to record.
	 *
	 * \param	newMaxNumberOfFrames	The new maximum number of frames.
	 */
	void setMaxNumberOfFrames(int newMaxNumberOfFrames);

	/**
	 * \fn	int SimpleRecordingConfiguration::getThreadCountToStart();
	 *
	 * \brief	Gets amount of threads to start.
	 *
	 * \return	The amount of threads to start.
	 */
	int getThreadCountToStart();

	/**
	 * \fn	void SimpleRecordingConfiguration::setThreadCountToStart(int amountOfThreads);
	 *
	 * \brief	Sets thread count to start.
	 *
	 * \param	amountOfThreads	The amount of threads.
	 */
	void setThreadCountToStart(int amountOfThreads);
private:

	/** \brief	prefix name of the files. */
	std::string			m_fileName;

	/** \brief	Full directory path where to save recorded files. */
	std::string			m_fullRecordingPath;

	/** \brief	The recording file format. */
	RecordingFileFormat m_recordingFileFormat;

	/** \brief	The maximum number of frames to record. */
	int					m_maximumNumberOfFrames;

	/** \brief	Number of threads to start. */
	int					m_threadsCount;
};
typedef std::shared_ptr<SimpleRecordingConfiguration> SimpleRecordingConfigurationPtr;

