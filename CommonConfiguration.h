#pragma once
#include "stdafx.h"

#include <string>
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <atlstr.h>
#include <regex>
#include "IRecordingConfiguration.h"
#include "NuiApi.h"

enum CommonConfigurationType{
	KinectV1_COMMON,
	KinectV2_COMMON,
	COMMON_CONFIGURATION_TYPE_COUNT
};

/**
 * \class	RecordingConfiguration
 *
 * \brief	The Recording Configuration stores all information neccessary to the KinectCloudFileWriter to save
 * 			ply and pcd files to the directory. It allows to specify the directory for the files and automatically
 * 			searches for the files. It constructs the directory path to save the point clouds in the following way:
 *			//outputfolder/timestamp/cloudtype/filename.fileformat.
 */

class CommonConfiguration{


public:
	CommonConfiguration();

	

	/**
	 * \fn	RecordingConfiguration::RecordingConfiguration(RecordingConfiguration& recordingConfiguration);
	 *
	 * \brief	Copy constructor.
	 *
	 * \param [in]	recordingConfiguration	The recording configuration.
	 */

	CommonConfiguration(CommonConfiguration& commonConfiguration);

	~CommonConfiguration();



	

	/**
	 * \fn	RecordingFileFormat RecordingConfiguration::getRecordFileFormat();
	 *
	 * \brief	Gets fileformat to record
	 *
	 * \return	The show  Option.
	 */
	RecordingShowOpt getShowOpt();
		
	void setEnabled(bool enabled){m_Enabled = enabled;};

		
	/**
	 * \fn	bool RecordingConfiguration::isEnabled();
	 *
	 * \brief	Query if Recording is enabled.
	 *
	 * \return	true if enabled, false if not.
	 */
	bool isEnabled(){return m_Enabled;};

	/**
	 * \fn	void RecordingConfiguration::setEnabled(bool enabled);
	 *
	 * \brief	Sets an enabled.
	 *
	 * \param	enabled	true to enable, false to disable.
	 */
	void setKeepBGEnabled(bool enabled);

	bool isKeepBGEnabled();

		/**
	 * \fn	void RecordingConfiguration::setShowOpt(RecordingShowOpt ShowOpt);
	 *
	 * \brief	Sets show option.
	 *
	 * \param	showOpt	Will show accoding to the ShowOpt.
	 */

	void setShowOpt(RecordingShowOpt ShowOpt);

	void setFacePointsShowOpt(FacePointsShowOpt FacePointsShowOpt);

	void setShowResolution(NUI_IMAGE_RESOLUTION	showResolution);

	FacePointsShowOpt getFacePointsShowOpt();


	static CString getShowOptAsString(RecordingShowOpt ShowOpt);

	void setThreadCountToStart(int threadsCount);
	int getThreadCountToStart();
	//boost::signal<void(RecordCloudType, bool)> recordConfigurationStatusChanged;
	//boost::signal<void(RecordCloudType)>	recordPathOrFileNameChanged;
private:

	
	/** \brief	The Show Option. */
	RecordingShowOpt         m_ShowOpt;

	bool					m_Enabled;
	/** \brief	true to enable recording, false to disable. */
	bool					m_keepBGEnabled;
	FacePointsShowOpt		m_FacePointsShowOpt;

	/** \brief	Number of threads to start for recording. */
	int						m_threadsCount;
	NUI_IMAGE_RESOLUTION	m_showResolution;						
};

typedef std::shared_ptr<CommonConfiguration> CommonConfigurationPtr;
typedef std::vector<CommonConfigurationPtr> SharedCommonConfiguration;
