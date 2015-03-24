#pragma once
#include "stdafx.h"
#include <queue>
#include <condition_variable>
#include <mutex>

#include <thread>
#include <vector>
#include <future>
#include "RecordingConfiguration.h"
#include "KinectFileWriterThread.h"
#include "CloudMeasurementSource.h"
#include <boost/signals2.hpp>

/**
 * \class	KinectCloudOutputWriter
 *
 * \brief	A kinect cloud output writer which writes Point Clouds to the hard disk according
 * 			to the set Configuration. Writing can be done multi-threaded.
 *
 * \tparam	PointCloudType	Type of the point cloud type.
 */

template < class PointCloudType >
class KinectCloudFileWriter : CloudMeasurementSource< PointCloudType >
{
public:
	
	KinectCloudFileWriter();
	
	~KinectCloudFileWriter();

	/**
	 * \fn	void KinectCloudFileWriter::pushCloudAsync(boost::shared_ptr<pcl::PointCloud<PointCloudType>> cloud);
	 *
	 * \brief	Pushes a cloud into the buffer holding the Point Clouds to be written to the hard drive.
	 *
	 * \param	cloud	The cloud to be pushed.
	 */

	void pushCloudAsync(boost::shared_ptr<pcl::PointCloud<PointCloudType>> cloud);

	/**
	 * \fn	void KinectCloudFileWriter::pushCloudsAsync(std::vector<boost::shared_ptr<pcl::PointCloud<PointCloudType>>> clouds);
	 *
	 * \brief	Pushes a list of Point Clouds into the buffer holding the Point Clouds to be written to the hard drive.
	 *
	 * \param	clouds	The clouds.
	 */

	void pushCloudsAsync(std::vector<boost::shared_ptr<pcl::PointCloud<PointCloudType>>> clouds);

	/**
	 * \fn	void KinectCloudFileWriter::startWritingClouds();
	 *
	 * \brief	Triggers the start of writing the buffered clouds.
	 */

	void startWritingClouds();

	/**
	 * \fn	void KinectCloudFileWriter::stopWritingClouds();
	 *
	 * \brief	Triggers the stop of writing the buffered clouds.
	 */

	void stopWritingClouds();

	/**
	 * \fn	void KinectCloudFileWriter::setRecordingConfiguration(IRecordingConfigurationPtr recordingConfiguration);
	 *
	 * \brief	Sets recording configuration for the writing.
	 *
	 * \param	recordingConfiguration	The recording configuration.
	 */

	void setRecordingConfiguration(IRecordingConfigurationPtr recordingConfiguration);

	/**
	* \fn	virtual bool CloudMeasurementSource::pullData(PointCloudMeasurement<PointCloudType>& measurement) = 0;
	*
	* \brief	Fills the given measurement with most recent data.
	*
	* \param [in,out]	measurement	The measurement.
	*
	* \return	true the source is empty or not running, otherwise false
	*/
	bool pullData(PointCloudMeasurement<PointCloudType>& measurement);


	/** \brief	The signal notfying about changes of the writing status. */
	boost::signals2::signal<void(std::wstring)> updateStatus;


	/** \brief	The signal notifying about the stop of the writing by the user. */
	boost::signals2::signal<void(void)> writingWasStopped;
private:

	/**
	 * \fn	bool KinectCloudFileWriter::isMaximumFramesReached();
	 *
	 * \brief	Query whether the maxmimum of frames was reached.
	 *
	 * \return	true if limit of frames reached, false if not.
	 */

	bool isMaximumFramesReached();

	/**
	 * \fn	void KinectCloudFileWriter::waitForWriterToFinish();
	 *
	 * \brief	Method supposed for a thread to wait for all writer threads to finish
	 * 			in order to update the status accordingly.
	 */

	void waitForWriterToFinish();

	/**
	 * \fn	void KinectCloudFileWriter::pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);
	 *
	 * \brief	Arctually pushes a cloud to the buffer in a blocking way. 
	 *
	 * \param	cloud	The cloud.
	 */

	void pushCloud(boost::shared_ptr<const pcl::PointCloud<PointCloudType>> cloud);


	
	/** \brief	The started writer threads. */
	std::vector<std::thread> m_writerThreads;


	/** \brief	The writer objectes associated to the writer threads. */
	std::vector<std::shared_ptr<KinectFileWriterThread<PointCloudType>>> m_writers;


	/** \brief	The buffered point clouds. */
	std::queue<PointCloudMeasurement<PointCloudType>> m_clouds;


	/** \brief	The check for existance of point clouds in the buffer. */
	std::condition_variable m_checkCloud;


	/** \brief	The lock for the cloud buffer. */
	std::mutex m_lockCloud;


	/** \brief	The recording configuration. */
	IRecordingConfigurationPtr m_recordingConfiguration;


	/** \brief	true if writing is currently running. */
	bool m_running;


	/** \brief	Number of buffered clouds. */
	int m_cloudCount;	
};

