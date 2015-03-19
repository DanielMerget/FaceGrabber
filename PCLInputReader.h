#pragma once
#include "stdafx.h"

#include <string>
#include <vector>
#include <thread>
#include <boost/signals2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <condition_variable>
#include <mutex>
#include "PlaybackConfiguration.h"
#include "Buffer.h"
#include "PCLInputReaderWorkerThread.h"

/**
 * \class	PCLInputReader
 *
 * \brief	The PCLInputReader is able to read point cloud files (ply, pcd) multi-threaded or single-threaded.
 * 			The files to be read must be specified a PlaybackConfiguration. Having started the reading, the files
 * 			are stored in the given buffer which provides them for further processing. THe PCLInputReader resizes
 * 			the Buffer to save all read files. So the Buffer will have the same size as number of files to be read.
 *
 * \tparam	PointType	Type of the point type.
 */

template <typename PointType>
class PCLInputReader
{
public:

	PCLInputReader();

	~PCLInputReader();

	/**
	 * \fn	void PCLInputReader::startCloudUpdateThread(bool isSingleThreatedReadingAndBlocking);
	 *
	 * \brief	Configures the Buffer of the PCLInputReader and triggers the starting of the reader threads. 
	 * 			Before, the Buffer to be used to store the read clouds and the PlaybackConfiguration has to be set.
	 *
	 * \param	isSingleThreatedReadingAndBlocking	true if reading is done single threated reading
	 *			and the method call should be blocking, false if multiple threads should be started and
	 *			the call to this method should be non-blocking.
	 */
	void startCloudUpdateThread(bool isSingleThreatedReadingAndBlocking);

	/**
	 * \fn	void PCLInputReader::stopReaderThreads();
	 *
	 * \brief	Stops the reader threads.
	 */
	void stopReaderThreads();

	

	/**
	 * \fn	std::shared_ptr<Buffer< boost::shared_ptr<pcl::PointCloud<PointType>>> > PCLInputReader::getBuffer();
	 *
	 * \brief	Getter for the Buffer of the PCLInputReader.
	 *
	 * \return	The buffer of the PCLInputReader.
	 */

	std::shared_ptr<Buffer< boost::shared_ptr<pcl::PointCloud<PointType>>> > getBuffer();

	/**
	 * \fn	void PCLInputReader::setBuffer(std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer);
	 *
	 * \brief	Setter for the Buffer of the PCLInputReader.
	 *
	 * \param	buffer	The buffer to be set.
	 */
	void setBuffer(std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer);

	/**
	 * \fn	void PCLInputReader::setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig);
	 *
	 * \brief	Setter for the playback configuration.
	 *
	 * \param	playbackConfig	The playback configuration to be set.
	 */

	void setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig);
	

	/** \brief	Signal notfying about the current reading status. */
	boost::signals2::signal<void(std::wstring)> updateStatus;
private:

	void printMessage(std::string msg);

	std::mutex m_printMutex;

	/**
	 * \fn	void PCLInputReader::createAndStartThreadForIndex(int index, int numOfThreads);
	 *
	 * \brief	Creates and starts thread with the specified index.
	 *
	 * \param	index			Zero-based index of the thread to be started.
	 * \param	numOfThreads	Number of threads to be started.
	 */
	void createAndStartThreadForIndex(int index, int numOfThreads);

	/**
	 * \fn	void PCLInputReader::startReaderThreads(bool isSingleThreatedReadingAndBlocking);
	 *
	 * \brief	Starts reader threads.
	 *
	 * \param	isSingleThreatedReadingAndBlocking	true if this call starts only one reader thread 
	 * 												and is blocking, false when non-blocking and
	 * 												multiple-reader threads have to be started.
	 */
	void startReaderThreads(bool isSingleThreatedReadingAndBlocking);

	/**
	 * \fn	void PCLInputReader::join();
	 *
	 * \brief	Blocks until all reader and the updater threads are finished.
	 */
	void join();

	/**
	 * \fn	void PCLInputReader::readerFinishedReadingAFile();
	 *
	 * \brief	Notifies that a reader thread has finished a file.
	 */
	void readerFinishedReadingAFile();
	

	/** \brief	The threads started for reading files. */
	std::vector<std::thread> m_readerThreads;


	/** \brief	The the objects associated with the reader threads. */
	std::vector<std::shared_ptr<PCLInputReaderWorkerThread<PointType>>> m_inputReaderWorkerThreads;


	/** \brief	The buffer storing the read point clouds. */
	std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> m_buffer;


	/** \brief	Number of files to read. */
	int m_numOfFilesRead;


	/** \brief	The mutex protecting m_numOfFilesRead. */
	std::mutex m_numOfFilesReadMutex;


	/** \brief	The current playback configuration. */
	PlaybackConfiguration m_playbackConfiguration;


	/** \brief	The previous playback configuration used to track if we can reuse the clouds in the buffer. */
	PlaybackConfiguration m_previousPlaybackConfiguration;
};


