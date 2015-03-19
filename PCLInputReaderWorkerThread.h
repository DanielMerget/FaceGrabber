#pragma once
#include "stdafx.h"

#include <memory>
#include "Buffer.h"
#include "PlaybackConfiguration.h"
#include <boost/signals2.hpp>

/**
 * \class	PCLInputReaderWorkerThread
 *
 * \brief	A PCL input reader worker object used by the PCLInputReader to read point cloud files (ply, pcd).
 *
 * \tparam	PointType	Type of the point type.
 */

template < typename PointType >
class PCLInputReaderWorkerThread 
{
public:

	PCLInputReaderWorkerThread();
	~PCLInputReaderWorkerThread();

	/**
	 * \fn	void PCLInputReaderWorkerThread::readCloudData(const int index, const int step, std::vector<CloudFile> cloudFilesToPlay, RecordingFileFormat format);
	 *
	 * \brief	Reads point cloud files. For the given index and step-size all files are red and stored in the
	 * 			set buffer. e.g. for index = 2 and step = 3: the files with indecies 2, 5, 7 of the
	 * 			cloudFilesToPlay list are read.
	 *
	 * \param	index				Zero-based index of the files in the cloudFilesToPlay to be read.
	 * \param	step				Amount to increment the index to calculate the next index to read.
	 * \param	cloudFilesToPlay	The cloud files to play.
	 * \param	format				Describes the format to use.
	 */
	void readCloudData(const int index, const int step, std::vector<CloudFile> cloudFilesToPlay, RecordingFileFormat format);

	/**
	 * \fn	void PCLInputReaderWorkerThread::stopReading();
	 *
	 * \brief	Stops a reader thread.
	 */
	void stopReading();

	/**
	 * \fn	void PCLInputReaderWorkerThread::setBuffer(std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer);
	 *
	 * \brief	Sets a buffer the read point clouds are saved to.
	 *
	 * \param	buffer	The buffer.
	 */
	void setBuffer(std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer);


	/** \brief	The finished reading a file signal. */
	boost::signals2::signal<void(void)> finishedReadingAFile;
private:

	void printMessage(std::string msg);


	/** \brief	true if the current thread is running. */
	bool m_isPlaybackRunning;


	/** \brief	The buffer to store read point clouds. */
	std::shared_ptr<Buffer<boost::shared_ptr<pcl::PointCloud<PointType>>>> m_buffer;
};

