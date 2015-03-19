#pragma once
#include "stdafx.h"
#include "Buffer.h"

#include <boost/signals2.hpp>
#include <thread>
#include <condition_variable>
#include <mutex>

/**
 * \class	BufferSynchronizer
 *
 * \brief	The BufferSynchronizer synchronizes the data that can be pulled from given buffers:
 * 			The BufferSynchronizer waits until data is available on all buffers. Afterwards,
 * 			the data is pulled and provided via the publishSynchronizedData signal. BufferSynchronizer is
 * 			supposed to run in its threading function updateThreadFunc.
 *
 * \tparam	BufferDataType	Type of the buffer data type.
 */

template < class BufferDataType >
class BufferSynchronizer
{
public:

	/**
	 * \fn	BufferSynchronizer::BufferSynchronizer(bool simulatePlaybackByWaitingEachFrame);
	 *
	 * \brief	Constructor.
	 *
	 * \param	simulatePlaybackByWaitingEachFrame	true to simulate playback by waiting between
	 * 												each new pushed frame.
	 */
	BufferSynchronizer(bool simulatePlaybackByWaitingEachFrame);

	~BufferSynchronizer();

	/**
	 * \fn	void BufferSynchronizer::updateThreadFunc();
	 *
	 * \brief	Function supposed to run in a thread. It synchronizes the data provided by the
	 * 			set Buffers and pushes them synchronized via publishSynchronizedData. THe method notifies
	 * 			about updateStatus and playbackFinished.
	 */
	void updateThreadFunc();

	/**
	 * \fn	void BufferSynchronizer::onApplicationQuit();
	 *
	 * \brief	Causes the update thread to terminate.
	 */
	void onApplicationQuit();

	/**
	 * \fn	void BufferSynchronizer::signalDataOfBufferWithIndexIsReady(int index);
	 *
	 * \brief	Signals the BufferSynchronizer that the data of the Buffers with the given index
	 * 			has data ready. If all other Buffers are ready, too. The Pulling of Data is triggered.
	 *
	 * \param	index	Zero-based index of the Buffer which is ready.
	 */
	void signalDataOfBufferWithIndexIsReady(int index);

	/**
	 * \fn	void BufferSynchronizer::setBuffer(std::vector<std::shared_ptr<Buffer<BufferDataType>>> buffers, int numOfFilesToRead);
	 *
	 * \brief	Sets the buffers to be synchronized
	 *
	 * \param	buffers				The buffers to be synchronized.
	 * \param	numOfFilesToRead	Number of files to reads/pulled from the buffers.
	 */
	void setBuffer(std::vector<std::shared_ptr<Buffer<BufferDataType>>> buffers, int numOfFilesToRead);
	

	/** \brief	Publishes stati. */
	boost::signals2::signal<void(std::wstring)> updateStatus;


	/** \brief	Notifies that the playback was finished. */
	boost::signals2::signal<void(void)> playbackFinished;

	/**
	 * \property	boost::signals2::signal<void(std::vector<BufferDataType> data)> publishSynchronizedData
	 *
	 * \brief	Publishes the data of the provided by Buffers synchronized.
	 *
	 */
	boost::signals2::signal<void(std::vector<BufferDataType> data)> publishSynchronizedData;

private:

	void BufferSynchronizer::printMessage(std::string msg);
	std::mutex m_printMutex;

	/** \brief	Number of files to read from the current Buffers. */
	int m_numOfFilesToRead;


	/** \brief	Number of files read from the current Buffers. */
	int m_numOfFilesRead;

	/**
	* \property	std::vector<std::pair<std::shared_ptr<Buffer<BufferDataType>>, bool>> m_bufferWithReadyState
	*
	* \brief	Map saving the Buffer together with the data-ready state.
	*
	*/
	std::vector<std::pair<std::shared_ptr<Buffer<BufferDataType>>, bool>> m_bufferWithReadyState;


	
	/** \brief	Mutex to synchronized m_bufferWithDataReadyState. */
	std::mutex m_updateBuffer;


	/** \brief	The is data available condition variable. */
	std::condition_variable m_isDataAvailableConditionVariable;


	/** \brief	true if all buffers have data ready. */
	bool m_isDataAvaiable;


	/** \brief	true if the synchronizer is running. */
	bool m_isRunning;


	/** \brief	true if wait after publishing data to simulate delay between each recorded frame. */
	bool m_waitEachFrame;
};

