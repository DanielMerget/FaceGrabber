#pragma once
#include "stdafx.h"
#include <mutex>
#include <condition_variable>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/signals2.hpp>

/**
 * \class	Buffer
 *
 * \brief	A buffer which allows to push new data and pull it afterwards. The buffer
 * 			allows multiple threads to push data. Pulling of data blocks until new data
 * 			is available. Pusing of data blocks until there is a free element in buffer.
 *
 * \tparam	DataType	Type of the data to be buffered.
 */
template < class DataType >
class Buffer
{
public:

	/**
	 * \fn	Buffer::Buffer();
	 *
	 * \brief	Default constructor.
	 */
	Buffer();

	/**
	 * \fn	Buffer::~Buffer();
	 *
	 * \brief	Destructor.
	 */
	~Buffer();

	/**
	 * \fn	void Buffer::setBufferSize(int bufferSize);
	 *
	 * \brief	Sets szie of the buffer.
	 *
	 * \param	bufferSize	Size of the buffer.
	 */

	void setBufferSize(int bufferSize);

	/**
	 * \fn	int Buffer::getBufferSize();
	 *
	 * \brief	Gets buffer size.
	 *
	 * \return	The buffer size.
	 */

	int getBufferSize();

	/**
	 * \fn	void Buffer::resetBuffer();
	 *
	 * \brief	Resets the buffer to the initial state.
	 */
	void resetBuffer();

	/**
	 * \fn	void Buffer::pushData(DataType newData, int index);
	 *
	 * \brief	Pushes data into the buffer at the given index.
	 *
	 * \param	newData	The data to be pushed to the buffer.
	 * \param	index  	Zero-based index in the buffer the data h
	 * 					should be pushed
	 */
	void pushData(DataType newData, int index);

	/**
	 * \fn	bool Buffer::isDataAvailable();
	 *
	 * \brief	Queries if a data is available in the buffer.
	 *
	 * \return	true if a data is available, false if not.
	 */
	bool isDataAvailable();
	

	/** \brief	Signals whether data is available for pulling. */
	std::shared_ptr<boost::signals2::signal<void(void)>> dataReady;

	/**
	 * \fn	DataType Buffer::pullData();
	 *
	 * \brief	Pulls the data "oldest" data of the buffer. Blocking until data is available.
	 *
	 * \return	The oldest buffered data. Null if the buffer is disabled or empty.
	 */

	DataType pullData();

	/**
	 * \fn	bool Buffer::isBufferAtIndexSet(const int index);
	 *
	 * \brief	Query if 'index' there is an alement in the buffer at given index.
	 *
	 * \param	index	Zero-based index where to look for buffered data.
	 *
	 * \return	true if buffer at index set or if overriding of all existing data
	 * 			is enabled, false if not. 
	 */
	bool isBufferAtIndexSet(const int index);

	/**
	 * \fn	void Buffer::enableBuffer();
	 *
	 * \brief	Enables the buffer.
	 */
	void enableBuffer();

	/**
	 * \fn	void Buffer::disableBuffer();
	 *
	 * \brief	Disables the buffer.
	 */
	void disableBuffer();

	/**
	 * \fn	void Buffer::setProducerFinished();
	 *
	 * \brief	Notifes the Buffer that the producer finished causing the buffer to
	 * 			let release all elements of the buffer even if the buffer is at the 
	 * 			moment not completely filled.
	 */
	void setProducerFinished();

	/**
	 * \fn	void Buffer::setReleaseDataAfterPull(bool enable);
	 *
	 * \brief	Sets the flag indicating to release the data after it was pulled.
	 *
	 * \param	enable	true to enable, false to disable.
	 */
	void setReleaseDataAfterPull(bool enable);

	/**
	 * \fn	bool Buffer::isDataReleasedAfterPull();
	 *
	 * \brief	Query if t data is released after pull.
	 *
	 * \return	true if data is released, false if not.
	 */
	bool isDataReleasedAfterPull();

	/**
	 * \fn	void Buffer::resetPullCounterAndPullAndNotifyConsumer();
	 *
	 * \brief	Resets the pull counter and notifies the consumer that the
	 * 			data is ready.
	 */

	void resetPullCounterAndPullAndNotifyConsumer();
private:

	void printMessage(std::string msg);
	std::shared_ptr<std::mutex> m_printMutex;


	/** \brief	The cloud buffer mutex. */
	std::shared_ptr<std::mutex>				 m_cloudBufferMutex;


	/** \brief	The cloud buffer is free condition. */
	std::shared_ptr<std::condition_variable> m_cloudBufferFree;


	/** \brief	The cloud buffer was updated condition. */
	std::shared_ptr<std::condition_variable> m_cloudBufferUpdated;


	/** \brief	The buffering active. */
	int m_bufferingActive;


	/** \brief	The index of the buffer where the next data to be pulled is stored. */
	int m_pullDataPosition;


	/** \brief	Buffer for cloud data. */
	std::vector<DataType> m_cloudBuffer;


	/** \brief	The buffer fill level. */
	int	m_bufferFillLevel;


	/** \brief	true if producer finished. */
	bool m_producerFinished;


	/** \brief	true if data is released after pull. */
	bool m_releaseDataAfterPull;
};

