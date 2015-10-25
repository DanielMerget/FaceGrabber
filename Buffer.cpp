#include "stdafx.h"
#include "Buffer.h"

#include <atlstr.h>
#include "CloudMeasurementSource.h"
template Buffer < pcl::PointCloud< pcl::PointXYZRGB>::Ptr >;
template Buffer < pcl::PointCloud< pcl::PointXYZ>::Ptr >;
template Buffer < std::shared_ptr<PointCloudMeasurement<pcl::PointXYZRGB>> > ;
template Buffer < std::shared_ptr<PointCloudMeasurement<pcl::PointXYZ>> > ;

template < class DataType >
Buffer< DataType >::Buffer() :
	m_cloudBufferMutex(new std::mutex),
	m_printMutex(new std::mutex),
	m_cloudBufferFree(new std::condition_variable),
	m_cloudBufferUpdated(new std::condition_variable),
	dataReady(new boost::signals2::signal<void(void)>),
	m_pullDataPosition(0),
	m_producerFinished(false),
	m_bufferingActive(true),
	m_releaseDataAfterPull(false),
	m_bufferFillLevel(0)
{
}

template < class DataType >
void Buffer< DataType >::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(*m_printMutex);

	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

template < class DataType >
Buffer< DataType >::~Buffer()
{

}

template < class DataType >
void Buffer< DataType >::resetPullCounterAndPullAndNotifyConsumer()
{
	//the buffer is filled with the correct data; just notify the observer
	//that we are ready for pulling 
	std::unique_lock<std::mutex> cloudBufferLock(*m_cloudBufferMutex);
	
	m_bufferFillLevel = m_cloudBuffer.size();
	m_pullDataPosition = 0;
	m_producerFinished = true;
	m_bufferingActive = true;
	(*dataReady)();
	m_cloudBufferUpdated->notify_all();
}

template < class DataType >
bool Buffer< DataType >::isDataReleasedAfterPull()
{
	return m_releaseDataAfterPull;
}

template < class DataType >
void Buffer< DataType >::resetBuffer()
{
	std::lock_guard<std::mutex> cloudBufferLock(*m_cloudBufferMutex);
	m_bufferFillLevel = 0;
	m_pullDataPosition = 0;
	m_producerFinished = false;
	m_bufferingActive = true;
	m_cloudBuffer.clear();
}

template < class DataType >
void Buffer< DataType >::setBufferSize(int bufferSize)
{
	m_cloudBuffer.resize(bufferSize);
}


template < class DataType >
void Buffer< DataType >::pushData(DataType newData, int index)
{
	std::unique_lock<std::mutex> cloudBufferLock(*m_cloudBufferMutex);

	//writing to that buffer is possible at that index?
	while (isBufferAtIndexSet(index)){
		//we have to wait until the buffer index gets freed
		m_cloudBufferFree->wait(cloudBufferLock);
		(*dataReady)();
		if (!m_bufferingActive){
			std::stringstream playbackFinishedMSG;
			playbackFinishedMSG << "thread for index: " << index << " done because of stop " << std::endl;
			printMessage(playbackFinishedMSG.str());
			return;
		}
	}
	//store the data inside the buffer
	m_cloudBuffer[index] = newData;
	m_bufferFillLevel++;

	//notify the updater that new data is available
	m_cloudBufferUpdated->notify_all();
	cloudBufferLock.unlock();
	
	//buffer fill level reached?
	if (m_bufferFillLevel == getBufferSize()){
		printMessage("buffer: data ready");
		(*dataReady)();
	}
}

template < class DataType >
bool Buffer< DataType >::isDataAvailable()
{
	return m_cloudBuffer[m_pullDataPosition];
}

template < class DataType >
void Buffer< DataType >::setReleaseDataAfterPull(bool enable)
{
	m_releaseDataAfterPull = enable;
}

template < class DataType >
DataType Buffer< DataType >::pullData(int pos)
{
	if (pos != -1){
		if (pos < 0 || pos >= m_cloudBuffer.size()){
			printMessage("Pulling at index out of range");
			return DataType(nullptr);
		}
		m_pullDataPosition = pos;
	}
	std::unique_lock<std::mutex> cloudBufferLock(*m_cloudBufferMutex);
	if (!m_bufferingActive || m_bufferFillLevel == 0){
		//we do not have any data ready?
		return DataType(nullptr);
	}
	//wait until buffer is filled
	while (m_bufferFillLevel != m_cloudBuffer.size()){
		if (m_producerFinished){
			//producer have finished => so we can just
			// take the left data out
			printMessage("break!");
			break;
		}
		m_cloudBufferUpdated->wait(cloudBufferLock);
	}
	//retrieve data
	auto result = m_cloudBuffer[m_pullDataPosition];

	//do we want to clear the data retrieved from the buffer?
	if (m_releaseDataAfterPull){
		m_cloudBuffer[m_pullDataPosition].reset();
		m_bufferFillLevel--;
	}

	//calc next pull index
	m_pullDataPosition = (m_pullDataPosition + 1) % m_cloudBuffer.size();
	m_cloudBufferFree->notify_all();
	return result;
}

template < class DataType >
bool Buffer< DataType >::isBufferAtIndexSet(const int index)
{
	if (index >= m_cloudBuffer.size()){
		return false;
	}
	if (!m_releaseDataAfterPull){
		//we never clear the data after pulling => buffer is always filled
		//new pushed data is just overrides the old data in the buffer
		return false;
	}
	if (m_cloudBuffer[index]){
		return true;
	}
	else{
		return false;
	}
}

template < class DataType >
int Buffer< DataType >::getBufferSize()
{
	return m_cloudBuffer.size();
}

template < class DataType >
void Buffer< DataType >::setProducerFinished()
{
	m_producerFinished = true;
}

template < class DataType >
void Buffer< DataType >::enableBuffer()
{
	m_bufferingActive = true;
}

template < class DataType >
void Buffer< DataType >::disableBuffer()
{
	m_bufferingActive = false;
	m_cloudBufferUpdated->notify_all();
	m_cloudBufferFree->notify_all();
}
