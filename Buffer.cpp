#include "Buffer.h"
#include "stdafx.h"
#include <atlstr.h>
#include "KinectFileWriterThread.h"

template Buffer < pcl::PointCloud< pcl::PointXYZRGB>::Ptr >;
template Buffer < std::shared_ptr<PointCloudMeasurement<pcl::PointXYZRGB>> > ;
template Buffer < std::shared_ptr<PointCloudMeasurement<pcl::PointXYZ>> > ;
//template KinectCloudOutputWriter < pcl::PointXYZRGB >;

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
	
	while (isBufferAtIndexSet(index)){
		std::stringstream waitMSg;
		waitMSg << "thread for index: " << index << " waiting for updater thread for slot " << index << std::endl;
		printMessage(waitMSg.str());
		//m_cloudBufferFree.wait(cloudBufferLock);
		m_cloudBufferFree->wait(cloudBufferLock);
		(*dataReady)();
		if (!m_bufferingActive){
			std::stringstream playbackFinishedMSG;
			playbackFinishedMSG << "thread for index: " << index << " done because of stop " << std::endl;
			printMessage(playbackFinishedMSG.str());
			return;
		}
	}

	m_cloudBuffer[index] = newData;
	m_bufferFillLevel++;
	m_cloudBufferUpdated->notify_all();
	cloudBufferLock.unlock();
	if (dataReady->empty()){
		printMessage("dont have a slot connected");
	}
	std::stringstream msg;
	msg << "push data: FillLevel" << m_bufferFillLevel << "of "<< getBufferSize();
	printMessage(msg.str());
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
DataType Buffer< DataType >::pullData()
{
	std::unique_lock<std::mutex> cloudBufferLock(*m_cloudBufferMutex);
	if (!m_bufferingActive){
		return DataType(nullptr);
	}
	 while (m_bufferFillLevel != m_cloudBuffer.size()){
		if (m_producerFinished){
			printMessage("break!");
			break;
		}
		m_cloudBufferUpdated->wait(cloudBufferLock);
	}
	 auto result = m_cloudBuffer[m_pullDataPosition];
	 m_bufferFillLevel--;
	 m_cloudBuffer[m_pullDataPosition].reset();
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
