#include "Buffer.h"
#include "stdafx.h"
#include <atlstr.h>
Buffer::Buffer():
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

void Buffer::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(*m_printMutex);

	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

Buffer::~Buffer()
{
}

void Buffer::resetBuffer()
{
	std::lock_guard<std::mutex> cloudBufferLock(*m_cloudBufferMutex);
	m_bufferFillLevel = 0;
	m_pullDataPosition = 0;
	m_producerFinished = false;
	m_bufferingActive = true;
	m_cloudBuffer.clear();
}

void Buffer::setBufferSize(int bufferSize)
{
	m_cloudBuffer.resize(bufferSize);
}

void Buffer::pushData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newData, int index)
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

bool Buffer::isDataAvailable()
{
	return m_cloudBuffer[m_pullDataPosition];
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Buffer::pullData()
{
	std::unique_lock<std::mutex> cloudBufferLock(*m_cloudBufferMutex);
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

bool Buffer::isBufferAtIndexSet(const int index)
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
int Buffer::getBufferSize()
{
	return m_cloudBuffer.size();
}
void Buffer::setProducerFinished()
{
	m_producerFinished = true;
}

void Buffer::enableBuffer()
{
	m_bufferingActive = true;
}
void Buffer::disableBuffer()
{
	m_bufferingActive = false;
	m_cloudBufferUpdated->notify_all();
	m_cloudBufferFree->notify_all();
}
