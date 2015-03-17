#pragma once
#include "stdafx.h"
#include "Buffer.h"

#include <boost/signals2.hpp>
#include <thread>
#include <condition_variable>
#include <mutex>

template < class BufferDataType >
class BufferSynchronizer
{
public:
	BufferSynchronizer();
	~BufferSynchronizer();

	void updateThreadFunc();

	void onApplicationQuit();
	
	void stopPlayback();

	void signalDataOfBufferWithIndexIsReady(int index);

	void setBuffer(std::vector<std::shared_ptr<Buffer<BufferDataType>>> buffers, int numOfFilesToRead);
	
	boost::signals2::signal<void(std::wstring)> updateStatus;

	boost::signals2::signal<void(void)> playbackFinished;
	
	boost::signals2::signal<void(std::vector<BufferDataType> cloud)> cloudsUpdated;


	
private:
	void BufferSynchronizer::printMessage(std::string msg);
	int m_numOfFilesToRead;
	int m_numOfFilesRead;

	std::vector<std::pair<std::shared_ptr<Buffer<BufferDataType>>, bool>> m_bufferWithReadyState;
	std::mutex m_printMutex;
	std::mutex m_updateBuffer;
	std::condition_variable m_isDataAvailableConditionVariable;
	//std::shared_ptr<std::mutex> m_printMutex;
	//std::shared_ptr<std::mutex> m_updateBuffer;
	//std::shared_ptr<std::condition_variable> m_isDataAvailableConditionVariable;
	bool m_isDataAvaiable;
	bool m_playbackActive;
	bool m_isRunning;
};

