#include "stdafx.h"
#include "BufferSynchronizer.h"
#include <thread>
#include <atlstr.h>

template BufferSynchronizer < pcl::PointCloud< pcl::PointXYZRGB>::Ptr >;
template BufferSynchronizer < pcl::PointCloud< pcl::PointXYZ>::Ptr >;

template < class BufferDataType >
BufferSynchronizer< BufferDataType >::BufferSynchronizer(bool simulatePlaybackByWaitingEachFrame) :
	m_isDataAvaiable(false),
	m_Paused(false),
	m_FPSLimit(0),
	m_waitEachFrame(simulatePlaybackByWaitingEachFrame)
	//m_printMutex(new std::mutex),
	//m_updateBuffer(new std::mutex),
	//m_isDataAvailableConditionVariable(new std::condition_variable)
{
	
}


template < class BufferDataType >
BufferSynchronizer< BufferDataType >::~BufferSynchronizer()
{
	
}

template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::onApplicationQuit()
{
	std::unique_lock<std::mutex> lock(m_updateBuffer);
	m_isRunning = false;
	m_isDataAvaiable = false;
	m_isDataAvailableConditionVariable.notify_all();
}

template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(m_printMutex);

	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}


template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::updateThreadFunc()
{
	m_isRunning = true;
	std::chrono::milliseconds dura(80);

	clock_t lastFrame;
	double timedelta;
	double actualFPS;

	std::stringstream FPSinfo;
	CString msgCstring;

	lastFrame = clock();

	while (m_isRunning)
	{
		//wait until we have data ready
		std::unique_lock<std::mutex> lock(m_updateBuffer);
		while (!m_isDataAvaiable && m_isRunning){
			printMessage("synchronier waiting");
			if (!m_isDataAvaiable){
				printMessage("No DATA availible");
			}
			m_isDataAvailableConditionVariable.wait(lock);
		}
		lock.unlock();

		while (m_isDataAvaiable){
			std::vector<BufferDataType> readyPointClouds;
			//pull data from each buffer and store it
			for (auto bufferWithState : m_bufferWithReadyState){
				printMessage("synchronier pulling data");
				auto cloud = bufferWithState.first->pullData(m_numOfFilesRead);
				printMessage("synchronier got data");
				if (!cloud){
					printMessage("synchronier received a null cloud");
					m_isDataAvaiable = false;
					break;
				}
				bufferWithState.second = false;
				//store it
				readyPointClouds.push_back(cloud);
			}

			//publish the data
			if (readyPointClouds.size() > 0){
				printMessage("synchronier updating");

				if (!m_Paused){
					m_numOfFilesRead++;
				}

				updatePlaybackSliderPos(m_numOfFilesRead);

				publishSynchronizedData(readyPointClouds);

				std::wstringstream message;
				message << "play: " << m_numOfFilesRead << "/" << m_numOfFilesToRead;
				updateStatus(message.str());

				// FPS limitation

				// save value from last frame
				// pullData with frame number
				// set slider to actual played frame

				if (m_FPSLimit != 0)
				{
					// timedelta in seconds
					timedelta = (double(clock() - lastFrame)) / CLOCKS_PER_SEC;
					// if faster than specified target fps: sleep
					if (timedelta < (1.0 / m_FPSLimit)) Sleep(((1.0 / m_FPSLimit) - timedelta) * 1000);
				}

				actualFPS = CLOCKS_PER_SEC / (float(clock() - lastFrame));
				lastFrame = clock();

				FPSinfo.str("");
				FPSinfo.clear();
				FPSinfo << "Playback Loop actual fps: " << actualFPS << " target fps: " << m_FPSLimit;
				msgCstring = CString(FPSinfo.str().c_str());
				msgCstring += L"\n";
				OutputDebugString(msgCstring);
			}

			//do we want to simpulate the live frame grabbing? 
			if (m_waitEachFrame){
				printMessage("synchronier sleepig");
				//wait for a few miliseconds to get delay between frames
				std::this_thread::sleep_for(dura);
				printMessage("synchronier woke up");
			}
			
		}
		printMessage("synchronier stopping");
		//all files we read => notify our observers about that
		if (m_numOfFilesToRead == m_numOfFilesRead){
			printMessage("Finished Playback");
			playbackFinished();
		}
	}
}


template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::setBuffer(std::vector<std::shared_ptr<Buffer<BufferDataType>>> buffers, int numOfFilesToRead)
{
	m_numOfFilesToRead = numOfFilesToRead;
	m_numOfFilesRead = 0;

	updatePlaybackSliderRange(1, numOfFilesToRead);
	updatePlaybackSliderPos(1);

	for (auto buffer : m_bufferWithReadyState){
		buffer.first->dataReady->disconnect_all_slots();
	}
	m_bufferWithReadyState.clear();
	for (int i = 0; i < buffers.size(); i++){
		auto currentBuffer = buffers[i];
		m_bufferWithReadyState.push_back(std::pair<std::shared_ptr<Buffer<BufferDataType>>, bool>(currentBuffer, false));
		currentBuffer->dataReady->connect(boost::bind(&BufferSynchronizer::signalDataOfBufferWithIndexIsReady, this, i));
	}
}


template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::signalDataOfBufferWithIndexIsReady(int index)
{
	printMessage("signal: Data for index now availbel for synchronizer" + index);
	std::unique_lock<std::mutex> lock(m_updateBuffer);
	m_bufferWithReadyState[index].second = true;
	for (auto bufferWithState : m_bufferWithReadyState){
		if (bufferWithState.second == false){
			return;
		}
	}
	printMessage("signal: Data now available for synchronizer");
	m_isDataAvaiable = true;
	m_isDataAvailableConditionVariable.notify_all();
}


template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::setFPSLimit(int fps)
{
	m_FPSLimit = fps;
}

template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::setPaused(bool enable)
{
	m_Paused = enable;
}

template < class BufferDataType >
void BufferSynchronizer< BufferDataType >::setPlaybackFrame(int pos)
{
	printMessage("Slider requested frame " + std::to_string(pos));
	if (pos >= 1 && pos <= m_numOfFilesToRead){
		m_numOfFilesRead = pos;
	}
}