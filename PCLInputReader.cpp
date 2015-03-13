#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <future>         // std::async
#include "PCLInputReaderWorkerThread.h"

template PCLInputReader < pcl::PointXYZRGB> ;
template PCLInputReader < pcl::PointXYZ>;
template <typename PointType>
PCLInputReader< PointType >::PCLInputReader() :
	m_readerThreads(),
	m_isPlaybackRunning(false),
	m_numOfFilesRead(0)
{
}

template <typename PointType>
void PCLInputReader< PointType >::setBuffer(std::shared_ptr<Buffer< boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer)
{
	m_buffer = buffer;
}
template <typename PointType>
void PCLInputReader< PointType >::join()
{
	if (m_updateThread.joinable()){
		m_updateThread.join();
	}
	for (auto& thread : m_readerThreads)
	{
		if (thread.joinable()){
			thread.join();
			std::cout << "main joining thread" << std::endl;
		}
	}
	
	m_readerThreads.clear();
}

template <typename PointType>
PCLInputReader< PointType >::~PCLInputReader()
{
	join();
}

template <typename PointType>
void PCLInputReader< PointType >::startCloudUpdateThread(bool isSingleThreatedReadingAndBlocking)
{
	if (!m_playbackConfiguration.isEnabled()){
		return;
	}
	join();

	//check for ability to reuse our buffer
	if (!m_buffer->isResetDataAfterPullEnabled() && m_previousPlaybackConfiguration.isEnabled() && m_previousPlaybackConfiguration.wasFullPlayed()){
		if (m_playbackConfiguration == m_previousPlaybackConfiguration){
			m_buffer->resetPullCounterAndPullAndNotifyConsumer();
			m_playbackConfiguration.setWasFullPlayed();
			printMessage("detected previous playbackconfiguration was the same! => reuse it; InputReader finished reading..");
			return;
		}
	}
	

	m_buffer->resetBuffer();

	int numOfFilesToRead = m_playbackConfiguration.getCloudFilesToPlay().size();

	m_buffer->setBufferSize(numOfFilesToRead);
	m_isPlaybackRunning = true;
	//m_updateThread = std::thread(&PCLInputReader::updateThreadFunc, this);
	if (isSingleThreatedReadingAndBlocking){
		startReaderThreads(isSingleThreatedReadingAndBlocking);
	}
	else{
		std::async(std::launch::async, &PCLInputReader::startReaderThreads, this, isSingleThreatedReadingAndBlocking);
	}
}

template <typename PointType>
void PCLInputReader< PointType >::createAndStartThreadForIndex(int index, int numOfThreads)
{
	auto recordingType = m_playbackConfiguration.getRecordFileFormat();
	auto filesToPlay = m_playbackConfiguration.getCloudFilesToPlay();
	std::shared_ptr<PCLInputReaderWorkerThread<PointType>> reader(new PCLInputReaderWorkerThread<PointType>);
	reader->setBuffer(m_buffer);
	reader->finishedReadingAFile.connect(boost::bind(&PCLInputReader<PointType>::readerFinishedReadingAFile, this));
	m_inputReaderWorkerThreads.push_back(reader);
	m_readerThreads.push_back(std::thread(&PCLInputReaderWorkerThread<PointType>::readCloudData, reader, index, numOfThreads, filesToPlay, recordingType));
}

template <typename PointType>
void PCLInputReader< PointType >::readerFinishedReadingAFile()
{
	std::lock_guard<std::mutex> lock(m_numOfFilesReadMutex);
	m_numOfFilesRead++;
	std::wstringstream statusMessage;
	statusMessage << "read: " <<  m_numOfFilesRead << "/" << m_playbackConfiguration.getCloudFilesToPlay().size();
	updateStatus(statusMessage.str());
	if (m_numOfFilesRead == m_playbackConfiguration.getCloudFilesToPlay().size()){
		m_playbackConfiguration.setWasFullPlayed();
	}
}

template <typename PointType>
void PCLInputReader< PointType >::startReaderThreads(bool isSingleThreatedReadingAndBlocking)
{ 
	updateStatus(L"");
	m_numOfFilesRead = 0;
	m_playbackConfiguration.sortCloudFilesForPlayback();
	m_isPlaybackRunning = true;
	m_buffer->enableBuffer();
	if (isSingleThreatedReadingAndBlocking){
		createAndStartThreadForIndex(0, 1);
		for (auto& thread : m_readerThreads){
			thread.join();
		}
	}
	else{
		const int numOfThreadsToStart = 5;
		for (int i = 0; i < numOfThreadsToStart; i++){
			createAndStartThreadForIndex(i, numOfThreadsToStart);
		}
	}
}

template <typename PointType>
void PCLInputReader< PointType >::stopReaderThreads()
{ 
	//if (!m_isPlaybackRunning){
	//	return;
	//}
	m_isPlaybackRunning = false;
	for (auto& reader : m_inputReaderWorkerThreads){
		reader->stopReading();
	}
	m_buffer->disableBuffer();
}


template <typename PointType>
void PCLInputReader< PointType >::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(m_printMutex);
	
	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

template <typename PointType>
void PCLInputReader< PointType >::setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig)
{
	if (m_playbackConfiguration.isEnabled()){
		m_previousPlaybackConfiguration = m_playbackConfiguration;
	}
	m_playbackConfiguration = *playbackConfig;
}

template <typename PointType>
std::shared_ptr<Buffer< boost::shared_ptr<pcl::PointCloud<PointType>>>> PCLInputReader< PointType >::getBuffer()
{
	return m_buffer;
}