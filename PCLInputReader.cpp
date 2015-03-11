#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <future>         // std::async
#include "PCLInputReaderWorkerThread.h"

PCLInputReader::PCLInputReader() :
	m_readerThreads(),
	m_isPlaybackRunning(false),
	m_numOfFilesRead(0)
{
}

void PCLInputReader::setBuffer(std::shared_ptr<Buffer< pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> buffer)
{
	m_buffer = buffer;
}
void PCLInputReader::join()
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

PCLInputReader::~PCLInputReader()
{
	join();
}

void PCLInputReader::startCloudUpdateThread()
{
	if (!m_playbackConfiguration->isEnabled()){
		return;
	}
	join();
	m_buffer->resetBuffer();

	int numOfFilesToRead = m_playbackConfiguration->getCloudFilesToPlay().size();

	m_buffer->setBufferSize(numOfFilesToRead);
	m_isPlaybackRunning = true;
	//m_updateThread = std::thread(&PCLInputReader::updateThreadFunc, this);

	std::async(&PCLInputReader::startReaderThreads, this);
}

void PCLInputReader::createAndStartThreadForIndex(int index, int numOfThreads)
{
	auto recordingType = m_playbackConfiguration->getRecordFileFormat();
	auto filesToPlay = m_playbackConfiguration->getCloudFilesToPlay();
	std::shared_ptr<PCLInputReaderWorkerThread> reader(new PCLInputReaderWorkerThread);
	reader->setBuffer(m_buffer);
	reader->finishedReadingAFile.connect(boost::bind(&PCLInputReader::readerFinishedReadingAFile, this));
	m_inputReaderWorkerThreads.push_back(reader);
	m_readerThreads.push_back(std::thread(&PCLInputReaderWorkerThread::readCloudData, reader, index, numOfThreads, filesToPlay, recordingType));
}

void PCLInputReader::readerFinishedReadingAFile()
{
	std::lock_guard<std::mutex> lock(m_numOfFilesReadMutex);
	m_numOfFilesRead++;
	std::wstringstream statusMessage;
	statusMessage << "read: " <<  m_numOfFilesRead << "/" << m_playbackConfiguration->getCloudFilesToPlay().size();
	updateStatus(statusMessage.str());
}

void PCLInputReader::startReaderThreads()
{ 
	updateStatus(L"");
	m_numOfFilesRead = 0;
	m_playbackConfiguration->sortCloudFilesForPlayback();
	m_isPlaybackRunning = true;
	m_buffer->enableBuffer();
	const int numOfThreadsToStart = 5;
	for (int i = 0; i < numOfThreadsToStart; i++){
		//m_readerThreads.push_back(std::thread(&PCLInputReader::readCloudData, this, i));
		createAndStartThreadForIndex(i, numOfThreadsToStart);
	}
}

void PCLInputReader::stopReaderThreads()
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



void PCLInputReader::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(m_printMutex);
	
	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

void PCLInputReader::setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig)
{
	m_playbackConfiguration = playbackConfig;
}

std::shared_ptr<Buffer< pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> PCLInputReader::getBuffer()
{
	return m_buffer;
}