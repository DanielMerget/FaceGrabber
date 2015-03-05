#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

PCLInputReader::PCLInputReader(const int bufferSize) :
	m_cloudBuffer(bufferSize),
//	m_cloudBufferIsFreeVariables(bufferSize),
//	m_cloudBufferPositionMutexes(bufferSize),
	m_readerThreads(),
	m_bufferSize(bufferSize),
	m_isPlaybackRunning(false)
{

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
	
	m_cloudBuffer.clear();
	m_cloudBuffer.resize(m_bufferSize);
	
	m_updateThread = std::thread(&PCLInputReader::updateThreadFunc, this);
}

void PCLInputReader::startReaderThreads()
{ 
	if (!m_playbackConfiguration->isEnabled()){
		return;
	}
	m_isPlaybackRunning = true;
	for (int i = 0; i < 1; i++){
		m_readerThreads.push_back(std::thread(&PCLInputReader::readPLYFile, this, i));
	}
}

void PCLInputReader::stopReaderThreads()
{ 
	if (!m_isPlaybackRunning){
		return;
	}
	m_isPlaybackRunning = false;
	m_cloudBufferUpdated.notify_all();
}


bool PCLInputReader::isBufferAtIndexSet(const int index)
{
	return m_cloudBuffer[index];
}

void PCLInputReader::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(m_printMutex);
	
	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

void PCLInputReader::updateThreadFunc()
{
	printMessage("update thread started");
	int currentUpdateIndex = 0;
	int numOfFilesRead = currentUpdateIndex;
	while (true)
	{
		if (numOfFilesRead >= m_playbackConfiguration->getCloudFilesToPlay().size() || !m_isPlaybackRunning){
			printMessage("update thread finished or read everything");
			playbackFinished();
			return;
		}
		std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		while (!isBufferAtIndexSet(currentUpdateIndex)){
			std::stringstream msg;
			msg << "update thread waiting for index " << currentUpdateIndex << " after reading files: " << numOfFilesRead << std::endl;
			printMessage(msg.str());
			m_cloudBufferUpdated.wait(cloudBufferLock);
			if (!m_isPlaybackRunning){
				std::stringstream msg;
				msg << "update done because of stop"<< std::endl;
				m_cloudBufferFree.notify_all();
				printMessage(msg.str());
				return;
			}
		}

		printMessage("update thread woke up - updating ");
		std::stringstream updateMsg;
		updateMsg << "updating: " << numOfFilesRead << std::endl;
		printMessage(updateMsg.str());
		cloudUpdated(m_cloudBuffer[currentUpdateIndex]);
		m_cloudBuffer[currentUpdateIndex].reset();
		m_cloudBufferFree.notify_all();
		printMessage("update thread sleeping");

		std::chrono::milliseconds dura(80);
		std::this_thread::sleep_for(dura);
		printMessage("update thread woke up from sleeping");
		numOfFilesRead++;
		currentUpdateIndex = (currentUpdateIndex + 1) % m_bufferSize;
	}
}

void PCLInputReader::readPLYFile(const int index)
{
	int indexOfFileToRead = index;
	std::stringstream msg;
	msg << "thread for index: " << index << "started. " << std::endl;
	printMessage(msg.str());

	auto cloudFilesToPlay = m_playbackConfiguration->getCloudFilesToPlay();
	
	while (true)
	{
		if (indexOfFileToRead >= cloudFilesToPlay.size() || !m_isPlaybackRunning){
			m_cloudBufferUpdated.notify_all();
			std::stringstream doneMsg;
			doneMsg<< "thread for index: " << index << " done because of stop or size end" << std::endl;
			printMessage(doneMsg.str());
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
		//construct filename
		//std::stringstream fileName;
		//fileName << m_fileNamePrefix << indexOfFileToRead << ".pcd";
		
		auto filePath = cloudFilesToPlay[indexOfFileToRead].fullFilePath;
		//pcl::io::loadPCDFile(currentFileName, *cloud);
		readCloudFromDisk(filePath, *cloud);
		//load the ply file
		//pcl::io::loadPCDFile(fileName.str(), *cloud);


		std::stringstream readingMsg;
		readingMsg << "ReadingFile: " << filePath << std::endl;
		printMessage(readingMsg.str());
//		m_printMutex.unlock();

		//store the cloud in the buffer it the index is free
		std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		const int cloudBufferIndex = indexOfFileToRead % m_bufferSize;
		while (isBufferAtIndexSet(cloudBufferIndex)){
			std::stringstream waitMSg;
			waitMSg << "thread for index: " << index << " waiting for updater thread for slot " << cloudBufferIndex << std::endl;
			printMessage(waitMSg.str());
			m_cloudBufferFree.wait(cloudBufferLock);
			if (!m_isPlaybackRunning){
				std::stringstream playbackFinishedMSG;
				playbackFinishedMSG << "thread for index: " << index << " done because of stop " << std::endl;
				printMessage(playbackFinishedMSG.str());
				return;
			}
		}
		//store the cloud
		m_cloudBuffer[cloudBufferIndex] = cloud;
		std::stringstream msg;

		//calc new buffer index
		int newIndexOfFileToRead = indexOfFileToRead + m_readerThreads.size();
		msg << "thread for index: " << index << " updated:" << cloudBufferIndex << " with: oldIndex: " << indexOfFileToRead << " new Index: " << newIndexOfFileToRead << std::endl;
		printMessage(msg.str());
		indexOfFileToRead = newIndexOfFileToRead;

		//notify the updater thread
		if (cloudBufferIndex == (m_bufferSize - 1)){
			m_cloudBufferUpdated.notify_all();
		}
	}
}

void PCLInputReader::setPlaybackConfiguration(PlaybackConfigurationPtr playbackConfig)
{
	m_playbackConfiguration = playbackConfig;
	auto recordingType = playbackConfig->getRecordFileFormat();
	readCloudFromDisk.disconnect_all_slots();
	switch (recordingType)
	{
	case PLY:
		readCloudFromDisk.connect(boost::bind(&pcl::io::loadPLYFile<pcl::PointXYZRGB>, _1, _2));
		break;
	case PLY_BINARY:
		readCloudFromDisk.connect(boost::bind(&pcl::io::loadPLYFile<pcl::PointXYZRGB>, _1, _2));
		break;
	case PCD:
		readCloudFromDisk.connect(boost::bind(&pcl::io::loadPCDFile<pcl::PointXYZRGB>, _1, _2));
		break;
	case PCD_BINARY:
		readCloudFromDisk.connect(boost::bind(&pcl::io::loadPCDFile<pcl::PointXYZRGB>, _1, _2));
		break;
	case RECORD_FILE_FORMAT_COUNT:
		break;
	default:
		break;
	}
}