#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

PCLInputReader::PCLInputReader(const int bufferSize) :
	m_cloudBuffer(bufferSize),
//	m_cloudBufferIsFreeVariables(bufferSize),
//	m_cloudBufferPositionMutexes(bufferSize),
	m_readerThreads(),
	m_bufferSize(bufferSize)
{

}

void PCLInputReader::join()
{
	for (auto& thread : m_readerThreads)
	{
		if (thread.joinable()){
			thread.join();
			std::cout << "main joining thread" << std::endl;
		}
	}
	if (m_updateThread.joinable()){
		m_updateThread.join();
	}
	
}

PCLInputReader::~PCLInputReader()
{
	//for (auto& thread : m_readerThreads)
	//{
	//	thread.join();
	//}
	////m_updateThread.join();
	join();
}

void PCLInputReader::startCloudUpdateThread()
{
	m_updateThread = std::thread(&PCLInputReader::updateThreadFunc, this);
}

void PCLInputReader::startReaderThreads()
{ 
	for (int i = 0; i < 1; i++){
		m_readerThreads.push_back(std::thread(&PCLInputReader::readPLYFile, this, i));
	}
}

bool PCLInputReader::isBufferAtIndexSet(const int index)
{
	return m_cloudBuffer[index];
}

void PCLInputReader::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(m_printMutex);
	std::cout << msg;
}

void PCLInputReader::updateThreadFunc()
{
	printMessage("update thread started");
	int currentUpdateIndex = 0;
	int numOfFilesRead = currentUpdateIndex;
	while (true)
	{
		if (numOfFilesRead >= m_playbackConfiguration->getCloudFilesToPlay().size()){
			printMessage("update thread finished");
			return;
		}
		std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		while (!isBufferAtIndexSet(currentUpdateIndex)){
			std::stringstream msg;
			msg << "update thread waiting for index " << currentUpdateIndex << " after reading files: " << numOfFilesRead << std::endl;
			//printMessage(msg.str());
			m_cloudBufferUpdated.wait(cloudBufferLock);
		}

		//printMessage("update thread woke up - updating ");
		std::stringstream updateMsg;
		updateMsg << "updating: " << numOfFilesRead << std::endl;
		printMessage(updateMsg.str());
		cloudUpdated(m_cloudBuffer[currentUpdateIndex]);
		m_cloudBuffer[currentUpdateIndex].reset();
		m_cloudBufferFree.notify_all();
		//printMessage("update thread sleeping");

		std::chrono::milliseconds dura(80);
		std::this_thread::sleep_for(dura);
		//printMessage("update thread woke up from sleeping");
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
		if (indexOfFileToRead >= cloudFilesToPlay.size()){
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
		//construct filename
		//std::stringstream fileName;
		//fileName << m_fileNamePrefix << indexOfFileToRead << ".pcd";
		
		auto currentFileName = cloudFilesToPlay[indexOfFileToRead];
		pcl::io::loadPCDFile(currentFileName, *cloud);
		//load the ply file
		//pcl::io::loadPCDFile(fileName.str(), *cloud);


		std::stringstream readingMsg;
		readingMsg << "ReadingFile: " << currentFileName << std::endl;
		printMessage(readingMsg.str());
//		m_printMutex.unlock();

		//store the cloud in the buffer it the index is free
		std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		const int cloudBufferIndex = indexOfFileToRead % m_bufferSize;
		while (isBufferAtIndexSet(cloudBufferIndex)){
			std::stringstream msg;
			msg << "thread for index: " << index << " waiting for updater thread for slot " << cloudBufferIndex << std::endl;
			//printMessage(msg.str());
			m_cloudBufferFree.wait(cloudBufferLock);
		}
		//store the cloud
		m_cloudBuffer[cloudBufferIndex] = cloud;
		std::stringstream msg;

		//calc new buffer index
		int newIndexOfFileToRead = indexOfFileToRead + m_readerThreads.size();
		msg << "thread for index: " << index << " updated:" << cloudBufferIndex << " with: oldIndex: " << indexOfFileToRead << " new Index: " << newIndexOfFileToRead << std::endl;
		//printMessage(msg.str());
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
}