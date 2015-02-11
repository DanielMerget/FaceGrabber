#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

PCLInputReader::PCLInputReader(const std::string inputPath, const std::string fileNamePrefix, const int bufferSize, const int numOfFilesToRead) :
	m_cloudBuffer(bufferSize),
//	m_cloudBufferIsFreeVariables(bufferSize),
//	m_cloudBufferPositionMutexes(bufferSize),
	m_readerThreads(),
	m_inputPath(inputPath),
	m_bufferSize(bufferSize),
	m_fileNamePrefix(fileNamePrefix),
	m_numOfFilesToRead(numOfFilesToRead)
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
	for (int i = 0; i < 5; i++){
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
		if (numOfFilesRead >= m_numOfFilesToRead){
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

		std::chrono::milliseconds dura(50);
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

	while (true)
	{
		if (indexOfFileToRead > m_numOfFilesToRead){
			return;
		}
		//pcl::PCLPointCloud2 blob;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
		//construct filename
		std::stringstream fileName;
		//fileName << m_fileNamePrefix << indexOfFileToRead << ".ply";
		fileName << m_fileNamePrefix << indexOfFileToRead << ".pcd";

		//load the ply file
		//m_printMutex.lock();
		//pcl::io::loadPLYFile(fileName.str(), *cloud);
		pcl::io::loadPCDFile(fileName.str(), *cloud);
		std::stringstream readingMsg;
		readingMsg << "ReadingFile: " << fileName.str() << std::endl;
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