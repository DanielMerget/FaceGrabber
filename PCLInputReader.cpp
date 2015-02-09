#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

PCLInputReader::PCLInputReader(const std::string inputPath, const std::string fileNamePrefix, const int bufferSize, const int numOfFilesToRead) :
	m_cloudBuffer(bufferSize),
//	m_cloudBufferIsFreeVariables(bufferSize),
//	m_cloudBufferPositionMutexes(bufferSize),
	m_readerThreads(bufferSize),
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
	for (int i = 0; i < m_bufferSize; i++){
		m_readerThreads.push_back(std::thread(&PCLInputReader::readPLYFile, this, i));
	}
}

bool PCLInputReader::isBufferAtIndexSet(const int index)
{
	return m_cloudBuffer[index];
	//return m_cloudBuffer[index] != nullptr;
}

void PCLInputReader::printMessage(std::string msg)
{
	std::lock_guard<std::mutex> lock(m_printMutex);
	std::cout << msg << std::endl;
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
			printMessage(msg.str());
			m_cloudBufferUpdated.wait(cloudBufferLock);
		}
		
		printMessage("update thread woke up - updating");
		cloudUpdated(m_cloudBuffer[currentUpdateIndex]);
		m_cloudBuffer[currentUpdateIndex].reset();
		m_cloudBufferFree.notify_all();
		printMessage("update thread sleeping");
		
		std::chrono::milliseconds dura(100);
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
	//while (true){
	//	indexOfFileToRead = indexOfFileToRead + index + m_bufferSize;
	//	std::cout << "thread for index: " << index << " now reading: " << indexOfFileToRead << std::endl;
	//	if (indexOfFileToRead > m_numOfFilesToRead){
	//		std::cout << "thread for index: " << index << " done " << std::endl;
	//		return;
	//	}
	//}
	//
	//return;
	while (true)
	{
		if (indexOfFileToRead > m_numOfFilesToRead){
			return;
		}
		
		pcl::PCLPointCloud2 blob;
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
		std::stringstream fileName;
		//fileName << m_inputPath << "/" << m_fileNamePrefix << indexOfFileToRead << ".ply";
		fileName << m_fileNamePrefix << indexOfFileToRead << ".ply";

		//std::string name = "C:/Users/mas/Documents/Repositories/IDP-KinectHDFaceGrabber/Cloud_0.ply";
		m_printMutex.lock();
		pcl::io::loadPLYFile(fileName.str(), blob );
		
		pcl::fromPCLPointCloud2(blob, *cloud);
		m_printMutex.unlock();
		std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		const int cloudBufferIndex = indexOfFileToRead % m_bufferSize;
		while (isBufferAtIndexSet(cloudBufferIndex)){
			std::stringstream msg;
			msg << "thread for index: " << index << " waiting for updater thread for slot " << cloudBufferIndex << std::endl;
			printMessage(msg.str());
			m_cloudBufferFree.wait(cloudBufferLock);
		}

		m_cloudBuffer[cloudBufferIndex] = cloud;
		std::stringstream msg;
		int newIndexOfFileToRead = indexOfFileToRead + m_bufferSize;

		msg << "thread for index: " << index << " updated:" << cloudBufferIndex << " with: oldIndex: " << indexOfFileToRead << " new Index: " << newIndexOfFileToRead << std::endl;
		printMessage(msg.str());
		indexOfFileToRead = newIndexOfFileToRead;
		m_cloudBufferUpdated.notify_all();
	}
}