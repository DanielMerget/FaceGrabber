#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

PCLInputReader::PCLInputReader(const int bufferSize) :
	//m_cloudBuffer(50),
	m_readerThreads(),
	//m_bufferSize(bufferSize),
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
	if (!m_playbackConfiguration->isEnabled() || m_isPlaybackRunning){
		return;
	}
	join();
	m_buffer.resetBuffer();

	//m_cloudBuffer.clear();
	int numOfFilesToRead = m_playbackConfiguration->getCloudFilesToPlay().size();
	//m_bufferSize = numOfFilesToRead;
	
	//m_cloudBuffer.resize(m_bufferSize);
	
	m_buffer.setBufferSize(numOfFilesToRead);

	m_updateThread = std::thread(&PCLInputReader::updateThreadFunc, this);
}

void PCLInputReader::startReaderThreads()
{ 
	if (!m_playbackConfiguration->isEnabled() || m_isPlaybackRunning){
		return;
	}
	m_isPlaybackRunning = true;
	for (int i = 0; i < 5; i++){
		m_readerThreads.push_back(std::thread(&PCLInputReader::readCloudData, this, i));
	}
}

void PCLInputReader::stopReaderThreads()
{ 
	if (!m_isPlaybackRunning){
		return;
	}
	m_isPlaybackRunning = false;
	//m_cloudBufferUpdated.notify_all();
	//m_buffer.m_cloudBufferUpdated->notify_all();
	m_buffer.disableBuffer();
}


//bool PCLInputReader::isBufferAtIndexSet(const int index)
//{
//	return m_cloudBuffer[index];
//}
//
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
	auto numOfFilesToRead = m_playbackConfiguration->getCloudFilesToPlay().size();
	
		
	while (true)
	{
		if (numOfFilesRead >= numOfFilesToRead || !m_isPlaybackRunning){
			printMessage("update thread finished or read everything");
			m_isPlaybackRunning = false;
			playbackFinished();

			return;
		}
		//std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		//std::unique_lock<std::mutex> cloudBufferLock(*m_buffer.m_cloudBufferMutex);
		//const int leftFilesToRead = (numOfFilesToRead - numOfFilesRead);
		//std::stringstream check;
		//check << "fill level: " << m_bufferFillLevel << "buffer: " << m_bufferSize << "A: " <<(m_bufferFillLevel != m_bufferSize)
		//	<< "B: " << ((leftFilesToRead < m_bufferSize) && !isBufferAtIndexSet(currentUpdateIndex)) <<
		//	"left files: " << leftFilesToRead << " buffer Set? " << isBufferAtIndexSet(currentUpdateIndex) << std::endl;
		//printMessage(check.str());
		////while (m_bufferFillLevel != m_bufferSize){
		////	if ((leftFilesToRead < m_bufferSize) && isBufferAtIndexSet(currentUpdateIndex)){
		////		printMessage("break!");
		////		break;
		////	}
		//while (!m_buffer.isDataAvailable()){
		//	
		//	std::stringstream msg;
		//	msg << "update thread waiting for index " << currentUpdateIndex << " after reading files: " << numOfFilesRead << std::endl;
		//	printMessage(msg.str());
		//	//m_cloudBufferUpdated.wait(cloudBufferLock);
		//	m_buffer.m_cloudBufferUpdated->wait(cloudBufferLock);
		//	if (!m_isPlaybackRunning){
		//		std::stringstream msg;
		//		msg << "update done because of stop" << std::endl;
		//		//m_cloudBufferFree.notify_all();
		//		m_buffer.m_cloudBufferFree->notify_all();
		//		printMessage(msg.str());
		//		return;
		//	}
		//}
		//
		//printMessage("update thread woke up - updating ");
		//std::stringstream updateMsg;
		//updateMsg << "updating: " << numOfFilesRead << std::endl;
		//printMessage(updateMsg.str());

		//cloudUpdated(m_cloudBuffer[currentUpdateIndex]);,
		printMessage("update thread pull new data");
		cloudUpdated(m_buffer.pullData());
		printMessage("update thread got new data");
		//m_cloudBuffer[currentUpdateIndex].reset();
		//m_bufferFillLevel--;

		//m_buffer.m_cloudBufferFree->notify_all();
		//m_cloudBufferFree.notify_all();
		

		std::chrono::milliseconds dura(80);
		std::this_thread::sleep_for(dura);
		printMessage("update thread woke up from sleeping");
		numOfFilesRead++;
		//currentUpdateIndex = (currentUpdateIndex + 1) % m_bufferSize;
	}
}

void PCLInputReader::readCloudData(const int index)
{
	int indexOfFileToRead = index;
	std::stringstream msg;
	msg << "thread for index: " << index << "started. " << std::endl;
	printMessage(msg.str());

	auto cloudFilesToPlay = m_playbackConfiguration->getCloudFilesToPlay();
	auto numberOfFilesToRead = cloudFilesToPlay.size();
	//pcl::PCDReader reader;

	while (true)
	{
		if (indexOfFileToRead >= numberOfFilesToRead || !m_isPlaybackRunning){
			//m_cloudBufferUpdated.notify_all();
			//m_buffer.m_cloudBufferUpdated->notify_all();!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			std::stringstream doneMsg;
			doneMsg<< "thread for index: " << index << " done because of stop or size end" << std::endl;
			printMessage(doneMsg.str());
			m_buffer.setProducerFinished();
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());

		
		auto filePath = cloudFilesToPlay[indexOfFileToRead].fullFilePath;
		std::unique_lock<std::mutex> cloudBufferLock(m_readMutex);
		readCloudFromDisk(filePath, *cloud);
		cloudBufferLock.unlock();
		//std::stringstream readingMsg;
		//readingMsg << "ReadingFile: " << filePath << std::endl;
		//printMessage(readingMsg.str());
		////store the cloud in the buffer it the index is free
		////std::unique_lock<std::mutex> cloudBufferLock(m_cloudBufferMutex);
		//std::unique_lock<std::mutex> cloudBufferLock(*m_buffer.m_cloudBufferMutex);
		
		const int cloudBufferIndex = indexOfFileToRead % m_buffer.getBufferSize();
		//while (isBufferAtIndexSet(cloudBufferIndex)){
		//while (m_buffer.isBufferAtIndexSet(cloudBufferIndex)){
		//	std::stringstream waitMSg;
		//	waitMSg << "thread for index: " << index << " waiting for updater thread for slot " << cloudBufferIndex << std::endl;
		//	printMessage(waitMSg.str());
		//	//m_cloudBufferFree.wait(cloudBufferLock);
		//	m_buffer.m_cloudBufferFree->wait(cloudBufferLock);
		//	if (!m_isPlaybackRunning){
		//		std::stringstream playbackFinishedMSG;
		//		playbackFinishedMSG << "thread for index: " << index << " done because of stop " << std::endl;
		//		printMessage(playbackFinishedMSG.str());
		//		return;
		//	}
		//}
		//store the cloud
		if (!cloud){
			OutputDebugString(L"read cloud was null");
		}
		m_buffer.pushData(cloud, cloudBufferIndex);
		//m_cloudBuffer[cloudBufferIndex] = cloud;
		//m_bufferFillLevel++;
		
		//cloudBufferLock.unlock();

		std::stringstream msg;

		//calc new buffer index
		int newIndexOfFileToRead = indexOfFileToRead + m_readerThreads.size();
		msg << "thread for index: " << index << " updated:" << cloudBufferIndex << " with: oldIndex: " << indexOfFileToRead << " new Index: " << newIndexOfFileToRead << std::endl;
		printMessage(msg.str());
		indexOfFileToRead = newIndexOfFileToRead;

		//auto isInFinishPhase = numberOfFilesToRead - indexOfFileToRead < m_buffer.getBufferSize();
		//if (isInFinishPhase){
		//	m_buffer.setProducerFinished();
		//}
		//int maxBufferIndex = m_bufferSize - 1;
		//m_cloudBufferUpdated.notify_all();
		
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