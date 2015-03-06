#include "PCLInputReader.h"
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

PCLInputReader::PCLInputReader() :
	m_readerThreads(),
	m_isPlaybackRunning(false)
{

}
void PCLInputReader::setBuffer(std::shared_ptr<Buffer> buffer)
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
	if (!m_playbackConfiguration->isEnabled() || m_isPlaybackRunning){
		return;
	}
	join();
	m_buffer->resetBuffer();

	int numOfFilesToRead = m_playbackConfiguration->getCloudFilesToPlay().size();

	m_buffer->setBufferSize(numOfFilesToRead);

	//m_updateThread = std::thread(&PCLInputReader::updateThreadFunc, this);
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
	m_buffer->disableBuffer();
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
	auto numOfFilesToRead = m_playbackConfiguration->getCloudFilesToPlay().size();
	
		
	while (true)
	{
		if (numOfFilesRead >= numOfFilesToRead || !m_isPlaybackRunning){
			printMessage("update thread finished or read everything");
			m_isPlaybackRunning = false;
			//playbackFinished();

			return;
		}

		printMessage("update thread pull new data");
		cloudUpdated(m_buffer->pullData());
		printMessage("update thread got new data");	

		std::chrono::milliseconds dura(80);
		std::this_thread::sleep_for(dura);
		printMessage("update thread woke up from sleeping");
		numOfFilesRead++;
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
			m_buffer->setProducerFinished();
			if (indexOfFileToRead == numberOfFilesToRead){
				m_isPlaybackRunning = false;
			}
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());

		
		auto filePath = cloudFilesToPlay[indexOfFileToRead].fullFilePath;
		std::unique_lock<std::mutex> cloudBufferLock(m_readMutex);
		readCloudFromDisk(filePath, *cloud);
		cloudBufferLock.unlock();
	
		const int cloudBufferIndex = indexOfFileToRead % m_buffer->getBufferSize();

		m_buffer->pushData(cloud, cloudBufferIndex);

		std::stringstream msg;
		//calc new buffer index
		int newIndexOfFileToRead = indexOfFileToRead + m_readerThreads.size();
		msg << "thread for index: " << index << " updated:" << cloudBufferIndex << " with: oldIndex: " << indexOfFileToRead << " new Index: " << newIndexOfFileToRead << std::endl;
		printMessage(msg.str());
		indexOfFileToRead = newIndexOfFileToRead;
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

std::shared_ptr<Buffer> PCLInputReader::getBuffer()
{
	return m_buffer;
}