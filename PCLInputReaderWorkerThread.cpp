#include "PCLInputReaderWorkerThread.h"
#include <iostream>
#include <atlstr.h>
#include <string>
#include <pcl/io/file_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

PCLInputReaderWorkerThread::PCLInputReaderWorkerThread()
{
}


PCLInputReaderWorkerThread::~PCLInputReaderWorkerThread()
{
}


void PCLInputReaderWorkerThread::printMessage(std::string msg)
{
	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

void PCLInputReaderWorkerThread::stopReading()
{
	m_isPlaybackRunning = false;
}
void PCLInputReaderWorkerThread::setBuffer(std::shared_ptr<Buffer> buffer)
{
	m_buffer = buffer;
}

void PCLInputReaderWorkerThread::readCloudData(const int index, const int step, std::vector<CloudFile> cloudFilesToPlay, RecordingFileFormat format)
{
	m_isPlaybackRunning = true;
	int indexOfFileToRead = index;
	std::stringstream msg;
	msg << "thread for index: " << index << "started. " << std::endl;
	printMessage(msg.str());
	
	std::shared_ptr<pcl::FileReader> fileReader;
	switch (format)
	{
	case PLY:
		fileReader = std::shared_ptr<pcl::FileReader>(new pcl::PLYReader);
		break;
	case PLY_BINARY:
		fileReader = std::shared_ptr<pcl::FileReader>(new pcl::PLYReader);
		break;
	case PCD:
		fileReader = std::shared_ptr < pcl::FileReader>(new pcl::PCDReader);
		break;
	case PCD_BINARY:
		fileReader = std::shared_ptr < pcl::FileReader>(new pcl::PCDReader);
		break;
	case RECORD_FILE_FORMAT_COUNT:
		break;
	default:
		break;
	}
	auto numberOfFilesToRead = cloudFilesToPlay.size();
	//pcl::PCDReader reader;

	while (true)
	{
		//if (indexOfFileToRead > numberOfFilesToRead || !m_isPlaybackRunning){
		if (indexOfFileToRead >= numberOfFilesToRead || !m_isPlaybackRunning){
			//m_cloudBufferUpdated.notify_all();
			//m_buffer.m_cloudBufferUpdated->notify_all();!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			std::stringstream doneMsg;
			doneMsg << "thread for index: " << index << " done because of stop or size end" << std::endl;
			printMessage(doneMsg.str());
			m_buffer->setProducerFinished();
			//if (indexOfFileToRead == numberOfFilesToRead){
			//	m_isPlaybackRunning = false;
			//}
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());


		auto filePath = cloudFilesToPlay[indexOfFileToRead].fullFilePath;
		fileReader->read<pcl::PointXYZRGB>(filePath, *cloud);
		//fileReader->read(filePath, *cloud);
		

		const int cloudBufferIndex = indexOfFileToRead % m_buffer->getBufferSize();

		m_buffer->pushData(cloud, cloudBufferIndex);

		std::stringstream msg;
		//calc new buffer index
		int newIndexOfFileToRead = indexOfFileToRead + step;
		msg << "thread for index: " << index << " updated:" << cloudBufferIndex << " with: oldIndex: " << indexOfFileToRead << " new Index: " << newIndexOfFileToRead << std::endl;
		printMessage(msg.str());
		indexOfFileToRead = newIndexOfFileToRead;
	}
}