#include "stdafx.h"
#include "PCLInputReaderWorkerThread.h"
#include <iostream>
#include <atlstr.h>
#include <string>
#include <pcl/io/file_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

template PCLInputReaderWorkerThread < pcl::PointXYZRGB> ;
template PCLInputReaderWorkerThread < pcl::PointXYZ>;
template < typename PointType >
PCLInputReaderWorkerThread< PointType >::PCLInputReaderWorkerThread()
{
}

template < typename PointType >
PCLInputReaderWorkerThread< PointType >::~PCLInputReaderWorkerThread()
{
}

template < typename PointType >
void PCLInputReaderWorkerThread< PointType >::printMessage(std::string msg)
{
	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	OutputDebugString(msgCstring);
}

template < typename PointType >
void PCLInputReaderWorkerThread< PointType >::stopReading()
{
	m_isPlaybackRunning = false;
}

template < typename PointType >
void PCLInputReaderWorkerThread< PointType >::setBuffer(std::shared_ptr<Buffer< boost::shared_ptr<pcl::PointCloud<PointType>>>> buffer)
{
	m_buffer = buffer;
}

template < typename PointType >
void PCLInputReaderWorkerThread< PointType >::readCloudData(const int index, const int step, std::vector<CloudFile> cloudFilesToPlay, RecordingFileFormat format)
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

		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud <PointType>());


		auto filePath = cloudFilesToPlay[indexOfFileToRead].fullFilePath;
		std::stringstream readingfileMessage;
		readingfileMessage << "Reading file: " << filePath;
		printMessage(readingfileMessage.str());

		fileReader->read<PointType>(filePath, *cloud);
		readingfileMessage << " finished";
		printMessage(readingfileMessage.str());
		//fileReader->read(filePath, *cloud);
		finishedReadingAFile();
		
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