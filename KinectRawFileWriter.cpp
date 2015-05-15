#include "stdafx.h"
#include "KinectRawFileWriter.h"


KinectRawFileWriter::KinectRawFileWriter() :
	m_running(false),
	m_imageCount(0),
	m_images(),
	m_writerThreads()
{
}


KinectRawFileWriter::~KinectRawFileWriter()
{
	m_running = false;
	for (auto& thread : m_writerThreads){
		if (thread.joinable()){
			thread.join();
		}
	}
}


void KinectRawFileWriter::waitForWriterToFinish()
{
	updateStatus(L"writing..");
	for (auto& thread : m_writerThreads){
		if (thread.joinable()){
			thread.join();
		}
	}
	updateStatus(L"");
	writingFinished();
}


void KinectRawFileWriter::startWriting()
{
	m_running = true;
	m_imageCount = 0;
	auto threadsToStartCount = m_recordingConfiguration->getThreadCountToStart();
	//create the writer objects and threads
	for (int i = 0; i < threadsToStartCount; i++){
		std::shared_ptr<KinectImageWriterThread> writer(new KinectImageWriterThread);
		writer->setKinectImageOutputWriter(this);
		m_writers.push_back(writer);
		m_writerThreads.push_back(std::thread(&KinectImageWriterThread::writeImagesToFile, writer, m_recordingConfiguration));
	}

	std::async(std::launch::async, &KinectRawFileWriter::waitForWriterToFinish, this);
}


void KinectRawFileWriter::stopWriting()
{
	m_running = false;
	m_imageCount = 0;
	m_checkImage.notify_all();
	writingWasStopped();
}


bool KinectRawFileWriter::pullData(ImageMeasurement& measurement)
{
	//threads pulling the data from the buffer
	std::unique_lock<std::mutex> imageLocker(m_lockImage);
	while (m_images.empty()){
		if (!m_checkImage.wait_for(imageLocker, std::chrono::milliseconds(100))){
			if (m_images.empty() && !m_running){
				return true;
			}
		}
	}
	if (m_images.empty() && !m_running){
		//buffer is empty or we were stopped
		return true;
	}

	measurement = m_images.front();
	m_images.pop();

	return false;
}


bool KinectRawFileWriter::isMaximumFramesReached()
{
	if (m_recordingConfiguration->isRecordingDurationUnLimited()){
		return false;
	}
	return m_imageCount == m_recordingConfiguration->getMaxNumberOfFrames();
}


void KinectRawFileWriter::pushImage(boost::shared_ptr<cv::Mat> imageToPush)
{
	std::unique_lock<std::mutex> imageLocker(m_lockImage);

	if (!m_running){
		m_running = false;
		m_checkImage.notify_all();
		return;
	}
	//construct the measurement
	ImageMeasurement imageMeasurement;
	imageMeasurement.image = imageToPush;
	imageMeasurement.index = m_imageCount;
	m_images.push(imageMeasurement);
	m_imageCount++;

	//check whether we captured enough frames
	if (isMaximumFramesReached()){
		stopWriting();
	}
	else{
		m_checkImage.notify_all();
	}
}

void KinectRawFileWriter::pushImagesAsync(std::vector<boost::shared_ptr<cv::Mat>> images)
{
	for (auto image : images){
		pushImageAsync(image);
	}
}

void KinectRawFileWriter::pushImageAsync(boost::shared_ptr<cv::Mat> image)
{
	if (!m_running){
		return;
	}
	std::async(std::launch::async, &KinectRawFileWriter::pushImage, this, image);
}


void KinectRawFileWriter::setRecordingConfiguration(IRecordingConfigurationPtr recordingConfiguration)
{
	m_recordingConfiguration = recordingConfiguration;
}