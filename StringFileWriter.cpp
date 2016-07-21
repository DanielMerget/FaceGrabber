#include "stdafx.h"
#include "StringFileWriter.h"


StringFileWriterThread::StringFileWriterThread()
{
}


StringFileWriterThread::~StringFileWriterThread()
{
}

void StringFileWriterThread::writeStringsToFile(StringFileRecordingConfigurationPtr recordingConfiguration)
{
		//collect important writing information
	auto recordingFileFormat =	recordingConfiguration->getRecordFileFormat();
	auto filePath =				recordingConfiguration->getFullRecordingPathString();
	auto fileName =				recordingConfiguration->getFileNameString();
	std::string fileFormatExtension;

	switch (recordingFileFormat)
	{
		case TXT:
			fileFormatExtension = ".txt";
			break;
		default:
			break;
	}

	while(true){
		StringMeasurement measurement;

		measurement.strings = nullptr;
		measurement.index = 0;
	

		bool isDone = m_source->pullData(measurement);
		if (!measurement.strings){
			OutputDebugString(L"writer finished cloud null");
			return;
		}
		std::stringstream outputFileWithPath;
		outputFileWithPath << filePath << "\\" << fileName << "_" << std::setfill('0') << std::setw(6) << measurement.index << fileFormatExtension;

		
		FILE *fp = nullptr;


		
		fp = fopen(outputFileWithPath.str().c_str(),"w+");
		if(fp != nullptr)
		{
			fwrite (measurement.strings->data() , sizeof(char), measurement.strings->size(), fp);
			//fprintf (fp, "%s",tmp.c_str());
			//fprintf (fp, "%s\n","test");
		}

	
		fclose(fp);

		if (isDone){
			OutputDebugString(L"writer finished isDone");
			return;
		}
	}


}

void StringFileWriterThread::setStringsOutputWriter(StringMeasurementSource* source)
{
	m_source = source;
}



StringFileWriter::StringFileWriter() :
	m_running(false),
	m_imageCount(0),
	m_stringStreams(),
	m_writerThreads()
{
}


StringFileWriter::~StringFileWriter()
{
	m_running = false;
	for (auto& thread : m_writerThreads){
		if (thread.joinable()){
			thread.join();
		}
	}
}


void StringFileWriter::waitForWriterToFinish()
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




void StringFileWriter::startWriting()
{
	m_running = true;
	m_imageCount = 0;
	auto threadsToStartCount = m_recordingConfiguration->getThreadCountToStart();
	//create the writer objects and threads
	for (int i = 0; i < threadsToStartCount; i++){
		std::shared_ptr<StringFileWriterThread> writer(new StringFileWriterThread);
		writer->setStringsOutputWriter(this);
		m_writers.push_back(writer);
		m_writerThreads.push_back(std::thread(&StringFileWriterThread::writeStringsToFile, writer, m_recordingConfiguration));
	}



	std::async(std::launch::async, &StringFileWriter::waitForWriterToFinish, this);
}


void StringFileWriter::stopWriting()
{
	m_running = false;
	m_imageCount = 0;
	m_checkString.notify_all();
	writingWasStopped();
}


bool StringFileWriter::pullData(StringMeasurement& measurement)
{
	//threads pulling the data from the buffer
	std::unique_lock<std::mutex> stringFileLocker(m_lockString);
	while (m_stringStreams.empty()){
		if (!m_checkString.wait_for(stringFileLocker, std::chrono::milliseconds(100))){
			if (m_stringStreams.empty() && !m_running){
				return true;
			}
		}
	}
	if (m_stringStreams.empty() && !m_running){
		//buffer is empty or we were stopped
		return true;
	}

	measurement = m_stringStreams.front();
	m_stringStreams.pop();

	return false;
}


bool StringFileWriter::isMaximumFramesReached()
{
	if (m_recordingConfiguration->isRecordingDurationUnLimited()){
		return false;
	}
	return m_imageCount == m_recordingConfiguration->getMaxNumberOfFrames();
}


void StringFileWriter::pushStrings(std::shared_ptr<std::string> StringsToPush)
{
	std::unique_lock<std::mutex> stringLocker(m_lockString);

	if (!m_running){
		m_running = false;
		m_checkString.notify_all();
		return;
	}
	//construct the measurement
	StringMeasurement stringsMeasurement;
	stringsMeasurement.strings = StringsToPush;
	stringsMeasurement.index = m_imageCount;
	m_stringStreams.push(stringsMeasurement);
	m_imageCount++;

	//check whether we captured enough frames
	if (isMaximumFramesReached()){
		stopWriting();
	}
	else{
		m_checkString.notify_all();
	}
}

void StringFileWriter::pushStringFilesAsync(std::vector<std::shared_ptr<std::string>> stringStreams)
{
	for (auto strings : stringStreams){
		pushStringFileAsync(strings);
	}
}

void StringFileWriter::pushStringFileAsync(std::shared_ptr<std::string> strings)
{
	if (!m_running){
		return;
	}
	std::async(std::launch::async, &StringFileWriter::pushStrings, this, strings);
}


void StringFileWriter::setRecordingConfiguration(StringFileRecordingConfigurationPtr recordingConfiguration)
{
	m_recordingConfiguration = recordingConfiguration;
}