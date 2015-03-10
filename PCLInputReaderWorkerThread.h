#pragma once
#include <memory>
#include "Buffer.h"
#include "PlaybackConfiguration.h"

class PCLInputReaderWorkerThread 
{
public:
	PCLInputReaderWorkerThread();
	~PCLInputReaderWorkerThread();

	void readCloudData(const int index, const int step, std::vector<CloudFile> cloudFilesToPlay, RecordingFileFormat format);
	void stopReading();
	void setBuffer(std::shared_ptr<Buffer> buffer);

private:
	void printMessage(std::string msg);
	bool m_isPlaybackRunning;
	std::shared_ptr<Buffer> m_buffer;
};

