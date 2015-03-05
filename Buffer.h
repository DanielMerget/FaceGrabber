#pragma once
#include <mutex>
#include <condition_variable>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/signals2.hpp>
class Buffer
{
public:
	Buffer();
	~Buffer();

	void setBufferSize(int bufferSize);
	int getBufferSize();
	void resetBuffer();
	void pushData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr newData, int index);
	
	bool isDataAvailable();
	std::shared_ptr<boost::signals2::signal<void(void)>> dataReady;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pullData();

	bool isBufferAtIndexSet(const int index);

	
	void enableBuffer();
	void disableBuffer();

	void setProducerFinished();
private:

	void printMessage(std::string msg);
	std::shared_ptr<std::mutex> m_printMutex;
	std::shared_ptr<std::mutex>				 m_cloudBufferMutex;
	std::shared_ptr<std::condition_variable> m_cloudBufferFree;
	std::shared_ptr<std::condition_variable> m_cloudBufferUpdated;

	int m_bufferingActive;
	int m_pullDataPosition;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_cloudBuffer;
	int	m_bufferFillLevel;
	bool m_producerFinished;
};

