#pragma once
#include <windowsx.h>
#include "stdafx.h"
#include <thread>
#include "resource.h"
#include "PlaybackConfiguration.h"
#include "BufferSynchronizer.h"
#include "SimpleRecordingConfiguration.h"
#include "KinectCloudOutputWriter.h"
#include "Buffer.h"
#include "PCLInputReader.h"
#include <memory>

typedef PCLInputReader<pcl::PointXYZRGB> ColoredCloudInputReader;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBCloud;
typedef Buffer<PointXYZRGBCloud> ColorBuffer;
typedef BufferSynchronizer<PointXYZRGBCloud> ColorBufferSynchronizer;

typedef PCLInputReader<pcl::PointXYZ>			NonColoredCloudInputReader;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr		PointXYZCloud;
typedef Buffer<PointXYZCloud>					NonColorBuffer;
typedef BufferSynchronizer<PointXYZCloud>		NonColorBufferSynchronizer;

class ConvertTabHandler
{
public:
	ConvertTabHandler();
	~ConvertTabHandler();

	
	LRESULT CALLBACK	DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
	static LRESULT CALLBACK	MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);;

	void onCreate(WPARAM wParam, LPARAM);
	
	void processUIMessage(WPARAM wParam, LPARAM);

	
private:
	
	void startFileConversion();
	void playbackConfigurationChanged();
	void recordingConfigurationChanged();

	void notifyWriterFinished();
	
	void updateWriterStatus(std::wstring newStatus);
	void updateReaderStatus(std::wstring newStatus);

	void onSelectionChanged(WPARAM wParam, LPARAM handle);
	void onButtonClicked(WPARAM wParam, LPARAM handle);
	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	
	void initColoredConversionPipeline();
	void nonColoredConversionPipeline();
	
	PCLInputReader<pcl::PointXYZ>		reader;
	PlaybackConfigurationPtr			m_playbackConfiguration;
	SimpleRecordingConfigurationPtr		m_recordingConfiguration;
	bool								m_enableColor;

	std::shared_ptr<ColoredCloudInputReader>	m_colorCloudReader;
	std::shared_ptr<ColorBuffer>				m_colorBuffer;
	std::shared_ptr<ColorBufferSynchronizer>	m_colorBufferSynchronizer;
	std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZRGB>>	m_colorWriter;

	
	std::shared_ptr<NonColoredCloudInputReader> m_nonColorCloudReader;
	std::shared_ptr<NonColorBuffer>				m_nonColorBuffer;
	std::shared_ptr<NonColorBufferSynchronizer>		m_nonColorBufferSynchronizer;
	std::shared_ptr<KinectCloudOutputWriter<pcl::PointXYZ>>	m_nonColorWriter;

	std::thread									m_colorBufferSynchronizerThread;
	std::thread									m_nonColorBufferSynchronizerThread;
	HWND										m_hWnd;
};
