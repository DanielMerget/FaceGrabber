#pragma once
#include "stdafx.h"

#include "MessageRouterHelper.h"
#include <windowsx.h>
#include <thread>
#include "resource.h"
#include "PlaybackConfiguration.h"
#include "BufferSynchronizer.h"
#include "SimpleRecordingConfiguration.h"
#include "KinectCloudFileWriter.h"
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

class ConvertTabHandler : public MessageRouterHelper
{
public:
	ConvertTabHandler();
	~ConvertTabHandler();
	
private:

	/**
	 * \fn	void ConvertTabHandler::onCreate();
	 *
	 * \brief	Creates the user interface and sets preconfigured values.
	 */

	void onCreate();

	/**
	 * \fn	void ConvertTabHandler::startFileConversion();
	 *
	 * \brief	Triggers the file conversion.
	 */
	void startFileConversion();

	/**
	 * \fn	void ConvertTabHandler::playbackConfigurationChanged();
	 *
	 * \brief	Playback configuration changed callback.
	 */

	void playbackConfigurationChanged();

	/**
	 * \fn	void ConvertTabHandler::recordingConfigurationChanged();
	 *
	 * \brief	Recording configuration changed callback.
	 */
	void recordingConfigurationChanged();

	/**
	 * \fn	void ConvertTabHandler::notifyWriterFinished();
	 *
	 * \brief	Callback for the writer finish signal.
	 */

	void notifyWriterFinished();

	/**
	 * \fn	void ConvertTabHandler::updateWriterStatus(std::wstring newStatus);
	 *
	 * \brief	Callback: Updates the writer status described by newStatus.
	 *
	 * \param	newStatus	The new status.
	 */
	void updateWriterStatus(std::wstring newStatus);

	/**
	 * \fn	void ConvertTabHandler::updateReaderStatus(std::wstring newStatus);
	 *
	 * \brief	Callback: Updates the reader status described by newStatus.
	 *
	 * \param	newStatus	The new status.
	 */

	void updateReaderStatus(std::wstring newStatus);

	void onSelectionChanged(WPARAM wParam, LPARAM handle);

	void onButtonClicked(WPARAM wParam, LPARAM handle);

	void onEditBoxeChanged(WPARAM wParam, LPARAM handle);

	/**
	 * \fn	void ConvertTabHandler::initColoredConversionPipeline();
	 *
	 * \brief	Initialises the colored conversion pipeline.
	 */
	void initColoredConversionPipeline();

	/**
	 * \fn	void ConvertTabHandler::initNonColoredConversionPipeline();
	 *
	 * \brief	Initialises the non colored conversion pipeline.
	 */
	void initNonColoredConversionPipeline();

	/** \brief	The playback configuration. */
	PlaybackConfigurationPtr			m_playbackConfiguration;

	/** \brief	The recording configuration. */
	SimpleRecordingConfigurationPtr		m_recordingConfiguration;

	/** \brief	true to enable, false to disable the color writing. */
	bool										m_enableColor;

	/** \brief	The color cloud reader. */
	std::shared_ptr<ColoredCloudInputReader>	m_colorCloudReader;

	/** \brief	Buffer for color point clouds. */
	std::shared_ptr<ColorBuffer>				m_colorBuffer;

	/** \brief	The color buffer synchronizer. */
	std::shared_ptr<ColorBufferSynchronizer>	m_colorBufferSynchronizer;

	/** \brief	The colored clouds writer. */
	std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZRGB>>	m_colorWriter;

	
	/** \brief	The non color cloud reader. */
	std::shared_ptr<NonColoredCloudInputReader> m_nonColorCloudReader;

	/** \brief	Buffer for non color clouds. */
	std::shared_ptr<NonColorBuffer>				m_nonColorBuffer;

	/** \brief	The non color buffer synchronizer. */
	std::shared_ptr<NonColorBufferSynchronizer>		m_nonColorBufferSynchronizer;


	/** \brief	The non colored cloud writer. */
	std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZ>>	m_nonColorWriter;

	/** \brief	The color buffer synchronizer thread. */
	std::thread	m_colorBufferSynchronizerThread;

	/** \brief	The non color buffer synchronizer thread. */
	std::thread	m_nonColorBufferSynchronizerThread;
};
