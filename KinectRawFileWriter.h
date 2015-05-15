#pragma once
#include "stdafx.h"
#include <queue>
#include <condition_variable>
#include <mutex>

#include <thread>
#include <vector>
#include <future>
#include "RecordingConfiguration.h"
#include "KinectImageWriterThread.h"
#include "ImageMeasurementSource.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
* \class	KinectRawFileWriter
*
* \brief	A kinect raw data output writer which writes color and depth raw images to the hard disk according
* 			to the set Configuration. Writing can be done multi-threaded.
*/


class KinectRawFileWriter : ImageMeasurementSource
{
public:

	KinectRawFileWriter();

	~KinectRawFileWriter();

	/**
	* \fn	void KinectRawFileWriter::pushImageAsync(boost::shared_ptr<cv::Mat> image);
	*
	* \brief	Pushes an image into the buffer holding the images to be written to the hard drive.
	*
	* \param	image	The image to be pushed.
	*/

	void pushImageAsync(boost::shared_ptr<cv::Mat> image);

	/**
	* \fn	void KinectRawFileWriter::void pushImagesAsync(std::vector<boost::shared_ptr<cv::Mat>> images);
	*
	* \brief	Pushes a list of images into the buffer holding the images to be written to the hard drive.
	*
	* \param	images	The images.
	*/

	void pushImagesAsync(std::vector<boost::shared_ptr<cv::Mat>> images);

	/**
	* \fn	void KinectRawFileWriter::startWriting();
	*
	* \brief	Triggers the start of writing the buffered images.
	*/

	void startWriting();

	/**
	* \fn	void KinectRawFileWriter::stopWriting();
	*
	* \brief	Triggers the stop of writing the buffered images.
	*/

	void stopWriting();

	/**
	* \fn	void KinectRawFileWriter::setRecordingConfiguration(IImageRecordingConfigurationPtr recordingConfiguration);
	*
	* \brief	Sets recording configuration for the writing.
	*
	* \param	recordingConfiguration	The recording configuration.
	*/

	void setRecordingConfiguration(IImageRecordingConfigurationPtr recordingConfiguration);

	/**
	* \fn	virtual bool ImageMeasurementSource::pullData(PointCloudMeasurement& measurement) = 0;
	*
	* \brief	Fills the given measurement with most recent data.
	*
	* \param [in,out]	measurement	The measurement.
	*
	* \return	true the source is empty or not running, otherwise false
	*/
	bool pullData(ImageMeasurement& measurement);

	/** \brief	The signal notfying that the writer finished. */
	boost::signals2::signal<void()> writingFinished;

	/** \brief	The signal notfying about changes of the writing status. */
	boost::signals2::signal<void(std::wstring)> updateStatus;


	/** \brief	The signal notifying about the stop of the writing by the user. */
	boost::signals2::signal<void(void)> writingWasStopped;
private:

	/**
	* \fn	bool KinectRawFileWriter::isMaximumFramesReached();
	*
	* \brief	Query whether the maxmimum of frames was reached.
	*
	* \return	true if limit of frames reached, false if not.
	*/

	bool isMaximumFramesReached();

	/**
	* \fn	void KinectRawFileWriter::waitForWriterToFinish();
	*
	* \brief	Method supposed for a thread to wait for all writer threads to finish
	* 			in order to update the status accordingly.
	*/

	void waitForWriterToFinish();

	/**
	* \fn	void KinectRawFileWriter::pushImage(boost::shared_ptr<cv::Mat> imageToPush);
	*
	* \brief	Arctually pushes an image to the buffer in a blocking way.
	*
	* \param	imageToPush	The image.
	*/

	void pushImage(boost::shared_ptr<cv::Mat> imageToPush);



	/** \brief	The started writer threads. */
	std::vector<std::thread> m_writerThreads;


	/** \brief	The writer objectes associated to the writer threads. */
	std::vector<std::shared_ptr<KinectImageWriterThread>> m_writers;


	/** \brief	The buffered images. */
	std::queue<ImageMeasurement> m_images;


	/** \brief	The check for existance of images in the buffer. */
	std::condition_variable m_checkImage;


	/** \brief	The lock for the image buffer. */
	std::mutex m_lockImage;


	/** \brief	The recording configuration. */
	IImageRecordingConfigurationPtr m_recordingConfiguration;


	/** \brief	true if writing is currently running. */
	bool m_running;


	/** \brief	Number of buffered images. */
	int m_imageCount;
};