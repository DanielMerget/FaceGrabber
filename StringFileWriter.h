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
//#include "ImageMeasurementSource.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "StringFileRecordingConfiguration.h"

class StringMeasurement{
public:

	/** \brief	The image of the measurment. */
	std::shared_ptr<std::string> strings;

	/** \brief	Zero-based index when the measurement was taken. */
	int index;
};


/**
* \class	StringMeasurementSource
*
* \brief	A image measurement source allows to pull ImageMeasurements.
*/


class StringMeasurementSource
{
public:

	/**
	* \fn	virtual bool ImageMeasurementSource::pullData(ImageMeasurement& measurement) = 0;
	*
	* \brief	Fills the given measurement with most recent data.
	*
	* \param [in,out]	measurement	The measurement.
	*
	* \return	true the source is empty or not running, otherwise false
	*/

	virtual bool pullData(StringMeasurement& measurement) = 0;
};


class StringFileWriterThread
{
public:

	StringFileWriterThread();
	~StringFileWriterThread();

	/**
	 * \fn	void KinectFileWriterThread::writeCloudsToFile(IRecordingConfigurationPtr recordingConfiguration);
	 *
	 * \brief	Writes point clouds to the hard disc as specified in the recording configuration.
	 * 			Poit Clouds are pulled from the CloudMeasurementSource.
	 *
	 * \param	recordingConfiguration	The recording configuration.
	 */

	void writeStringsToFile(StringFileRecordingConfigurationPtr recordingConfiguration);

	/**
	 * \fn	void KinectFileWriterThread::setKinectCloudOutputWriter(CloudMeasurementSource<PointCloudType>* source);
	 *
	 * \brief	Sets kinect cloud output writer for pulling the CloudMeasurements to be written.
	 *
	 * \param [in,out]	source	If non-null, source for the.
	 */

	void setStringsOutputWriter(StringMeasurementSource* source);


	/** \brief	Source for CloudMeasurements. */
	StringMeasurementSource* m_source;
};




/**
* \class	StringFileWriter
*
* \brief	A kinect raw data output writer which writes color and depth raw images to the hard disk according
* 			to the set Configuration. Writing can be done multi-threaded.
*/


class StringFileWriter : StringMeasurementSource
{
public:

	StringFileWriter();

	~StringFileWriter();

	/**
	* \fn	void StringFileWriter::pushStringFileAsync(boost::shared_ptr<BYTE>);
	*
	* \brief	Pushes an string file into the buffer holding the strings to be written to the hard drive.
	*
	* \param	strings	The strings to be pushed.
	*/

	void pushStringFileAsync(std::shared_ptr<std::string> string);

	/**
	* \fn	void StringFileWriter::void pushStringFileAsync(std::vector<boost::shared_ptr<BYTE>> strings);
	*
	* \brief	Pushes a list of strings into the buffer holding the strings to be written to the hard drive.
	*
	* \param	strings	The strings.
	*/

	void pushStringFilesAsync(std::vector<std::shared_ptr<std::string>> stringStreams);

	/**
	* \fn	void StringFileWriter::startWriting();
	*
	* \brief	Triggers the start of writing the buffered strings.
	*/

	void startWriting();

	/**
	* \fn	void StringFileWriter::stopWriting();
	*
	* \brief	Triggers the stop of writing the buffered string.
	*/

	void stopWriting();

	

	/**
	* \fn	void StringFileWriter::setRecordingConfiguration(IImageRecordingConfigurationPtr recordingConfiguration);
	*
	* \brief	Sets recording configuration for the writing.
	*
	* \param	recordingConfiguration	The recording configuration.
	*/

	void setRecordingConfiguration(StringFileRecordingConfigurationPtr recordingConfiguration);

	/**
	* \fn	virtual bool ImageMeasurementSource::pullData(PointCloudMeasurement& measurement) = 0;
	*
	* \brief	Fills the given measurement with most recent data.
	*
	* \param [in,out]	measurement	The measurement.
	*
	* \return	true the source is empty or not running, otherwise false
	*/
	bool pullData(StringMeasurement& measurement);

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

	void pushStrings(std::shared_ptr<std::string> StringsToPush);



	/** \brief	The started writer threads. */
	std::vector<std::thread> m_writerThreads;


	/** \brief	The writer objectes associated to the writer threads. */
	std::vector<std::shared_ptr<StringFileWriterThread>> m_writers;


	/** \brief	The buffered images. */
	std::queue<StringMeasurement> m_stringStreams;


	/** \brief	The check for existance of images in the buffer. */
	std::condition_variable m_checkString;


	/** \brief	The lock for the image buffer. */
	std::mutex m_lockString;


	/** \brief	The recording configuration. */
	StringFileRecordingConfigurationPtr m_recordingConfiguration;


	/** \brief	true if writing is currently running. */
	bool m_running;


	/** \brief	Number of buffered images. */
	int m_imageCount;
};