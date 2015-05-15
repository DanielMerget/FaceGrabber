#pragma once
#include "stdafx.h"
#include <memory>
#include "ImageRecordingConfiguration.h"
#include "ImageMeasurementSource.h"
#include "opencv2\highgui\highgui.hpp"

/**
* \class	KinectImageWriterThread
*
* \brief	A kinect file writer thread supposed to write PortableAnyMap files to the hard disk.
*			ImageMeasurements are pulled from the set source.
*
* \tparam	PixelType	Type of the image.
*/

class KinectImageWriterThread
{
public:

	KinectImageWriterThread();
	~KinectImageWriterThread();

	/**
	* \fn	void KinectImageWriterThread::writeImagesToFile(IRecordingConfigurationPtr recordingConfiguration);
	*
	* \brief	Writes images to the hard disc as specified in the recording configuration.
	* 			Images are pulled from the ImageMeasurementSource.
	*
	* \param	recordingConfiguration	The recording configuration.
	*/

	void writeImagesToFile(IImageRecordingConfigurationPtr recordingConfiguration);

	/**
	* \fn	void KinectImageWriterThread::setKinectImageOutputWriter(ImageMeasurementSource* source);
	*
	* \brief	Sets kinect image writer for pulling the ImageMeasurements to be written.
	*
	* \param [in,out]	source	If non-null, source for the.
	*/

	void setKinectImageOutputWriter(ImageMeasurementSource* source);


	/** \brief	Source for ImageMeasurements. */
	ImageMeasurementSource* m_source;
};
