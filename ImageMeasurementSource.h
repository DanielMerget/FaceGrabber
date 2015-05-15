#pragma once
#include "stdafx.h"
#include <opencv2/imgproc/imgproc.hpp>

/**
* \class	ImageMeasurement
*
* \brief	An image measurement encapsulating an image and the index when it was
* 			captured.
*/

class ImageMeasurement{
public:

	/** \brief	The image of the measurment. */
	boost::shared_ptr<cv::Mat> image;

	/** \brief	Zero-based index when the measurement was taken. */
	int index;
};

/**
* \class	ImageMeasurementSource
*
* \brief	A image measurement source allows to pull ImageMeasurements.
*/

class ImageMeasurementSource
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

	virtual bool pullData(ImageMeasurement& measurement) = 0;
};

