#pragma once
#include "stdafx.h"

/**
 * \class	PointCloudMeasurement
 *
 * \brief	A point cloud measurement encapsulating a point cloud and the index when it was
 * 			captured.
 *
 * \tparam	MeasurementCloudType	Type of the measurement cloud type.
 */

template < typename MeasurementCloudType >
class PointCloudMeasurement{
public:

	/** \brief	The point cloud of the measurment. */
	boost::shared_ptr<const pcl::PointCloud<MeasurementCloudType>> cloud;
	
	/** \brief	Zero-based index when the measurement was taken. */
	int index;
};

/**
 * \class	CloudMeasurementSource
 *
 * \brief	A cloud measurement source allows to pull PointCloudMeasurements.
 *
 * \tparam	PointCloudType	Type of the point cloud type.
 */

template < class PointCloudType >
class CloudMeasurementSource
{
public:

	/**
	 * \fn	virtual bool CloudMeasurementSource::pullData(PointCloudMeasurement<PointCloudType>& measurement) = 0;
	 *
	 * \brief	Fills the given measurement with most recent data.
	 *
	 * \param [in,out]	measurement	The measurement.
	 *
	 * \return	true the source is empty or not running, otherwise false
	 */

	virtual bool pullData(PointCloudMeasurement<PointCloudType>& measurement) = 0;
};

