#pragma once
#include "stdafx.h"


template < typename MeasurementCloudType >
class PointCloudMeasurement{
public:
	boost::shared_ptr<const pcl::PointCloud<MeasurementCloudType>> cloud;
	int index;
};

template < class PointCloudType >
class CloudMeasurementSource
{
public:
	virtual bool pullData(PointCloudMeasurement<PointCloudType>& measurement) = 0;
};

