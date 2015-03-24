#pragma once
#include "stdafx.h"
#include <memory>
#include "RecordingConfiguration.h"
#include "CloudMeasurementSource.h"

/**
 * \class	KinectFileWriterThread
 *
 * \brief	A kinect file writer thread supposed to write Point Cloud files in the given generates
 * 			to the hard disk. CloudMeasurements are pulled from the set source.
 *
 * \tparam	PointCloudType	Type of the point cloud type.
 */

template < class PointCloudType >
class KinectFileWriterThread
{
public:

	KinectFileWriterThread();
	~KinectFileWriterThread();

	/**
	 * \fn	void KinectFileWriterThread::writeCloudsToFile(IRecordingConfigurationPtr recordingConfiguration);
	 *
	 * \brief	Writes point clouds to the hard disc as specified in the recording configuration.
	 * 			Poit Clouds are pulled from the CloudMeasurementSource.
	 *
	 * \param	recordingConfiguration	The recording configuration.
	 */

	void writeCloudsToFile(IRecordingConfigurationPtr recordingConfiguration);

	/**
	 * \fn	void KinectFileWriterThread::setKinectCloudOutputWriter(CloudMeasurementSource<PointCloudType>* source);
	 *
	 * \brief	Sets kinect cloud output writer for pulling the CloudMeasurements to be written.
	 *
	 * \param [in,out]	source	If non-null, source for the.
	 */

	void setKinectCloudOutputWriter(CloudMeasurementSource<PointCloudType>* source);


	/** \brief	Source for CloudMeasurements. */
	CloudMeasurementSource<PointCloudType>* m_source;
};

