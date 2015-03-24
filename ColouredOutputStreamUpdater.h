#pragma once
#include "stdafx.h"

#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <limits>

/**
 * \class	ColouredOutputStreamUpdater
 *
 * \brief	The ColouredOutputStreamUpdater takes the buffers and face information provided by the
 * 			updateOutputStreams, extracts the HDFace, FaceRaw and FullRawDepth point clouds and publishes
 * 			them via cloudsUpdated or cloudUpdated. Beforehand the ColouredOutputStreamUpdater has to be
 * 			initialied with the coordinatemapper provided by the kinect and information about the resolution
 * 			of the update-streams.
 */

class ColouredOutputStreamUpdater :
	public OutputStreamsUpdaterStragedy
{
public:

	ColouredOutputStreamUpdater();
	~ColouredOutputStreamUpdater();

	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace);

	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>)> cloudsUpdated;

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)> cloudUpdated[3];

	void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	void stopFaceCollection();

	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

private:
	void allocateClouds();
	bool extractColoredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV);

	void extractColoredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDepthBufferToPointCloud();


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_HDFacePointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_FaceRawPointCloud;

	std::vector<UINT16>				m_pDepthVisibilityTestMap;
	std::vector<ColorSpacePoint>	m_pColorCoordinates;
	bool							m_isValidFaceFrame;

	RGBQUAD* m_colorBuffer;
	UINT16*	m_depthBuffer;
};
