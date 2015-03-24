#pragma once
#include "stdafx.h"
#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class NonColouredOutputStreamsUpdater :
	public OutputStreamsUpdaterStragedy
{
public:
	NonColouredOutputStreamsUpdater();
	~NonColouredOutputStreamsUpdater();


	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace);


	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>)> cloudsUpdated;

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)> cloudUpdated[3];

	void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	void stopFaceCollection();

private:

	void allocateClouds();

	bool extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	void extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthBufferToPointCloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_HDFacePointCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_FaceRawPointCloud;

	bool							m_isValidFaceFrame;
	UINT16*	m_depthBuffer;

};

