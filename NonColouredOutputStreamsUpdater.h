#pragma once
#include "stdafx.h"

#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
class NonColouredOutputStreamsUpdater :
	public OutputStreamsUpdaterStragedy
{
public:
	NonColouredOutputStreamsUpdater();
	~NonColouredOutputStreamsUpdater();


	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer);


	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>)> cloudsUpdated;

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)> cloudUpdated[3];
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> cloudUpdated;
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> depthCloudUpdated;

private:

	

	bool extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer, pcl::PointCloud<pcl::PointXYZ>::Ptr);

	pcl::PointCloud<pcl::PointXYZ>::Ptr extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer);

	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthBufferToPointCloud(UINT16* depthBuffer);
};

