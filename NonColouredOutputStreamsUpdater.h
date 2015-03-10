#pragma once
#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
class NonColouredOutputStreamsUpdater :
	public OutputStreamsUpdaterStragedy
{
public:
	NonColouredOutputStreamsUpdater();
	~NonColouredOutputStreamsUpdater();


	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer);


	boost::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>)> cloudsUpdated;

	boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> cloudUpdated[3];
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> cloudUpdated;
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> depthCloudUpdated;

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer);
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer);
};

