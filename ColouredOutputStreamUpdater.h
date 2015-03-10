#pragma once
#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ColouredOutputStreamUpdater :
	public OutputStreamsUpdaterStragedy
{
public:
	ColouredOutputStreamUpdater();
	~ColouredOutputStreamUpdater();
	
	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace, 
		ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer);

	boost::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>)> cloudsUpdated;

	boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> cloudUpdated[2];
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> depthCloudUpdated;

private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractColouredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack, 
		std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractClolouredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints, 
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer);
};

