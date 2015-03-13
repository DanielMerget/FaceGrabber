#pragma once
#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <limits>


class ColouredOutputStreamUpdater :
	public OutputStreamsUpdaterStragedy
{
public:
	
	ColouredOutputStreamUpdater();
	~ColouredOutputStreamUpdater();
	
	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace, 
		ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer);

	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>)> cloudsUpdated;

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)> cloudUpdated[3];
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> depthCloudUpdated;


	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

private:

	bool extractColouredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractColouredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack, 
	//	std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractClolouredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints, 
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDepthBufferToPointCloud(RGBQUAD* colorBuffer, UINT16* depthBuffer);

	
	
	std::vector<UINT16>				m_pDepthVisibilityTestMap;
	std::vector<ColorSpacePoint>	m_pColorCoordinates;
};

