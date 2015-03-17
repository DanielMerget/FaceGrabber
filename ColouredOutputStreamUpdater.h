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
		ColorSpacePoint* detectedHDFacePointsColorSpace);

	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>)> cloudsUpdated;

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)> cloudUpdated[3];
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> depthCloudUpdated;

	void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	void stopFaceCollection();

	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

private:

	bool extractColoredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV);

	void extractColoredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints, 
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	void convertDepthBufferToPointCloud();

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_HDFacePointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_FaceRawPointCloud;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_fullRawPointCloud;
	std::vector<UINT16>				m_pDepthVisibilityTestMap;
	std::vector<ColorSpacePoint>	m_pColorCoordinates;
	bool							m_isValidFaceFrame;

	RGBQUAD* m_colorBuffer;
	UINT16*	m_depthBuffer;
};

