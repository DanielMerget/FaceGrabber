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
		ColorSpacePoint* detectedHDFacePointsColorSpace);


	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>)> cloudsUpdated;

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)> cloudUpdated[3];
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> cloudUpdated;
	//boost::signal<void(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)> depthCloudUpdated;

	void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	void stopFaceCollection();

	/**
	* \fn	void NonColouredOutputStreamsUpdater::initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);
	*
	* \brief	Initializes the NonColouredOutputStreamsUpdater.
	*
	* \param [in]	m_pCoordinateMapper	Kinect coordinate mapper.
	* \param	depthWidth				   	Width of the depth.
	* \param	depthHeight				   	Height of the depth.
	* \param	colorWidth				   	Width of the color.
	* \param	colorHeight				   	Height of the color.
	*/
	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);
private:

	

	bool extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	void extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	void convertDepthBufferToPointCloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_HDFacePointCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_FaceRawPointCloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_fullRawPointCloud;
	bool							m_isValidFaceFrame;
	UINT16*	m_depthBuffer;

};

