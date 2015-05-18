#include "stdafx.h"
#include "UncoloredOutputStreamsUpdater.h"
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

UncoloredOutputStreamsUpdater::UncoloredOutputStreamsUpdater() :
m_centerEnabled(false),
m_colorBuffer(nullptr),
m_depthBuffer(nullptr)
{
}


UncoloredOutputStreamsUpdater::~UncoloredOutputStreamsUpdater()
{
}

void UncoloredOutputStreamsUpdater::allocateClouds()
{
	m_HDFacePointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>());
	m_HDFacePointCloud->is_dense = false;
	m_HDFacePointCloud_centered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>());
	m_HDFacePointCloud_centered->is_dense = false;
	m_FaceRawPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>());
	m_FaceRawPointCloud->is_dense = false;
	m_FaceRawPointCloud_centered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud <pcl::PointXYZ>());
	m_FaceRawPointCloud_centered->is_dense = false;
}

void UncoloredOutputStreamsUpdater::setCeterEnabled(bool enable)
{
	m_centerEnabled = enable;
}

void UncoloredOutputStreamsUpdater::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer)
{
	m_colorBuffer = colorBuffer;
	m_depthBuffer = depthBuffer;
	allocateClouds();
	m_isValidFaceFrame = true;
}
void UncoloredOutputStreamsUpdater::stopFaceCollection()
{	
	if (!m_isValidFaceFrame){
		m_depthBuffer = nullptr;
		return;
	}

	//center the point clouds
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*m_HDFacePointCloud, centroid);
	Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
	Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

	pcl::transformPointCloud(*m_HDFacePointCloud, *m_HDFacePointCloud_centered, m);
	pcl::transformPointCloud(*m_FaceRawPointCloud, *m_FaceRawPointCloud_centered, m);

	//update HDFace & Raw Face Depth
	if (!cloudsUpdated.empty()){
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> updatedClouds;
		updatedClouds.push_back(m_HDFacePointCloud_centered);
		updatedClouds.push_back(m_FaceRawPointCloud_centered);
		//update viewer
		cloudsUpdated(updatedClouds);
	}

	//HDface
	if (!cloudUpdated[0].empty()){
		// update writer
		if (m_centerEnabled) cloudUpdated[0](m_HDFacePointCloud_centered);
		else cloudUpdated[0](m_HDFacePointCloud);
	}

	//Raw Face Depth
	if (!cloudUpdated[1].empty()){
		// update writer
		if (m_centerEnabled) cloudUpdated[1](m_FaceRawPointCloud_centered);
		else cloudUpdated[1](m_FaceRawPointCloud);
	}

	//create the full depth only if requested und notify
	if (!cloudUpdated[2].empty()){
		auto fullDepthBufferCloud = convertDepthBufferToPointCloud();
		// update writer
		cloudUpdated[2](fullDepthBufferCloud);
	}

	//KinectColorRaw
	if (!imageUpdated[0].empty()){
		// update writer
		cv::Mat m_colorImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_colorBuffer, cv::Mat::AUTO_STEP);
		boost::shared_ptr<cv::Mat> m_colorImagePtr(new cv::Mat());
		*m_colorImagePtr = m_colorImage.clone();
		imageUpdated[0](m_colorImagePtr);
	}

	//KinectDepthRaw
	if (!imageUpdated[1].empty()){
		// update writer
		cv::Mat m_depthImage = cv::Mat(m_depthHeight, m_depthWidth, CV_16UC1, m_depthBuffer, cv::Mat::AUTO_STEP);
		boost::shared_ptr<cv::Mat> m_depthImagePtr(new cv::Mat());
		*m_depthImagePtr = m_depthImage.clone();
		imageUpdated[1](m_depthImagePtr);
	}

	m_depthBuffer = nullptr;
	m_colorBuffer = nullptr;
	m_isValidFaceFrame = false;
}



HRESULT UncoloredOutputStreamsUpdater::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize,
	CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace)
{

	if (m_depthBuffer == nullptr){
		return -1;
	}
	//create vertices of HD Face points
	HRESULT hr = faceModel->CalculateVerticesForAlignment(faceAlignment, bufferSize, detectedHDFacePointsCamSpace);
	if (SUCCEEDED(hr)){
		//map them into color space for convex hull later
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(bufferSize, detectedHDFacePointsCamSpace, bufferSize, detectedHDFacePointsColorSpace);
	}
	if (SUCCEEDED(hr)){
		CameraSpacePoint boundingBoxPointTopLeft;
		CameraSpacePoint boundingBoxPointBottomRight;
		std::vector<cv::Point2f> hdFacePointsInCamSpaceOpenCV;

		extractFaceHDPoinCloudAndBoundingBox(bufferSize, detectedHDFacePointsCamSpace, detectedHDFacePointsColorSpace,
			boundingBoxPointTopLeft, boundingBoxPointBottomRight, hdFacePointsInCamSpaceOpenCV);


		m_isValidFaceFrame &= extractDepthCloudFromBoundingBox(boundingBoxPointTopLeft, boundingBoxPointBottomRight,
			hdFacePointsInCamSpaceOpenCV);
	}
	return hr;
}


void UncoloredOutputStreamsUpdater::extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints,
	ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeft, CameraSpacePoint& camBottomRight, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV)
{
	//init bounding box
	float bottom = FLT_MAX;
	float top = -FLT_MAX;
	float right = -FLT_MAX;
	float left = FLT_MAX;
	float back = -FLT_MAX;
	float front = FLT_MAX;
	for (int i = 0; i < bufferSize; i++){
		const auto& cameraSpacePoint = *cameraSpacePoints;
		const auto& colorSpacePoint = *colorSpacePoints;

		//copy 3D coordinates
		pcl::PointXYZ point;
		point.x = cameraSpacePoint.X;
		point.y = cameraSpacePoint.Y;
		point.z = cameraSpacePoint.Z;
		
		//get color index for convex hull raw face depth point
		int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
		int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));

		if (colorY > m_colorHeight || colorX > m_colorWidth || colorY < 0 || colorX < 0)
			continue;

		hdFacePointsInCamSpaceOpenCV.push_back(cv::Point2f(colorX, colorY));

		//calc bounding box
		bottom = std::min(point.y, bottom);
		top = std::max(point.y, top);
		right = std::max(point.x, right);
		left = std::min(point.x, left);
		back = std::max(point.z, back);
		front = std::min(point.z, front);

		m_HDFacePointCloud->push_back(point);

		cameraSpacePoints++;
		colorSpacePoints++;
	}

	//construct bounding box
	//widen the box 1cm in each direction
	top = top + 0.01;
	bottom = bottom - 0.01;
	left = left - 0.01;
	right = right + 0.01;
	back = back + 0.01;
	front = front - 0.01;

	camTopLeft.X = left;
	camTopLeft.Y = top;
	if (left < 0) camTopLeft.Z = front;
	else camTopLeft.Z = back;


	camBottomRight.X = right;
	camBottomRight.Y = bottom;
	if (right > 0) camBottomRight.Z = front;
	else camBottomRight.Z = back;
}

bool UncoloredOutputStreamsUpdater::extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeft, CameraSpacePoint camBottomRight,
	std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV)
{
	//map corners of bounding box into color space for convex hull test
	DepthSpacePoint depthTopLeftBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopLeft, &depthTopLeftBack);

	DepthSpacePoint depthBottomRightBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomRight, &depthBottomRightBack);

	if (!isValidDepthPoint(depthTopLeftBack) || !isValidDepthPoint(depthBottomRightBack)){
		return false;
	}
	
	//create convex hull with opencv 
	cv::vector<cv::Point2f> hullPoints;
	cv::convexHull(hdFacePointsInCamSpaceOpenCV, hullPoints);
	for (int x = static_cast<int>(depthTopLeftBack.X); x < static_cast<int>(depthBottomRightBack.X); x++){
		for (int y = static_cast<int>(depthTopLeftBack.Y); y < static_cast<int>(depthBottomRightBack.Y); y++){

			pcl::PointXYZ point;
			//get the depth value
			DepthSpacePoint depthPoint;
			depthPoint.X = static_cast<float>(x);
			depthPoint.Y = static_cast<float>(y);
			UINT16 depthOfCurrentPoint = m_depthBuffer[y * m_depthWidth + x];

			//map to color space for acurate face extraction in color space
			ColorSpacePoint colorPoint;
			HRESULT hr = m_pCoordinateMapper->MapDepthPointToColorSpace(depthPoint, depthOfCurrentPoint, &colorPoint);
			if (FAILED(hr)){
				continue;
			}

			//inside the convex hull of HD Face points?
			int colorPixelMidX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
			int colorPixelMidY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));
			auto result = cv::pointPolygonTest(hullPoints, cv::Point2f(colorPoint.X, colorPoint.Y), false);
			if (result < 0){
				continue;
			}

			CameraSpacePoint camPoint;
			hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthOfCurrentPoint, &camPoint);

			if (FAILED(hr)){
				continue;
			}
			//get 3D point coordinates
			if ((0 <= colorPixelMidX) && (colorPixelMidX < m_colorWidth) && (0 <= colorPixelMidY) && (colorPixelMidY < m_colorHeight)){
				point.x = camPoint.X;
				point.y = camPoint.Y;
				point.z = camPoint.Z;
			}

			if (point.x < camTopLeft.X || point.x > camBottomRight.X)
				continue;
			
			m_FaceRawPointCloud->push_back(point);

		}
	}

	return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr  UncoloredOutputStreamsUpdater::convertDepthBufferToPointCloud()
{
	//create the full depth buffer cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullDepthBufferCloud(new pcl::PointCloud<pcl::PointXYZ>);
	HRESULT hr;

	//iterate over the depth buffer
	for (int y = 0; y < m_depthHeight; y++){
		for (int x = 0; x < m_depthWidth; x++){
			pcl::PointXYZ point;

			//calc depth value
			DepthSpacePoint depthPoint;
			depthPoint.X = static_cast<float>(x);
			depthPoint.Y = static_cast<float>(y);
			UINT16 depthOfCurrentPoint = m_depthBuffer[y * m_depthWidth + x];
	
			//calc 3D Coordinate of depth point
			CameraSpacePoint camPoint;
			hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthOfCurrentPoint, &camPoint);
	
			if (FAILED(hr) || !isValidCamSpacePoint(camPoint)){
				continue;
			}
			//copy coordinates and add to the cloud
			point.x = camPoint.X;
			point.y = camPoint.Y;
			point.z = camPoint.Z;
				
			fullDepthBufferCloud->push_back(point);
		}
	}
	return fullDepthBufferCloud;
}