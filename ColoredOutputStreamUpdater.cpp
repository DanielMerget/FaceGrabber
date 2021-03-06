
#include "stdafx.h"
#include "ColoredOutputStreamUpdater.h"
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <future>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/imgproc/imgproc.hpp>


#include <atlstr.h>
#include <cmath>

ColoredOutputStreamUpdater::ColoredOutputStreamUpdater() :
m_colorBuffer(nullptr),
m_centerEnabled(false),
m_depthBuffer(nullptr),
m_alignedDepthBuffer(nullptr),
m_infraredBuffer(nullptr)
{
}

static const int            PATCHDIVISIONSHIFT = 2;
static const UINT16         VISIBILITY_MAX_THRESHHOLD = 50;
ColoredOutputStreamUpdater::~ColoredOutputStreamUpdater()
{
}


void ColoredOutputStreamUpdater::initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight)
{
	OutputStreamsUpdaterStragedy::initialize(m_pCoordinateMapper, depthWidth, depthHeight, colorWidth, colorHeight);
	m_pDepthVisibilityTestMap = std::vector<UINT16>((colorWidth >> PATCHDIVISIONSHIFT) * (colorHeight >> PATCHDIVISIONSHIFT));
	m_pColorCoordinates = std::vector<ColorSpacePoint>(depthWidth * depthHeight);
}

void ColoredOutputStreamUpdater::allocateClouds()
{
	
	m_pFiveKeyPoints =  std::shared_ptr<std::string> (new std::string(""));
	m_HDFacePointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_HDFacePointCloud->is_dense = false;
	m_HDFacePointCloud_centered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_HDFacePointCloud_centered->is_dense = false;
	m_FaceRawPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_FaceRawPointCloud->is_dense = false;
	m_FaceRawPointCloud_centered = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_FaceRawPointCloud_centered->is_dense = false;
	m_HDFacePointCloud2D = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_HDFacePointCloud2D->is_dense = false;
}

void ColoredOutputStreamUpdater::setCeterEnabled(bool enable)
{
	m_centerEnabled = enable;
}

void ColoredOutputStreamUpdater::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer,UINT16* alignedDepthBuffer,RGBQUAD* infraredBuffer,RGBQUAD* alignedInfraredBuffer)
{
	allocateClouds();
	m_isValidFaceFrame = true;

	m_colorBuffer = colorBuffer;
	m_depthBuffer = depthBuffer;
	m_alignedDepthBuffer = alignedDepthBuffer;
	m_infraredBuffer = infraredBuffer;
	m_alignedInfraredBuffer = alignedInfraredBuffer;
}

void ColoredOutputStreamUpdater::stopFaceCollection()
{	
	if (!m_isValidFaceFrame){
		m_colorBuffer = nullptr;
		m_depthBuffer = nullptr;
		m_alignedDepthBuffer = nullptr;
		m_infraredBuffer = nullptr;
		m_alignedInfraredBuffer = nullptr;
		return;
	}

	//center the point clouds
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*m_HDFacePointCloud, centroid);
	Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
	Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

	pcl::transformPointCloud(*m_HDFacePointCloud, *m_HDFacePointCloud_centered, m);
	pcl::transformPointCloud(*m_FaceRawPointCloud, *m_FaceRawPointCloud_centered, m);

	//update as vector
	if (!cloudsUpdated.empty()){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> updatedClouds;
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

	//create and update full depth before cloud
	if (!cloudUpdated[2].empty()){
		auto depthBufferCloud = convertDepthBufferToPointCloud();
		// update writer
		cloudUpdated[2](depthBufferCloud);
	}

	//HDFace2D
	if (!cloudUpdated[3].empty()){
		// update writer
		cloudUpdated[3](m_HDFacePointCloud2D);
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

	//KinectAlignedDepthRaw
	if (!imageUpdated[2].empty()){
		// update writer
		//cv::Mat m_alignedDepthImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_alignedDepthBuffer, cv::Mat::AUTO_STEP);   
		cv::Mat m_alignedDepthImage = cv::Mat(m_colorHeight, m_colorWidth, CV_16UC1, m_alignedDepthBuffer, cv::Mat::AUTO_STEP);   
		boost::shared_ptr<cv::Mat> m_alignedDepthImagePtr(new cv::Mat());
		*m_alignedDepthImagePtr = m_alignedDepthImage.clone();
		imageUpdated[2](m_alignedDepthImagePtr);
	}

	//KinectInfrared
	if (!imageUpdated[3].empty()){
		// update writer
		//cv::Mat m_alignedDepthImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_alignedDepthBuffer, cv::Mat::AUTO_STEP);   
		cv::Mat m_infraredImage = cv::Mat(m_depthHeight, m_depthWidth, CV_8UC4, m_infraredBuffer, cv::Mat::AUTO_STEP);   
		boost::shared_ptr<cv::Mat> m_infraredImagePtr(new cv::Mat());
		*m_infraredImagePtr = m_infraredImage.clone();
		imageUpdated[3](m_infraredImagePtr);
	}

		//KinectInfrared
	if (!imageUpdated[4].empty()){
		// update writer
		//cv::Mat m_alignedDepthImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_alignedDepthBuffer, cv::Mat::AUTO_STEP);   
		cv::Mat m_infraredImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_alignedInfraredBuffer, cv::Mat::AUTO_STEP);   
		boost::shared_ptr<cv::Mat> m_infraredImagePtr(new cv::Mat());
		*m_infraredImagePtr = m_infraredImage.clone();
		imageUpdated[4](m_infraredImagePtr);
	}

		//KinectInfrared
	if (!keyPointsUpdated[0].empty()){

		
		keyPointsUpdated[0](m_pFiveKeyPoints);
	}


	m_isValidFaceFrame = false;
	m_colorBuffer = nullptr;
	m_depthBuffer = nullptr;
	m_alignedDepthBuffer = nullptr;
	m_infraredBuffer = nullptr;
}


HRESULT ColoredOutputStreamUpdater::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize,
	CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace, std::string sKeyPoints)
{
	if (m_colorBuffer == nullptr || m_depthBuffer == nullptr){
		return -1;
	}
	//create vectices of face
	HRESULT hr = faceModel->CalculateVerticesForAlignment(faceAlignment, bufferSize, detectedHDFacePointsCamSpace);
	if (SUCCEEDED(hr)){
		//map them to color space for creating the convex hull later on
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(bufferSize, detectedHDFacePointsCamSpace, bufferSize, detectedHDFacePointsColorSpace);
	}

	if (SUCCEEDED(hr)){
		CameraSpacePoint boundingBoxPointTopLeft;
		CameraSpacePoint boundingBoxPointBottomRight;
		std::vector<cv::Point2f> hdFacePointsInColorSpaceSpaceOpenCV;

		extractColoredFaceHDPoinCloudAndBoundingBox(bufferSize, detectedHDFacePointsCamSpace, detectedHDFacePointsColorSpace,
			boundingBoxPointTopLeft, boundingBoxPointBottomRight, hdFacePointsInColorSpaceSpaceOpenCV);


		m_isValidFaceFrame &= extractColoredDepthCloudFromBoundingBox(boundingBoxPointTopLeft, boundingBoxPointBottomRight,
			hdFacePointsInColorSpaceSpaceOpenCV);
	}
	

	m_pFiveKeyPoints->append(sKeyPoints.c_str());

	return hr;
}


void ColoredOutputStreamUpdater::extractColoredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints,
	ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeft, CameraSpacePoint& camBottomRight, std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV)
{
	//init for bounding box
	float bottom = FLT_MAX;
	float top = -FLT_MAX;
	float right = -FLT_MAX;
	float left = FLT_MAX;
	float back = -FLT_MAX;
	float front = FLT_MAX;

	std::vector<cv::Point2f> ellipsePoints;
	for (int i = 0; i < bufferSize; i++){
		const auto& cameraSpacePoint = *cameraSpacePoints;
		const auto& colorSpacePoint = *colorSpacePoints;
		pcl::PointXYZRGB point;
		pcl::PointXYZRGB point2D;
		point.x = cameraSpacePoint.X;
		point.y = cameraSpacePoint.Y;
		point.z = cameraSpacePoint.Z;
		point2D.x = colorSpacePoint.X;
		point2D.y = colorSpacePoint.Y;
		point2D.z = 0;

		//find color buffer indices
		int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
		int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
		//valid index?
		if (colorY > m_colorHeight || colorX > m_colorWidth || colorY < 0 || colorX < 0)
			continue;

		//add the color point for creating convex hull for raw depth extraction 
		hdFacePointsInColorSpaceSpaceOpenCV.push_back(cv::Point2f(colorX, colorY));

		bottom = std::min(point.y, bottom);
		top = std::max(point.y, top);
		right = std::max(point.x, right);
		left = std::min(point.x, left);
		back = std::max(point.z, back);
		front = std::min(point.z, front);
		
		//color the point
		int colorImageIndex = ((m_colorWidth * colorY) + colorX);
		RGBQUAD pixel = m_colorBuffer[colorImageIndex];
		point.r = pixel.rgbRed;
		point.g = pixel.rgbGreen;
		point.b = pixel.rgbBlue;
		point2D.r = pixel.rgbRed;
		point2D.g = pixel.rgbGreen;
		point2D.b = pixel.rgbBlue;

		m_HDFacePointCloud->push_back(point);
		m_HDFacePointCloud2D->push_back(point2D);

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


bool ColoredOutputStreamUpdater::extractColoredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeft, CameraSpacePoint camBottomRight,
	std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV)
{
	//map the bounding box points into depth space
	DepthSpacePoint depthTopLeftBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopLeft, &depthTopLeftBack);

	DepthSpacePoint depthBottomRightBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomRight, &depthBottomRightBack);

	cv::vector<cv::Point2f> hullPoints;
	//check vor validty of transformed box corners
	if (!isValidDepthPoint(depthTopLeftBack) || !isValidDepthPoint(depthBottomRightBack)){
		return false;
	}

	//create a concvex hull around the hd face points to be able to check each point for beeing inside or outside
	cv::convexHull(hdFacePointsInColorSpaceSpaceOpenCV, hullPoints);

	//iterate over bounding box 
	for (int x = static_cast<int>(depthTopLeftBack.X); x < static_cast<int>(depthBottomRightBack.X); x++){
		for (int y = static_cast<int>(depthTopLeftBack.Y); y < static_cast<int>(depthBottomRightBack.Y); y++){

			pcl::PointXYZRGB point;

			//find depth buffer index
			DepthSpacePoint depthPoint;
			depthPoint.X = static_cast<float>(x);
			depthPoint.Y = static_cast<float>(y);
			UINT16 depthOfCurrentPoint = m_depthBuffer[y * m_depthWidth + x];

			//find color point for coloring and testing inside the HDFace hull
			ColorSpacePoint colorPoint;
			HRESULT hr = m_pCoordinateMapper->MapDepthPointToColorSpace(depthPoint, depthOfCurrentPoint, &colorPoint);
			if (FAILED(hr)){
				continue;
			}
			int colorPixelMidX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
			int colorPixelMidY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));

			//inside the convex hull?
			auto result = cv::pointPolygonTest(hullPoints, cv::Point2f(colorPoint.X, colorPoint.Y), false);
			if (result < 0){
				continue;
			}

			bool isInColor = false;
			if ((0 <= colorPixelMidX) && (colorPixelMidX < m_colorWidth) && (0 <= colorPixelMidY) && (colorPixelMidY < m_colorHeight)){
				RGBQUAD color = m_colorBuffer[colorPixelMidY * m_colorWidth + colorPixelMidX];
				point.b = color.rgbBlue;
				point.g = color.rgbGreen;
				point.r = color.rgbRed;
				isInColor = true;
			}

			//map the depth point to cam space for 3D coordinates
			CameraSpacePoint camPoint;
			hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthOfCurrentPoint, &camPoint);

			if (FAILED(hr)){
				continue;
			}
			bool isInDepth = false;
			if ((0 <= colorPixelMidX) && (colorPixelMidX < m_colorWidth) && (0 <= colorPixelMidY) && (colorPixelMidY < m_colorHeight)){
				point.x = camPoint.X;
				point.y = camPoint.Y;
				point.z = camPoint.Z;
				isInDepth = true;
			}

			if (point.x < camTopLeft.X || point.x > camBottomRight.X)
				continue;

			if (isInColor && isInDepth){
				m_FaceRawPointCloud->push_back(point);
			}
		}
	}

	return true;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredOutputStreamUpdater::convertDepthBufferToPointCloud()
{
	//allocate new cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullDepthBufferCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	fullDepthBufferCloud->width = static_cast<uint32_t>(m_depthWidth);
	fullDepthBufferCloud->height = static_cast<uint32_t>(m_depthHeight);
	fullDepthBufferCloud->is_dense = false;

	//validity call check
	if (m_pDepthVisibilityTestMap.size() == 0 || m_pColorCoordinates.size() == 0){
		return fullDepthBufferCloud;
	}

	HRESULT hr;
	auto depthBufferSize = m_depthWidth * m_depthHeight;
	//find the correspoinding color space points to the depth points
	hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(depthBufferSize, m_depthBuffer,
		depthBufferSize, m_pColorCoordinates.data());

	if (FAILED(hr))
	{
		return fullDepthBufferCloud;
	}


	const UINT16* const pDepthEnd = m_depthBuffer + depthBufferSize;
	auto pColorPoint = m_pColorCoordinates.begin();

	const UINT testMapWidth = UINT(m_colorWidth >> PATCHDIVISIONSHIFT);
	const UINT testMapHeight = UINT(m_colorHeight >> PATCHDIVISIONSHIFT);

	//clear the old map
	ZeroMemory(m_pDepthVisibilityTestMap.data(), testMapWidth * testMapHeight * sizeof(UINT16));

	//different depth point can be mapped to the same color point if objects are hidden, because of
	//distance between color camera and depth camera
	//only store the depth of the closest of those depth points to be able to ignore those depth points later on
	//which are not seen by the color camera
	//algorithm similar to z-Buffering/Shadow Mapping/Kinect Fusion Explorer Sample
	for (const UINT16* pDepth = m_depthBuffer; pDepth < pDepthEnd; pDepth++, pColorPoint++)
	{
		//color cam resolution is 1920x1080; depth: 512x424 => divide the color coodinates by 4 to be able
		//get the same coordinates for very close color space points => small patches in color space
		const UINT patchColorX = UINT(pColorPoint->X + 0.5f) >> PATCHDIVISIONSHIFT;
		const UINT patchColorY = UINT(pColorPoint->Y + 0.5f) >> PATCHDIVISIONSHIFT;
		if (patchColorX < testMapWidth && patchColorY < testMapHeight)
		{
			const UINT currentPatchIndex = patchColorY * testMapWidth + patchColorX;
			const UINT16 oldDepth = m_pDepthVisibilityTestMap[currentPatchIndex];
			const UINT16 newDepth = *pDepth;
			if (!oldDepth || oldDepth > newDepth)
			{
				m_pDepthVisibilityTestMap[currentPatchIndex] = newDepth;
			}
		}
	}

	//create the colored depth cloud by iterating over the depth buffer
	for (int yDepthHeight = 0; yDepthHeight < m_depthHeight; yDepthHeight++)
	{
		const UINT testMapWidth = UINT(m_colorWidth >> PATCHDIVISIONSHIFT);

		UINT destIndex = yDepthHeight * m_depthWidth;
		for (UINT xDepthWidth = 0; xDepthWidth < m_depthWidth; ++xDepthWidth, ++destIndex)
		{
			//coordinate of the depth point in color coordinates
			const ColorSpacePoint colorPoint = m_pColorCoordinates[destIndex];
			const UINT colorX = (UINT)(colorPoint.X + 0.5f);
			const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
			if (colorX < m_colorWidth && colorY < m_colorHeight)
			{
				const UINT16 depthValue = m_depthBuffer[destIndex];

				//find the smallest depth point value mapped to that color point
				const UINT testX = colorX >> PATCHDIVISIONSHIFT;
				const UINT testY = colorY >> PATCHDIVISIONSHIFT;
				const UINT testIdx = testY * testMapWidth + testX;
				const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];

				auto testDiff = std::abs(depthValue - depthTestValue);
				//test whether the current point is the one closest to the color image plane
				if (testDiff < VISIBILITY_MAX_THRESHHOLD)
				{
					//get color of pixel
					const UINT colorIndex = colorX + (colorY * m_colorWidth);
					auto pixelColor = m_colorBuffer[colorIndex];

					//get 3D coordinates in cam space
					DepthSpacePoint depthPoint;
					depthPoint.X = static_cast<float>(xDepthWidth);
					depthPoint.Y = static_cast<float>(yDepthHeight);
					CameraSpacePoint camPoint;
					hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthValue, &camPoint);

					//some points get coordinates which are infinity
					if (FAILED(hr) || !isValidCamSpacePoint(camPoint)){
						continue;
					}

					pcl::PointXYZRGB point;
					point.x = camPoint.X;
					point.y = camPoint.Y;
					point.z = camPoint.Z;

					point.b = pixelColor.rgbBlue;
					point.g = pixelColor.rgbGreen;
					point.r = pixelColor.rgbRed;

					fullDepthBufferCloud->push_back(point);
				}
			}
		}
	}

	return fullDepthBufferCloud;

}