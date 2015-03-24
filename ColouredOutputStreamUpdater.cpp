
#include "stdafx.h"
#include "ColouredOutputStreamUpdater.h"
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <future>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/imgproc/imgproc.hpp>


#include <atlstr.h>
#include <cmath>

ColouredOutputStreamUpdater::ColouredOutputStreamUpdater() :
m_colorBuffer(nullptr),
m_depthBuffer(nullptr)
{
}

static const int            PATCHDIVISIONSHIFT = 2;
static const UINT16         VISIBILITY_MAX_THRESHHOLD = 50;
ColouredOutputStreamUpdater::~ColouredOutputStreamUpdater()
{
}


void ColouredOutputStreamUpdater::initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight)
{
	OutputStreamsUpdaterStragedy::initialize(m_pCoordinateMapper, depthWidth, depthHeight, colorWidth, colorHeight);
	m_pDepthVisibilityTestMap = std::vector<UINT16>((colorWidth >> PATCHDIVISIONSHIFT) * (colorHeight >> PATCHDIVISIONSHIFT));
	m_pColorCoordinates = std::vector<ColorSpacePoint>(depthWidth * depthHeight);
}

void ColouredOutputStreamUpdater::allocateClouds()
{
	m_HDFacePointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_HDFacePointCloud->is_dense = false;

	m_FaceRawPointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud <pcl::PointXYZRGB>());
	m_FaceRawPointCloud->is_dense = false;
}

void ColouredOutputStreamUpdater::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer)
{
	allocateClouds();
	m_isValidFaceFrame = true;

	m_colorBuffer = colorBuffer;
	m_depthBuffer = depthBuffer;
}

void ColouredOutputStreamUpdater::stopFaceCollection()
{	
	if (!m_isValidFaceFrame){
		m_colorBuffer = nullptr;
		m_depthBuffer = nullptr;
		return;
	}
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*m_HDFacePointCloud, centroid);
	Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
	Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

	pcl::transformPointCloud(*m_HDFacePointCloud, *m_HDFacePointCloud, m);
	pcl::transformPointCloud(*m_FaceRawPointCloud, *m_FaceRawPointCloud, m);


	if (!cloudsUpdated.empty()){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> updatedClouds;
		updatedClouds.push_back(m_HDFacePointCloud);
		updatedClouds.push_back(m_FaceRawPointCloud);
		cloudsUpdated(updatedClouds);
	}

	if (!cloudUpdated[0].empty()){
		cloudUpdated[0](m_HDFacePointCloud);
	}

	if (!cloudUpdated[1].empty()){
		cloudUpdated[1](m_FaceRawPointCloud);
	}

	if (!cloudUpdated[2].empty()){
		auto depthBufferCloud = convertDepthBufferToPointCloud();
		cloudUpdated[2](depthBufferCloud);
	}
	m_isValidFaceFrame = false;
	m_colorBuffer = nullptr;
	m_depthBuffer = nullptr;
}


HRESULT ColouredOutputStreamUpdater::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize,
	CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace)
{
	if (m_colorBuffer == nullptr || m_depthBuffer == nullptr){
		return -1;
	}
	HRESULT hr = faceModel->CalculateVerticesForAlignment(faceAlignment, bufferSize, detectedHDFacePointsCamSpace);
	if (SUCCEEDED(hr)){
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
	return hr;
}


void ColouredOutputStreamUpdater::extractColoredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints,
	ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV)
{
	float bottom = FLT_MAX;
	float top = -FLT_MAX;
	float right = -FLT_MAX;
	float left = FLT_MAX;
	float back = -FLT_MAX;
	std::vector<cv::Point2f> ellipsePoints;
	for (int i = 0; i < bufferSize; i++){
		const auto& cameraSpacePoint = *cameraSpacePoints;
		const auto& colorSpacePoint = *colorSpacePoints;
		pcl::PointXYZRGB point;
		point.x = cameraSpacePoint.X;
		point.y = cameraSpacePoint.Y;
		point.z = cameraSpacePoint.Z;

		int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
		int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));

		if (colorY > m_colorHeight || colorX > m_colorWidth || colorY < 0 || colorX < 0)
			continue;

		hdFacePointsInColorSpaceSpaceOpenCV.push_back(cv::Point2f(colorX, colorY));

		bottom = std::min(point.y, bottom);
		top = std::max(point.y, top);
		right = std::max(point.x, right);
		left = std::min(point.x, left);
		back = std::max(point.z, back);

		int colorImageIndex = ((m_colorWidth * colorY) + colorX);
		RGBQUAD pixel = m_colorBuffer[colorImageIndex];
		point.r = pixel.rgbRed;
		point.g = pixel.rgbGreen;
		point.b = pixel.rgbBlue;

		m_HDFacePointCloud->push_back(point);

		cameraSpacePoints++;
		colorSpacePoints++;
	}


	camTopLeftBack.X = left;
	camTopLeftBack.Y = top;
	camTopLeftBack.Z = back;


	camBottomRightBack.X = right;
	camBottomRightBack.Y = bottom;
	camBottomRightBack.Z = back;
}


bool ColouredOutputStreamUpdater::extractColoredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
	std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV)
{
	DepthSpacePoint depthTopLeftBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopLeftBack, &depthTopLeftBack);

	DepthSpacePoint depthBottomRightBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomRightBack, &depthBottomRightBack);

	cv::vector<cv::Point2f> hullPoints;
	if (!isValidDepthPoint(depthTopLeftBack) || !isValidDepthPoint(depthBottomRightBack)){
		return false;
	}


	cv::convexHull(hdFacePointsInColorSpaceSpaceOpenCV, hullPoints);
	for (int x = static_cast<int>(depthTopLeftBack.X); x < static_cast<int>(depthBottomRightBack.X); x++){
		for (int y = static_cast<int>(depthTopLeftBack.Y); y < static_cast<int>(depthBottomRightBack.Y); y++){

			pcl::PointXYZRGB point;
			DepthSpacePoint depthPoint;
			depthPoint.X = static_cast<float>(x);
			depthPoint.Y = static_cast<float>(y);

			UINT16 depthOfCurrentPoint = m_depthBuffer[y * m_depthWidth + x];

			ColorSpacePoint colorPoint;
			HRESULT hr = m_pCoordinateMapper->MapDepthPointToColorSpace(depthPoint, depthOfCurrentPoint, &colorPoint);
			if (FAILED(hr)){
				continue;
			}
			int colorPixelMidX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
			int colorPixelMidY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));
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

			if (point.x < camTopLeftBack.X || point.x > camBottomRightBack.X)
				continue;

			if (isInColor && isInDepth){
				m_FaceRawPointCloud->push_back(point);
			}
		}
	}

	return true;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColouredOutputStreamUpdater::convertDepthBufferToPointCloud()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fullDepthBufferCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	fullDepthBufferCloud->width = static_cast<uint32_t>(m_depthWidth);
	fullDepthBufferCloud->height = static_cast<uint32_t>(m_depthHeight);
	fullDepthBufferCloud->is_dense = false;
	if (m_pDepthVisibilityTestMap.size() == 0 || m_pColorCoordinates.size() == 0){
		return fullDepthBufferCloud;
	}

	HRESULT hr;
	auto depthBufferSize = m_depthWidth * m_depthHeight;
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

	ZeroMemory(m_pDepthVisibilityTestMap.data(), testMapWidth * testMapHeight * sizeof(UINT16));

	for (const UINT16* pDepth = m_depthBuffer; pDepth < pDepthEnd; pDepth++, pColorPoint++)
	{
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

	for (int yDepthHeight = 0; yDepthHeight < m_depthHeight; yDepthHeight++)
	{
		const UINT testMapWidth = UINT(m_colorWidth >> PATCHDIVISIONSHIFT);

		UINT destIndex = yDepthHeight * m_depthWidth;
		for (UINT xDepthWidth = 0; xDepthWidth < m_depthWidth; ++xDepthWidth, ++destIndex)
		{
			const ColorSpacePoint colorPoint = m_pColorCoordinates[destIndex];
			const UINT colorX = (UINT)(colorPoint.X + 0.5f);
			const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
			if (colorX < m_colorWidth && colorY < m_colorHeight)
			{
				const UINT16 depthValue = m_depthBuffer[destIndex];
				const UINT testX = colorX >> PATCHDIVISIONSHIFT;
				const UINT testY = colorY >> PATCHDIVISIONSHIFT;
				const UINT testIdx = testY * testMapWidth + testX;
				const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];

				auto testDiff = std::abs(depthValue - depthTestValue);
				if (testDiff < VISIBILITY_MAX_THRESHHOLD)
				{
					const UINT colorIndex = colorX + (colorY * m_colorWidth);
					auto pixelColor = m_colorBuffer[colorIndex];

					DepthSpacePoint depthPoint;
					depthPoint.X = static_cast<float>(xDepthWidth);
					depthPoint.Y = static_cast<float>(yDepthHeight);

					CameraSpacePoint camPoint;
					hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthValue, &camPoint);

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