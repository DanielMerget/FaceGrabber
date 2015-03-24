#include "stdafx.h"
#include "ColouredOutputStreamUpdater.h"
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <future>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "stdafx.h"
#include <atlstr.h>
#include <cmath>

ColouredOutputStreamUpdater::ColouredOutputStreamUpdater()
{
}

static const int            PATCHDIVISIONSHIFT = 2 ;
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

void printMessage(std::string msg)
{
	auto msgCstring = CString(msg.c_str());
	msgCstring += L"\n";
	//OutputDebugString(msgCstring);

}
HRESULT ColouredOutputStreamUpdater::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, 
	CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer)
{

	HRESULT hr = faceModel->CalculateVerticesForAlignment(faceAlignment, bufferSize, detectedHDFacePointsCamSpace);
	if (SUCCEEDED(hr)){
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(bufferSize, detectedHDFacePointsCamSpace, bufferSize, detectedHDFacePointsColorSpace);
	}
	if (SUCCEEDED(hr)){
		CameraSpacePoint boundingBoxPointTopLeft;
		CameraSpacePoint boundingBoxPointBottomRight;
		std::vector<cv::Point2f> hdFacePointsInColorSpaceSpaceOpenCV;		

		auto hdFaceCloud = extractClolouredFaceHDPoinCloudAndBoundingBox(bufferSize, detectedHDFacePointsCamSpace, detectedHDFacePointsColorSpace,
			boundingBoxPointTopLeft, boundingBoxPointBottomRight, hdFacePointsInColorSpaceSpaceOpenCV, colorBuffer);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr hdFaceRawDepthCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		bool isCorrectFrame = extractColouredDepthCloudFromBoundingBox(boundingBoxPointTopLeft, boundingBoxPointBottomRight,
			hdFacePointsInColorSpaceSpaceOpenCV, colorBuffer, depthBuffer, hdFaceRawDepthCloud);
		if (!isCorrectFrame){
			return hr;
		}

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*hdFaceCloud, centroid);
		Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
		Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

		pcl::transformPointCloud(*hdFaceCloud, *hdFaceCloud, m);
		pcl::transformPointCloud(*hdFaceRawDepthCloud, *hdFaceRawDepthCloud, m);


		if (!cloudsUpdated.empty()){
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> updatedClouds;
			updatedClouds.push_back(hdFaceCloud);
			auto fullDepthCloud = convertDepthBufferToPointCloud(colorBuffer, depthBuffer);
			updatedClouds.push_back(fullDepthCloud);
			cloudsUpdated(updatedClouds);
		}
		if (!cloudUpdated[0].empty()){
			cloudUpdated[0](hdFaceCloud);
		}

		if (!cloudUpdated[1].empty()){
			cloudUpdated[1](hdFaceRawDepthCloud);
		}
		if (!cloudUpdated[2].empty()){
			auto fullDepthCloud = convertDepthBufferToPointCloud(colorBuffer, depthBuffer);
			cloudUpdated[2](fullDepthCloud);
		}

	}
	return hr;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColouredOutputStreamUpdater::extractClolouredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, 
	ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV,
	RGBQUAD* colorBuffer)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
	cloud->is_dense = false;
	float bottom	=	 FLT_MAX;
	float top		= -	 FLT_MAX;
	float right		= -	 FLT_MAX;
	float left		=	 FLT_MAX;
	float back		=  - FLT_MAX;
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
		RGBQUAD pixel = colorBuffer[colorImageIndex];
		point.r = pixel.rgbRed;
		point.g = pixel.rgbGreen;
		point.b = pixel.rgbBlue;

		cloud->push_back(point);

		cameraSpacePoints++;
		colorSpacePoints++;
	}


	camTopLeftBack.X = left;
	camTopLeftBack.Y = top;
	camTopLeftBack.Z = back;


	camBottomRightBack.X = right;
	camBottomRightBack.Y = bottom;
	camBottomRightBack.Z = back;

	return cloud;
}


bool ColouredOutputStreamUpdater::extractColouredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack, 
	std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud)
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

			UINT16 depthOfCurrentPoint = depthBuffer[y * m_depthWidth + x];

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
				RGBQUAD color = colorBuffer[colorPixelMidY * m_colorWidth + colorPixelMidX];
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
				depthCloud->push_back(point);
			}
		}
	}

	return true;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColouredOutputStreamUpdater::convertDepthBufferToPointCloud(RGBQUAD* colorBuffer, UINT16* depthBuffer)
{
	
	if (m_pDepthVisibilityTestMap.size() == 0 || m_pColorCoordinates.size() == 0){
		return nullptr;
	}
	//Transform Depth Points into Color Space
	HRESULT hr;
	auto depthBufferSize = m_depthWidth * m_depthHeight;

	hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(depthBufferSize, depthBuffer,
		depthBufferSize, m_pColorCoordinates.data());

	if (FAILED(hr))
	{
		return nullptr;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pointCloud->width = static_cast<uint32_t>(m_depthWidth);
	pointCloud->height = static_cast<uint32_t>(m_depthHeight);
	pointCloud->is_dense = false;

	const UINT16* const pDepthEnd = depthBuffer + depthBufferSize;
	auto pColorPoint = m_pColorCoordinates.begin();

	const UINT testMapWidth = UINT(m_colorWidth >> PATCHDIVISIONSHIFT);
	const UINT testMapHeight = UINT(m_colorHeight >> PATCHDIVISIONSHIFT);

	ZeroMemory(m_pDepthVisibilityTestMap.data(), testMapWidth * testMapHeight * sizeof(UINT16));

	//iterate through color buffer 
	for (const UINT16* pDepth = depthBuffer; pDepth < pDepthEnd; pDepth++, pColorPoint++)
	{
		const UINT patchColorX = UINT(pColorPoint->X + 0.5f) >> PATCHDIVISIONSHIFT;
		const UINT patchColorY = UINT(pColorPoint->Y + 0.5f) >> PATCHDIVISIONSHIFT;
		//check whether inside the color buffer
		if (patchColorX < testMapWidth && patchColorY < testMapHeight)
		{
			//calculate index by color space points => more than one color space point is mapped
			//to the same index, as PATCHDIVISIONSHIFT divides the color space from 1920x1080 
			//into a 480x270 grid  (depth 512×424)
			//take only the smallest depth value
			const UINT currentPatchIndex = patchColorY * testMapWidth + patchColorX;
			const UINT16 oldDepth = m_pDepthVisibilityTestMap[currentPatchIndex];
			const UINT16 newDepth = *pDepth;
			if (!oldDepth || oldDepth > newDepth)
			{
				m_pDepthVisibilityTestMap[currentPatchIndex] = newDepth;
			}
		}
	}

	//iterate throught the deph array again
	for (int yDepthHeight = 0; yDepthHeight < m_depthHeight; yDepthHeight++)
	{
		const UINT testMapWidth = UINT(m_colorWidth >> PATCHDIVISIONSHIFT);

		UINT destIndex = yDepthHeight * m_depthWidth;
		for (UINT xDepthWidth = 0; xDepthWidth < m_depthWidth; ++xDepthWidth, ++destIndex)
		{
			const ColorSpacePoint colorPoint = m_pColorCoordinates[destIndex];
			const UINT colorX = (UINT)(colorPoint.X + 0.5f);
			const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
			//check vor valid color index
			if (colorX < m_colorWidth && colorY < m_colorHeight)
			{
				const UINT16 depthValue = depthBuffer[destIndex];

				//calculate corresponding patch index
				const UINT testX = colorX >> PATCHDIVISIONSHIFT;
				const UINT testY = colorY >> PATCHDIVISIONSHIFT;

				const UINT testIdx = testY * testMapWidth + testX;

				const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];

				//compare measured depth with depth in color space 
				auto testDiff = std::abs(depthValue - depthTestValue);
				//check whether depth point is in sight of color camera 
				if (testDiff < VISIBILITY_MAX_THRESHHOLD)
				{
					const UINT colorIndex = colorX + (colorY * m_colorWidth);
					auto pixelColor = colorBuffer[colorIndex];

					DepthSpacePoint depthPoint;
					depthPoint.X = static_cast<float>(xDepthWidth);
					depthPoint.Y = static_cast<float>(yDepthHeight);

					//map color space point into camera space instead?
					CameraSpacePoint camPoint;
					hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthValue, &camPoint);

					if (FAILED(hr)){
						continue;
					}

					pcl::PointXYZRGB point;
					point.x = camPoint.X;
					point.y = camPoint.Y;
					point.z = camPoint.Z;
					
					point.b = pixelColor.rgbBlue;
					point.g = pixelColor.rgbGreen;
					point.r = pixelColor.rgbRed;

					pointCloud->push_back(point);
				}
			}	
		}
	};
	
	return pointCloud;
}