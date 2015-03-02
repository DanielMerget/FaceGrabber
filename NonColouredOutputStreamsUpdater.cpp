#include "NonColouredOutputStreamsUpdater.h"
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

NonColouredOutputStreamsUpdater::NonColouredOutputStreamsUpdater()
{
}


NonColouredOutputStreamsUpdater::~NonColouredOutputStreamsUpdater()
{
}



HRESULT NonColouredOutputStreamsUpdater::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize,
	CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer)
{

	HRESULT hr = faceModel->CalculateVerticesForAlignment(faceAlignment, bufferSize, detectedHDFacePointsCamSpace);
	if (SUCCEEDED(hr)){
		hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(bufferSize, detectedHDFacePointsCamSpace, bufferSize, detectedHDFacePointsColorSpace);
	}
	if (SUCCEEDED(hr)){
		CameraSpacePoint boundingBoxPointTopLeft;
		CameraSpacePoint boundingBoxPointBottomRight;
		std::vector<cv::Point2f> hdFacePointsInCamSpaceOpenCV;

		auto hdFaceCloud = extractFaceHDPoinCloudAndBoundingBox(bufferSize, detectedHDFacePointsCamSpace, detectedHDFacePointsColorSpace,
			boundingBoxPointTopLeft, boundingBoxPointBottomRight, hdFacePointsInCamSpaceOpenCV, colorBuffer);
		auto hdFaceRawDepthCloud = extractDepthCloudFromBoundingBox(boundingBoxPointTopLeft, boundingBoxPointBottomRight,
			hdFacePointsInCamSpaceOpenCV, colorBuffer, depthBuffer);

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*hdFaceCloud, centroid);
		Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
		Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

		pcl::transformPointCloud(*hdFaceCloud, *hdFaceCloud, m);
		pcl::transformPointCloud(*hdFaceRawDepthCloud, *hdFaceRawDepthCloud, m);


		cloudUpdated(hdFaceCloud);
		depthCloudUpdated(hdFaceRawDepthCloud);
	}
	return hr;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr NonColouredOutputStreamsUpdater::extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints,
	ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV,
	RGBQUAD* colorBuffer)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>());
	cloud->is_dense = false;
	float bottom = FLT_MAX;
	float top = -FLT_MAX;
	float right = -FLT_MAX;
	float left = FLT_MAX;
	float back = -FLT_MAX;
	std::vector<cv::Point2f> ellipsePoints;
	for (int i = 0; i < bufferSize; i++){
		const auto& cameraSpacePoint = *cameraSpacePoints;
		const auto& colorSpacePoint = *colorSpacePoints;
		pcl::PointXYZ point;
		point.x = cameraSpacePoint.X;
		point.y = cameraSpacePoint.Y;
		point.z = cameraSpacePoint.Z;
	
		int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
		int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));

		if (colorY > m_colorHeight || colorX > m_colorWidth || colorY < 0 || colorX < 0)
			continue;

		hdFacePointsInCamSpaceOpenCV.push_back(cv::Point2f(colorX, colorY));

		bottom = std::min(point.y, bottom);
		top = std::max(point.y, top);
		right = std::max(point.x, right);
		left = std::min(point.x, left);
		back = std::max(point.z, back);

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



pcl::PointCloud<pcl::PointXYZ>::Ptr NonColouredOutputStreamsUpdater::extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
	std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer)
{
	DepthSpacePoint depthTopLeftBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopLeftBack, &depthTopLeftBack);

	DepthSpacePoint depthBottomRightBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomRightBack, &depthBottomRightBack);

	pcl::PointCloud<pcl::PointXYZ>::Ptr depthCloud(new pcl::PointCloud <pcl::PointXYZ>());

	cv::vector<cv::Point2f> hullPoints;
	cv::convexHull(hdFacePointsInCamSpaceOpenCV, hullPoints);
	for (int x = static_cast<int>(depthTopLeftBack.X); x < static_cast<int>(depthBottomRightBack.X); x++){
		for (int y = static_cast<int>(depthTopLeftBack.Y); y < static_cast<int>(depthBottomRightBack.Y); y++){

			pcl::PointXYZ point;
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

			CameraSpacePoint camPoint;
			hr = m_pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint, depthOfCurrentPoint, &camPoint);

			if (FAILED(hr)){
				continue;
			}
			

			if (point.x < camTopLeftBack.X || point.x > camBottomRightBack.X)
				continue;

			
			depthCloud->push_back(point);
			
		}
	}

	return depthCloud;
}

