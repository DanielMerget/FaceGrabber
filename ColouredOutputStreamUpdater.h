#pragma once
#include "stdafx.h"

#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <limits>

/**
 * \class	ColouredOutputStreamUpdater
 *
 * \brief	The ColouredOutputStreamUpdater takes the buffers and face information provided by the
 * 			updateOutputStreams, extracts the HDFace, FaceRaw and FullRawDepth point clouds and publishes
 * 			them via cloudsUpdated or cloudUpdated. Beforehand the ColouredOutputStreamUpdater has to be
 * 			initialied with the coordinatemapper provided by the kinect and information about the resolution
 * 			of the update-streams.
 */

class ColouredOutputStreamUpdater :
	public OutputStreamsUpdaterStragedy
{
public:
	
	ColouredOutputStreamUpdater();
	~ColouredOutputStreamUpdater();

	/**
	 * \fn	HRESULT ColouredOutputStreamUpdater::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer);
	 *
	 * \brief	Updates the output streams.
	 *
	 * \param [in]	faceModel					  	The face model accociated with the detectedHDFacePointsCamSpace.
	 * \param [in]	faceAlignment				  	The current faceAlignement of the face model.
	 * \param	hdFacePointCamSpaceBufferSize	  	Size of the detectedHDFacePointsCamSpace buffer.
	 * \param [out]	detectedHDFacePointsCamSpace  	the detected HD face points in Cam space to be fil.ed.
	 *  camera space.
	 * \param [out]	detectedHDFacePointsColorSpace	the detected HD face points in Color space to be filled.
	 *  color space.
	 * \param [in]	colorBuffer					  	The current color buffer.
	 * \param [in]	depthBuffer					  	The current depth buffer.
	 *
	 * \return	A hResult.
	 */
	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int hdFacePointCamSpaceBufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace, 
		ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer);


	/** \brief	The signal providing HDFace and FaceRaw point clouds if consumer is connected*/
	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>)> cloudsUpdated;

	/**
	 * \property	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)> cloudUpdated[3]
	 *
	 * \brief	The signal providing the point clouds:
	 * 			HDFace at index 0
	 * 			FaceRaw at index 1
	 * 			FullFaceRaw at index 3
	 * 			
	 *
	 */
	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)> cloudUpdated[3];

	/**
	 * \fn	void ColouredOutputStreamUpdater::initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);
	 *
	 * \brief	Initializes the ColordOutputStreamUpdater.
	 *
	 * \param [in]	m_pCoordinateMapper	Kinect coordinate mapper.
	 * \param	depthWidth				   	Width of the depth.
	 * \param	depthHeight				   	Height of the depth.
	 * \param	colorWidth				   	Width of the color.
	 * \param	colorHeight				   	Height of the color.
	 */
	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

private:

	bool extractColouredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV, RGBQUAD* colorBuffer, UINT16* depthBuffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractClolouredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints, 
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV, RGBQUAD* colorBuffer);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDepthBufferToPointCloud(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	
	std::vector<UINT16>				m_pDepthVisibilityTestMap;
	std::vector<ColorSpacePoint>	m_pColorCoordinates;
};

