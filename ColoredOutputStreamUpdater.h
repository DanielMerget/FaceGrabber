#pragma once
#include "stdafx.h"

#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <limits>

/**
 * \class	ColoredOutputStreamUpdater
 *
 * \brief	The ColoredOutputStreamUpdater takes the buffers and face information provided by the
 * 			updateOutputStreams, extracts the HDFace, FaceRaw and FullRawDepth point clouds and publishes
 * 			them via cloudsUpdated or cloudUpdated. Beforehand the ColoredOutputStreamUpdater has to be
 * 			initialied with the coordinatemapper provided by the kinect and information about the resolution
 * 			of the update-streams.
 */
class ColoredOutputStreamUpdater :
	public OutputStreamsUpdaterStragedy
{
public:

	ColoredOutputStreamUpdater();
	~ColoredOutputStreamUpdater();

	/**
	* \fn	virtual HRESULT OutputStreamsUpdaterStragedy::updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace, ColorSpacePoint* detectedHDFacePointsColorSpace) = 0;
	*
	* \brief	Updates and processes the output streams.
	*
	* \param [in,out]	faceModel					  	the face model of the current face
	* \param [in,out]	faceAlignment				  	the face alignment of the current face
	* \param	bufferSize							  	Size of the detectedHDFacePointsCamSpace buffer.
	* \param [in,out]	detectedHDFacePointsCamSpace  	The detected HD face points in camera space
	*  camera space.
	* \param [in,out]	detectedHDFacePointsColorSpace	The detected HD face points in color space
	*  color space.
	*
	* \return	A hResult.
	*/
	HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace,std::string sKeyPoints);


	/** \brief	The clouds updated. */
	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>)> cloudsUpdated;

	/** \brief	The clouds updated:  cloudUpdated[0]: HDFace; cloudUpdated[1]: RawFaceDepth; cloudUpdated[2]: FullRawpDeth; cloudUpdated[3]: HDFace2D */
	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)> cloudUpdated[4];

	/** \brief	The images updated */
	boost::signals2::signal<void(boost::shared_ptr<cv::Mat>)> imageUpdated[5]; // 1 -> color 2 ->depth 3->aligned depth 4-> infrared 5->alignedInfrared


	boost::signals2::signal<void(std::shared_ptr<std::string>)> keyPointsUpdated[1];
	/**
	 * \fn	void ColoredOutputStreamUpdater::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	 *
	 * \brief	Starts collecting point clouds of all tracked users.
	 * \param [in]	colorBuffer the current color Buffer
	 * \param [in]	depthBuffer the current depth Buffer
	 */
	void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer,UINT16* alignedDepthBuffer,RGBQUAD* infraredBuffer,RGBQUAD* alignedInfraredBuffer);

	/**
	 * \fn	void ColoredOutputStreamUpdater::stopFaceCollection();
	 *
	 * \brief	Stops the collection of the faces, runs the conversion of the entire depth buffer if neccesary and
	 * 			 calls the signals.
	 */
	void stopFaceCollection();

	/**
	* \fn	void RecordTabHandler::setCeterEnabled(bool enable);
	*
	* \brief	Enables or disables the centering of recorded Clouds.
	*
	* \param	enable	true to enable, false to disable.
	*/
	void setCeterEnabled(bool enable);

	/**
	* \fn	virtual void OutputStreamsUpdaterStragedy::initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);
	*
	* \brief	Initializes the OutputStreamUpdaterStragedy with the frame sizes.
	*
	* \param [in,out]	m_pCoordinateMapper	If non-null, the coordinate mapper.
	* \param	depthWidth				   	Width of the depth.
	* \param	depthHeight				   	Height of the depth.
	* \param	colorWidth				   	Width of the color.
	* \param	colorHeight				   	Height of the color.
	*/
	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

private:

	/**
	 * \fn	void ColoredOutputStreamUpdater::allocateClouds();
	 *
	 * \brief	Allocate new face cloud buffers.
	 */
	void allocateClouds();

	/**
	 * \fn	bool ColoredOutputStreamUpdater::extractColoredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV);
	 *
	 * \brief	Extracts the colored depth cloud from the given bounding box.
	 *
	 * \param	camTopLeftBack							   	The top left back corner of bounding box in cam space
	 * \param	camBottomRightBack						   	The bottom right back corner of bounding box in cam sapce
	 * \param [in]	hdFacePointsInColorSpaceSpaceOpenCV	The HD face points in color space as opencv points.
	 *
	 * \return	true if it succeeds, false if it fails.
	 */
	bool extractColoredDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInColorSpaceSpaceOpenCV);

	/**
	 * \fn	void ColoredOutputStreamUpdater::extractColoredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);
	 *
	 * \brief	Extracts the colored face HD point cloud and bounding box.
	 *
	 * \param	bufferSize								Size of the cameraSpacePoints buffer.
	 * \param [in]	cameraSpacePoints				the hdface camera space points.
	 * \param [in]	colorSpacePoints				the hdface color space points.
	 * \param [in,out]	camTopLeftBack					The camera top left back.
	 * \param [in,out]	camBottomRightBack				The camera bottom right back.
	 * \param [in,out]	hdFacePointsInCamSpaceOpenCV	The HD face points in camera space open cv.
	 */
	void extractColoredFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	/**
	 * \fn	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredOutputStreamUpdater::convertDepthBufferToPointCloud();
	 *
	 * \brief	Convert depth buffer to a colored point cloud.
	 *
	 * \return	converted point cloud.
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDepthBufferToPointCloud();


	/** \brief	The HD face point cloud. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_HDFacePointCloud;

	/** \brief	The centered HD face point cloud. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_HDFacePointCloud_centered;

	/** \brief	The face raw point cloud. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_FaceRawPointCloud;

	/** \brief	The centered face raw point cloud. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_FaceRawPointCloud_centered;

	/** \brief	The HD face point cloud 2D ColorSpace representation. */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_HDFacePointCloud2D;
	

	/** \brief	The depth visibility test map for checking whether color camera is able to see provided depth points. */
	std::vector<UINT16>				m_pDepthVisibilityTestMap;

	/** \brief	The color coordinates of depth points. */
	std::vector<ColorSpacePoint>	m_pColorCoordinates;

	/** \brief	true if face frame is valid. */
	bool							m_isValidFaceFrame;

	std::shared_ptr<std::string> m_pFiveKeyPoints;
	/** \brief	true to enable, false to disable the centering. */
	bool m_centerEnabled;

	/** \brief	Buffer for color data. */
	RGBQUAD* m_colorBuffer;

	/** \brief	Buffer for infrared data. */
	RGBQUAD*  m_infraredBuffer;

	/** \brief	Buffer for infrared data. */
	RGBQUAD*  m_alignedInfraredBuffer;

	/** \brief	Buffer for depth data. */
	UINT16*	m_depthBuffer;

	/** \brief	Buffer for alighned depth data. */
	UINT16* m_alignedDepthBuffer;
};
