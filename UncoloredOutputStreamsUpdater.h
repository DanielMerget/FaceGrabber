#pragma once
#include "stdafx.h"
#include "OutputStreamsUpdaterStragedy.h"
#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * \class	UncoloredOutputStreamsUpdater
 *
 * \brief	Implementation of the OutputStreamsUpdaterStragedy which extracts point clouds using the
 * 			buffers provided method calls. The processed point clouds are provided without colored.
 * 			Listeners can registered on cloudsUpdated and cloudUpdated. CloudsUpdated provides the
 * 			HDFace and RawFaceDepth point clouds. 
 * 			cloudUpdated[0]: HDFace
 * 			cloudUpdated[1]: RawFaceDepth
 * 			cloudUpdated[2]: FullRawpDeth
 */

class UncoloredOutputStreamsUpdater :
	public OutputStreamsUpdaterStragedy
{
public:

	/**
	 * \fn	UncoloredOutputStreamsUpdater::UncoloredOutputStreamsUpdater();
	 *
	 * \brief	Default constructor.
	 */
	UncoloredOutputStreamsUpdater();

	/**
	 * \fn	UncoloredOutputStreamsUpdater::~UncoloredOutputStreamsUpdater();
	 *
	 * \brief	Destructor.
	 */
	~UncoloredOutputStreamsUpdater();


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
		ColorSpacePoint* detectedHDFacePointsColorSpace);


	/** \brief	The clouds updated. */
	boost::signals2::signal<void(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>)> cloudsUpdated;

	/** \brief	The clouds updated:  cloudUpdated[0]: HDFace; cloudUpdated[1]: RawFaceDepth; cloudUpdated[2]: FullRawpDeth; cloudUpdated[3]: HDFace2D */
	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)> cloudUpdated[4];

	/** \brief	The image updated */
	boost::signals2::signal<void(boost::shared_ptr<cv::Mat>)> imageUpdated[2];

	/**
	 * \fn	void UncoloredOutputStreamsUpdater::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	 *
	 * \brief	Starts collecting point clouds of all tracked users.
	 * \param [in]	colorBuffer the current color Buffer
	 * \param [in]	depthBuffer the current depth Buffer
	 */
	void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);

	/**
	 * \fn	void UncoloredOutputStreamsUpdater::stopFaceCollection();
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

private:

	/**
	 * \fn	void UncoloredOutputStreamsUpdater::allocateClouds();
	 *
	 * \brief	Allocate new face cloud buffers.
	 */
	void allocateClouds();

	/**
	 * \fn	bool UncoloredOutputStreamsUpdater::extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);
	 *
	 * \brief	Extracts the colored depth cloud from the given bounding box.
	 *
	 * \param	camTopLeftBack							   	The top left back corner of bounding box in cam space
	 * \param	camBottomRightBack						   	The bottom right back corner of bounding box in cam sapce
	 * \param [in]	hdFacePointsInColorSpaceSpaceOpenCV	The HD face points in color space as opencv points.
	 *
	 * \return	true if it succeeds, false if it fails.
	 */
	bool extractDepthCloudFromBoundingBox(CameraSpacePoint camTopLeftBack, CameraSpacePoint camBottomRightBack,
		std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	/**
	 * \fn	void UncoloredOutputStreamsUpdater::extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints, CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);
	 *
	 * \brief	Extracts the uncolored face HD point cloud and bounding box.
	 *
	 * \param	bufferSize								Size of the cameraSpacePoints buffer.
	 * \param [in]	cameraSpacePoints				the hdface camera space points.
	 * \param [in]	colorSpacePoints				the hdface color space points.
	 * \param [in,out]	camTopLeftBack					The camera top left back.
	 * \param [in,out]	camBottomRightBack				The camera bottom right back.
	 * \param [in,out]	hdFacePointsInCamSpaceOpenCV	The HD face points in camera space open cv.
	 */
	void extractFaceHDPoinCloudAndBoundingBox(int bufferSize, CameraSpacePoint* cameraSpacePoints, ColorSpacePoint* colorSpacePoints,
		CameraSpacePoint& camTopLeftBack, CameraSpacePoint& camBottomRightBack, std::vector<cv::Point2f>& hdFacePointsInCamSpaceOpenCV);

	/**
	 * \fn	pcl::PointCloud<pcl::PointXYZ>::Ptr UncoloredOutputStreamsUpdater::convertDepthBufferToPointCloud();
	 *
	 * \brief	Convert depth buffer to point cloud.
	 *
	 * \return	converted depth point cloud.
	 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthBufferToPointCloud();

	/** \brief	The HD face point cloud buffer. */
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_HDFacePointCloud;

	/** \brief	The centered HD face point cloud buffer. */
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_HDFacePointCloud_centered;

	/** \brief	The face raw point cloud. buffer */
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_FaceRawPointCloud;

	/** \brief	The centred face raw point cloud. buffer */
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_FaceRawPointCloud_centered;

	/** \brief	The HD face point cloud 2D ColorSpace representation. */
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_HDFacePointCloud2D;

	/** \brief	true if this object is valid face frame. */
	bool m_isValidFaceFrame;

	/** \brief	true to enable, false to disable the centering. */
	bool m_centerEnabled;

	/** \brief	Buffer for color data. */
	RGBQUAD* m_colorBuffer;

	/** \brief	Buffer for depth data. */
	UINT16*	m_depthBuffer;

};

