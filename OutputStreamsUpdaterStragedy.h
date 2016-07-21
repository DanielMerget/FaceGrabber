#pragma once
#include "stdafx.h"

#include <Kinect.h>
#include <Kinect.Face.h>
#include <limits>

/**
 * \class	OutputStreamsUpdaterStragedy
 *
 * \brief	The OutputStreamsUpdaterStragedy defines method stubs which should called by the KinectHDFaceGrabber for each new
 * 			FaceFrame. Inherited classes can use the provided data to run various algorithmns. startFaceCollection should be called
 * 			before calling updateOutputStreams for tracked Kinect-FaceFrame. Afterwards stopFaceCollection should be called. 
 * 			OutputStreamsUpdaterStragedy hides the implementation to allow switching the implementation at run time.
 */

class OutputStreamsUpdaterStragedy
{
public:

	/**
	 * \fn	OutputStreamsUpdaterStragedy::OutputStreamsUpdaterStragedy();
	 *
	 * \brief	Default constructor.
	 */
	OutputStreamsUpdaterStragedy();

	/**
	 * \fn	OutputStreamsUpdaterStragedy::~OutputStreamsUpdaterStragedy();
	 *
	 * \brief	Destructor.
	 */
	~OutputStreamsUpdaterStragedy();

	/**
	 * \fn	bool OutputStreamsUpdaterStragedy::isValidCamSpacePoint(CameraSpacePoint camSpacePoint)
	 *
	 * \brief	Query if 'camSpacePoint' is valid camera space point.
	 *
	 * \param	camSpacePoint	The camera space point.
	 *
	 * \return	true if valid camera space point, false if not. Points with coordinates of infinity are treated as invalid points.
	 */
	bool isValidCamSpacePoint(CameraSpacePoint camSpacePoint){
		return (camSpacePoint.X >= - FLT_MAX && camSpacePoint.X <= FLT_MAX)
			&& (camSpacePoint.Y >= - FLT_MAX && camSpacePoint.Y <= FLT_MAX)
			&& (camSpacePoint.Z >= - FLT_MAX && camSpacePoint.Z <= FLT_MAX);
	}

	/**
	 * \fn	bool OutputStreamsUpdaterStragedy::isValidDepthPoint(DepthSpacePoint depthPoint)
	 *
	 * \brief	Query if 'depthPoint' is valid depth point. Coordinates of invalid points are outside 
	 * 			the depth frame size.
	 *
	 * \param	depthPoint	The depth point.
	 *
	 * \return	true if valid depth point, false if not.
	 */
	bool isValidDepthPoint(DepthSpacePoint depthPoint){
		return (depthPoint.X > 0 && depthPoint.X < m_depthWidth) && (depthPoint.Y > 0 && depthPoint.Y < m_depthWidth);
	}


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
	virtual HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace,std::string sKeyPoints) = 0;

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
	virtual void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

	/**
	 * \fn	virtual void OutputStreamsUpdaterStragedy::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer) = 0;
	 *
	 * \brief	Indicates that tracked/found faces will be provided with calls to updateOutputStreams.
	 *
	 * \param [in]	colorBuffer the current color Buffer
	 * \param [in]	depthBuffer the current depth Buffer
	 */
	virtual void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer,UINT16* alignedDepthBuffer,RGBQUAD* infraredBuffer,RGBQUAD* alignedInfraredBuffer) = 0;

	/**
	 * \fn	virtual void OutputStreamsUpdaterStragedy::stopFaceCollection() = 0;
	 *
	 * \brief	Stops face collection/processing.
	 */
	virtual void stopFaceCollection() = 0;

	/**
	* \fn	void RecordTabHandler::setCeterEnabled(bool enable);
	*
	* \brief	Enables or disables the centering of recorded Clouds.
	*
	* \param	enable	true to enable, false to disable.
	*/
	virtual void setCeterEnabled(bool enable) = 0;
protected:
	
	/** \brief	The coordinate mapper. */
	ICoordinateMapper*			m_pCoordinateMapper;


	/** \brief	Width of the depth frame. */
	int							m_depthWidth;

	/** \brief	Height of the depth frame. */
	int							m_depthHeight;

	/** \brief	Width of the color frame. */
	int							m_colorWidth;

	/** \brief	Height of the color frame. */
	int							m_colorHeight;
	
};

