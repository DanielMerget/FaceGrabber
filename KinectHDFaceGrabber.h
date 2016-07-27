//------------------------------------------------------------------------------
// <copyright file="FaceBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include "stdafx.h"

#include "resource.h"
#include "ImageRenderer.h"

#include <boost/signals2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mutex>
#include <condition_variable>
#include <opencv2/imgproc/imgproc.hpp>
#include "RecordingConfiguration.h"
#include "OutputStreamsUpdaterStragedy.h"
#include <array>
#include "CommonConfiguration.h"

// InfraredSourceValueMaximum is the highest value that can be returned in the InfraredFrame.
// It is cast to a float for readability in the visualization code.
#define InfraredSourceValueMaximum static_cast<float>(USHRT_MAX)

// The InfraredOutputValueMinimum value is used to set the lower limit, post processing, of the
// infrared data that we will render.
// Increasing or decreasing this value sets a brightness "wall" either closer or further away.
#define InfraredOutputValueMinimum 0.01f 

// The InfraredOutputValueMaximum value is the upper limit, post processing, of the
// infrared data that we will render.
#define InfraredOutputValueMaximum 1.0f

// The InfraredSceneValueAverage value specifies the average infrared value of the scene.
// This value was selected by analyzing the average pixel intensity for a given scene.
// Depending on the visualization requirements for a given application, this value can be
// hard coded, as was done here, or calculated by averaging the intensity for each pixel prior
// to rendering.
#define InfraredSceneValueAverage 0.08f

/// The InfraredSceneStandardDeviations value specifies the number of standard deviations
/// to apply to InfraredSceneValueAverage. This value was selected by analyzing data
/// from a given scene.
/// Depending on the visualization requirements for a given application, this value can be
/// hard coded, as was done here, or calculated at runtime.
#define InfraredSceneStandardDeviations 3.0f

/**
 * \class	KinectHDFaceGrabber
 *
 * \brief	KinectHDFaceGrabber uses the Kinect v2 to process the users faces found and tracked by the Kinect API.
 * 			With the set OutputStreamsUpdaterStragedy the information about the faces found by HDFace API can be
 * 			further	processed. The KinectHDFaceGrabber initializes the Depth, Color and HDFace Reader and grabs
 * 			the frames provided by the the reades each time the update() method is called. The current color
 * 			camera image with the tracked face points are render using the set ImageRenderer.
 */
class KinectHDFaceGrabber
{
    static const int cColorWidth  = 1920;
    static const int cColorHeight = 1080;

public:

	KinectHDFaceGrabber();

	~KinectHDFaceGrabber();

	/**
	 * \fn	void KinectHDFaceGrabber::update();
	 *
	 * \brief	This method have to be called if a new frame should be grabbed. With this call
	 * 			all  Depth, Color and HDFace Reader are updated and the results are provided to the
	 * 			OutputStreamsUpdaterStragedy for further processing.
	 */
	void update();

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initializeDefaultSensor();
	 *
	 * \brief	Initializes the Kinect v2 and all streams required by the KinectHDFaceGrabber:
	 *
	 * \return	A hResult.
	 */
	HRESULT	initializeDefaultSensor();

	/**
	 * \fn	ICoordinateMapper* KinectHDFaceGrabber::getCoordinateMapper();
	 *
	 * \brief	Getter for the Kinect Coordinate Mapper.
	 *
	 * \return	The coordinate mapper.
	 */
	ICoordinateMapper* getCoordinateMapper();

	/**
	 * \fn	void KinectHDFaceGrabber::setImageRenderer(ImageRenderer* renderer);
	 *
	 * \brief	Sets image renderer.
	 *
	 * \param [in]	renderer	ImageRenderer to render the current color image and the tracked face points.
	 */
	void setImageRenderer(ImageRenderer* renderer);

	/**
	 * \fn	void KinectHDFaceGrabber::setOutputStreamUpdater(std::shared_ptr<OutputStreamsUpdaterStragedy> outputStreamUpdater);
	 *
	 * \brief	Sets output stream updater.
	 *
	 * \param	outputStreamUpdater	The output stream updater for further processing of the streams provided by the kinect.
	 */
	void setOutputStreamUpdater(std::shared_ptr<OutputStreamsUpdaterStragedy> outputStreamUpdater);


	/** \brief	The status of the Kinect and Kinect HD face modelling. */
	boost::signals2::signal<bool(std::wstring, bool)> statusChanged;

	/**
	 * \fn	void KinectHDFaceGrabber::getColourAndDepthSize(int& depthWidth, int& depthHeight, int& colorWidth, int& colorHeight);
	 *
	 * \brief	Getter for the size of the colour and depth streams.
	 *
	 * \param [in,out]	depthWidth 	Width of the depth.
	 * \param [in,out]	depthHeight	Height of the depth.
	 * \param [in,out]	colorWidth 	Width of the color.
	 * \param [in,out]	colorHeight	Height of the color.
	 */
	void getColourAndDepthSize(int& depthWidth, int& depthHeight, int& colorWidth, int& colorHeight);

	void SetConfiguration(CommonConfigurationPtr commonConfiguration)
	{
		m_commonConfiguration = commonConfiguration;
	}

private:

	/**
	 * \fn	std::wstring KinectHDFaceGrabber::getCaptureStatusText(FaceModelBuilderCollectionStatus status);
	 *
	 * \brief	Gets capture status text.
	 *
	 * \param	status	The FaceModelBuilderCollectionStatus status.
	 *
	 * \return	The capture status as wstring.
	 */

	std::wstring getCaptureStatusText(FaceModelBuilderCollectionStatus status);

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initColorFrameReader();
	 *
	 * \brief	Initialises the color frame reader.
	 *
	 * \return	A hResult.
	 */
	HRESULT initColorFrameReader();

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initDepthFrameReader();
	 *
	 * \brief	Initialises the depth frame reader.
	 *
	 * \return	A hResult.
	 */
	HRESULT initDepthFrameReader();

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initHDFaceReader();
	 *
	 * \brief	Initialises the HD face reader.
	 *
	 * \return	A hResult.
	 */
	HRESULT initHDFaceReader();

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initDepthAndColorAlignment();
	 *
	 * \brief	Initialises Aligment of Depth and Color.
	 *
	 * \return	A hResult.
	 */

	HRESULT initDepthAndColorAlignment();
	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initInfraredFrameReader();
	 *
	 * \brief	Initialises the infrared frame reader.
	 *
	 * \return	A hResult.
	 */
	HRESULT initInfraredFrameReader();
		
	/**
	 * \fn	HRESULT KinectHDFaceGrabber::initBodyIndexReader();
	 *
	 * \brief	Initialises the Body index frame reader.
	 *
	 * \return	A hResult.
	 */
	HRESULT initBodyIndexFrameReader();

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::alighDepthWithColor();
	 *
	 * \brief	aligh depth data with color data.
	 *
	 * \return	A hResult.
	 */
	HRESULT alighDepthWithColor();

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::getInfraredFrame();
	 *
	 * \brief	get infrared data.
	 *
	 * \return	A hResult.
	 */
	HRESULT getInfraredFrame();

	/// <summary>
    /// Handle new infrared data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    /// </summary>
    void                    ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nHeight, int nWidth);

	    /// <summary>
    /// Computes the face result text layout position by adding an offset to the corresponding 
    /// body's head joint in camera space and then by projecting it to screen space
    /// </summary>
    /// <param name="pBody">pointer to the body data</param>
    /// <param name="pFaceTextLayout">pointer to the text layout position in screen space</param>
    /// <returns>indicates success or failure</returns>
		HRESULT GetFaceTextPositionInColorSpace(IBody* pBody, D2D1_POINT_2F* pFaceTextLayout);

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::getFiveKeyPointsOnFaces(bool bFaceTracked);
	 *
	 * \brief	get five key points on faces.
	 *
	 * \param [in]	bFaceTracked  	
	 * 	
	 * \return	A hResult.
	 */

	HRESULT getFiveKeyPointsOnFaces(int iFace,IBody* ppBodies,bool bHaveBodyData, bool bFaceTracked);

	/**
	 * \fn	HRESULT KinectHDFaceGrabber::updateHDFaceTrackingID(IHighDefinitionFaceFrameSource* faceFrame, IBody* trackedBody);
	 *
	 * \brief	Associates the HD face frame with the tracked body.
	 *
	 * \param [in,out]	faceFrame  	to be updated
	 * \param [in,out]	trackedBody	the tracked body to Associates with the IHighDefinitionFaceFrameSource
	 *
	 * \return	A hResult.
	 */
	HRESULT updateHDFaceTrackingID(IHighDefinitionFaceFrameSource* faceFrame, IBody* trackedBody);

	/**
	 * \fn	void KinectHDFaceGrabber::updateFaceModelStatusOfFaceModelBuilder(IFaceModelBuilder** faceModelBuilder, IFaceModel* faceModel);
	 *
	 * \brief	Updates the face model status using the face model builder.
	 *
	 * \param [in]	faceModelBuilder	The FaceModelBuilder
	 * \param [in]	faceModel			The faceModel
	 */
	void updateFaceModelStatusOfFaceModelBuilder(IFaceModelBuilder** faceModelBuilder, IFaceModel* faceModel);

    /**
     * \fn	void KinectHDFaceGrabber::renderColorFrameAndProcessFaces();
     *
     * \brief	renders the current color image and
     * 			processes the HDFace, depth and color streams by triggering the OutputStreamStragedy
     * 			and ImageRenderer.
     */

	void renderColorFrameAndProcessFaces();

	/**
	 * \fn	void KinectHDFaceGrabber::processFaces();
	 *
	 * \brief	updates the face data and triggers the OutputStreamStragedy.
	 */
	void processFaces();

    /**
     * \fn	HRESULT KinectHDFaceGrabber::updateBodyData(IBody** ppBodies);
     *
     * \brief	Updates the body data with the current body frame.
     *
     * \param [in,out]	ppBodies	the body to be updated.
     *
     * \return	A hResult.
     */
    HRESULT	updateBodyData(IBody** body);

    
    /** \brief	The kinect sensor. */
    IKinectSensor*			m_pKinectSensor;

    
    /** \brief	The coordinate mapper. */
    ICoordinateMapper*		m_pCoordinateMapper;

    
    /** \brief	The color frame reader. */
    IColorFrameReader*		m_pColorFrameReader;


	/** \brief	The depth frame reader. */
	IDepthFrameReader*		m_pDepthFrameReader;

    
    /** \brief	The body frame reader. */
    IBodyFrameReader*		m_pBodyFrameReader;

	// Infrared reader
    IInfraredFrameReader*  m_pInfraredFrameReader;


	/** \brief	The multi source frame reader. */
	IMultiSourceFrameReader * m_pMultiSourceFrameReader;
	IBodyIndexFrameReader * m_pBodyIndexFrameReader;

    /** \brief	The face frame sources for each. */
    IFaceFrameSource*		m_pFaceFrameSources[BODY_COUNT];

    
    /** \brief	The face frame readers for each body*/
    IFaceFrameReader*		m_pFaceFrameReaders[BODY_COUNT];

	
	/** \brief	The HD face source fo each body*/
	IHighDefinitionFaceFrameSource* m_pHDFaceSource[BODY_COUNT];

	/** \brief	The HD face reader for each body. */
	IHighDefinitionFaceFrameReader* m_pHDFaceReader[BODY_COUNT];

	/** \brief	The face alignment for each body. */
	IFaceAlignment* m_pFaceAlignment[BODY_COUNT];


	/** \brief	The face model builder for each body. */
	IFaceModelBuilder* m_pFaceModelBuilder[BODY_COUNT];


	/** \brief	The face model for each body. */
	IFaceModel* m_pFaceModel[BODY_COUNT];

    
    /** \brief	The Image Reader to draw the color stream & HD Face points. */
    ImageRenderer*				m_pDrawDataStreams;


	/** \brief	The output stream updater for processing kinect data streams. */
	std::shared_ptr<OutputStreamsUpdaterStragedy> m_outputStreamUpdater;


	/** \brief	The HDFace points in Cam Space for each body. */
	std::vector<CameraSpacePoint>m_HDFaceDetectedPointsCamSpace[BODY_COUNT];

	/** \brief	The HDFace points in Color Space for each body. */
	std::vector<ColorSpacePoint> m_HDFaceDetectedPointsColorSpace[BODY_COUNT];
	

	/** \brief	Buffer for depth data. */
	std::vector<UINT16>			m_depthBuffer;
	std::vector<UINT16>			m_alignedRawDepthBuffer;

	/** \brief	Buffer for color data. */
	std::vector<RGBQUAD>		m_colorBuffer;

	/** \brief	Buffer for aligned depth data. */
	std::vector<RGBQUAD>		m_alighedDepthBuffer;

	/** \brief	Buffer for infrared data. */
	std::vector<RGBQUAD>		m_alighedInfraredBuffer;

	/** \brief	Buffer for aligned color data. */
	std::vector<RGBQUAD>		m_alighedColorBuffer;

		/** \brief	Buffer for aligned color data. */
	std::vector<RGBQUAD>		m_infraredBuffer;


	std:: vector <std::array<PointF, FacePointType::FacePointType_Count>> m_facePoints;
	Vector4 m_faceRotation[BODY_COUNT];
	RectI m_faceBox[BODY_COUNT];
	
	/** \brief	Width of the depth. */
	int							m_depthWidth;

	/** \brief	Height of the depth. */
	int							m_depthHeight;

		/** \brief	Width of the depth. */
	int							m_infraredWidth;

	/** \brief	Height of the depth. */
	int							m_infraredHeight;


	/** \brief	Width of the color. */
	int							m_colorWidth;


	/** \brief	Height of the color. */
	int							m_colorHeight;



	CommonConfigurationPtr m_commonConfiguration;


	void UpdateFrameRate();
	UINT                m_fps;
    UINT                m_frameCount;
    UINT                m_lastFrameCount;
    DWORD               m_lastTick;
};

