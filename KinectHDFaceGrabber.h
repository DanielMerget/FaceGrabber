//------------------------------------------------------------------------------
// <copyright file="FaceBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#undef max
#undef min
#include <boost/signals.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mutex>
#include <condition_variable>

class KinectHDFaceGrabber
{
    static const int       cColorWidth  = 1920;
    static const int       cColorHeight = 1080;

public:
    /// <summary>
    /// Constructor
    /// </summary>
	KinectHDFaceGrabber();

    /// <summary>
    /// Destructor
    /// </summary>
	~KinectHDFaceGrabber();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
   /* static LRESULT CALLBACK	MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);*/

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    /*LRESULT CALLBACK		DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);*/

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
   /* int						Run(HINSTANCE hInstance, int nCmdShow);*/

	/// <summary>
	/// Main processing function
	/// </summary>
	void					update();
	
	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success else the failure code</returns>
	HRESULT					initializeDefaultSensor();


	void setImageRenderer(ImageRenderer* renderer);

	boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> cloudUpdated;
	boost::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> depthCloudUpdated;

	boost::signal<void(const unsigned char *data, unsigned width, unsigned height)> imageUpdated;

	boost::signal<bool(std::wstring, bool)> statusChanged;
	
private:
	std::wstring getCaptureStatusText(FaceModelBuilderCollectionStatus status);

	HRESULT initColorFrameReader();
	HRESULT initDepthFrameReader();
	HRESULT initHDFaceReader();

	HRESULT updateHDFaceTrackingID(IHighDefinitionFaceFrameSource* faceFrame, IBody* trackedBody);
	void updateFaceModelStatusOfFaceModelBuilder(IFaceModelBuilder** faceModelBuilder, IFaceModel* faceModel);

	void updateDepthCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDepthBufferToPointCloud();
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertKinectRGBPointsToPointCloud(std::vector<CameraSpacePoint>& renderPoints, const RGBQUAD* pBuffer, const int imageWidth, const int imageHeight);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertKinectRGBPointsToPointCloud(const std::vector<CameraSpacePoint>& renderPoints, const std::vector<ColorSpacePoint>& imagePoints);

    /// <summary>
    /// Renders the color and face streams
    /// </summary>			
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="pBuffer">pointer to frame data</param>
    /// <param name="nWidth">width (in pixels) of input image data</param>
    /// <param name="nHeight">height (in pixels) of input image data</param>
    void					updateHDFaceAndColor();

    /// <summary>
    /// Processes new face frames
    /// </summary>
	void					processFaces();


    /// <summary>
    /// Updates body data
    /// </summary>
    /// <param name="ppBodies">pointer to the body data storage</param>
    /// <returns>indicates success or failure</returns>
    HRESULT					updateBodyData(IBody** ppBodies);


    // Current Kinect
    IKinectSensor*			m_pKinectSensor;

    // Coordinate mapper
    ICoordinateMapper*		m_pCoordinateMapper;

    // Color reader
    IColorFrameReader*		m_pColorFrameReader;

	IDepthFrameReader*		m_pDepthFrameReader;

    // Body reader
    IBodyFrameReader*		m_pBodyFrameReader;

    // Face sources
    IFaceFrameSource*		m_pFaceFrameSources[BODY_COUNT];

    // Face readers
    IFaceFrameReader*		m_pFaceFrameReaders[BODY_COUNT];

	//HDFace
	IHighDefinitionFaceFrameSource* m_pHDFaceSource[BODY_COUNT];;

	IHighDefinitionFaceFrameReader* m_pHDFaceReader[BODY_COUNT];

	IFaceAlignment* m_pFaceAlignment[BODY_COUNT];

	IFaceModelBuilder* m_pFaceModelBuilder[BODY_COUNT];

	IFaceModel* m_pFaceModel[BODY_COUNT];

    //// Direct2D
    ImageRenderer*			m_pDrawDataStreams;
    RGBQUAD*				m_pColorRGBX;	

	std::vector<CameraSpacePoint> m_HDFaceDetectedPointsCamSpace[BODY_COUNT];
	std::vector<ColorSpacePoint> m_HDFaceDetectedPointsColorSpace[BODY_COUNT];
	

	std::vector<UINT16> m_depthBuffer;
	std::vector<RGBQUAD> m_colorBuffer;
	std::mutex m_depthBufferMutex;
	std::condition_variable m_bufferCondVariable;

	bool m_depthImageProcessed;
	std::mutex m_depthImageProcessedLock;

	int m_depthWidth;
	int m_depthHeight;
	int m_colorWidth;
	int m_colorHeight;
};

