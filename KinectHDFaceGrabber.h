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
#include <boost/signals2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mutex>
#include <condition_variable>
#include <opencv2/imgproc/imgproc.hpp>
#include "OutputStreamsUpdaterStragedy.h"

class KinectHDFaceGrabber
{
    static const int       cColorWidth  = 1920;
    static const int       cColorHeight = 1080;

public:

	KinectHDFaceGrabber();

	~KinectHDFaceGrabber();


	void					update();

	HRESULT					initializeDefaultSensor();
	ICoordinateMapper*		getCoordinateMapper();

	void setImageRenderer(ImageRenderer* renderer);
	void setOutputStreamUpdater(std::shared_ptr<OutputStreamsUpdaterStragedy> outputStreamUpdater);

	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> cloudUpdated;
	boost::signals2::signal<void(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)> depthCloudUpdated;

	boost::signals2::signal<void(const unsigned char *data, unsigned width, unsigned height)> imageUpdated;

	boost::signals2::signal<bool(std::wstring, bool)> statusChanged;
	
	void getColourAndDepthSize(int& depthWidth, int& depthHeight, int& colorWidth, int& colorHeight);
private:
	std::wstring getCaptureStatusText(FaceModelBuilderCollectionStatus status);

	HRESULT initColorFrameReader();
	HRESULT initDepthFrameReader();
	HRESULT initHDFaceReader();

	HRESULT updateHDFaceTrackingID(IHighDefinitionFaceFrameSource* faceFrame, IBody* trackedBody);
	void updateFaceModelStatusOfFaceModelBuilder(IFaceModelBuilder** faceModelBuilder, IFaceModel* faceModel);

	void updateDepthCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertDepthBufferToPointCloud();


    void					updateHDFaceAndColor();

	void					processFaces();

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
    ImageRenderer*				m_pDrawDataStreams;

	std::shared_ptr<OutputStreamsUpdaterStragedy> m_outputStreamUpdater;

	std::vector<CameraSpacePoint>m_HDFaceDetectedPointsCamSpace[BODY_COUNT];
	std::vector<ColorSpacePoint> m_HDFaceDetectedPointsColorSpace[BODY_COUNT];
	

	std::vector<UINT16>			m_depthBuffer;
	std::vector<RGBQUAD>		m_colorBuffer;
	std::mutex					m_depthBufferMutex;
	std::condition_variable		m_bufferCondVariable;

	bool						m_depthImageProcessed;
	std::mutex					m_depthImageProcessedLock;

	int							m_depthWidth;
	int							m_depthHeight;
	int							m_colorWidth;
	int							m_colorHeight;
};

