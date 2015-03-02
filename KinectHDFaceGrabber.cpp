//------------------------------------------------------------------------------
// <copyright file="FaceBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "KinectHDFaceGrabber.h"
#include <iostream>
#include <vector>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <future>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/imgproc/imgproc.hpp>

// face property text layout offset in X axis
static const float c_FaceTextLayoutOffsetX = -0.1f;

// face property text layout offset in Y axis
static const float c_FaceTextLayoutOffsetY = -0.125f;

// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures = 
    FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
    | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
    | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
    | FaceFrameFeatures::FaceFrameFeatures_Happy
    | FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
    | FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
    | FaceFrameFeatures::FaceFrameFeatures_MouthOpen
    | FaceFrameFeatures::FaceFrameFeatures_MouthMoved
    | FaceFrameFeatures::FaceFrameFeatures_LookingAway
    | FaceFrameFeatures::FaceFrameFeatures_Glasses
    | FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;


/// <summary>
/// Constructor
/// </summary>
KinectHDFaceGrabber::KinectHDFaceGrabber() :
    m_pKinectSensor(nullptr),
    m_pCoordinateMapper(nullptr),
	m_pColorFrameReader(nullptr),
    m_pDrawDataStreams(nullptr),
    m_pBodyFrameReader(nullptr),
	m_pDepthFrameReader(nullptr),
	m_depthImageProcessed(true)
{
    for (int i = 0; i < BODY_COUNT; i++)
    {
        m_pFaceFrameSources[i] = nullptr;
        m_pFaceFrameReaders[i] = nullptr;
    }
}

//// Direct2D
ImageRenderer*				m_pDrawDataStreams;
/// <summary>
/// Destructor
/// </summary>
KinectHDFaceGrabber::~KinectHDFaceGrabber()
{
    // clean up Direct2D renderer
    if (m_pDrawDataStreams)
    {
        delete m_pDrawDataStreams;
        m_pDrawDataStreams = nullptr;
    }

    //// clean up Direct2D
    //SafeRelease(m_pD2DFactory);

    // done with face sources and readers
    for (int i = 0; i < BODY_COUNT; i++)
    {
        SafeRelease(m_pFaceFrameSources[i]);
        SafeRelease(m_pFaceFrameReaders[i]);		
		SafeRelease(m_pHDFaceSource[i]);
		SafeRelease(m_pHDFaceReader[i]);

		SafeRelease(m_pFaceAlignment[i]);
		SafeRelease(m_pFaceModelBuilder[i]);
		SafeRelease(m_pFaceModel[i]);
    }

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with color frame reader
    SafeRelease(m_pColorFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

	SafeRelease(m_pDepthFrameReader);
    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}




HRESULT KinectHDFaceGrabber::initColorFrameReader()
{
	IColorFrameSource* pColorFrameSource = nullptr;
	HRESULT hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
	
	if (SUCCEEDED(hr)){
		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}

	IFrameDescription* pFrameDescription = nullptr;
	if (SUCCEEDED(hr))
	{
		hr = pColorFrameSource->get_FrameDescription(&pFrameDescription);
	}

	if (SUCCEEDED(hr))
	{
		hr = pFrameDescription->get_Width(&m_colorWidth);
	}

	if (SUCCEEDED(hr))
	{
		hr = pFrameDescription->get_Height(&m_colorHeight);
	}

	if (SUCCEEDED(hr)){
		m_colorBuffer.resize(m_colorHeight * m_colorWidth);
	}

	SafeRelease(pFrameDescription);
	SafeRelease(pColorFrameSource);
	
	return hr;
}

HRESULT KinectHDFaceGrabber::initDepthFrameReader()
{
	
	IDepthFrameSource* depthFrameSource = nullptr;
	
	HRESULT hr = m_pKinectSensor->get_DepthFrameSource(&depthFrameSource);
	
	IFrameDescription* frameDescription = nullptr;
	if (SUCCEEDED(hr)){
		hr = depthFrameSource->get_FrameDescription(&frameDescription);
	}

	if (SUCCEEDED(hr)){
		hr = frameDescription->get_Width(&m_depthWidth);
	}

	if (SUCCEEDED(hr)){
		hr = frameDescription->get_Height(&m_depthHeight);
	}

	if (SUCCEEDED(hr)){
		m_depthBuffer.resize(m_depthHeight * m_depthWidth);
	}

	SafeRelease(frameDescription);
	if (SUCCEEDED(hr)){
		hr = depthFrameSource->OpenReader(&m_pDepthFrameReader);
	}

	SafeRelease(depthFrameSource);
	return hr;
}

HRESULT KinectHDFaceGrabber::initHDFaceReader()
{
	IBodyFrameSource* pBodyFrameSource = nullptr;
	UINT32 vertices = 0;
	HRESULT hr = GetFaceModelVertexCount(&vertices);

	if (SUCCEEDED(hr)){
		hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
	}
	std::vector<std::vector<float>> deformations(BODY_COUNT, std::vector<float>(FaceShapeDeformations::FaceShapeDeformations_Count));

	if (SUCCEEDED(hr)){
		// create a face frame source + reader to track each body in the fov
		for (int i = 0; i < BODY_COUNT; i++){
			if (SUCCEEDED(hr)){
				// create the face frame source by specifying the required face frame features
				hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
			}

			if (SUCCEEDED(hr)){
				// open the corresponding reader
				hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
			}
			std::vector<std::vector<float>> deformations(BODY_COUNT, std::vector<float>(FaceShapeDeformations::FaceShapeDeformations_Count));

			if (SUCCEEDED(hr)){
				hr = CreateHighDefinitionFaceFrameSource(m_pKinectSensor, &m_pHDFaceSource[i]);
				m_pHDFaceSource[i]->put_TrackingQuality(FaceAlignmentQuality_High);
			}

			if (SUCCEEDED(hr)){
				hr = m_pHDFaceSource[i]->OpenReader(&m_pHDFaceReader[i]);
			}

			if (SUCCEEDED(hr)){
				hr = m_pHDFaceSource[i]->OpenModelBuilder(FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &m_pFaceModelBuilder[i]);
			}

			if (SUCCEEDED(hr)){
				hr = m_pFaceModelBuilder[i]->BeginFaceDataCollection();
			}

			if (SUCCEEDED(hr)){
				hr = CreateFaceAlignment(&m_pFaceAlignment[i]);
			}
			if (SUCCEEDED(hr)){
				m_HDFaceDetectedPointsCamSpace[i].resize(vertices);
				m_HDFaceDetectedPointsColorSpace[i].resize(vertices);
			}
			// Create Face Model
			hr = CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, deformations[i].data(), &m_pFaceModel[i]);
			if (FAILED(hr)){
				std::cerr << "Error : CreateFaceModel()" << std::endl;
				return -1;
			}
		}
		
		if (SUCCEEDED(hr)){
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		SafeRelease(pBodyFrameSource);
	}

	
	if (SUCCEEDED(hr)){
		//std::thread updateThread(&KinectHDFaceGrabber::updateDepthCloud, this);
		//updateThread.detach();
	}
	return hr;
}
/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>S_OK on success else the failure code</returns>
HRESULT KinectHDFaceGrabber::initializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }
	
    if (m_pKinectSensor)
    {
        // Initialize Kinect and get color, body and face readers
        
        
		IMultiSourceFrameReader* reader;
		
        hr = m_pKinectSensor->Open();
		
		if (SUCCEEDED(hr)){
			hr = initColorFrameReader();
		}
		
		if (SUCCEEDED(hr)){
			hr = initDepthFrameReader();
		}
		
		if (SUCCEEDED(hr)){
			hr = initHDFaceReader();
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
    }
	
    if (!m_pKinectSensor || FAILED(hr))
    {
		statusChanged(L"No ready Kinect found!", true);
        return E_FAIL;
    }
	
    return hr;
}

/// <summary>
/// Main processing function
/// </summary>
void KinectHDFaceGrabber::update()
{
	if (!m_pColorFrameReader || !m_pBodyFrameReader){
		return;
	}

	//m_depthImageProcessedLock.lock();
	//if (!m_depthImageProcessed){
	//	OutputDebugString(L"discarding\n");
	//	m_depthImageProcessedLock.unlock();
	//	return;
	//}
	//m_depthImageProcessedLock.unlock();
	//OutputDebugString(L"got the lock\n");

	bool produce[BODY_COUNT] = { false };
	
	//std::unique_lock<std::mutex> lock(m_depthBufferMutex);
	//if (!m_depthBufferMutex.try_lock()){
	//	//conversion is done on a seperate thread => conversion thread still working on old buffer
	//	//discard this frame and try with next one
	//	OutputDebugString(L"discarding\n");
	//	return;
	//}
	
    IColorFrame* pColorFrame = nullptr;
    HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	IDepthFrame* depthFrame = nullptr;
	if (SUCCEEDED(hr)){
		hr = m_pDepthFrameReader->AcquireLatestFrame(&depthFrame);
	}
	
    if (SUCCEEDED(hr)){
        ColorImageFormat imageFormat = ColorImageFormat_None;
       
        if (SUCCEEDED(hr)){
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }
	
        if (SUCCEEDED(hr)){
			UINT nBufferSize = m_colorWidth * m_colorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(m_colorBuffer.data()), ColorImageFormat_Bgra);
        }
		
		
		if (SUCCEEDED(hr)){
			hr = depthFrame->CopyFrameDataToArray(m_depthBuffer.size(), &m_depthBuffer[0]);
		}
		if (SUCCEEDED(hr)){
			updateHDFaceAndColor();
			//const unsigned *data, unsigned width, unsigned height)
			//imageUpdated(reinterpret_cast<unsigned char*>(m_colorBuffer.data()), m_colorWidth, m_colorHeight);
		}
		
		
        //if (SUCCEEDED(hr)){
		//	std::lock_guard<std::mutex> lock(m_depthBufferMutex);
			
		//	updateDepthCloud();
		//	//std::async(std::launch::async, &KinectHDFaceGrabber::updateDepthCloud, this);
		//	
		//	//std::async
		//	std::thread depthThread(&KinectHDFaceGrabber::updateDepthCloud, this);
		//	depthThread.detach();
        //}
		
	}
	
	//m_depthBufferMutex.unlock();
	//m_bufferCondVariable.notify_all();
	SafeRelease(depthFrame);
	SafeRelease(pColorFrame);  
}



void KinectHDFaceGrabber::updateDepthCloud()
{
	while (true){
		std::unique_lock<std::mutex> lock(m_depthBufferMutex);
		m_bufferCondVariable.wait(lock);
		
		m_depthImageProcessedLock.lock();
		m_depthImageProcessed = false;
		m_depthImageProcessedLock.unlock();

		LARGE_INTEGER before = { 0 };
		QueryPerformanceCounter(&before);
	

		
		//std::vector<CameraSpacePoint> depthPointsInCameraSpace(m_depthBuffer.size());
		//std::vector<ColorSpacePoint> renderPoints(vertex);
		//m_pCoordinateMapper->MapDepthFrameToCameraSpace(m_depthBuffer.size(), m_depthBuffer.data(), depthPointsInCameraSpace.size(), depthPointsInCameraSpace.data());
		auto cloud = convertDepthBufferToPointCloud();

		LARGE_INTEGER afterCopy = { 0 };
		QueryPerformanceCounter(&afterCopy);
		double time1 = afterCopy.QuadPart - before.QuadPart;
		//618452
		//759616
		m_depthImageProcessedLock.lock();
		m_depthImageProcessed = true;
		m_depthImageProcessedLock.unlock();

		depthCloudUpdated(cloud);

		LARGE_INTEGER last = { 0 };
		QueryPerformanceCounter(&last);

		double time2 = last.QuadPart - afterCopy.QuadPart;
		std::cout << "time1" << time1 << "time: " << time2 << std::endl;
		std::wstring result = L"time1: ";
		result += time1;
		result += L"\ntime2:";
		result += time2;
		result += L"\n";
		_In_z_ WCHAR* szMessage = &result[0];
		OutputDebugString(szMessage);	
	}
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectHDFaceGrabber::convertDepthBufferToPointCloud()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pointCloud->width = static_cast<uint32_t>(m_depthWidth);
	pointCloud->height = static_cast<uint32_t>(m_depthHeight);
	pointCloud->is_dense = false;
	HRESULT hr;
	for (int y = 0; y < m_depthHeight; y++){
		for (int x = 0; x < m_depthWidth; x++){
			pcl::PointXYZRGB point;
			DepthSpacePoint depthPoint;
			depthPoint.X = static_cast<float>(x);
			depthPoint.Y = static_cast<float>(y);
			UINT16 depthOfCurrentPoint = m_depthBuffer[y * m_depthWidth + x];

			ColorSpacePoint colorPoint;
			hr = m_pCoordinateMapper->MapDepthPointToColorSpace(depthPoint, depthOfCurrentPoint, &colorPoint);
			if (FAILED(hr)){
				continue;
			}
			int colorPixelMidX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
			int colorPixelMidY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));
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
			if (isInColor && isInDepth){
				pointCloud->push_back(point);
			}
		}
	}
	return pointCloud;
}


/// <summary>
/// Renders the color and face streams
/// </summary>
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
void KinectHDFaceGrabber::updateHDFaceAndColor()
{

    HRESULT hr;
    hr = m_pDrawDataStreams->beginDrawing();

    if (SUCCEEDED(hr))
    {
        // Make sure we've received valid color data
		if ((m_colorWidth > 0 ) && (m_colorHeight > 0))
        {
            // Draw the data with Direct2D
            hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_colorBuffer.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
        }
        else
        {
            // Recieved invalid data, stop drawing
            hr = E_INVALIDARG;
        }

        if (SUCCEEDED(hr))
        {
            // begin processing the face frames
			processFaces();
        }

        m_pDrawDataStreams->endDrawing();
    }
}

HRESULT KinectHDFaceGrabber::updateHDFaceTrackingID(IHighDefinitionFaceFrameSource* faceFrame, IBody* trackedBody)
{
	BOOLEAN bTrackingIdValid = false;
	HRESULT hr = faceFrame->get_IsTrackingIdValid(&bTrackingIdValid);
	if (!bTrackingIdValid){
		BOOLEAN bTracked = false;
		hr = trackedBody->get_IsTracked(&bTracked);
		if (SUCCEEDED(hr) && bTracked){
			// Set TrackingID to Detect Face
			UINT64 trackingId = _UI64_MAX;
			hr = trackedBody->get_TrackingId(&trackingId);
			if (SUCCEEDED(hr)){
				hr = faceFrame->put_TrackingId(trackingId);
			}
		}
	}
	return hr;
}

void KinectHDFaceGrabber::updateFaceModelStatusOfFaceModelBuilder(IFaceModelBuilder** faceModelBuilder, IFaceModel* faceModel)
{
	FaceModelBuilderCollectionStatus status;
	HRESULT hr = faceModelBuilder[0]->get_CollectionStatus(&status);
	if (SUCCEEDED(hr)){
		std::wstring statusString = getCaptureStatusText(status);
		statusChanged(statusString, true);
		if (status == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete){
			std::cout << "Status : Complete" << std::endl;

			IFaceModelData* pFaceModelData = nullptr;
			hr = faceModelBuilder[0]->GetFaceData(&pFaceModelData);
			if (SUCCEEDED(hr) && pFaceModelData != nullptr){
				hr = pFaceModelData->ProduceFaceModel(&faceModel);
			}
			faceModelBuilder[0]->Release();
			SafeRelease(pFaceModelData);
			faceModelBuilder[0] = nullptr;
		}
	}
}

/// <summary>
/// Processes new face frames
/// </summary>
void KinectHDFaceGrabber::processFaces()
{
    HRESULT hr;
    IBody* ppBodies[BODY_COUNT] = {0};
    bool bHaveBodyData = SUCCEEDED( updateBodyData(ppBodies) );
	if (!bHaveBodyData)
		return;

	UINT32 vertex = 0;
	hr = GetFaceModelVertexCount(&vertex); // 1347
    // iterate through each face reader
    for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
    {
		updateHDFaceTrackingID(m_pHDFaceSource[iFace], ppBodies[iFace]);

		IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
		hr = m_pHDFaceReader[iFace]->AcquireLatestFrame(&pHDFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && pHDFaceFrame != nullptr){
			hr = pHDFaceFrame->get_IsFaceTracked(&bFaceTracked);
		}
		if (SUCCEEDED(hr) && bFaceTracked){
			hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment[iFace]);
		}
		if (FAILED(hr) || m_pFaceAlignment[iFace] == nullptr){
			continue;
		}
		if (m_pFaceModelBuilder[iFace] != nullptr){
			updateFaceModelStatusOfFaceModelBuilder(&m_pFaceModelBuilder[iFace], m_pFaceModel[iFace]);
		}
		
		if (m_outputStreamUpdater){
			hr = m_outputStreamUpdater->updateOutputStreams(m_pFaceModel[iFace], m_pFaceAlignment[iFace], 
				std::min(m_HDFaceDetectedPointsCamSpace[iFace].size(), m_HDFaceDetectedPointsColorSpace[iFace].size()), 
				m_HDFaceDetectedPointsCamSpace[iFace].data(), m_HDFaceDetectedPointsColorSpace[iFace].data(),
				m_colorBuffer.data(), m_depthBuffer.data());
		}
			
		if (SUCCEEDED(hr)){
			m_pDrawDataStreams->drawPoints(m_HDFaceDetectedPointsColorSpace[iFace]);
		}		
    }

    if (bHaveBodyData)
    {
        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }
}
ICoordinateMapper* KinectHDFaceGrabber::getCoordinateMapper()
{
	return m_pCoordinateMapper;
}

/// <summary>
/// Updates body data
/// </summary>
/// <param name="ppBodies">pointer to the body data storage</param>
/// <returns>indicates success or failure</returns>
HRESULT KinectHDFaceGrabber::updateBodyData(IBody** ppBodies)
{
    HRESULT hr = E_FAIL;

    if (m_pBodyFrameReader != nullptr)
    {
        IBodyFrame* pBodyFrame = nullptr;
        hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
        }
        SafeRelease(pBodyFrame);    
    }

    return hr;
}


std::wstring KinectHDFaceGrabber::getCaptureStatusText(FaceModelBuilderCollectionStatus status)
{

	std::wstring result = L"";
	if ((status & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded) != 0){
		result += L"  Front View Needed";
	}
	if ((status & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded) != 0){
		result += L" Left Views Needed";
	}
	if ((status & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_MoreFramesNeeded) != 0){
		result += L" More Frames needed";
	}
	if ((status & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded) != 0){
		result += L" Right Views needed";
	}
	if ((status & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded) != 0){
		result += L" Tilted Up Views needed";
	}
	if ((status & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete) != 0){
		result += L" Completed";
	}
	return result;
}


void KinectHDFaceGrabber::setImageRenderer(ImageRenderer* renderer){
	m_pDrawDataStreams = renderer;
}

void KinectHDFaceGrabber::setOutputStreamUpdater(std::shared_ptr<OutputStreamsUpdaterStragedy> outputStreamUpdater)
{
	m_outputStreamUpdater = outputStreamUpdater;
}

void KinectHDFaceGrabber::getColourAndDepthSize(int& depthWidth, int& depthHeight, int& colorWidth, int& colorHeight)
{
	depthWidth = m_depthWidth;
	depthHeight = m_depthHeight;
	colorWidth = m_colorWidth;
	colorHeight = m_colorHeight;
}