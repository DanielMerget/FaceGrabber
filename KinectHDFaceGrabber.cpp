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
    m_pColorRGBX(nullptr),
    m_pBodyFrameReader(nullptr),
	m_pDepthFrameReader(nullptr),
	m_depthImageProcessed(true)
{
    for (int i = 0; i < BODY_COUNT; i++)
    {
        m_pFaceFrameSources[i] = nullptr;
        m_pFaceFrameReaders[i] = nullptr;
    }
	
    // create heap storage for color pixel data in RGBX format
    m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}


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

    if (m_pColorRGBX)
    {
        delete [] m_pColorRGBX;
        m_pColorRGBX = nullptr;
    }

    //// clean up Direct2D
    //SafeRelease(m_pD2DFactory);

    // done with face sources and readers
    for (int i = 0; i < BODY_COUNT; i++)
    {
        SafeRelease(m_pFaceFrameSources[i]);
        SafeRelease(m_pFaceFrameReaders[i]);		
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


void KinectHDFaceGrabber::setImageRenderer(ImageRenderer* renderer){
	m_pDrawDataStreams = renderer;
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
	HRESULT hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
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


	UINT32 vertices = 0;

	if (SUCCEEDED(hr)){
		hr = GetFaceModelVertexCount(&vertices);
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
		BOOLEAN bTrackingIdValid = false;
		hr = m_pHDFaceSource[iFace]->get_IsTrackingIdValid(&bTrackingIdValid);
		if (!bTrackingIdValid){
			BOOLEAN bTracked = false;
			hr = ppBodies[iFace]->get_IsTracked(&bTracked);
			if (SUCCEEDED(hr) && bTracked){

				// Set TrackingID to Detect Face
				UINT64 trackingId = _UI64_MAX;
				hr = ppBodies[iFace]->get_TrackingId(&trackingId);
				if (SUCCEEDED(hr)){
					m_pHDFaceSource[iFace]->put_TrackingId(trackingId);
				}
			}
		}
	
		IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
		hr = m_pHDFaceReader[iFace]->AcquireLatestFrame(&pHDFaceFrame);
		
		if (SUCCEEDED(hr) && pHDFaceFrame != nullptr){
			BOOLEAN bFaceTracked = false;
			hr = pHDFaceFrame->get_IsFaceTracked(&bFaceTracked);
			if (SUCCEEDED(hr) && bFaceTracked){
				hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment[iFace]);
				if (SUCCEEDED(hr) && m_pFaceModelBuilder[iFace] != nullptr && m_pFaceAlignment[iFace] != nullptr && m_pFaceModel[iFace] != nullptr){
					static bool isCompleted = false;
					
					FaceModelBuilderCollectionStatus status;
					hr = m_pFaceModelBuilder[iFace]->get_CollectionStatus(&status);
					std::wstring statusString = getCaptureStatusText(status);
					statusChanged(statusString, true);
					if (status == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete){
						std::cout << "Status : Complete" << std::endl;
						
						IFaceModelData* pFaceModelData = nullptr;
						hr = m_pFaceModelBuilder[iFace]->GetFaceData(&pFaceModelData);
						if (SUCCEEDED(hr) && pFaceModelData != nullptr){
							if (!isCompleted){
								hr = pFaceModelData->ProduceFaceModel(&m_pFaceModel[iFace]);
								isCompleted = true;
							}
						}
						m_pFaceModelBuilder[iFace]->Release();
						SafeRelease(pFaceModelData);
						//m_pFaceModelBuilder[iFace]->Release();
						m_pFaceModelBuilder[iFace] = nullptr;
					}
				}
				std::vector<CameraSpacePoint> facePoints(vertex);
				std::vector<ColorSpacePoint> renderPoints(vertex);
				hr = m_pFaceModel[iFace]->CalculateVerticesForAlignment(m_pFaceAlignment[iFace], vertex, &facePoints[0]);

				if (SUCCEEDED(hr)){
					hr = m_pCoordinateMapper->MapCameraPointsToColorSpace(facePoints.size(), facePoints.data(), renderPoints.size(), renderPoints.data());
				}
				if (SUCCEEDED(hr)){
					auto cloud = convertKinectRGBPointsToPointCloud(facePoints, renderPoints);
					cloudUpdated(cloud);
					m_pDrawDataStreams->drawPoints(renderPoints);
				}
			}
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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectHDFaceGrabber::convertKinectRGBPointsToPointCloud(const std::vector<CameraSpacePoint>& cameraSpacePoints, const std::vector<ColorSpacePoint>& colorSpacePoints)
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>(imageWidth, imageHeight));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
	cloud->is_dense = false;
	auto colorSpacePoint = colorSpacePoints.begin();
//	float bottom	= - FLT_MAX;
//	float top		=   FLT_MAX;
	float bottom	=	FLT_MAX;
	float top		= - FLT_MAX;

	//float right		=   FLT_MAX;
	//float left		= - FLT_MAX;
	float right		= - FLT_MAX;
	float left		=   FLT_MAX;
	float front		=   FLT_MAX;
	float back		= - FLT_MAX;
	std::vector<cv::Point2f> ellipsePoints;
	for (auto& cameraSpacePoint : cameraSpacePoints){
		pcl::PointXYZRGB point;
		point.x = cameraSpacePoint.X;
		point.y = cameraSpacePoint.Y;
		point.z = cameraSpacePoint.Z;
		pcl::PointXYZRGB point2;

		
		int colorX = static_cast<int>(std::floor(colorSpacePoint->X + 0.5f));
		int colorY = static_cast<int>(std::floor(colorSpacePoint->Y + 0.5f));
		
		if (colorY > m_colorHeight || colorX > m_colorWidth || colorY < 0 || colorX < 0)
			continue;
		
		ellipsePoints.push_back(cv::Point2f(colorX, colorY));

		bottom = std::min(point.y, bottom);

		top = std::max(point.y, top);

		right = std::max(point.x, right);

		left = std::min(point.x, left);

		front = std::min(point.z, front);

		back = std::max(point.z, back);

		int colorImageIndex = ((m_colorWidth * colorY) + colorX);
		RGBQUAD pixel = m_colorBuffer[colorImageIndex];
		point.r = pixel.rgbRed;
		point.g = pixel.rgbGreen;
		point.b = pixel.rgbBlue;

		colorSpacePoint++;
		cloud->push_back(point);
	}
	
	
	Eigen::Vector4f centroid;


	pcl::compute3DCentroid(*cloud, centroid);
	Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
	Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

	pcl::transformPointCloud(*cloud, *cloud, m);

	CameraSpacePoint camTopLeftFront;
	camTopLeftFront.X = left;
	camTopLeftFront.Y = top;
	camTopLeftFront.Z = front;
	DepthSpacePoint depthTopLeftFront;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopLeftFront, &depthTopLeftFront);

	CameraSpacePoint camTopLeftBack;
	camTopLeftBack.X = left;
	camTopLeftBack.Y = top;
	camTopLeftBack.Z = back;
	DepthSpacePoint depthTopLeftBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopLeftBack, &depthTopLeftBack);

	CameraSpacePoint camTopRightFront;
	camTopRightFront.X = right;
	camTopRightFront.Y = top;
	camTopRightFront.Z = front;
	DepthSpacePoint depthTopRightFront;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopRightFront, &depthTopRightFront);

	CameraSpacePoint camTopRightBack;
	camTopRightBack.X = right;
	camTopRightBack.Y = top;
	camTopRightBack.Z = back;
	DepthSpacePoint depthTopRightBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camTopRightBack, &depthTopRightBack);

	CameraSpacePoint camBottomLeftFront;
	camBottomLeftFront.X = left;
	camBottomLeftFront.Y = bottom;
	camBottomLeftFront.Z = front;
	DepthSpacePoint depthBottomLeftFront;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomLeftFront, &depthBottomLeftFront);

	CameraSpacePoint camBottomLeftBack;
	camBottomLeftBack.X = left;
	camBottomLeftBack.Y = bottom;
	camBottomLeftBack.Z = back;
	DepthSpacePoint depthBottomLeftBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomLeftBack, &depthBottomLeftBack);

	CameraSpacePoint camBottomRightFront;
	camBottomRightFront.X = right;
	camBottomRightFront.Y = bottom;
	camBottomRightFront.Z = front;
	DepthSpacePoint depthBottomRightFront;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomRightFront, &depthBottomRightFront);

	CameraSpacePoint camBottomRightBack;
	camBottomRightBack.X = right;
	camBottomRightBack.Y = bottom;
	camBottomRightBack.Z = back;
	DepthSpacePoint depthBottomRightBack;
	m_pCoordinateMapper->MapCameraPointToDepthSpace(camBottomRightBack, &depthBottomRightBack);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthCloud(new pcl::PointCloud <pcl::PointXYZRGB>());
	cv::vector<cv::Point2f> hullPoints;
	
	cv::convexHull(ellipsePoints, hullPoints);
	//for (int x = static_cast<int>(depthTopRightBack.X); x < static_cast<int>(depthTopLeftBack.X); x++){
	//	for (int y = static_cast<int>(depthBottomLeftBack.Y); y < static_cast<int>(depthTopLeftBack.Y); y++){
	for (int x = static_cast<int>(depthTopLeftBack.X); x < static_cast<int>(depthTopRightBack.X); x++){
		for (int y = static_cast<int>(depthTopLeftBack.Y); y < static_cast<int>(depthBottomLeftBack.Y); y++){

			pcl::PointXYZRGB point;
			DepthSpacePoint depthPoint;
			depthPoint.X = static_cast<float>(x);
			depthPoint.Y = static_cast<float>(y);
			
			UINT16 depthOfCurrentPoint = m_depthBuffer[y * m_depthWidth + x];

			ColorSpacePoint colorPoint;
			HRESULT hr = m_pCoordinateMapper->MapDepthPointToColorSpace(depthPoint, depthOfCurrentPoint, &colorPoint);
			if (FAILED(hr)){
				continue;
			}
			int colorPixelMidX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
			int colorPixelMidY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));
			auto result = cv::pointPolygonTest(hullPoints, cv::Point2f(colorPoint.X, colorPoint.Y), false);
			if(result < 0){
				continue;
			}

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

			if (point.x < left || point.x > right)
				continue;
			
			if (isInColor && isInDepth){
				depthCloud->push_back(point);
			}
		}
	}
	
	pcl::transformPointCloud(*depthCloud, *depthCloud, m);
	depthCloudUpdated(depthCloud);
	
	return cloud;
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
