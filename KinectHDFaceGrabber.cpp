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
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0),
    m_pKinectSensor(nullptr),
    m_pCoordinateMapper(nullptr),
	m_pColorFrameReader(nullptr),
    m_pDrawDataStreams(nullptr),
    m_pColorRGBX(nullptr),
    m_pBodyFrameReader(nullptr),
	m_pDepthFrameReader(nullptr)
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
			hr = CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &deformations[i][0], &m_pFaceModel[i]);
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

	bool produce[BODY_COUNT] = { false };
	
    IColorFrame* pColorFrame = nullptr;
    HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	
    if (SUCCEEDED(hr)){
        INT64 nTime = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;
        UINT nBufferSize = 0;
        RGBQUAD *pBuffer = nullptr;
	
        hr = pColorFrame->get_RelativeTime(&nTime);
	
        if (SUCCEEDED(hr)){
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }
	
        if (SUCCEEDED(hr)){
			nBufferSize = m_colorWidth * m_colorHeight * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(m_colorBuffer.data()), ColorImageFormat_Bgra);
        }			
	
        if (SUCCEEDED(hr)){
			updateHDFaceAndColor(nTime);
			updateDepthCloud();
        }
        
    }
	SafeRelease(pColorFrame);  
}

void KinectHDFaceGrabber::updateDepthCloud()
{
	IDepthFrame* depthFrame = nullptr;
	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&depthFrame);

	if (SUCCEEDED(hr)){
		hr = depthFrame->CopyFrameDataToArray(m_depthBuffer.size(), &m_depthBuffer[0]);
	}
	//std::vector<CameraSpacePoint> depthPointsInCameraSpace(m_depthBuffer.size());
	//std::vector<ColorSpacePoint> renderPoints(vertex);
	//m_pCoordinateMapper->MapDepthFrameToCameraSpace(m_depthBuffer.size(), m_depthBuffer.data(), depthPointsInCameraSpace.size(), depthPointsInCameraSpace.data());
	auto cloud = convertDepthBufferToPointCloud();
	depthCloudUpdated(cloud);
	SafeRelease(depthFrame);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectHDFaceGrabber::convertDepthBufferToPointCloud()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pointCloud->width = static_cast<uint32_t>(m_depthWidth);
	pointCloud->height = static_cast<uint32_t>(m_depthHeight);
	pointCloud->is_dense = false;
	HRESULT hr;
	for (int y = 0; y < m_depthHeight; y++)
	{
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
void KinectHDFaceGrabber::updateHDFaceAndColor(INT64 nTime)
{
//    if (m_hWnd)
//    {
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

        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

		if (statusChanged(szStatusMessage, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
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
						//SafeRelease(pFaceModelData);
						//m_pFaceModelBuilder[iFace]->Release();
						//m_pFaceModelBuilder[iFace] = nullptr;
					}
					std::vector<CameraSpacePoint> facePoints(vertex);
					std::vector<ColorSpacePoint> renderPoints(vertex);
					hr = m_pFaceModel[iFace]->CalculateVerticesForAlignment(m_pFaceAlignment[iFace], vertex, &facePoints[0]);
					
					if (SUCCEEDED(hr)){
						m_pCoordinateMapper->MapCameraPointsToColorSpace(facePoints.size(), facePoints.data(), renderPoints.size(), renderPoints.data());
					}
					auto cloud = convertKinectRGBPointsToPointCloud(facePoints, renderPoints);

					cloudUpdated(cloud);
					
					//first = false;
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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectHDFaceGrabber::convertKinectRGBPointsToPointCloud(const std::vector<CameraSpacePoint>& renderPoints, const std::vector<ColorSpacePoint>& imagePoints)
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>(imageWidth, imageHeight));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>());
	cloud->is_dense = false;
	auto imageSpacePoint = imagePoints.begin();
	for (auto& colorSpacePoint : renderPoints){
		pcl::PointXYZRGB point;
		point.x = colorSpacePoint.X;
		point.y = colorSpacePoint.Y;
		point.z = colorSpacePoint.Z;

		int colorX = static_cast<int>(std::floor(imageSpacePoint->X + 0.5f));
		int colorY = static_cast<int>(std::floor(imageSpacePoint->Y + 0.5f));
		if (colorY > m_colorHeight || colorX > m_colorWidth || colorY < 0 || colorX < 0)
			continue;

		int colorImageIndex = ((m_colorWidth * colorY) + colorX);
		RGBQUAD pixel = m_colorBuffer[colorImageIndex];
		point.r = pixel.rgbRed;
		point.g = pixel.rgbGreen;
		point.b = pixel.rgbBlue;

		imageSpacePoint++;
		cloud->push_back(point);
	}
	
	Eigen::Vector4f centroid;
	
	
	pcl::compute3DCentroid(*cloud, centroid);
	Eigen::Vector3f center(-centroid.x(), -centroid.y(), -centroid.z());
	Eigen::Matrix4f m = Eigen::Affine3f(Eigen::Translation3f(center)).matrix();

	pcl::transformPointCloud(*cloud, *cloud, m);
	
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
