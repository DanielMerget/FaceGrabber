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


KinectHDFaceGrabber::KinectHDFaceGrabber() :
    m_pKinectSensor(nullptr),
    m_pCoordinateMapper(nullptr),
	m_pColorFrameReader(nullptr),
    m_pDrawDataStreams(nullptr),
    m_pBodyFrameReader(nullptr),
	m_pDepthFrameReader(nullptr),
	m_depthWidth(-1),
	m_depthHeight(-1),
	m_colorWidth(-1),
	m_colorHeight(-1)
{
    for (int i = 0; i < BODY_COUNT; i++)
    {
        m_pFaceFrameSources[i] = nullptr;
        m_pFaceFrameReaders[i] = nullptr;
    }
}

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
				return hr;
			}
		}
		
		if (SUCCEEDED(hr)){
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		SafeRelease(pBodyFrameSource);
	}

	return hr;
}

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

void KinectHDFaceGrabber::update()
{
	if (!m_pColorFrameReader || !m_pBodyFrameReader){
		return;
	}

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
		}
				
	}
	
	SafeRelease(depthFrame);
	SafeRelease(pColorFrame);  
}



void KinectHDFaceGrabber::renderColorFrameAndProcessFaces()
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

void KinectHDFaceGrabber::processFaces()
{
    //update the tracked bodys
	HRESULT hr;
    IBody* ppBodies[BODY_COUNT] = {0};
    bool bHaveBodyData = SUCCEEDED( updateBodyData(ppBodies) );
	if (!bHaveBodyData)
		return;

	//indicate the start of data providing
	m_outputStreamUpdater->startFaceCollection(m_colorBuffer.data(), m_depthBuffer.data());
	bool updatedOneFace = false;
	UINT32 vertex = 0;
	hr = GetFaceModelVertexCount(&vertex); // 1347

    // iterate through each face reader
    for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
    {
		//asociate the faces with the bodies
		updateHDFaceTrackingID(m_pHDFaceSource[iFace], ppBodies[iFace]);

		IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
		hr = m_pHDFaceReader[iFace]->AcquireLatestFrame(&pHDFaceFrame);
		
		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && pHDFaceFrame != nullptr){
			hr = pHDFaceFrame->get_IsFaceTracked(&bFaceTracked);
		}
		//update face aligment
		if (SUCCEEDED(hr) && bFaceTracked){
			hr = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment[iFace]);
		}
		
		if (FAILED(hr) || m_pFaceAlignment[iFace] == nullptr){
			continue;
		}
		//have we finished building our model?
		if (m_pFaceModelBuilder[iFace] != nullptr){
			updateFaceModelStatusOfFaceModelBuilder(&m_pFaceModelBuilder[iFace], m_pFaceModel[iFace]);
		}
		
		if (m_outputStreamUpdater){
			hr = m_outputStreamUpdater->updateOutputStreams(m_pFaceModel[iFace], m_pFaceAlignment[iFace], 
				std::min(m_HDFaceDetectedPointsCamSpace[iFace].size(), m_HDFaceDetectedPointsColorSpace[iFace].size()), 
				m_HDFaceDetectedPointsCamSpace[iFace].data(), m_HDFaceDetectedPointsColorSpace[iFace].data());
			updatedOneFace = true;
		}
			
		if (SUCCEEDED(hr)){
			m_pDrawDataStreams->drawPoints(m_HDFaceDetectedPointsColorSpace[iFace]);
		}		
    }
	if (updatedOneFace){
		m_outputStreamUpdater->stopFaceCollection();
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

HRESULT KinectHDFaceGrabber::updateBodyData(IBody** body)
{
    HRESULT hr = E_FAIL;

    if (m_pBodyFrameReader != nullptr)
    {
        IBodyFrame* pBodyFrame = nullptr;
        hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
        if (SUCCEEDED(hr))
        {
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, body);
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