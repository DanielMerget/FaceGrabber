//------------------------------------------------------------------------------
// <copyright file="FaceBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"


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
#include <strsafe.h>

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
	m_pBodyIndexFrameReader(nullptr),
	m_pInfraredFrameReader(nullptr),
	m_depthWidth(-1),
	m_depthHeight(-1),
	m_colorWidth(-1),
	m_colorHeight(-1)
{
    for (int i = 0; i < BODY_COUNT; i++)
    {
        m_pFaceFrameSources[i] = nullptr;
        m_pFaceFrameReaders[i] = nullptr;
		m_pHDFaceSource[i] = nullptr;
		m_pHDFaceReader[i] = nullptr;

		m_pFaceAlignment[i] = nullptr;
		m_pFaceModelBuilder[i] = nullptr;
		m_pFaceModel[i] = nullptr;
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

	SafeRelease(m_pBodyIndexFrameReader);
	SafeRelease(m_pInfraredFrameReader);
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
				hr = CreateFaceFrameSource(m_pKinectSensor, 0,  
					FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace   | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace  | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation, 
					&m_pFaceFrameSources[i]);  //c_FaceFrameFeatures
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
			m_facePoints.resize(BODY_COUNT);
		}

		if (SUCCEEDED(hr)){
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		SafeRelease(pBodyFrameSource);
	}

	return hr;
}

HRESULT KinectHDFaceGrabber::initDepthAndColorAlignment()
{
	HRESULT hr = initBodyIndexFrameReader();

    // for align depth data with color data
    if (SUCCEEDED(hr) && m_pDepthFrameReader && m_pColorFrameReader){
		m_alighedDepthBuffer.resize(m_colorHeight * m_colorWidth);
		m_alighedColorBuffer.resize(m_colorHeight * m_colorWidth);
		m_alignedRawDepthBuffer.resize(m_colorHeight * m_colorWidth);
	}

	return hr;
}
/* */
HRESULT KinectHDFaceGrabber::initBodyIndexFrameReader()
{
	
	// open Bodey index Frame Reader 
	HRESULT hr;
	IBodyIndexFrameSource *pBodyIndexFrameSource = nullptr;
	if(m_pKinectSensor)
	{
		hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
	}

	if(SUCCEEDED(hr))
	{
		pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
	}

	SafeRelease(pBodyIndexFrameSource);
	//HRESULT hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_BodyIndex,&m_pMultiSourceFrameReader);

	return hr;
}


HRESULT KinectHDFaceGrabber::initInfraredFrameReader()
{
	IInfraredFrameSource* pInfraredFrameSource = nullptr;
	HRESULT hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);
	
	IFrameDescription* frameDescription = nullptr;
	if (SUCCEEDED(hr)){
		hr = pInfraredFrameSource->get_FrameDescription(&frameDescription);
	}

	if (SUCCEEDED(hr)){
		hr = frameDescription->get_Width(&m_infraredWidth);
	}

	if (SUCCEEDED(hr)){
		hr = frameDescription->get_Height(&m_infraredHeight);
	}


	if (SUCCEEDED(hr))
    {
        hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
    }

	if (SUCCEEDED(hr)){
		m_infraredBuffer.resize(m_infraredHeight * m_infraredWidth);
	}

	m_alighedInfraredBuffer.resize(m_colorHeight * m_colorWidth);;

	SafeRelease(frameDescription);
	SafeRelease(pInfraredFrameSource);
	
	return hr;
}

HRESULT KinectHDFaceGrabber::initializeDefaultSensor()
{
    HRESULT hr;
	BOOLEAN isAvailable = true;
    hr = GetDefaultKinectSensor(&m_pKinectSensor);

	if(FAILED(hr))
    {
        return hr;
    }
	
    if (m_pKinectSensor)
    {        		
        hr = m_pKinectSensor->Open();
		
		
		if (SUCCEEDED(hr))
		{
			Sleep(2000);
			hr = m_pKinectSensor->get_IsAvailable(&isAvailable);
			if(SUCCEEDED(hr))
			{
				if(isAvailable ==false)
				{
					m_pKinectSensor->Close();
					return E_FAIL;
				}
			}
			else{
					m_pKinectSensor->Close();
					return E_FAIL;
			}
		}
		
		if (SUCCEEDED(hr)){
			hr = initColorFrameReader();
		}
		
		if (SUCCEEDED(hr)){
			hr = initDepthFrameReader();
		}

				
		if (SUCCEEDED(hr)){
			hr = initInfraredFrameReader();
		}
		
		if (SUCCEEDED(hr)){
			hr = initHDFaceReader();
		}
				
								
		//
		if (SUCCEEDED(hr)){
			hr = initDepthAndColorAlignment(); // 
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
			 hr = getInfraredFrame();
		}
			
		/* */
		if (SUCCEEDED(hr)){
			hr = alighDepthWithColor();
		}
		/* */
		
		if (SUCCEEDED(hr)){
			renderColorFrameAndProcessFaces();
		}
				
	}
	
	SafeRelease(depthFrame);
	SafeRelease(pColorFrame);  
	
}

 
HRESULT KinectHDFaceGrabber::getInfraredFrame() 
{
	if (!m_pInfraredFrameReader)
    {
        return -1;
    }

    IInfraredFrame* pInfraredFrame = NULL;

    HRESULT hr = m_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

        hr = pInfraredFrame->get_RelativeTime(&nTime);

        
        if (SUCCEEDED(hr))
        {
            hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
        }

        if (SUCCEEDED(hr))
        {
            ProcessInfrared(nTime, pBuffer, m_infraredWidth, m_infraredHeight);
        }

    }

    SafeRelease(pInfraredFrame);

	return hr;
}

void KinectHDFaceGrabber::ProcessInfrared(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight)
{
    

    if (pBuffer && (nWidth == m_infraredWidth) && (nHeight == m_infraredHeight))
    {
        //RGBQUAD* pDest = m_infraredBuffer;

        // end pixel is start + width*height - 1
        //const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		for (int i = 0; i < m_infraredWidth * m_infraredHeight; ++i)
		{
			// normalize the incoming infrared data (ushort) to a float ranging from 
			// [InfraredOutputValueMinimum, InfraredOutputValueMaximum] by
			// 1. dividing the incoming value by the source maximum value
			float intensityRatio = static_cast<float>(*pBuffer) / InfraredSourceValueMaximum;

			// 2. dividing by the (average scene value * standard deviations)
			intensityRatio /= InfraredSceneValueAverage * InfraredSceneStandardDeviations;
		
			// 3. limiting the value to InfraredOutputValueMaximum
			intensityRatio = std::min(InfraredOutputValueMaximum, intensityRatio);

			// 4. limiting the lower value InfraredOutputValueMinimym
			intensityRatio = std::max(InfraredOutputValueMinimum, intensityRatio);
	
			// 5. converting the normalized value to a byte and using the result
			// as the RGB components required by the image
			byte intensity = static_cast<byte>(intensityRatio * 255.0f); 
			m_infraredBuffer[i].rgbRed = intensity;
			m_infraredBuffer[i].rgbGreen = intensity;
			m_infraredBuffer[i].rgbBlue = intensity;
			m_infraredBuffer[i].rgbReserved = 255;
			++pBuffer;
        }

       
    }
}


HRESULT KinectHDFaceGrabber::alighDepthWithColor()
{
	HRESULT hr;
	//IMultiSourceFrame* pMultiSourceFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	//IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
	IFrameDescription* pBodyIndexFrameDescription = NULL;
	BYTE* m_bodyIndexBuffer = NULL;
	UINT  m_bodyIndexBufferSize;
	int	m_bodyIndexWidth;
	int	m_bodyIndexHeight;
	bool RmBG = m_commonConfiguration->isKeepBGEnabled();

	if (RmBG == false)
	{
		
		hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);	

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Width(&m_bodyIndexWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Height(&m_bodyIndexWidth);
		}
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&m_bodyIndexBufferSize, &m_bodyIndexBuffer); 
		}
			

		
	}
	
	DepthSpacePoint * m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];

	hr = getCoordinateMapper()->MapColorFrameToDepthSpace(m_depthHeight*m_depthWidth, (UINT16*)m_depthBuffer.data(), m_colorWidth * m_colorHeight, m_pDepthCoordinates);

	//CameraIntrinsics  pCameraIntrinsics;
	//UINT32 u32TableCount;
	//PointF *pPointF;
	//getCoordinateMapper()->GetDepthCameraIntrinsics(&pCameraIntrinsics);
	//getCoordinateMapper()->GetDepthFrameToCameraSpaceTable(&u32TableCount,&pPointF);
	
	if (SUCCEEDED(hr))
	{
		RGBQUAD rgb_zero;
		rgb_zero.rgbRed = 0;
		rgb_zero.rgbBlue = 0;
		rgb_zero.rgbGreen = 0;
		rgb_zero.rgbReserved = 255;
		std::fill(m_alighedDepthBuffer.begin(), m_alighedDepthBuffer.end(), rgb_zero);
		std::fill(m_alighedColorBuffer.begin(), m_alighedColorBuffer.end(), rgb_zero);
		std::fill(m_alighedInfraredBuffer.begin(), m_alighedInfraredBuffer.end(), rgb_zero);
		std::fill(m_alignedRawDepthBuffer.begin(), m_alignedRawDepthBuffer.end(), 0);
		
		for (int i = 0; i < m_colorWidth * m_colorHeight; i++)
		{
			DepthSpacePoint depthPoint = m_pDepthCoordinates[i];
			BYTE intensity = 0;
			BYTE infraredIntensity = 0;		
			int depthX = static_cast<int>(depthPoint.X + 0.5f);                 
			int depthY = static_cast<int>(depthPoint.Y + 0.5f);
			if (depthPoint.X >= 0 && depthPoint.Y >= 0)
			{
				int depthIdx = (int)(depthPoint.X + depthPoint.Y * m_depthWidth);
				USHORT depth = m_depthBuffer.data()[depthIdx];

				intensity = static_cast<BYTE>(depth % 256);
				infraredIntensity = m_infraredBuffer[depthIdx].rgbBlue;
				if (RmBG == false && m_bodyIndexBuffer)
				{
					BYTE player = m_bodyIndexBuffer[depthX + (depthY* m_depthWidth)];

					if (player != 0xff)                   
					{
						//
						//m_pDepthRGBX = m_colorBuffer[i].rgbBlue;
						m_alighedColorBuffer[i].rgbRed = m_colorBuffer[i].rgbRed;
						m_alighedColorBuffer[i].rgbGreen = m_colorBuffer[i].rgbGreen;
						m_alighedColorBuffer[i].rgbBlue = m_colorBuffer[i].rgbBlue;

						//m_pDepthRGBX[i] = m_colorBuffer.data()[i];
						m_alighedDepthBuffer[i].rgbRed = intensity; //depth
						m_alighedDepthBuffer[i].rgbGreen = intensity;
						m_alighedDepthBuffer[i].rgbBlue = intensity;
						m_alignedRawDepthBuffer[i] = depth;

						m_alighedInfraredBuffer[i].rgbRed = infraredIntensity; //depth
						m_alighedInfraredBuffer[i].rgbGreen = infraredIntensity;
						m_alighedInfraredBuffer[i].rgbBlue = infraredIntensity;

					}
				}
				else
				{
						m_alighedColorBuffer[i].rgbRed = m_colorBuffer[i].rgbRed;
						m_alighedColorBuffer[i].rgbGreen = m_colorBuffer[i].rgbGreen;
						m_alighedColorBuffer[i].rgbBlue = m_colorBuffer[i].rgbBlue;

						//m_pDepthRGBX[i] = m_colorBuffer.data()[i];
						m_alighedDepthBuffer[i].rgbRed = intensity; //depth
						m_alighedDepthBuffer[i].rgbGreen = intensity;
						m_alighedDepthBuffer[i].rgbBlue = intensity;

						m_alignedRawDepthBuffer[i] = depth;

						m_alighedInfraredBuffer[i].rgbRed = infraredIntensity; //depth
						m_alighedInfraredBuffer[i].rgbGreen = infraredIntensity;
						m_alighedInfraredBuffer[i].rgbBlue = infraredIntensity;
				}

			}

		}
		
	}
								
	delete m_pDepthCoordinates;		
	SafeRelease(pBodyIndexFrameDescription);
	SafeRelease(pBodyIndexFrame);

	return hr;
}

void KinectHDFaceGrabber::renderColorFrameAndProcessFaces()
{

    HRESULT hr = E_FAIL;
    
	UpdateFrameRate();
    //if (SUCCEEDED(hr))
    {
        // Make sure we've received valid color data
		 // Draw the data with Direct2D
		
			// View Depth Sensor Output
		
		std::vector<RGBQUAD> depth(m_depthWidth * m_depthHeight);
		RGBQUAD rgb_zero;
		rgb_zero.rgbRed = 255;
		rgb_zero.rgbBlue = 0;
		rgb_zero.rgbGreen = 0;
		rgb_zero.rgbReserved = 255;
		std::fill(depth.begin(), depth.end(), rgb_zero);

		int value;
		int k = 0;
		for (int i = 0; i < m_depthHeight ; ++i)
		{
			for (int j = 0; j < m_depthWidth; ++j )
			{
				USHORT depth_v = m_depthBuffer.data()[k++];
				
				//value = static_cast<BYTE>((depth_v >= 0) && (depth_v <= 5000) ? (depth_v % 256) : 0);
				value =  static_cast<BYTE>(depth_v % 256);

				depth[i*m_depthWidth+j].rgbRed = value;
				depth[i*m_depthWidth+j].rgbGreen = value;
				depth[i*m_depthWidth+j].rgbBlue = value;
			}
		}

		
		
		std::vector<RGBQUAD>		*pShowBuffer = nullptr;

		
		int showWidth = 0;
		int showHeight = 0;
		if ((m_colorWidth > 0 ) && (m_colorHeight > 0))
		{
			switch(m_commonConfiguration->getShowOpt()){
				case 	Color_Raw:
					pShowBuffer = &m_colorBuffer;
					showWidth = m_colorWidth;
					showHeight = m_colorHeight;
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_colorBuffer.data()), c * m_colorHeight* sizeof(RGBQUAD));
					break;
				case Depth_Raw:
					pShowBuffer = &depth;
					showWidth = m_depthWidth;
					showHeight = m_depthHeight;
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(depth.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
					break;
				case Color_Body:
					pShowBuffer = &m_alighedColorBuffer;
					showWidth = m_colorWidth;
					showHeight = m_colorHeight;
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_alighedColorBuffer.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
					break;
				case Depth_Body:
					pShowBuffer = &m_alighedDepthBuffer;
					showWidth = m_colorWidth;
					showHeight = m_colorHeight;
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_alighedDepthBuffer.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
					
				    break;
				case Infrared_Raw:
					pShowBuffer = &m_infraredBuffer;
					showWidth = m_depthWidth;
					showHeight = m_depthHeight;
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_alighedInfraredBuffer.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(infrared.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
				    break;
					
				default:
					//hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_colorBuffer.data()), m_colorWidth * m_colorHeight* sizeof(RGBQUAD));
					break;
			}

			if(pShowBuffer!=nullptr)
			{
				
				m_pDrawDataStreams->setSize(showWidth,showHeight,showWidth * sizeof(RGBQUAD));
				hr = m_pDrawDataStreams->beginDrawing();
				hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(pShowBuffer->data()), showWidth * showHeight* sizeof(RGBQUAD));
				m_pDrawDataStreams->DrawFPS(m_fps);
			}
			//test code end


			
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
			m_pDrawDataStreams->endDrawing();
        }

       
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
	m_outputStreamUpdater->startFaceCollection(m_colorBuffer.data(), m_depthBuffer.data(),m_alignedRawDepthBuffer.data(),m_infraredBuffer.data(),m_alighedInfraredBuffer.data()); //m_alighedColorBuffer m_alighedDepthBuffer m_infraredBuffer
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
		if (SUCCEEDED(hr)) {
			getFiveKeyPointsOnFaces(iFace, ppBodies[iFace],bHaveBodyData, bFaceTracked);
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
		
		        // extract face rotation in degrees as Euler angles
        int pitch, yaw, roll;
        m_pDrawDataStreams->extractFaceRotationInDegrees(&m_faceRotation[iFace], &pitch, &yaw, &roll);
		std::string keyPoints;
		PointF * pFiveP = m_facePoints[iFace].data();
		//if (m_recordingConfiguration->isEnabled
		for(int i=0; i < 5; ++i)
		{
			keyPoints += std::to_string((pFiveP+i)->X)  + '\t' +std::to_string((pFiveP+i)->Y)  + "\n";
		}
			
		keyPoints +="BBox : " +std::to_string(m_faceBox[iFace].Left)+ '\t'+std::to_string(m_faceBox[iFace].Top) +'\t'+std::to_string(m_faceBox[iFace].Right)+'\t'+std::to_string(m_faceBox[iFace].Bottom) + "\n";;
		keyPoints += "FaceYaw : " + std::to_string(yaw) + "\n";
        keyPoints += "FacePitch : " + std::to_string(pitch) + "\n";
        keyPoints += "FaceRoll : " + std::to_string(roll) + "\n";

		
		if (m_outputStreamUpdater){
			hr = m_outputStreamUpdater->updateOutputStreams(m_pFaceModel[iFace], m_pFaceAlignment[iFace], 
				std::min(m_HDFaceDetectedPointsCamSpace[iFace].size(), m_HDFaceDetectedPointsColorSpace[iFace].size()), 
				m_HDFaceDetectedPointsCamSpace[iFace].data(), m_HDFaceDetectedPointsColorSpace[iFace].data(),keyPoints);
			updatedOneFace = true;
		}
			
		if (SUCCEEDED(hr)){
			if (m_commonConfiguration->getFacePointsShowOpt() == FacePointsShowOpt::HDFacePoints_Opt &&
				m_commonConfiguration->getShowOpt() != Depth_Raw &&
				m_commonConfiguration->getShowOpt() != Infrared_Raw)
			{
				m_pDrawDataStreams->drawPoints(m_HDFaceDetectedPointsColorSpace[iFace]);
			}
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

HRESULT KinectHDFaceGrabber::getFiveKeyPointsOnFaces(int iFace, IBody* ppBodies,bool bHaveBodyData, bool bFaceTracked)
{
    HRESULT hr;
    //IBody* ppBodies = {0};
    


    // retrieve the latest face frame from this reader
    IFaceFrame* pFaceFrame = nullptr;
    hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

    //BOOLEAN bFaceTracked = false;
    //if (SUCCEEDED(hr) && nullptr != pFaceFrame)
    //{
        // check if a valid face is tracked in this face frame
        //hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
    //}

    if (SUCCEEDED(hr))
    {
        if (bFaceTracked)
        {
            IFaceFrameResult* pFaceFrameResult = nullptr;
			memset(&m_faceBox[iFace],0,sizeof(RECT));
            //PointF facePoints[FacePointType::FacePointType_Count];
			
            DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
            D2D1_POINT_2F faceTextLayout;

            hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

            // need to verify if pFaceFrameResult contains data before trying to access it
            if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
            {
                hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&m_faceBox[iFace]);

                if (SUCCEEDED(hr))
                {										
                    hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, m_facePoints[iFace].data());
                }
				
                if (SUCCEEDED(hr))
                {
                    hr = pFaceFrameResult->get_FaceRotationQuaternion(&m_faceRotation[iFace]);
                }

                if (SUCCEEDED(hr))
                {
                    hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
                }

                if (SUCCEEDED(hr))
                {
                    hr = GetFaceTextPositionInColorSpace(ppBodies, &faceTextLayout);
                }

                if (SUCCEEDED(hr))
                {
					if (m_commonConfiguration->getFacePointsShowOpt() == FacePointsShowOpt::FiveKeyPoints_Opt  &&
						m_commonConfiguration->getShowOpt() != Depth_Raw &&
						m_commonConfiguration->getShowOpt() != Infrared_Raw)
					{
                    // draw face frame results
						m_pDrawDataStreams->drawFaceFrameResults(iFace, &m_faceBox[iFace], m_facePoints[iFace].data(), &m_faceRotation[iFace], faceProperties, &faceTextLayout);
					}
                }							
            }

            SafeRelease(pFaceFrameResult);	
        }
        else 
        {	
            // face tracking is not valid - attempt to fix the issue
            // a valid body is required to perform this step
            if (bHaveBodyData)
            {
                // check if the corresponding body is tracked 
                // if this is true then update the face frame source to track this body
                IBody* pBody = ppBodies;
                if (pBody != nullptr)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    UINT64 bodyTId;
                    if (SUCCEEDED(hr) && bTracked)
                    {
                        // get the tracking ID of this body
                        hr = pBody->get_TrackingId(&bodyTId);
                        if (SUCCEEDED(hr))
                        {
                            // update the face frame source with the tracking ID
                            m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
                        }
                    }
                }
            }
        }
    }			

    SafeRelease(pFaceFrame);

	return hr;
}

HRESULT KinectHDFaceGrabber::GetFaceTextPositionInColorSpace(IBody* pBody, D2D1_POINT_2F* pFaceTextLayout)
{
    HRESULT hr = E_FAIL;

    if (pBody != nullptr)
    {
        BOOLEAN bTracked = false;
        hr = pBody->get_IsTracked(&bTracked);

        if (SUCCEEDED(hr) && bTracked)
        {
            Joint joints[JointType_Count]; 
            hr = pBody->GetJoints(_countof(joints), joints);
            if (SUCCEEDED(hr))
            {
                CameraSpacePoint headJoint = joints[JointType_Head].Position;
                CameraSpacePoint textPoint = 
                {
                    headJoint.X + c_FaceTextLayoutOffsetX,
                    headJoint.Y + c_FaceTextLayoutOffsetY,
                    headJoint.Z
                };

                ColorSpacePoint colorPoint = {0};
                hr = m_pCoordinateMapper->MapCameraPointToColorSpace(textPoint, &colorPoint);

                if (SUCCEEDED(hr))
                {
                    pFaceTextLayout->x = colorPoint.X;
                    pFaceTextLayout->y = colorPoint.Y;
                }
            }
        }
    }

    return hr;
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


/// <summary>
/// Update frame rate
/// </summary>
void KinectHDFaceGrabber::UpdateFrameRate()
{
    m_frameCount++;

    DWORD tickCount = GetTickCount();
    DWORD span      = tickCount - m_lastTick;
    if (span >= 1000)
    {
        m_fps            = (UINT)((double)(m_frameCount - m_lastFrameCount) * 1000.0 / (double)span + 0.5);
        m_lastTick       = tickCount;
        m_lastFrameCount = m_frameCount;
    }
}