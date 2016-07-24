//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------


#include "resource.h"
#include "stdafx.h"
//#include <strsafe.h>

#include "KinectV1Controller.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

KinectV1Controller::KinectV1Controller():m_pNuiSensor(nullptr),m_pDrawDataStreams(nullptr),m_colorFrameCount(0),m_depthFrameCount(0)
{		
	DWORD width = 0;
    DWORD height = 0;
	m_paused = false;
	m_nearMode =false;
	m_depthTreatment = TINT_UNRELIABLE_DEPTHS; //DISPLAY_ALL_DEPTHS CLAMP_UNRELIABLE_DEPTHS TINT_UNRELIABLE_DEPTHS
    NuiImageResolutionToSize(cDepthResolution, width, height);
    m_depthWidth  = static_cast<LONG>(width);
    m_depthHeight = static_cast<LONG>(height);

	m_depthD16 = new USHORT[m_depthWidth*m_depthHeight];
	

    NuiImageResolutionToSize(cColorResolution, width, height);
    m_colorWidth  = static_cast<LONG>(width);
    m_colorHeight = static_cast<LONG>(height);

    m_colorToDepthDivisor = m_colorWidth/m_depthWidth;
	m_alignedDepthD16 = new USHORT[m_colorWidth*m_colorHeight];


	m_colorCoordinates = new LONG[m_colorWidth*m_colorHeight*2];
	//m_colorRGBX = new NuiImageBuffer();
	// m_alignedInfraredRGBX = new RGBQUAD[m_colorWidth * m_colorHeight];
	
	m_showOpt = RecordingShowOpt::Color_Raw;
	m_showResolution = NUI_IMAGE_RESOLUTION_640x480;
	m_drawRGBX =   &m_colorRGBX;//&m_colorRGBX;

	m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	//m_hNextInfaredFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
}


KinectV1Controller::~KinectV1Controller()
{
	//if (m_alignedInfraredRGBX){
	//	delete []m_alignedInfraredRGBX;
	//}

	  // clean up Direct2D renderer
    if (m_pDrawDataStreams)
    {
        delete m_pDrawDataStreams;
        m_pDrawDataStreams = nullptr;
    }


}

HRESULT KinectV1Controller::init()
{

	HRESULT hr = CreateFirstConnected();
	if(FAILED(hr))
	{
		return hr;
	}


	
	m_depthResolution = NUI_IMAGE_RESOLUTION_320x240;
	hr = openDepthStream(); //NUI_IMAGE_RESOLUTION_320x240 NUI_IMAGE_RESOLUTION_640x480
	m_colorImageType = NUI_IMAGE_TYPE_COLOR;

	m_colorResolution = NUI_IMAGE_RESOLUTION_640x480;
	hr = openColorStream();

	//hr = openInfraredStream();
	m_alignedDepthRGBX.SetImageSize(NUI_IMAGE_RESOLUTION_640x480); 
	//hr = openInfraredStream(NUI_IMAGE_RESOLUTION_640x480);

	return hr;
}


HRESULT KinectV1Controller:: CreateFirstConnected()
{
	INuiSensor * pNuiSensor;
    HRESULT hr=E_FAIL;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr)||iSensorCount==0)
    {
        return E_FAIL;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            m_pNuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if (NULL != m_pNuiSensor)
    {
		    // Initialize Nui sensor
     hr = m_pNuiSensor->NuiInitialize(
        NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
        | NUI_INITIALIZE_FLAG_USES_SKELETON
        | NUI_INITIALIZE_FLAG_USES_COLOR
        | NUI_INITIALIZE_FLAG_USES_AUDIO);

        
    }

    if (NULL == m_pNuiSensor || FAILED(hr))
    {
        //SetStatusMessage(L"No ready Kinect found!");
        return E_FAIL;
    }

    return hr;

}

HANDLE KinectV1Controller::getCorlorFrameEvent()
{
	return m_hNextColorFrameEvent;
}

HANDLE KinectV1Controller::getDepthFrameEvent()
{
	return m_hNextDepthFrameEvent;
}

void KinectV1Controller::Update()
{

    if (NULL == m_pNuiSensor)
    {
        return;
    }
	const int eventCount = 2;
    HANDLE hEvents[eventCount];

	hEvents[0] = m_hNextColorFrameEvent;
	hEvents[1] = m_hNextDepthFrameEvent;
	//hEvents[2] = m_hNextInfaredFrameEvent;


	MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

		if(WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0))
	{
		ProcessColor();
	}

		if(WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
	{
		ProcessDepth();
	}


	//show();
	

}

void KinectV1Controller::showOptUpdated(RecordingShowOpt showOpt)
{
	if(m_showOpt == showOpt)
	{
		return;
	}
	m_showResolution = NUI_IMAGE_RESOLUTION_640x480;
	switch(showOpt)
	{
		case Color_Raw:

			m_colorImageType = NUI_IMAGE_TYPE_COLOR;
			m_colorResolution = NUI_IMAGE_RESOLUTION_640x480;
			openColorStream();
			m_drawRGBX = &m_colorRGBX;
			
			break;

		case Depth_Raw:
			m_depthResolution = NUI_IMAGE_RESOLUTION_640x480;
			openDepthStream();
			m_drawRGBX = &m_DepthRGBX;
			break;
		default:break;
	}
	m_showOpt = showOpt;
	
	return ;

}
void KinectV1Controller::showResolutionUpdated(int showResolution)
{
	switch(m_showOpt)
	{
		case Color_Raw:

			showcolorResolutionUpdated((v1ColorType)(showResolution));
			break;

		case Depth_Raw:
			showdepthResolutionUpdated((v1DepthType)(showResolution));
			break;
		default:break;
	}
}

void KinectV1Controller::showcolorResolutionUpdated(v1ColorType showResolution)
{
	
	if(m_showOpt != Color_Raw)
	{
		return;
	}
	//m_showResolution = showResolution;
	switch(showResolution)
	{
		case RESOLUTION_RGBRESOLUTION640X480FPS30:
			m_colorImageType = NUI_IMAGE_TYPE_COLOR;
			m_colorResolution = NUI_IMAGE_RESOLUTION_640x480;                
			break;
		case RESOLUTION_RGBRESOLUTION1280X960FPS12:
			m_colorImageType = NUI_IMAGE_TYPE_COLOR;
			m_colorResolution = NUI_IMAGE_RESOLUTION_1280x960; 
			break;
		case RESOLUTION_YUVRESOLUTION640X480FPS15:
			m_colorImageType = NUI_IMAGE_TYPE_COLOR_YUV;
			m_colorResolution = NUI_IMAGE_RESOLUTION_640x480; 
			break;
		case RESOLUTION_INFRAREDRESOLUTION640X480FPS30:
			m_colorImageType = NUI_IMAGE_TYPE_COLOR_INFRARED;
			m_colorResolution = NUI_IMAGE_RESOLUTION_640x480; 
			break;
		case RESOLUTION_RAWBAYERRESOLUTION640X480FPS30:
			m_colorImageType = NUI_IMAGE_TYPE_COLOR_RAW_BAYER;
			m_colorResolution = NUI_IMAGE_RESOLUTION_640x480; 
			break;
		case RESOLUTION_RAWBAYERRESOLUTION1280X960FPS12:
			m_colorImageType = NUI_IMAGE_TYPE_COLOR_RAW_BAYER;
			m_colorResolution = NUI_IMAGE_RESOLUTION_1280x960; 
			break;
		default:break;
	}
	openColorStream();
			
	return;
}

void KinectV1Controller::showdepthResolutionUpdated(v1DepthType showResolution)
{
	
	if(m_showOpt != Depth_Raw)
	{
		return;

	}
	//m_showResolution = showResolution;
	switch(showResolution)
	{
		case RESOLUTION_DEPTHESOLUTION640X480FPS30:
			m_depthResolution = NUI_IMAGE_RESOLUTION_640x480;                
			break;
		case RESOLUTION_DEPTHRESOLUTION320X240FPS30:
			m_depthResolution = NUI_IMAGE_RESOLUTION_320x240; 
			break;
		case RESOLUTION_DEPTHRESOLUTION80X60FP30:
			m_depthResolution = NUI_IMAGE_RESOLUTION_80x60; 
			break;

		default:break;
	}
	openDepthStream();

	return;
}


void KinectV1Controller::show()
{

	 
	m_pDrawDataStreams->setSize(m_drawRGBX->GetWidth(),m_drawRGBX->GetHeight(),m_drawRGBX->GetWidth() * sizeof(RGBQUAD));
	//m_pDrawDataStreams->initialize(m_liveViewWindow, m_pD2DFactory, cColorWidth, cColorHeight, cColorWidth * sizeof(RGBQUAD));
	//m_pDrawDataStreams->initialize(m_hWnd, m_pD2DFactory, m_drawRGBX->GetWidth(),m_drawRGBX->GetHeight(),m_drawRGBX->GetWidth() * sizeof(RGBQUAD));
	HRESULT hr = m_pDrawDataStreams->beginDrawing();
	hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_drawRGBX->GetBuffer()), m_drawRGBX->GetWidth() * m_drawRGBX->GetHeight()* sizeof(RGBQUAD));
	m_pDrawDataStreams->DrawFPS(m_showFps);

	hr = m_pDrawDataStreams->endDrawing();
	
}


 
void KinectV1Controller::ProcessDepth()
{

	HRESULT hr;
	//if (WAIT_OBJECT_0 == WaitForSingleObject(GetFrameReadyEvent(), 0))
	
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the depth frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return;
    }

    if (m_paused)
    {
        // Stream paused. Skip frame process and release the frame.
        goto ReleaseFrame;
    }

    BOOL nearMode;
    INuiFrameTexture* pTexture;

    // Get the depth image pixel texture
    hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
    if (FAILED(hr))
    {
        goto ReleaseFrame;
    }

    NUI_LOCKED_RECT lockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &lockedRect, NULL, 0);

    // Make sure we've received valid data
    if (lockedRect.Pitch != 0)
    {
        // Conver depth data to color image and copy to image buffer
		m_DepthRGBX.CopyDepth(lockedRect.pBits, lockedRect.size, nearMode, m_depthTreatment);
	
		
		
        //memcpy(m_depthD16,lockedRect.pBits,lockedRect.size);

		//AlignDepthToColorSpace();
		/*
		m_pDrawDataStreams->setSize(m_DepthRGBX.GetWidth(),m_DepthRGBX.GetHeight(),m_DepthRGBX.GetWidth() * sizeof(RGBQUAD));
		hr = m_pDrawDataStreams->beginDrawing();
		hr = m_pDrawDataStreams->drawBackground(reinterpret_cast<BYTE*>(m_DepthRGBX.GetBuffer()), m_DepthRGBX.GetWidth() * m_DepthRGBX.GetHeight()* sizeof(RGBQUAD));
		m_pDrawDataStreams->endDrawing();
		*/

    }


    // Done with the texture. Unlock and release it
    pTexture->UnlockRect(0);
    pTexture->Release();

	UpdateDepthFrameRate();

	if(m_showOpt ==  Depth_Raw) 
	{
		m_showFps = m_depthFps;
		show();
		//int a = 5;
		//cv::Mat m_depthImage = cv::Mat(m_DepthRGBX.GetHeight(), m_DepthRGBX.GetWidth(), CV_8UC4, m_DepthRGBX.GetBuffer(), cv::Mat::AUTO_STEP);
		//cv::imshow("DepthWindow",m_depthImage);
		//boost::shared_ptr<cv::Mat> m_depthImagePtr(new cv::Mat());
		//*m_depthImagePtr = m_depthImage.clone();
		//imageUpdated[1](m_depthImagePtr);


	}
ReleaseFrame:
    // Release the frame
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	return;
}





void KinectV1Controller::UpdateSensorAndStatus(DWORD changedFlags)
{

}


void KinectV1Controller::UpdateNscControlStatus()
{

}

void KinectV1Controller::setImageRenderer(ImageRenderer* renderer,ID2D1Factory* pD2DFactory ){
	m_pDrawDataStreams = renderer; 
	m_pD2DFactory = pD2DFactory;
}

void KinectV1Controller::SetIcon(HWND hWnd)
{
	m_hWnd = hWnd;
}





HRESULT KinectV1Controller::openColorStream()
{
    // Open color stream.

	HRESULT hr;

	if (m_pNuiSensor)
    {
     
     hr = m_pNuiSensor->NuiImageStreamOpen(m_colorImageType,
                                                  m_colorResolution,
                                                  0,
                                                  2,
                                                  m_hNextColorFrameEvent,
                                                  &m_pColorStreamHandle);

	}

	if (SUCCEEDED(hr))
    {
        //m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_hStreamHandle, m_nearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);   // Set image flags
		m_colorRGBX.SetImageSize(m_colorResolution); // Set source image resolution to image buffer
    }

    return hr;
}


HRESULT KinectV1Controller::openDepthStream()
{
	HRESULT hr;
    // Open color stream.

	NUI_IMAGE_TYPE imageType = HasSkeletalEngine(m_pNuiSensor) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH;
	//NUI_IMAGE_TYPE imageType = NUI_IMAGE_TYPE_DEPTH;
    // Open depth stream
    hr = m_pNuiSensor->NuiImageStreamOpen(imageType,
                                                  m_depthResolution,
                                                  0,
                                                  2,
                                                  m_hNextDepthFrameEvent,
                                                  &m_pDepthStreamHandle);
    if (SUCCEEDED(hr))
    {
        m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, m_nearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);   // Set image flags

		m_DepthRGBX.SetImageSize(m_depthResolution); // Set source image resolution to image buffer
    }

    return hr;


}

HRESULT KinectV1Controller::openIbodyStream(){
	HRESULT hr = E_FAIL;

	return hr;
}

HRESULT KinectV1Controller::openAudioStream(){
	HRESULT hr = E_FAIL;

	return hr;
}


void KinectV1Controller::ProcessColor()
{

    HRESULT hr;

    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the color frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return;
    }

    if (m_paused)
    {
        // Stream paused. Skip frame process and release the frame.
        goto ReleaseFrame;
    }

    INuiFrameTexture* pTexture = imageFrame.pFrameTexture;

    // Lock the frame data so the Kinect knows not to modify it while we are reading it
    NUI_LOCKED_RECT lockedRect;
    pTexture->LockRect(0, &lockedRect, NULL, 0);

    // Make sure we've received valid data
    if (lockedRect.Pitch != 0)
    {
 
			//m_colorRGBX.CopyRGB(lockedRect.pBits, lockedRect.size);
			switch (m_colorImageType)
			{
				case NUI_IMAGE_TYPE_COLOR_RAW_BAYER:    // Convert raw bayer data to color image and copy to image buffer
					m_colorRGBX.CopyBayer(lockedRect.pBits, lockedRect.size);
					break;

				case NUI_IMAGE_TYPE_COLOR_INFRARED:     // Convert infrared data to color image and copy to image buffer
					m_colorRGBX.CopyInfrared(lockedRect.pBits, lockedRect.size);
					break;

				default:    // Copy color data to image buffer
					m_colorRGBX.CopyRGB(lockedRect.pBits, lockedRect.size);
					break;
			}
			UpdateColorFrameRate();
			if(m_showOpt ==  Color_Raw) 
			{
				m_showFps = m_colorFps;
				show();
			}

    }

    // Unlock frame data
    pTexture->UnlockRect(0);

ReleaseFrame:
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

}

void	KinectV1Controller::AlignDepthToColorSpace()
{
	    // Get of x, y coordinates for color in depth space
    // This will allow us to later compensate for the differences in location, angle, etc between the depth and color cameras
    HRESULT hr = m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(cColorResolution,cDepthResolution, 
		m_depthWidth*m_depthHeight,m_depthD16,m_depthWidth*m_depthHeight*2,m_colorCoordinates);
	if( hr == E_INVALIDARG)
	{
		int a =10;
	}
	else  if( hr == E_INVALIDARG)
	{
		int a =10;
	}
	else if( hr == E_POINTER)
	{
		int a =10;
	}

	int outputIndex = 0;
	USHORT* pSrc = m_alignedDepthD16;

    // loop over each row and column of the color
	//for (int i = 0; i < m_colorHeight*m_colorWidth; ++i)
    for (LONG y= 0; y < m_colorHeight; ++y)
    {
        for (LONG x = 0; x < m_colorWidth; ++x)
        {
            // calculate index into depth array
            int depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * m_depthWidth;

            USHORT depth  = m_depthD16[depthIndex];

			//int cordinatesIndex = x + y * m_colorWidth;
			
            // retrieve the depth to color mapping for the current depth pixel
            LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
            LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];
			            // retrieve the depth to color mapping for the current depth pixel
            //LONG colorInDepthX = m_colorCoordinates[i*2 ];
            //LONG colorInDepthY = m_colorCoordinates[i*2 + 1];
			
			//*pSrc = 0;
            // make sure the depth pixel maps to a valid point in color space
            //if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
			//if ( colorInDepthX >= 0 && colorInDepthX < m_depthWidth && colorInDepthY >= 0 && colorInDepthY < m_depthHeight )
            {
				//USHORT depth  = m_depthD16[colorInDepthX+colorInDepthY*m_depthWidth];
                // calculate index into color array
                LONG colorIndex = colorInDepthX + colorInDepthY * m_colorWidth;

                // set source for copy to the color pixel
				*(pSrc+outputIndex)  = depth;
            }
            
			//++pSrc;
			outputIndex++;
        }
    }
	m_alignedDepthRGBX.CopyDepth(m_alignedDepthD16, m_colorHeight*m_colorWidth*sizeof(RGBQUAD), m_nearMode, m_depthTreatment);


}


/// <summary>
/// Set and reset near mode
/// </summary>
/// <param name="nearMode">True to enable near mode. False to disable</param>
void KinectV1Controller::SetNearMode(bool nearMode)
{
    m_nearMode = nearMode;
    if (INVALID_HANDLE_VALUE != m_pDepthStreamHandle)
    {
        m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, (m_nearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0));
    }
}


CString KinectV1Controller::getColorTypeAsString(v1ColorType colorType)
{
	switch (colorType)
	{
	case RESOLUTION_RGBRESOLUTION640X480FPS30:
		return CString(L"RGB640x480");
	case RESOLUTION_RGBRESOLUTION1280X960FPS12:
		return CString(L"RGB1280x960");
	case RESOLUTION_YUVRESOLUTION640X480FPS15:
		return CString(L"YUVF640x480");
	case RESOLUTION_INFRAREDRESOLUTION640X480FPS30:
		return CString(L"INFRARED640x480");
	case RESOLUTION_RAWBAYERRESOLUTION640X480FPS30:
		return CString(L"BAYER640x480");
	case RESOLUTION_RAWBAYERRESOLUTION1280X960FPS12:
		return CString(L"BAYER1280x960");
	default:
		return CString(L"UNKNOWN TYPE");
		break;
	}
}

CString KinectV1Controller::getDepthTypeAsString(v1DepthType depthType)
{
	switch (depthType)
	{
	case RESOLUTION_DEPTHESOLUTION640X480FPS30:
		return CString(L"DEPTH640x480");
	case RESOLUTION_DEPTHRESOLUTION320X240FPS30:
		return CString(L"DEPTH320x240");
	case RESOLUTION_DEPTHRESOLUTION80X60FP30:
		return CString(L"DEPTH80x60");

	default:
		return CString(L"UNKNOWN TYPE");
		break;
	}
}


/// <summary>
/// Update frame rate
/// </summary>
void KinectV1Controller::UpdateColorFrameRate()
{
    m_colorFrameCount++;

    DWORD tickCount = GetTickCount();
    DWORD span      = tickCount - m_lastColorTick;
    if (span >= 1000)
    {
        m_colorFps            = (UINT)((double)(m_colorFrameCount - m_lastColorFrameCount) * 1000.0 / (double)span + 0.5);
        m_lastColorTick       = tickCount;
        m_lastColorFrameCount = m_colorFrameCount;
    }
}

/// <summary>
/// Update frame rate
/// </summary>
void KinectV1Controller::UpdateDepthFrameRate()
{
    m_depthFrameCount++;

    DWORD tickCount = GetTickCount();
    DWORD span      = tickCount - m_lastDepthTick;
    if (span >= 1000)
    {
        m_depthFps            = (UINT)((double)(m_depthFrameCount - m_lastDepthFrameCount) * 1000.0 / (double)span + 0.5);
        m_lastDepthTick       = tickCount;
        m_lastDepthFrameCount = m_depthFrameCount;
    }
}