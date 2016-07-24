//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "NuiApi.h"
#include "ImageRenderer.h"
#include "CommonConfiguration.h"
#include <NuiSensorChooser.h>
#include "NuiSensorChooserUI.h"
#include "NuiImageBuffer.h"
#include "KinectV1Controller.h"

//#include "WindowsApplication.h"

#define MIN_DEPTH                   400
#define MAX_DEPTH                   16383

#define UNKNOWN_DEPTH               0
#define UNKNOWN_DEPTH_COLOR         0x003F3F07
#define TOO_NEAR_COLOR              0x001F7FFF
#define TOO_FAR_COLOR               0x007F0F3F
#define NEAREST_COLOR               0x00FFFFFF

 enum v1ColorType{
		RESOLUTION_RGBRESOLUTION640X480FPS30,
		RESOLUTION_RGBRESOLUTION1280X960FPS12, 
		RESOLUTION_YUVRESOLUTION640X480FPS15,
		RESOLUTION_INFRAREDRESOLUTION640X480FPS30,
		RESOLUTION_RAWBAYERRESOLUTION640X480FPS30,
		RESOLUTION_RAWBAYERRESOLUTION1280X960FPS12,
		V1_COLOR_TYPE_COUNT
} ;

 enum v1DepthType{
		RESOLUTION_DEPTHESOLUTION640X480FPS30,
		RESOLUTION_DEPTHRESOLUTION320X240FPS30, 
		RESOLUTION_DEPTHRESOLUTION80X60FP30,
		V1_DEPTH_TYPE_COUNT
} ;

class KinectV1Controller
{
    static const int        cBytesPerPixel    = 4;

    static const NUI_IMAGE_RESOLUTION cDepthResolution = NUI_IMAGE_RESOLUTION_320x240;
    
    // green screen background will also be scaled to this resolution
    static const NUI_IMAGE_RESOLUTION cColorResolution = NUI_IMAGE_RESOLUTION_640x480;

    static const int        cStatusMessageMaxLen = MAX_PATH*2;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    KinectV1Controller();

    /// <summary>
    /// Destructor
    /// </summary>
    ~KinectV1Controller();

	HRESULT init();

	    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();
	
	void setImageRenderer(ImageRenderer* renderer,ID2D1Factory * pD2DFactory );

	void SetConfiguration(CommonConfigurationPtr commonConfiguration)
	{
		m_commonConfiguration = commonConfiguration;
	}

	void showOptUpdated(RecordingShowOpt showOpt);
	void showResolutionUpdated(int showResolution);

	void showcolorResolutionUpdated(v1ColorType showResolution);

	void showdepthResolutionUpdated(v1DepthType showResolution);

	void SetIcon(HWND hWnd);

	static CString getColorTypeAsString(v1ColorType colorType);

	static CString getDepthTypeAsString(v1DepthType depthType);

	//DWORD run(WindowsApplication* pThis);

	   /// <summary>
    /// Handle new depth data
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    void                 ProcessDepth();

    /// <summary>
    /// Handle new color data
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    void                 ProcessColor();

	HANDLE getCorlorFrameEvent();

	HANDLE getDepthFrameEvent();

private:
    
    // Current Kinect
    INuiSensor*             m_pNuiSensor;

    // Direct2D
    //ImageRenderer*          m_pDrawCoordinateMappingBasics;
   
    
    


    HANDLE                  m_pColorStreamHandle;
	HANDLE                  m_pDepthStreamHandle;


    HANDLE                  m_hNextColorFrameEvent;
	HANDLE                  m_hNextDepthFrameEvent;


    LONG                    m_depthWidth;
    LONG                    m_depthHeight;

    LONG                    m_colorWidth;
    LONG                    m_colorHeight;

    LONG                    m_colorToDepthDivisor;

    USHORT*                 m_depthD16;
	USHORT*					m_alignedDepthD16;
    NuiImageBuffer          m_colorRGBX;
    
	NuiImageBuffer			m_DepthRGBX;
    NuiImageBuffer			m_alignedDepthRGBX;
	NuiImageBuffer          m_infraredRGBX;
	NuiImageBuffer          m_alignedInfraredRGBX;
	NuiImageBuffer*          m_drawRGBX;
	

    LONG*                   m_colorCoordinates;

    LARGE_INTEGER           m_depthTimeStamp;
    LARGE_INTEGER           m_colorTimeStamp;

    NuiSensorChooser*       m_pSensorChooser;
    NuiSensorChooserUI*     m_pSensorChooserUI;

	ImageRenderer*			m_pDrawDataStreams;
	ID2D1Factory*           m_pD2DFactory;

    /// <summary>
    /// Create the first connected Kinect found 
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 CreateFirstConnected();

 



    void                 AlignDepthToColorSpace();

	
	HRESULT openColorStream();
	HRESULT openDepthStream();
	HRESULT openIbodyStream();
	HRESULT openAudioStream();

	void SetNearMode(bool nearMode);

    /// <summary>
    /// Update the sensor and status based on the input changed flags
    /// </summary>
    /// <param name="changedFlags">The device change flags</param>
    void UpdateSensorAndStatus(DWORD changedFlags);

    /// <summary>
    /// Update the Nui Sensor Chooser UI control status
    /// </summary>
	void UpdateNscControlStatus();

	void show();

	void UpdateColorFrameRate();
	void UpdateDepthFrameRate();
	bool m_paused;

	DEPTH_TREATMENT				m_depthTreatment;

	UINT*						m_depthColorTable[MAX_PLAYER_INDEX + 1]; //[USHRT_MAX + 1]

	bool						m_nearMode;

	CommonConfigurationPtr		m_commonConfiguration;


	NUI_IMAGE_RESOLUTION		m_colorResolution;
	NUI_IMAGE_RESOLUTION		m_depthResolution;
	NUI_IMAGE_TYPE				m_colorImageType;
	//NUI_IMAGE_TYPE				m_depthImageType;

	NUI_IMAGE_RESOLUTION		m_showResolution;

	RecordingShowOpt			m_showOpt;

	HWND m_hWnd; 


	UINT                m_showFps;
	UINT                m_colorFps;
    UINT                m_colorFrameCount;
    UINT                m_lastColorFrameCount;
    DWORD               m_lastColorTick;

	UINT                m_depthFps;
    UINT                m_depthFrameCount;
    UINT                m_lastDepthFrameCount;
    DWORD               m_lastDepthTick;
};
