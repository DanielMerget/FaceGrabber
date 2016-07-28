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
#include "KinectV1OutputStreamUpdater.h"
#include "IImageREcordingConfiguration.h"
#include <thread>
#include <mutex>
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
	
	/// <summary>
    /// set the imagee renderer for showing in the screen
    /// </summary>
    /// <returns>void</returns>
	void setImageRenderer(ImageRenderer* renderer,ID2D1Factory * pD2DFactory );

	void SetConfiguration(CommonConfigurationPtr commonConfiguration)
	{
		m_commonConfiguration = commonConfiguration;
	}

	/// <summary>
    /// change the show option, rgb or depth
    /// </summary>
    /// <returns>void</returns>
	void showOptUpdated(KinectV1ImageRecordType showOpt);

	
	/// <summary>
    /// change the show resolution
    /// </summary>
    /// <returns>void</returns>
	void showResolutionUpdated(int showResolution);

	/// <summary>
    /// change the show resolution of color images
    /// </summary>
    /// <returns>void</returns>
	void showcolorResolutionUpdated(v1ColorType showResolution);

	/// <summary>
    /// change the show resolution of depth images
    /// </summary>
    /// <returns>void</returns>
	void showdepthResolutionUpdated(v1DepthType showResolution);

	void SetIcon(HWND hWnd);
	
	/// <summary>
    /// get the string for showing in UI
    /// </summary>
    /// <returns>CString</returns>
	static CString getColorTypeAsString(v1ColorType colorType);

	/// <summary>
    /// get the string for showing in UI
    /// </summary>
    /// <returns>CString</returns>
	static CString getDepthTypeAsString(v1DepthType depthType);


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

	/// <summary>
    /// get the event handle of color frame 
    /// </summary>
    /// <returns>HANDLE</returns>
	HANDLE getCorlorFrameEvent();

	/// <summary>
    /// get the event handle of depth frame 
    /// </summary>
    /// <returns>HANDLE</returns>
	HANDLE getDepthFrameEvent();

	/// <summary>
    /// set the stream updater 
    /// </summary>
    /// <returns>void</returns>
	void setOutPutStreamUpdater(std::shared_ptr<KinectV1OutPutStreamUpdater> KinectV1OutPutStreamUpdater);

	/// <summary>
    /// get the  time stamp of the last color stream  
    /// </summary>
    /// <returns>LARGE_INTEGER</returns>
	LARGE_INTEGER getLastColorFrameStamp();

	/// <summary>
    /// get the  time stamp of the last depth stream  
    /// </summary>
    /// <returns>LARGE_INTEGER</returns>
	LARGE_INTEGER getLastDepthFrameStamp();

	/// <summary>
    /// check if needs to update the stream to oupput stream writer  
    /// </summary>
    /// <returns>void</returns>
	void updateWriter();

	
	/// <summary>
    /// cchange the resolution for recording
    /// </summary>
    /// <returns>void</returns>
	void recordingResolutionUpdated(KinectV1ImageRecordType opt,int showResolution);


	/// <summary>
    /// get the FPS of depth streams
    /// </summary>
    /// <returns>UINT</returns>
	UINT getDepthFrameFPS();

	/// <summary>
    /// get the FPS of color streams
    /// </summary>
    /// <returns>UINT</returns>
	UINT getColorFrameFPS();

	/// <summary>
    /// set the limited FPS
    /// </summary>
    /// <returns>void</returns>
	void setLimitedFPS(int fps);

	/// <summary>
    /// enable or disable the aligment of depth to color
    /// </summary>
    /// <returns>void</returns>
	void setAlignmentEnable(bool enable);

	/// <summary>
    /// check if the aligment is either enabled or disabled
    /// </summary>
    /// <returns>bool</returns>
	bool getAlignmentEnable();



	//std::mutex m_updateMutex;
private:
    
    // Current Kinect
    INuiSensor*             m_pNuiSensor;

    // Direct2D
    //ImageRenderer*          m_pDrawCoordinateMappingBasics;
   
    
    

	// handle of color stream
    HANDLE                  m_pColorStreamHandle;

	// handle of depth stream
	HANDLE                  m_pDepthStreamHandle;

	// event handle of color stream
    HANDLE                  m_hNextColorFrameEvent;

	// event handle of depth stream
	HANDLE                  m_hNextDepthFrameEvent;

	// current width of depth streams
    LONG                    m_depthWidth;

	// current height of depth streams
    LONG                    m_depthHeight;

	// current width of color streams
    LONG                    m_colorWidth;

	// current height of color streams
    LONG                    m_colorHeight;

	/* the ratio of color to depth in width*/
    LONG                    m_colorToDepthDivisor;

	/* the current received raw depth data*/
    USHORT*                 m_depthD16;

	/* the current aligned  depth data if the aligment enabled*/
	USHORT*					m_alignedDepthD16;

	/* buffer for save color imgages */
    NuiImageBuffer          m_colorRGBX;
    
	/*resverve, buffer for save depth intensity imgages */
	NuiImageBuffer			m_DepthRGBX;

	/* resverve, buffer for save aligned depth intensity imgages */
   // NuiImageBuffer			m_alignedDepthRGBX;
		
	/* pointer to buffer for showing */
	NuiImageBuffer*          m_drawRGBX;
	    

    LARGE_INTEGER           m_depthTimeStamp;
    LARGE_INTEGER           m_colorTimeStamp;

	/* ImageRenderer */
	ImageRenderer*			m_pDrawDataStreams;

	ID2D1Factory*           m_pD2DFactory;

    /// <summary>
    /// Create the first connected Kinect found 
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 CreateFirstConnected();

     /// <summary>
    /// convert to raw depth data 
    /// </summary>
    /// <returns>void</returns>
	void getRawDepthData(const BYTE* srcpImage, UINT size, USHORT* depthpImage, int width, int height);

	 /// <summary>
    /// align the depth to color space
    /// </summary>
    /// <returns>void</returns>
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

		
	/// <summary>
    /// show the image in the screen 
    /// </summary>
    /// <returns>void</returns>
	void show();
		
	/// <summary>
    /// caculat the current frame rate of color streams
    /// </summary>
    /// <returns>void</returns>
	void UpdateColorFrameRate();

	/// <summary>
    /// caculat the current frame rate of depth streams
    /// </summary>
    /// <returns>void</returns>
	void UpdateDepthFrameRate();

	bool m_paused;

	DEPTH_TREATMENT				m_depthTreatment;

	//UINT*						m_depthColorTable[MAX_PLAYER_INDEX + 1]; //[USHRT_MAX + 1]

	bool						m_nearMode;

	CommonConfigurationPtr		m_commonConfiguration;

	// the current  resolution of color streams
	NUI_IMAGE_RESOLUTION		m_colorResolution;

	// the current the resolution of depth streams
	NUI_IMAGE_RESOLUTION		m_depthResolution;

	// the image type  resolution of depth streams
	NUI_IMAGE_TYPE				m_colorImageType;
	//NUI_IMAGE_TYPE				m_depthImageType;

	// the current resolution of images will be showed
	NUI_IMAGE_RESOLUTION		m_showResolution;

	// show option
	KinectV1ImageRecordType			m_showOpt;

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

	//bool				m_colorFrameArrived;
	//bool				m_depthFrameArrived;


	// for recording the images
	std::shared_ptr<KinectV1OutPutStreamUpdater> m_outPutStreamUpdater;

	LARGE_INTEGER		m_LastColorTimeStamp;
	LARGE_INTEGER		m_LastDepthTimeStamp;

	/** \brief	FPS Limit. */
	int					m_FPSLimit;

	
	// check if the color frame should be dumped because of the limited FPS
	bool  ifDumpColorFrame(NUI_IMAGE_FRAME *frame);

	// check if the depth frame should be dumped because of the limited FPS
	bool  ifDumpDepthFrame(NUI_IMAGE_FRAME *frame);

	

	bool  m_alignmentEnabled;

	std::mutex m_colorMutex;
	std::mutex m_depthMutex;

	

	
};
