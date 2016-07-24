#include "stdafx.h"
#include "RecordTabHandler.h"
#include "WindowsAppDialogHelper.h"

#define MAX_DATE 20
#include <time.h>

RecordTabHandler::RecordTabHandler() : 
	m_colorEnabled(true),
	m_centerEnabled(false),
	m_isRecording(false),
	m_KinectEnableOpt(3)
{
}


RecordTabHandler::~RecordTabHandler()
{
}

bool RecordTabHandler::isRecording()
{
	return m_isRecording;
}

void RecordTabHandler::setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration)
{
	m_recordingConfiguration = recordingConfiguration;
}

void RecordTabHandler::setSharedImageRecordingConfiguration(SharedImageRecordingConfiguration imageRecordingConfiguration)
{
	m_imageRecordingConfiguration = imageRecordingConfiguration;
}

void RecordTabHandler::setSharedStringStringRecordingConfiguration(SharedStringFileRecordingConfiguration stringFileRecordingConfiguration)
{
	m_KeyPointsRecordingConfiguration = stringFileRecordingConfiguration;
}

void RecordTabHandler::setSharedCommonConfiguration(SharedCommonConfiguration commonConfiguration)
{
	 m_commonConfiguration = commonConfiguration;
}

SharedCommonConfiguration RecordTabHandler::getSharedCommonConfiguration()
{
	return m_commonConfiguration;
}


void RecordTabHandler::movieShowOptWindosOfV1()
{
	RECT tabRecord;
	GetWindowRect(m_hWnd, &tabRecord);

	RECT showOptGroupRect;
	RECT showOptGroupV1Rect;
	RECT tmpRect;
	HWND tabShowOPTHandle = GetDlgItem(m_hWnd, IDC_GROUP_SHOW_OPT);
	HWND tabShowOPTForV1Handle = GetDlgItem(m_hWnd, IDC_GROUP_SHOW_OPT2);	
	GetWindowRect(tabShowOPTHandle, &showOptGroupRect);
	GetWindowRect(tabShowOPTForV1Handle, &showOptGroupV1Rect);
	//GetClientRect(tabShowOPTHandle, &showOptGroupRect);

	/* show opt group*/
	MoveWindow(GetDlgItem(m_hWnd, IDC_GROUP_SHOW_OPT2),
					showOptGroupRect.left-tabRecord.left,
					showOptGroupRect.top-tabRecord.top,
					showOptGroupRect.right - showOptGroupRect.left,
					showOptGroupRect.bottom - showOptGroupRect.top,
					TRUE);
		
	HWND v1ShowType = GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_STATIC);
	GetWindowRect(v1ShowType, &tmpRect);
			/* show opt group*/
	MoveWindow(v1ShowType,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

	/*
	HWND v1ShowResolution =GetDlgItem(m_hWnd, IDC_V1_SHOW_RESOLUTION_STATIC);
	GetWindowRect(v1ShowResolution, &tmpRect);
	MoveWindow(v1ShowResolution,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);*/

	HWND v1ColorShow =GetDlgItem(m_hWnd, IDC_V1_COLOR_SHOW_RADIO);
	GetWindowRect(v1ColorShow, &tmpRect);
	MoveWindow(v1ColorShow,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

	HWND v1DepthShow =GetDlgItem(m_hWnd, IDC_V1_DEPTH_SHOW_RADIO);
	GetWindowRect(v1DepthShow, &tmpRect);
	MoveWindow(v1DepthShow,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);
	
	HWND v1IDShowType =GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_COMBO);
	GetWindowRect(v1IDShowType, &tmpRect);
	MoveWindow(v1IDShowType,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

	/*
	HWND v1InfraredShow =GetDlgItem(m_hWnd, IDC_V1_INFRARED_SHOW_RADIO);
	GetWindowRect(v1InfraredShow, &tmpRect);
	MoveWindow(v1InfraredShow,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);
					*/

		
	/*
	HWND v1R640 =GetDlgItem(m_hWnd, IDC_V1_R640X480_RADIO);
	GetWindowRect(v1R640, &tmpRect);
	MoveWindow(v1R640,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

	HWND v1R320 =GetDlgItem(m_hWnd, IDC_V1_R320X240_RADIO);
	GetWindowRect(v1R320, &tmpRect);
	MoveWindow(v1R320,
		showOptGroupRect.left-tabRecord.left+tmpRect.left-showOptGroupV1Rect.left,
		showOptGroupRect.top-tabRecord.top+tmpRect.top-showOptGroupV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

	*/
	HWND recordingOPTV1Handle = GetDlgItem(m_hWnd, IDC_RECORDING_OPT_V1);
	HWND recordingOPTV2Handle = GetDlgItem(m_hWnd, IDC_RECORDING_OPT_V2);
	
	RECT recordingOPTV1Rect;
	RECT recordingOPTV2Rect;
	GetWindowRect(recordingOPTV1Handle, &recordingOPTV1Rect);
	GetWindowRect(recordingOPTV2Handle, &recordingOPTV2Rect);

	MoveWindow(recordingOPTV1Handle,
					recordingOPTV2Rect.left-tabRecord.left,
					recordingOPTV2Rect.top-tabRecord.top,
					recordingOPTV2Rect.right - recordingOPTV2Rect.left,
					(recordingOPTV1Rect.bottom - recordingOPTV1Rect.top)*2/3,
					TRUE);

	RECT tmpRect_CIG;
	HWND v1ColorInfGroup = GetDlgItem(m_hWnd, IDC_STATIC_COLOR_INF_V);
	GetWindowRect(v1ColorInfGroup, &tmpRect_CIG);
			/* show opt group*/
	MoveWindow(v1ColorInfGroup,
		recordingOPTV2Rect.left-tabRecord.left+tmpRect_CIG.left-recordingOPTV1Rect.left,
		recordingOPTV2Rect.top-tabRecord.top+tmpRect_CIG.top-recordingOPTV1Rect.top,
					tmpRect_CIG.right - tmpRect_CIG.left,
					tmpRect_CIG.bottom - tmpRect_CIG.top,
					TRUE);

	HWND v1Color = GetDlgItem(m_hWnd, IDC_V1_RECORD_COLOR_CHECK);
	GetWindowRect(v1Color, &tmpRect);
			/* show opt group*/
	MoveWindow(v1Color,
		recordingOPTV2Rect.left-tabRecord.left+tmpRect.left-recordingOPTV1Rect.left,
		recordingOPTV2Rect.top-tabRecord.top+tmpRect.top-recordingOPTV1Rect.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

	

	RECT tmpRect_Infrared;
	HWND v1Infrared = GetDlgItem(m_hWnd, IDC_V1_RECORD_INFRARED_CHECK);
	GetWindowRect(v1Infrared, &tmpRect_Infrared);
			
	MoveWindow(v1Infrared,
		recordingOPTV2Rect.left-tabRecord.left+tmpRect_Infrared.left-recordingOPTV1Rect.left,
		recordingOPTV2Rect.top-tabRecord.top+tmpRect_Infrared.top-recordingOPTV1Rect.top,
					tmpRect_Infrared.right - tmpRect_Infrared.left,
					tmpRect_Infrared.bottom - tmpRect_Infrared.top,
					TRUE);
	

	//IDC_V1_RECORD_DEPTH_CHECK
	HWND v1Depth = GetDlgItem(m_hWnd, IDC_V1_RECORD_DEPTH_CHECK);
	GetWindowRect(v1Depth, &tmpRect);
			/* show opt group*/
	MoveWindow(v1Depth,
		recordingOPTV2Rect.left-tabRecord.left+tmpRect.left-recordingOPTV1Rect.left + tmpRect_CIG.right - tmpRect_CIG.left+10,
		recordingOPTV2Rect.top-tabRecord.top+tmpRect.top-recordingOPTV1Rect.top - tmpRect.top + tmpRect_Infrared.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);

		//IDC_V1_RECORD_DEPTH_CHECK
	HWND v1Aligned = GetDlgItem(m_hWnd, IDC_V1_RECORDING_ALIGNED_CHECKBOX);
	GetWindowRect(v1Aligned, &tmpRect);
			/* show opt group*/
	MoveWindow(v1Aligned,
		recordingOPTV2Rect.left-tabRecord.left+tmpRect.left-recordingOPTV1Rect.left + tmpRect_CIG.right - tmpRect_CIG.left+10,
		recordingOPTV2Rect.top-tabRecord.top+tmpRect.top-recordingOPTV1Rect.top - tmpRect.top + tmpRect_Infrared.top,
					tmpRect.right - tmpRect.left,
					tmpRect.bottom - tmpRect.top,
					TRUE);
}
void RecordTabHandler::onCreate()
{

	//preset the edit boxes with the values of the model objects
	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX),			m_recordingConfiguration[    HDFace	   ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX),			m_recordingConfiguration[   FaceRaw	   ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX),	m_recordingConfiguration[ FullDepthRaw ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_2D_EDIT_BOX),		m_recordingConfiguration[   HDFace2D   ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_EDIT_BOX), m_imageRecordingConfiguration[KinectColorRaw]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_EDIT_BOX), m_imageRecordingConfiguration[KinectDepthRaw]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_EDIT_BOX), m_imageRecordingConfiguration[KinectAlignedDepthRaw]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_INFRARED_EDIT_BOX), m_imageRecordingConfiguration[KinectInfrared]->getFileNameCString());

	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_KEYPOINTS_EDIT_BOX), m_KeyPointsRecordingConfiguration[FiveKeyPoints]->getFileNameCString());

	CheckDlgButton(m_hWnd, IDC_RECORD_COLOR, m_colorEnabled);
	CheckDlgButton(m_hWnd, IDC_CENTER_CLOUDS, m_centerEnabled);

	//create combo box items for the recording file formats (ply, pcd, binary etc.)
	HWND hdFaceComboBox = GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX); 
	HWND hdFace2DComboBox = GetDlgItem(m_hWnd, IDC_HD_FACE_2D_COMBO_BOX);
	HWND facerawDepthComboBox = GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX);
	HWND fullRawDepthCombobox = GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX);

	HWND hd3DFaceComboBox = GetDlgItem(m_hWnd, IDC_3D_POINTS_CLOUDS_FORMAT_COMBOX);

	
	
	HWND kinectColorComboBox = GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_COMBO_BOX);
	HWND kinectDepthComboBox = GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_COMBO_BOX);
	HWND kinectAlignedDepthComboBox = GetDlgItem(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_COMBO_BOX);
	HWND kinectInfraredComboBox = GetDlgItem(m_hWnd, IDC_KINECT_INFRARED_COMBO_BOX);

	HWND kinectImageFormatComboBox = GetDlgItem(m_hWnd, IDC_IMAGE_FORMAT_COMBO);

	//HWND kinectKeypointsComboBox = GetDlgItem(m_hWnd, IDC_KINECT_KEYPOINTS_COMBO_BOX);
	HWND kinectKeypointsComboBox = GetDlgItem(m_hWnd, IDC_KP_FORMAT);
	
	HWND kinectShowOptComboBox = GetDlgItem(m_hWnd, IDC_COMBO_BOX_SHOW_OPT);
	
	if(m_KinectEnableOpt == 1)
	{
		
		movieShowOptWindosOfV1();
		ShowWindow(GetDlgItem(m_hWnd, IDC_GROUP_SHOW_OPT), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_STATIC_V2_SHOW), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_COMBO_BOX_SHOW_OPT), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_KINECT_REMOVE_BG_CHECKBOX), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, HD_FACE_SHOW_RADIO), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, FIVE_KEY_POINTS_SHOW_RADIO), SW_HIDE);

		ShowWindow(GetDlgItem(m_hWnd, IDC_STATIC_INFRARED_V2), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_STATIC_DEPTH_V2), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_STATIC_KEYPOINTS_V2), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_OPT_V2), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_ALIGNED_INFRARED_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_DEPTH_RAW_CHECK), SW_HIDE);
	
		ShowWindow(GetDlgItem(m_hWnd, IDC_ALIGNED_DEPTH_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_DEPTH_3D_RAW_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_HD2D_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_HD3D_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_FACE_RAW_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_KEY_POINTS_CHECK), SW_HIDE);

		ShowWindow(GetDlgItem(m_hWnd, IDC_RAW_COLOR_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_OPT_RESET), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_INRARED_RAW_CHECK), SW_HIDE);


		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_V1_ENABLE_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_V2_ENABLE_CHECK), SW_HIDE);
			
		CheckDlgButton(m_hWnd, IDC_V1_COLOR_SHOW_RADIO, true);

		
		//CheckDlgButton(m_hWnd, IDC_V1_R640X480_RADIO, true);
		HWND v1ShowTypeComboBox = GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_COMBO);
		
		for (int i = 0; i < V1_COLOR_TYPE_COUNT; i++){
			CString colorType = KinectV1Controller::getColorTypeAsString(static_cast<v1ColorType>( i));
			ComboBox_AddString(v1ShowTypeComboBox, colorType);
			if(i==0)
				ComboBox_SetCurSel(v1ShowTypeComboBox, 0);
			//ComboBox_ResetContent
		}

		
		

	}
	else if (m_KinectEnableOpt == 0)
	{
		ShowWindow(GetDlgItem(m_hWnd, IDC_GROUP_SHOW_OPT2), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_STATIC), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_SHOW_RESOLUTION_STATIC), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_COLOR_SHOW_RADIO), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_DEPTH_SHOW_RADIO), SW_HIDE);
		//ShowWindow(GetDlgItem(m_hWnd, IDC_V1_INFRARED_SHOW_RADIO), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_R640X480_RADIO), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_R320X240_RADIO), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_OPT_V1), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_STATIC_COLOR_INF_V), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_RECORD_COLOR_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_RECORD_INFRARED_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_RECORD_DEPTH_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_V1_RECORDING_ALIGNED_CHECKBOX), SW_HIDE);

		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_V1_ENABLE_CHECK), SW_HIDE);
		ShowWindow(GetDlgItem(m_hWnd, IDC_RECORDING_V2_ENABLE_CHECK), SW_HIDE);

	}
	else 
	{
		CheckDlgButton(m_hWnd, IDC_V1_COLOR_SHOW_RADIO, true);

		
		//CheckDlgButton(m_hWnd, IDC_V1_R640X480_RADIO, true);

		HWND v1ShowTypeComboBox = GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_COMBO);
		
		for (int i = 0; i < V1_COLOR_TYPE_COUNT; i++){
			CString colorType = KinectV1Controller::getColorTypeAsString(static_cast<v1ColorType>( i));
			ComboBox_AddString(v1ShowTypeComboBox, colorType);
			if(i==0)
				ComboBox_SetCurSel(v1ShowTypeComboBox, 0);
			//ComboBox_ResetContent
		}
	}
	
	for (int i = 0; i < RECORD_SHOW_OPT_COUNT; i++){
		CString ShowOpt = CommonConfiguration::getShowOptAsString(static_cast<RecordingShowOpt>(i));
		ComboBox_AddString(kinectShowOptComboBox, ShowOpt);
		if(i ==0)
			ComboBox_SetCurSel(kinectShowOptComboBox, 0);
	}	
	for (int i = 0; i < RECORD_FILE_FORMAT_COUNT; i++){
		CString fileFormatName = RecordingConfiguration::getFileFormatAsString(static_cast<RecordingFileFormat>(i));
		ComboBox_AddString(hdFaceComboBox, fileFormatName);
		
		ComboBox_AddString(facerawDepthComboBox, fileFormatName);
		ComboBox_AddString(fullRawDepthCombobox, fileFormatName);
		ComboBox_AddString(hdFace2DComboBox, fileFormatName);

		ComboBox_AddString(hd3DFaceComboBox, fileFormatName);

		if (i == 0){
			ComboBox_SetCurSel(hdFaceComboBox, i);
			ComboBox_SetCurSel(facerawDepthComboBox, i);
			ComboBox_SetCurSel(fullRawDepthCombobox, i);
			ComboBox_SetCurSel(hdFace2DComboBox, i);

			ComboBox_SetCurSel(hd3DFaceComboBox, i);
		}
	}

	for (int i = 0; i < IMAGE_RECORD_FILE_FORMAT_COUNT; i++){
		CString fileFormatName = ImageRecordingConfiguration::getFileFormatAsString(static_cast<ImageRecordingFileFormat>(i));
		ComboBox_AddString(kinectColorComboBox, fileFormatName);
		ComboBox_AddString(kinectDepthComboBox, fileFormatName);
		ComboBox_AddString(kinectAlignedDepthComboBox, fileFormatName);
		ComboBox_AddString(kinectInfraredComboBox, fileFormatName);

		ComboBox_AddString(kinectImageFormatComboBox, fileFormatName);
		
			if (i == 0){
			ComboBox_SetCurSel(kinectColorComboBox, i);
			ComboBox_SetCurSel(kinectDepthComboBox, i);
			ComboBox_SetCurSel(kinectAlignedDepthComboBox, i);
			ComboBox_SetCurSel(kinectInfraredComboBox, i);

			ComboBox_SetCurSel(kinectImageFormatComboBox, i);

		}
	}

	for (int i = 0; i < STRING_FILE_RECORD_FILE_FORMAT_COUNT; i++){
		CString fileFormatName = StringFileRecordingConfiguration::getFileFormatAsString(static_cast<StringFileRecordingFileFormat>(i));

		ComboBox_AddString(kinectKeypointsComboBox, fileFormatName);
		if (i == 0){

			ComboBox_SetCurSel(kinectKeypointsComboBox, i);
		}
	}


	//create combo box items for the amount of threads to start
	HWND hdFaceComboBoxThreads			= GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX_THREADS);
	HWND facerawDepthComboBoxThreads	= GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX_THREADS);
	HWND fullRawDepthComboboxThreads	= GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX_THREADS);
	HWND kinectColorRawComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_COMBO_BOX_THREADS);
	HWND kinectDepthRawComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_COMBO_BOX_THREADS);
	HWND kinectAlignedDepthRawComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_COMBO_BOX_THREADS);
	HWND kinectInfraredComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_INFRARED_COMBO_BOX_THREADS);
	HWND kinectKeyPointsComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_KEYPOINTS_COMBO_BOX_THREADS);

	HWND allComboboxThreads	= GetDlgItem(m_hWnd, IDC_ALL_COM_BOX_THREADS);

	HWND hdFace2DComboBoxThreads		= GetDlgItem(m_hWnd, IDC_HD_FACE_2D_COMBO_BOX_THREADS);
	for (int i = 1; i < 5; i++){
		CString counter;
		counter.Format(L"%d", i);
		ComboBox_AddString(hdFaceComboBoxThreads, counter);
		ComboBox_AddString(facerawDepthComboBoxThreads, counter);
		ComboBox_AddString(fullRawDepthComboboxThreads, counter);
		ComboBox_AddString(kinectColorRawComboboxThreads, counter);
		ComboBox_AddString(kinectDepthRawComboboxThreads, counter);
		ComboBox_AddString(kinectAlignedDepthRawComboboxThreads, counter);
		ComboBox_AddString(kinectInfraredComboboxThreads, counter);
		ComboBox_AddString(kinectKeyPointsComboboxThreads, counter);
		ComboBox_AddString(hdFace2DComboBoxThreads, counter);

		ComboBox_AddString(allComboboxThreads, counter);
		
	}
	ComboBox_SetCurSel(hdFaceComboBoxThreads,			m_recordingConfiguration[HDFace]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(facerawDepthComboBoxThreads,		m_recordingConfiguration[FaceRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(fullRawDepthComboboxThreads,		m_recordingConfiguration[FullDepthRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(hdFace2DComboBoxThreads,			m_recordingConfiguration[HDFace2D]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectColorRawComboboxThreads,	m_imageRecordingConfiguration[KinectColorRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectDepthRawComboboxThreads,	m_imageRecordingConfiguration[KinectDepthRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectAlignedDepthRawComboboxThreads,	m_imageRecordingConfiguration[KinectAlignedDepthRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectInfraredComboboxThreads,	m_imageRecordingConfiguration[KinectInfrared]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectKeyPointsComboboxThreads,	m_KeyPointsRecordingConfiguration[FiveKeyPoints]->getThreadCountToStart() - 1);


	ComboBox_SetCurSel(allComboboxThreads,			m_recordingConfiguration[HDFace]->getThreadCountToStart() - 1);


	CheckDlgButton(m_hWnd, HD_FACE_SHOW_RADIO, true);
	m_commonConfiguration[KinectV2_COMMON]->setFacePointsShowOpt(FacePointsShowOpt::HDFacePoints_Opt);

	CheckDlgButton(m_hWnd, IDC_KINECT_REMOVE_BG_CHECKBOX, m_commonConfiguration[KinectV2_COMMON]->isKeepBGEnabled());

}

bool RecordTabHandler::isColorEnabled()
{
	return m_colorEnabled;
}

bool RecordTabHandler::isCenterEnabled()
{
	return m_centerEnabled;
}

void RecordTabHandler::updateWriterStatus(RecordCloudType recordType, std::wstring status)
{
	_In_z_ WCHAR* newStatus = &status[0];
	switch (recordType)
	{
	case HDFace:
		SetDlgItemText(m_hWnd, IDC_HD_FACE_STATUS, newStatus);
		break;
	case FaceRaw:
		SetDlgItemText(m_hWnd, IDC_FACE_RAW_DEPTH_STATUS, newStatus);
		break;
	case FullDepthRaw:
		SetDlgItemText(m_hWnd, IDC_FULL_RAW_DEPTH_STATUS, newStatus);
		break;
	case HDFace2D:
		SetDlgItemText(m_hWnd, IDC_HD_FACE_2D_STATUS, newStatus);
		break;
	case RECORD_CLOUD_TYPE_COUNT:
		break;
	default:
		break;
	}
}

void RecordTabHandler::updateWriterStatus(ImageRecordType recordType, std::wstring status)
{
	_In_z_ WCHAR* newStatus = &status[0];
	switch (recordType)
	{
	case KinectColorRaw:
		SetDlgItemText(m_hWnd, IDC_KINECT_RAW_COLOR_STATUS, newStatus);
		break;
	case KinectDepthRaw:
		SetDlgItemText(m_hWnd, IDC_KINECT_RAW_DEPTH_STATUS, newStatus);
		break;
	case KinectAlignedDepthRaw:
		SetDlgItemText(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_STATUS, newStatus);
		break;
	case KinectInfrared:
		SetDlgItemText(m_hWnd, IDC_KINECT_INFRARED_STATUS, newStatus);
		break;
	case IMAGE_RECORD_TYPE_COUNT:
		break;
	default:
		break;
	}
}

void RecordTabHandler::updateWriterStatus(StringFileRecordType recordType, std::wstring status)
{
	_In_z_ WCHAR* newStatus = &status[0];
	switch (recordType)
	{
	case FiveKeyPoints:
		SetDlgItemText(m_hWnd, IDC_KINECT_KEYPOINTS_STATUS, newStatus);
		break;

	case STRING_FILE_RECORD_TYPE_COUNT:
		break;
	default:
		break;
	}
}


void RecordTabHandler::onSelectionChanged(WPARAM wParam, LPARAM handle)
{
	int currentSelection = ComboBox_GetCurSel(GetDlgItem(m_hWnd, LOWORD(wParam)));
	switch (LOWORD(wParam))
	{
	case IDC_3D_POINTS_CLOUDS_FORMAT_COMBOX:
		m_recordingConfiguration[HDFace]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		m_recordingConfiguration[FaceRaw]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		m_recordingConfiguration[HDFace2D]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		m_recordingConfiguration[FullDepthRaw]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;


	case IDC_HD_FACE_COMBO_BOX:
		m_recordingConfiguration[HDFace]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;
	case IDC_FACE_RAW_DEPTH_COMBO_BOX:
		m_recordingConfiguration[FaceRaw]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;
	case IDC_FULL_RAW_DEPTH_COMBO_BOX:
		m_recordingConfiguration[FullDepthRaw]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;
	case IDC_HD_FACE_2D_COMBO_BOX:
		m_recordingConfiguration[HDFace2D]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;

	case IDC_IMAGE_FORMAT_COMBO:
		m_imageRecordingConfiguration[KinectColorRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		m_imageRecordingConfiguration[KinectDepthRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		m_imageRecordingConfiguration[KinectInfrared]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setFileFormat(static_cast<StringFileRecordingFileFormat>(currentSelection));
		break;

	case IDC_KINECT_RAW_COLOR_COMBO_BOX:
		m_imageRecordingConfiguration[KinectColorRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		break;
	case IDC_KINECT_RAW_DEPTH_COMBO_BOX:
		m_imageRecordingConfiguration[KinectDepthRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		break;
	case IDC_KINECT_ALIGHNED_DEPTH_COMBO_BOX:
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		break;
	case IDC_KINECT_INFRARED_COMBO_BOX:
		m_imageRecordingConfiguration[KinectInfrared]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		break;

	case IDC_KP_FORMAT:
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setFileFormat(static_cast<StringFileRecordingFileFormat>(currentSelection));
		break;

	case IDC_KINECT_KEYPOINTS_COMBO_BOX:
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setFileFormat(static_cast<StringFileRecordingFileFormat>(currentSelection));
		break;
		
	case IDC_ALL_COM_BOX_THREADS:
		m_recordingConfiguration[HDFace]->setThreadCountToStart(currentSelection+1);
		m_recordingConfiguration[FaceRaw]->setThreadCountToStart(currentSelection + 1);
		m_recordingConfiguration[FullDepthRaw]->setThreadCountToStart(currentSelection+1);
		m_recordingConfiguration[HDFace2D]->setThreadCountToStart(currentSelection + 1);
		m_imageRecordingConfiguration[KinectColorRaw]->setThreadCountToStart(currentSelection + 1);
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setThreadCountToStart(currentSelection + 1);
		m_imageRecordingConfiguration[KinectInfrared]->setThreadCountToStart(currentSelection + 1);
		m_imageRecordingConfiguration[KinectInfrared]->setThreadCountToStart(currentSelection + 1);
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setThreadCountToStart(currentSelection + 1);
		break;

	case IDC_HD_FACE_COMBO_BOX_THREADS:
		m_recordingConfiguration[HDFace]->setThreadCountToStart(currentSelection+1);
		break;
	case IDC_FACE_RAW_DEPTH_COMBO_BOX_THREADS:
		m_recordingConfiguration[FaceRaw]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_FULL_RAW_DEPTH_COMBO_BOX_THREADS:
		m_recordingConfiguration[FullDepthRaw]->setThreadCountToStart(currentSelection+1);
		break;
	case IDC_HD_FACE_2D_COMBO_BOX_THREADS:
		m_recordingConfiguration[HDFace2D]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_KINECT_RAW_COLOR_COMBO_BOX_THREADS:
		m_imageRecordingConfiguration[KinectColorRaw]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_KINECT_RAW_DEPTH_COMBO_BOX_THREADS:
		m_imageRecordingConfiguration[KinectDepthRaw]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_KINECT_ALIGHNED_DEPTH_COMBO_BOX_THREADS:
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_KINECT_INFRARED_COMBO_BOX_THREADS:
		m_imageRecordingConfiguration[KinectInfrared]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_KINECT_KEYPOINTS_COMBO_BOX_THREADS:
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setThreadCountToStart(currentSelection + 1);
		break;
	case IDC_COMBO_BOX_SHOW_OPT:
		m_commonConfiguration[KinectV2_COMMON]->setShowOpt(static_cast<RecordingShowOpt>(currentSelection));
		break;
	case IDC_V1_SHOW_TYPE_COMBO:
		v1ShowResolutionChanged(currentSelection);
		break;
	default:break;
	}
}


void RecordTabHandler::checkRecordingConfigurationPossible()
{
	bool oneEnabled = false;
	bool allValid = true;
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		oneEnabled |=	m_recordingConfiguration[i]->isEnabled();
		allValid &=		m_recordingConfiguration[i]->isRecordConfigurationValid();
	}
	for (int i = 0; i < IMAGE_RECORD_TYPE_COUNT; i++){
		oneEnabled |= m_imageRecordingConfiguration[i]->isEnabled();
		allValid &= m_imageRecordingConfiguration[i]->isRecordConfigurationValid();
	}

	if (!oneEnabled || !allValid){
		Button_Enable(GetDlgItem(m_hWnd, IDC_RECORD_BUTTON), false);
	}
	else{
		Button_Enable(GetDlgItem(m_hWnd, IDC_RECORD_BUTTON), true);
	}

}

CString getRecordTimeStamp()
{
	//create the current timestamp for folder naming
	time_t now;
	char the_date[MAX_DATE];

	the_date[0] = '\0';

	now = time(NULL);

	if (now != -1)
	{
		strftime(the_date, MAX_DATE, "%Y_%m_%d_%H_%M_%S", gmtime(&now));
	}
	
	return CString(the_date);
}

void RecordTabHandler::setupRecording()
{
	//set the full recording path & timestamp
	auto timeStamp = getRecordTimeStamp();
	for (auto& recordConfig : m_recordingConfiguration){
		recordConfig->setTimeStampFolderName(timeStamp);
		if (recordConfig->isEnabled()){
			auto fullRecordingPath = recordConfig->getFullRecordingPath();
			SHCreateDirectoryEx(m_hWnd, fullRecordingPath, NULL);
		}
	}

	for (auto& recordConfig : m_imageRecordingConfiguration){
		recordConfig->setTimeStampFolderName(timeStamp);
		if (recordConfig->isEnabled()){
			auto fullRecordingPath = recordConfig->getFullRecordingPath();
			SHCreateDirectoryEx(m_hWnd, fullRecordingPath, NULL);
		}
	}

	for (auto& recordConfig : m_KeyPointsRecordingConfiguration){
		recordConfig->setTimeStampFolderName(timeStamp);
		if (recordConfig->isEnabled()){
			auto fullRecordingPath = recordConfig->getFullRecordingPath();
			SHCreateDirectoryEx(m_hWnd, fullRecordingPath, NULL);
		}
	}
}

void RecordTabHandler::recordingStopped()
{
	SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Record");
	m_isRecording = false;
}

void RecordTabHandler::setRecording(bool enable)
{
	if (enable == m_isRecording){
		return;
	}

	m_isRecording = enable;
	
	if (enable){
		setupRecording();
		startWriting(m_colorEnabled, m_recordingConfiguration, m_imageRecordingConfiguration,m_KeyPointsRecordingConfiguration);
		SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Stop");
	}
	else{
		stopWriting(m_colorEnabled, m_recordingConfiguration, m_imageRecordingConfiguration,m_KeyPointsRecordingConfiguration);
	}	
}

void RecordTabHandler::setColorEnabled(bool enable)
{
	m_colorEnabled = enable;
	colorConfigurationChanged(enable);
}

void RecordTabHandler::setCeterEnabled(bool enable)
{
	m_centerEnabled = enable;
	centerConfigurationChanged(enable);
}

void RecordTabHandler::updateFrameLimit()
{
	bool isLimited = IsDlgButtonChecked(m_hWnd, IDC_LIMIT_FRAMES_CHECK);
	Edit_Enable(GetDlgItem(m_hWnd, IDC_LIMIT_FRAMES_EDIT_BOX), isLimited);
	
	int limitAsInt = UNLIMITED_FRAMES;

	//if the user has set an limit it is now parsed and overridden
	if (isLimited){
		auto editBoxHandle = GetDlgItem(m_hWnd, IDC_LIMIT_FRAMES_EDIT_BOX);
		std::vector<wchar_t> buffer(MAX_PATH);
		Edit_GetText(editBoxHandle, buffer.data(), buffer.size());
		limitAsInt = _tstoi(buffer.data());
	}

	//store the limit in each config
	for (auto recordConfig : m_recordingConfiguration){
		recordConfig->setMaxNumberOfFrames(limitAsInt);
	}
	for (auto recordConfig : m_imageRecordingConfiguration){
		recordConfig->setMaxNumberOfFrames(limitAsInt);
	}

	for (auto recordConfig : m_KeyPointsRecordingConfiguration){
		recordConfig->setMaxNumberOfFrames(limitAsInt);
	}
}

void RecordTabHandler::updateFPSLimit()
{
	bool isLimited = IsDlgButtonChecked(m_hWnd, IDC_LIMIT_FRAMERATE_CHECK);
	Edit_Enable(GetDlgItem(m_hWnd, IDC_LIMIT_FRAMRATE_EDIT_BOX), isLimited);

	int fps = 0;

	//if the user has set an limit it is now parsed and overridden
	if (isLimited){
		auto editBoxHandle = GetDlgItem(m_hWnd, IDC_LIMIT_FRAMRATE_EDIT_BOX);
		std::vector<wchar_t> buffer(MAX_PATH);
		Edit_GetText(editBoxHandle, buffer.data(), buffer.size());
		fps = _tstoi(buffer.data());
	}

	fpsLimitUpdated(fps);
}

void RecordTabHandler::onButtonClicked(WPARAM wParam, LPARAM handle)
{
	
	switch (LOWORD(wParam))
	{
	case IDC_RECORD_COLOR:
		setColorEnabled(IsDlgButtonChecked(m_hWnd, IDC_RECORD_COLOR));
		break;
	case IDC_CENTER_CLOUDS:
		setCeterEnabled(IsDlgButtonChecked(m_hWnd, IDC_CENTER_CLOUDS));
		break;
	case IDC_LIMIT_FRAMES_CHECK:
	{
		updateFrameLimit();
		break;
	}
	case IDC_RECORD_BUTTON:
	{
		setRecording(!m_isRecording);
		break;
	}
	case IDC_LIMIT_FRAMERATE_CHECK:
		updateFPSLimit();
		break;
	case IDC_RAW_COLOR_CHECK:
		m_imageRecordingConfiguration[KinectColorRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_RAW_COLOR_CHECK));
		break;


	case IDC_DEPTH_RAW_CHECK:
		m_imageRecordingConfiguration[KinectDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_DEPTH_RAW_CHECK));
		break;
	case IDC_ALIGNED_DEPTH_CHECK:
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_ALIGNED_DEPTH_CHECK));
		break;


	case IDC_HD2D_CHECK:
		m_recordingConfiguration[HDFace2D]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD2D_CHECK));
		break;
	case IDC_HD3D_CHECK:
		m_recordingConfiguration[HDFace]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD3D_CHECK));
		break;
	case IDC_FACE_RAW_CHECK:
		m_recordingConfiguration[FaceRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FACE_RAW_CHECK));
		break;
	case IDC_KEY_POINTS_CHECK:
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_KEY_POINTS_CHECK));
		break;

	case IDC_INRARED_RAW_CHECK:
		m_imageRecordingConfiguration[KinectInfrared]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_INRARED_RAW_CHECK));
		break;
	case IDC_ALIGNED_INFRARED_CHECK:
		m_imageRecordingConfiguration[KinectAlignedInfrared]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_ALIGNED_INFRARED_CHECK));
		break;


	case IDC_DEPTH_3D_RAW_CHECK:
		m_recordingConfiguration[FullDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_DEPTH_3D_RAW_CHECK));
		break;

	


	case IDC_RECORDING_OPT_RESET:
		
		CheckDlgButton(m_hWnd, IDC_HD2D_CHECK, false);
		CheckDlgButton(m_hWnd, IDC_HD3D_CHECK, false);
		CheckDlgButton(m_hWnd, IDC_FACE_RAW_CHECK, false);
		CheckDlgButton(m_hWnd, IDC_KEY_POINTS_CHECK, false);
		m_recordingConfiguration[HDFace]->setEnabled(false);
		m_recordingConfiguration[HDFace2D]->setEnabled(false);
		m_recordingConfiguration[FaceRaw]->setEnabled(false);
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setEnabled(false);

		CheckDlgButton(m_hWnd, IDC_RAW_COLOR_CHECK, false);
		m_imageRecordingConfiguration[KinectColorRaw]->setEnabled(false);
		

		CheckDlgButton(m_hWnd, IDC_DEPTH_RAW_CHECK, false);
		CheckDlgButton(m_hWnd, IDC_ALIGNED_DEPTH_CHECK, false);
		m_imageRecordingConfiguration[KinectDepthRaw]->setEnabled(false);
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setEnabled(false);


		CheckDlgButton(m_hWnd, IDC_INRARED_RAW_CHECK, false);
		CheckDlgButton(m_hWnd, IDC_ALIGNED_INFRARED_CHECK, false);
		m_imageRecordingConfiguration[KinectInfrared]->setEnabled(false);
		m_imageRecordingConfiguration[KinectAlignedInfrared]->setEnabled(false);

		CheckDlgButton(m_hWnd, IDC_DEPTH_3D_RAW_CHECK, false);
		m_recordingConfiguration[FullDepthRaw]->setEnabled(false);
		break;

		
	
	case IDC_KINECT_REMOVE_BG_CHECKBOX:
		m_commonConfiguration[KinectV2_COMMON]->setKeepBGEnabled(IsDlgButtonChecked(m_hWnd, IDC_KINECT_REMOVE_BG_CHECKBOX));
		//m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setPointsEnabled(IsDlgButtonChecked(m_hWnd, IDC_KINECT_KEYPOINTS_CHECKBOX));
		break;

	case HD_FACE_SHOW_RADIO:
		m_commonConfiguration[KinectV2_COMMON]->setFacePointsShowOpt(FacePointsShowOpt::HDFacePoints_Opt);
	
		break;
	case FIVE_KEY_POINTS_SHOW_RADIO:
		m_commonConfiguration[KinectV2_COMMON]->setFacePointsShowOpt(FacePointsShowOpt::FiveKeyPoints_Opt);
		
		break;
	case IDC_BUTTON_CHOOSE_OUTPUT_DIRECTORY:
	{

		WCHAR szDir[MAX_PATH];
		if (WindowsAppDialogHelper::openDirectoryDialog(szDir, m_hWnd)){
			SetDlgItemText(m_hWnd, IDC_FILE_PATH_EDIT_BOX, szDir);
			for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
				m_recordingConfiguration[i]->setFilePath(szDir);
			}
			for (int i = 0; i < IMAGE_RECORD_TYPE_COUNT; i++){
				m_imageRecordingConfiguration[i]->setFilePath(szDir);
			}
					
			for (auto recordConfig : m_KeyPointsRecordingConfiguration){
					recordConfig->setFilePath(szDir);
			}
		}
		break;
	}
	case IDC_V1_COLOR_SHOW_RADIO:
		//m_commonConfiguration[KinectV1_COMMON]->setShowOpt(RecordingShowOpt::Color_Raw);
		HWND v1ShowTypeComboBox;
		v1ShowTypeComboBox = GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_COMBO);
		ComboBox_ResetContent(v1ShowTypeComboBox);
		for (int i = 0; i < v1ColorType::V1_COLOR_TYPE_COUNT; i++){
			CString colorType = KinectV1Controller::getColorTypeAsString(static_cast<v1ColorType>( i));
			ComboBox_AddString(v1ShowTypeComboBox, colorType);
			if(i == 0)
				ComboBox_SetCurSel(v1ShowTypeComboBox, i);
			//ComboBox_ResetContent
		}
		

		v1ShowOptChanged(RecordingShowOpt::Color_Raw);
		//EnableWindow(GetDlgItem(m_hWnd,IDC_V1_R320X240_RADIO), FALSE);
		break;
	case IDC_V1_DEPTH_SHOW_RADIO:
				//m_commonConfiguration[KinectV1_COMMON]->setShowOpt(RecordingShowOpt::Color_Raw);
		HWND v1ShowTypeComboBox1;
		v1ShowTypeComboBox1 = GetDlgItem(m_hWnd, IDC_V1_SHOW_TYPE_COMBO);
		ComboBox_ResetContent(v1ShowTypeComboBox1);
		for (int i = 0; i < v1DepthType::V1_DEPTH_TYPE_COUNT; i++){
		CString depthType = KinectV1Controller::getDepthTypeAsString(static_cast<v1DepthType>( i));
			ComboBox_AddString(v1ShowTypeComboBox1, depthType);
			if(i == 0)
				ComboBox_SetCurSel(v1ShowTypeComboBox1,i);
			//ComboBox_ResetContent
		}
		

		//m_commonConfiguration[KinectV1_COMMON]->setShowOpt(RecordingShowOpt::Depth_Raw);
		v1ShowOptChanged(RecordingShowOpt::Depth_Raw);
		//EnableWindow(GetDlgItem(m_hWnd,IDC_V1_R320X240_RADIO), TRUE);
		break;
	case IDC_V1_INFRARED_SHOW_RADIO:
		//v1ShowOptChanged(RecordingShowOpt::Infrared_Raw);
		//EnableWindow(GetDlgItem(m_hWnd,IDC_V1_R320X240_RADIO), TRUE);
		//m_commonConfiguration[KinectV1_COMMON]->setShowOpt(RecordingShowOpt::Infrared_Raw);
		break;

	case IDC_V1_R640X480_RADIO:
		//v1ShowResolutionChanged(NUI_IMAGE_RESOLUTION_640x480);
		//m_commonConfiguration[KinectV1_COMMON]->setShowResolution(NUI_IMAGE_RESOLUTION_640x480);
		break;
	case IDC_V1_R320X240_RADIO:
		//v1ShowResolutionChanged(NUI_IMAGE_RESOLUTION_320x240);
		//m_commonConfiguration[KinectV1_COMMON]->setShowResolution(NUI_IMAGE_RESOLUTION_320x240);
		break;
	
	default:
		break;
	}
}


void RecordTabHandler::onEditBoxeChanged(WPARAM wParam, LPARAM handle)
{
	HWND editBoxHandle = GetDlgItem(m_hWnd, LOWORD(wParam));
	CString editBoxText;
	
	Edit_GetText(editBoxHandle, editBoxText.GetBuffer(MAX_PATH), MAX_PATH);
	switch (LOWORD(wParam)){
	case IDC_FACE_RAW_EDIT_BOX:
		m_recordingConfiguration[FaceRaw]->setFileName(editBoxText);
		break;
	case IDC_HDFACE_EDIT_BOX:
		m_recordingConfiguration[HDFace]->setFileName(editBoxText);
		break;
	case IDC_FULL_RAW_DEPTH_EDIT_BOX:
		m_recordingConfiguration[FullDepthRaw]->setFileName(editBoxText);
		break;
	case IDC_HDFACE_2D_EDIT_BOX:
		m_recordingConfiguration[HDFace2D]->setFileName(editBoxText);
		break;
	case IDC_KINECT_RAW_COLOR_EDIT_BOX:
		m_imageRecordingConfiguration[KinectColorRaw]->setFileName(editBoxText);
		break;
	case IDC_KINECT_RAW_DEPTH_EDIT_BOX:
		m_imageRecordingConfiguration[KinectDepthRaw]->setFileName(editBoxText);
		break;
	case IDC_KINECT_ALIGHNED_DEPTH_EDIT_BOX:
		m_imageRecordingConfiguration[KinectAlignedDepthRaw]->setFileName(editBoxText);
		break;
	case IDC_KINECT_INFRARED_EDIT_BOX:
		m_imageRecordingConfiguration[KinectInfrared]->setFileName(editBoxText);
		break;
	case IDC_KINECT_KEYPOINTS_EDIT_BOX:
		m_KeyPointsRecordingConfiguration[FiveKeyPoints]->setFileName(editBoxText);
		break;
	case IDC_LIMIT_FRAMES_EDIT_BOX:
		updateFrameLimit();
		break;
	case IDC_LIMIT_FRAMRATE_EDIT_BOX:
		updateFPSLimit();
		break;
	case IDC_FILE_PATH_EDIT_BOX:
	{
		for (auto recordConfig : m_recordingConfiguration){
			recordConfig->setFilePath(editBoxText);
		}
		for (auto recordConfig : m_imageRecordingConfiguration){
			recordConfig->setFilePath(editBoxText);
		}

		for (auto recordConfig : m_KeyPointsRecordingConfiguration){
			recordConfig->setFilePath(editBoxText);
		}
		break;
	}
	default:
		break;
	}
}

void RecordTabHandler::recordConfigurationStatusChanged(RecordCloudType type, bool newState)
{
	switch (type)
	{
	case HDFace:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX_THREADS), newState);
		break;
	case FaceRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX_THREADS), newState);
		break;
	case FullDepthRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX_THREADS), newState);
		break;
	case HDFace2D:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_HDFACE_2D_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_HD_FACE_2D_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_HD_FACE_2D_COMBO_BOX_THREADS), newState);
		break;
	case RECORD_CLOUD_TYPE_COUNT:
		break;
	default:
		break;
	}

	checkRecordingConfigurationPossible();
}

void RecordTabHandler::recordConfigurationStatusChanged(ImageRecordType type, bool newState)
{
	switch (type)
	{
	case KinectColorRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_COMBO_BOX_THREADS), newState);
		break;
	case KinectDepthRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_COMBO_BOX_THREADS), newState);
		break;
	case KinectAlignedDepthRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_ALIGHNED_DEPTH_COMBO_BOX_THREADS), newState);
		break;
	case KinectInfrared:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_KINECT_INFRARED_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_INFRARED_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_INFRARED_COMBO_BOX_THREADS), newState);
		break;		
	default:
		break;
	}

	checkRecordingConfigurationPossible();
}

void RecordTabHandler::recordConfigurationStatusChanged(StringFileRecordType type, bool newState)
{
	switch (type)
	{
	case FiveKeyPoints:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_KINECT_KEYPOINTS_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_KEYPOINTS_COMBO_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_KINECT_KEYPOINTS_COMBO_BOX_THREADS), newState);
		break;
	
	default:
		break;
	}

	checkRecordingConfigurationPossible();
}

void RecordTabHandler::recordPathChanged(RecordCloudType type)
{
	checkRecordingConfigurationPossible();
}

void RecordTabHandler::recordPathChanged(ImageRecordType type)
{
	checkRecordingConfigurationPossible();
}

void RecordTabHandler::recordPathChanged(StringFileRecordType type)
{
	checkRecordingConfigurationPossible();
}

SharedRecordingConfiguration RecordTabHandler::getRecordConfiguration()
{
	return m_recordingConfiguration;
}

SharedImageRecordingConfiguration RecordTabHandler::getImageRecordConfiguration()
{
	return m_imageRecordingConfiguration;
}


void RecordTabHandler::setKinectEnableOpt(bool v1Enabled, bool v2Enabled)
{
	if(v1Enabled && v2Enabled)
	{
		m_KinectEnableOpt = 2;
	}
	else if(!v1Enabled && v2Enabled)
	{
		m_KinectEnableOpt = 0;
	}
	else if(v1Enabled && !v2Enabled)
	{
		m_KinectEnableOpt = 1;
	}
	else
	{
		m_KinectEnableOpt=3;
	}
}