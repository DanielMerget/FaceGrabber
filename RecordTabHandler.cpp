#include "stdafx.h"
#include "RecordTabHandler.h"
#include "WindowsAppDialogHelper.h"

#define MAX_DATE 20
#include <time.h>

RecordTabHandler::RecordTabHandler() : 
	m_colorEnabled(true),
	m_centerEnabled(false),
	m_isRecording(false)
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


void RecordTabHandler::onCreate()
{
	//preset the edit boxes with the values of the model objects
	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX),			m_recordingConfiguration[    HDFace	   ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX),			m_recordingConfiguration[   FaceRaw	   ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX),	m_recordingConfiguration[ FullDepthRaw ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_2D_EDIT_BOX),		m_recordingConfiguration[   HDFace2D   ]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_EDIT_BOX), m_imageRecordingConfiguration[KinectColorRaw]->getFileNameCString());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_EDIT_BOX), m_imageRecordingConfiguration[KinectDepthRaw]->getFileNameCString());
	CheckDlgButton(m_hWnd, IDC_RECORD_COLOR, m_colorEnabled);
	CheckDlgButton(m_hWnd, IDC_CENTER_CLOUDS, m_centerEnabled);

	//create combo box items for the recording file formats (ply, pcd, binary etc.)
	HWND hdFaceComboBox = GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX);
	HWND facerawDepthComboBox = GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX);
	HWND fullRawDepthCombobox = GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX);
	HWND hdFace2DComboBox = GetDlgItem(m_hWnd, IDC_HD_FACE_2D_COMBO_BOX);
	HWND kinectColorComboBox = GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_COMBO_BOX);
	HWND kinectDepthComboBox = GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_COMBO_BOX);
	for (int i = 0; i < RECORD_FILE_FORMAT_COUNT; i++){
		CString fileFormatName = RecordingConfiguration::getFileFormatAsString(static_cast<RecordingFileFormat>(i));
		ComboBox_AddString(hdFaceComboBox, fileFormatName);
		ComboBox_AddString(facerawDepthComboBox, fileFormatName);
		ComboBox_AddString(fullRawDepthCombobox, fileFormatName);
		ComboBox_AddString(hdFace2DComboBox, fileFormatName);
		if (i == 0){
			ComboBox_SetCurSel(hdFaceComboBox, i);
			ComboBox_SetCurSel(facerawDepthComboBox, i);
			ComboBox_SetCurSel(fullRawDepthCombobox, i);
			ComboBox_SetCurSel(hdFace2DComboBox, i);
		}
	}

	for (int i = 0; i < IMAGE_RECORD_FILE_FORMAT_COUNT; i++){
		CString fileFormatName = ImageRecordingConfiguration::getFileFormatAsString(static_cast<ImageRecordingFileFormat>(i));
		ComboBox_AddString(kinectColorComboBox, fileFormatName);
		ComboBox_AddString(kinectDepthComboBox, fileFormatName);
		if (i == 0){
			ComboBox_SetCurSel(kinectColorComboBox, i);
			ComboBox_SetCurSel(kinectDepthComboBox, i);
		}
	}

	//create combo box items for the amount of threads to start
	HWND hdFaceComboBoxThreads			= GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX_THREADS);
	HWND facerawDepthComboBoxThreads	= GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX_THREADS);
	HWND fullRawDepthComboboxThreads	= GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX_THREADS);
	HWND kinectColorRawComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_RAW_COLOR_COMBO_BOX_THREADS);
	HWND kinectDepthRawComboboxThreads	= GetDlgItem(m_hWnd, IDC_KINECT_RAW_DEPTH_COMBO_BOX_THREADS);
	HWND hdFace2DComboBoxThreads		= GetDlgItem(m_hWnd, IDC_HD_FACE_2D_COMBO_BOX_THREADS);
	for (int i = 1; i < 5; i++){
		CString counter;
		counter.Format(L"%d", i);
		ComboBox_AddString(hdFaceComboBoxThreads, counter);
		ComboBox_AddString(facerawDepthComboBoxThreads, counter);
		ComboBox_AddString(fullRawDepthComboboxThreads, counter);
		ComboBox_AddString(kinectColorRawComboboxThreads, counter);
		ComboBox_AddString(kinectDepthRawComboboxThreads, counter);
		ComboBox_AddString(hdFace2DComboBoxThreads, counter);
	}
	ComboBox_SetCurSel(hdFaceComboBoxThreads,			m_recordingConfiguration[HDFace]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(facerawDepthComboBoxThreads,		m_recordingConfiguration[FaceRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(fullRawDepthComboboxThreads,		m_recordingConfiguration[FullDepthRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(hdFace2DComboBoxThreads,			m_recordingConfiguration[HDFace2D]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectColorRawComboboxThreads,	m_imageRecordingConfiguration[KinectColorRaw]->getThreadCountToStart() - 1);
	ComboBox_SetCurSel(kinectDepthRawComboboxThreads,	m_imageRecordingConfiguration[KinectDepthRaw]->getThreadCountToStart() - 1);
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
	case IMAGE_RECORD_TYPE_COUNT:
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
	case IDC_KINECT_RAW_COLOR_COMBO_BOX:
		m_imageRecordingConfiguration[KinectColorRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
		break;
	case IDC_KINECT_RAW_DEPTH_COMBO_BOX:
		m_imageRecordingConfiguration[KinectDepthRaw]->setFileFormat(static_cast<ImageRecordingFileFormat>(currentSelection));
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
		startWriting(m_colorEnabled, m_recordingConfiguration, m_imageRecordingConfiguration);
		SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Stop");
	}
	else{
		stopWriting(m_colorEnabled, m_recordingConfiguration, m_imageRecordingConfiguration);
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
	case IDC_HD_FACE_CHECKBOX:
		m_recordingConfiguration[HDFace]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD_FACE_CHECKBOX));
		break;
	case IDC_FACE_RAW_DEPTH_CHECKBOX:
		m_recordingConfiguration[FaceRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FACE_RAW_DEPTH_CHECKBOX));
		break;
	case IDC_FULL_RAW_DEPTH_CHECKBOX:
		m_recordingConfiguration[FullDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FULL_RAW_DEPTH_CHECKBOX));
		break;
	case IDC_HD_FACE_2D_CHECKBOX:
		m_recordingConfiguration[HDFace2D]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD_FACE_2D_CHECKBOX));
		break;
	case IDC_KINECT_RAW_COLOR_CHECKBOX:
		m_imageRecordingConfiguration[KinectColorRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_KINECT_RAW_COLOR_CHECKBOX));
		break;
	case IDC_KINECT_RAW_DEPTH_CHECKBOX:
		m_imageRecordingConfiguration[KinectDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_KINECT_RAW_DEPTH_CHECKBOX));
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
		}
		break;
	}
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
	case IDC_LIMIT_FRAMES_EDIT_BOX:
		updateFrameLimit();
		break;
	case IDC_FILE_PATH_EDIT_BOX:
	{
		for (auto recordConfig : m_recordingConfiguration){
			recordConfig->setFilePath(editBoxText);
		}
		for (auto recordConfig : m_imageRecordingConfiguration){
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
	case IMAGE_RECORD_TYPE_COUNT:
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

SharedRecordingConfiguration RecordTabHandler::getRecordConfiguration()
{
	return m_recordingConfiguration;
}

SharedImageRecordingConfiguration RecordTabHandler::getImageRecordConfiguration()
{
	return m_imageRecordingConfiguration;
}