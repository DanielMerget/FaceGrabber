#include "RecordTabHandler.h"
#include "WindowsApplication.h"


RecordTabHandler::RecordTabHandler() : 
	m_colorEnabled(true),
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

LRESULT CALLBACK RecordTabHandler::MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	RecordTabHandler* pThis = nullptr;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<RecordTabHandler*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<RecordTabHandler*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProcTab(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}

void RecordTabHandler::onCreate(WPARAM wParam, LPARAM)
{
	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX),			m_recordingConfiguration[   HDFace	 ]->getFileName());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX),			m_recordingConfiguration[  FaceRaw	 ]->getFileName());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX),	m_recordingConfiguration[FullDepthRaw]->getFileName());
	CheckDlgButton(m_hWnd, IDC_RECORD_COLOR, m_colorEnabled);
	HWND hdFaceComboBox = GetDlgItem(m_hWnd, IDC_HD_FACE_COMBO_BOX);
	HWND facerawDepthComboBox = GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX);
	HWND fullRawDepthCombobox = GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX);

	//for (int i = RECORD_FILE_FORMAT_COUNT-1; i >= 0; --i){
	for (int i = 0; i < RECORD_FILE_FORMAT_COUNT; i++){
		LPTSTR fileFormatName = RecordingConfiguration::getFileFormatAsString(static_cast<RecordingFileFormat>(i));
		ComboBox_AddString(hdFaceComboBox, fileFormatName);
		ComboBox_AddString(facerawDepthComboBox, fileFormatName);
		ComboBox_AddString(fullRawDepthCombobox, fileFormatName);
		if (i == 0){
			ComboBox_SetCurSel(hdFaceComboBox, i);
			ComboBox_SetCurSel(facerawDepthComboBox, i);
			ComboBox_SetCurSel(fullRawDepthCombobox, i);
		}
	}
}

bool RecordTabHandler::isColorEnabled()
{
	return m_colorEnabled;
}
LRESULT CALLBACK RecordTabHandler::DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
		m_hWnd = hWnd;
		onCreate(wParam, lParam);
		break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		//m_pclFaceRawViewer->stopViewer();
		//m_pclFaceViewer->stopViewer();
		break;

	case WM_DESTROY:
		// Quit the main message pump
		//m_listView.OnDestroy(m_hWnd);
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
	{
		processUIMessage(wParam, lParam);
		break;
	}
	case WM_SIZE:
		//m_listView.OnSize(m_hWnd, LOWORD(lParam), HIWORD(lParam), static_cast<UINT>(wParam));
		//m_listView.OnSize(m_hWnd, LOWORD(lParam), HIWORD(lParam), static_cast<UINT>(wParam));
		break;
	case WM_NOTIFY:
		break;

	}

	return FALSE;
}


void RecordTabHandler::processUIMessage(WPARAM wParam, LPARAM handle)
{

	switch (HIWORD(wParam))
	{
	case CBN_SELCHANGE:
		onSelectionChanged(wParam, handle);
		break;
	case BN_CLICKED:
		onButtonClicked(wParam, handle);
		break;
	case EN_CHANGE:
		onEditBoxeChanged(wParam, handle);
	default:
		break;
	}
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
	case RECORD_CLOUD_TYPE_COUNT:
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
	case IDC_FACE_RAW_DEPTH_COMBO_BOX:
	{
		m_recordingConfiguration[FaceRaw]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;
	}
	case IDC_FULL_RAW_DEPTH_COMBO_BOX:
		m_recordingConfiguration[FullDepthRaw]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;
	case IDC_HD_FACE_COMBO_BOX:
		m_recordingConfiguration[HDFace]->setFileFormat(static_cast<RecordingFileFormat>(currentSelection));
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
	if (!oneEnabled || !allValid){
		Button_Enable(GetDlgItem(m_hWnd, IDC_RECORD_BUTTON), false);
	}
	else{
		Button_Enable(GetDlgItem(m_hWnd, IDC_RECORD_BUTTON), true);
	}

}
#define MAX_DATE 20
#include <time.h>
CString getRecordTimeStamp()
{
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
	auto timeStamp = getRecordTimeStamp();
	for (auto& recordConfig : m_recordingConfiguration){
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
		startWriting(m_colorEnabled);
		SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Stop");
	}
	else{
		stopWriting(m_colorEnabled);
	}	
}

void RecordTabHandler::setColorEnabled(bool enable)
{
	m_colorEnabled = enable;
	colorConfigurationChanged(enable);
}

void RecordTabHandler::updateFrameLimit()
{
	bool isLimited = IsDlgButtonChecked(m_hWnd, IDC_LIMIT_FRAMES_CHECK);
	Edit_Enable(GetDlgItem(m_hWnd, IDC_LIMIT_FRAMES_EDIT_BOX), isLimited);

	int limitAsInt = UNLIMITED_FRAMES;
	if (isLimited){
		auto editBoxHandle = GetDlgItem(m_hWnd, IDC_LIMIT_FRAMES_EDIT_BOX);
		CString limit;
		std::vector<wchar_t> buffer(MAX_PATH);
		Edit_GetText(editBoxHandle, buffer.data(), buffer.size());
		limitAsInt = _tstoi(buffer.data());
	}
	for (auto recordConfig : m_recordingConfiguration){
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
	case IDC_LIMIT_FRAMES_CHECK:
	{
		updateFrameLimit();
		break;
	}
	case IDC_RECORD_BUTTON:
	{
		
		setRecording(!m_isRecording);
		//if (!m_isCloudWritingStarted)
		//{
		//	m_cloudOutputWriter->startWritingClouds();
		//	SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Stop");
		//}
		//else{
		//	SetDlgItemText(m_hWnd, IDC_RECORD_BUTTON, L"Record");
		//	m_cloudOutputWriter->stopWritingClouds();
		//}
		//m_isCloudWritingStarted = !m_isCloudWritingStarted;
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
	case IDC_BUTTON_CHOOSE_OUTPUT_DIRECTORY:
	{

		WCHAR szDir[MAX_PATH];
		if (WindowsApplication::openDirectoryDialog(szDir, m_hWnd)){
			SetDlgItemText(m_hWnd, IDC_FILE_PATH_EDIT_BOX, szDir);
			for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
				m_recordingConfiguration[i]->setFilePath(szDir);
			}
		}
		//WCHAR szDir[MAX_PATH];
		//BROWSEINFO bInfo;
		//bInfo.hwndOwner = m_hWnd;
		//bInfo.pidlRoot = NULL;
		//bInfo.pszDisplayName = szDir; // Address of a buffer to receive the display name of the folder selected by the user
		//bInfo.lpszTitle = L"Please, select a output folder"; // Title of the dialog
		//bInfo.ulFlags = 0;
		//bInfo.lpfn = NULL;
		//bInfo.lParam = 0;
		//bInfo.iImage = -1;

		//LPITEMIDLIST lpItem = SHBrowseForFolder(&bInfo);
		//if (lpItem != NULL)
		//{
		//	if (SHGetPathFromIDList(lpItem, szDir)){
		//		OutputDebugString(szDir);
		//		SetDlgItemText(m_hWnd, IDC_FILE_PATH_EDIT_BOX, szDir);
		//		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		//			m_recordingConfiguration->at(i).setFilePath(szDir);
		//		}
		//	}
		//}
		break;
	}
	default:
		break;
	}
}


void RecordTabHandler::onEditBoxeChanged(WPARAM wParam, LPARAM handle)
{
	HWND editBoxHandle = GetDlgItem(m_hWnd, LOWORD(wParam));

	//std::vector<wchar_t> buffer(Edit_GetTextLength(editBoxHandle));
	std::vector<wchar_t> buffer(MAX_PATH);
	Edit_GetText(editBoxHandle, buffer.data(), buffer.size());
	switch (LOWORD(wParam)){
	case IDC_FACE_RAW_EDIT_BOX:
		m_recordingConfiguration[FaceRaw]->setFileName(buffer.data());
		break;
	case IDC_HDFACE_EDIT_BOX:
		m_recordingConfiguration[HDFace]->setFileName(buffer.data());
		break;
	case IDC_FULL_RAW_DEPTH_EDIT_BOX:
		m_recordingConfiguration[FullDepthRaw]->setFileName(buffer.data());
		break;
	case IDC_LIMIT_FRAMES_EDIT_BOX:
		updateFrameLimit();
		break;
	case IDC_FILE_PATH_EDIT_BOX:
	{
		for (auto recordConfig : m_recordingConfiguration){
			recordConfig->setFilePath(buffer.data());
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
		break;
	case FaceRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_FACE_RAW_DEPTH_COMBO_BOX), newState);
		break;
	case FullDepthRaw:
		Edit_Enable(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), newState);
		ComboBox_Enable(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_COMBO_BOX), newState);
		break;
	case RECORD_CLOUD_TYPE_COUNT:
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

SharedRecordingConfiguration RecordTabHandler::getRecordConfiguration()
{
	return m_recordingConfiguration;
}
