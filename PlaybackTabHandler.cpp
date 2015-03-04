#include "PlaybackTabHandler.h"
#include "WindowsApplication.h"

PlaybackTabHandler::PlaybackTabHandler() :
	m_playbackConfiguration(RECORD_CLOUD_TYPE_COUNT),
	m_isPlaybackRunning(false)
{
}


PlaybackTabHandler::~PlaybackTabHandler()
{
}

void PlaybackTabHandler::resetUIElements()
{
	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), L"");
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), L"");
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), L"");

	DlgDirList(m_hWnd,
		L"",
		IDC_RECODINGS_LIST_BOX,
		IDC_FILE_PATH_EDIT_BOX,
		DDL_EXCLUSIVE | DDL_READWRITE | DDL_DIRECTORY);
}
void PlaybackTabHandler::setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration)
{
	
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_playbackConfiguration[i] = std::shared_ptr<PlaybackConfiguration>(new PlaybackConfiguration(*recordingConfiguration[i]));
		m_playbackConfiguration[i]->playbackConfigurationChanged.connect(boost::bind(&PlaybackTabHandler::playbackConfigurationChanged, this));
	}
	auto outputFolderPath = recordingConfiguration[HDFace]->getFilePathCString();
	

	DlgDirList(m_hWnd,
		outputFolderPath.GetBuffer(),
		IDC_RECODINGS_LIST_BOX,
		IDC_FILE_PATH_EDIT_BOX,
		DDL_EXCLUSIVE | DDL_READWRITE | DDL_DIRECTORY);
	auto timeStampFolder = recordingConfiguration[HDFace]->getTimeStampFolderName();
	if (!timeStampFolder.IsEmpty()){
		
		SendMessage(GetDlgItem(m_hWnd, IDC_RECODINGS_LIST_BOX), LB_SELECTSTRING,
			1, (LPARAM)timeStampFolder.GetBuffer());
	}
	m_playbackConfiguration[HDFace]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD_FACE_CHECKBOX));
	m_playbackConfiguration[FaceRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FACE_RAW_DEPTH_CHECKBOX));
	m_playbackConfiguration[FullDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FULL_RAW_DEPTH_CHECKBOX));
	//DlgDirList(m_hWnd,
	//	outputFolderPath.GetBuffer(),
	//	IDC_RECODINGS_LIST_BOX,
	//	IDC_FILE_PATH_EDIT_BOX,
	//	DDL_EXCLUSIVE | DDL_READWRITE | DDL_DIRECTORY);
}



void PlaybackTabHandler::onCreate(WPARAM wParam, LPARAM)
{
	
	//if (!m_playbackConfiguration[0]){
	//	return;
	//}
	//auto filePath = m_playbackConfiguration[0]->getFilePath();
	//CString testDir(filePath.c_str());
	//
	//DlgDirList(m_hWnd,
	//	testDir.GetBuffer(0),
	//	IDC_RECODINGS_LIST_BOX,
	//	IDC_FILE_PATH_EDIT_BOX,
	//	DDL_EXCLUSIVE | DDL_READWRITE | DDL_DIRECTORY);
	//
	//Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX),			m_playbackConfiguration[	HDFace	 ]->getFirstPlaybackFile());
	//Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX),			m_playbackConfiguration[	FaceRaw	 ]->getFirstPlaybackFile());
	//Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX),	m_playbackConfiguration[FullDepthRaw ]->getFirstPlaybackFile());

}
void PlaybackTabHandler::checkPlayBackPossible()
{

}

void PlaybackTabHandler::processUIMessage(WPARAM wParam, LPARAM handle)
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


void PlaybackTabHandler::onSelectionChanged(WPARAM wParam, LPARAM handle)
{
	switch (LOWORD(wParam))
	{
	case IDC_RECODINGS_LIST_BOX:
	{
		CString timeStampFolderNameBuffer;
		DlgDirSelectEx(m_hWnd, timeStampFolderNameBuffer.GetBuffer(MAX_PATH), MAX_PATH, IDC_RECODINGS_LIST_BOX);
		CString inputFolder;
		Edit_GetText(GetDlgItem(m_hWnd, IDC_FILE_PATH_EDIT_BOX), inputFolder.GetBuffer(MAX_PATH), MAX_PATH);

		for (auto& playbackConfig : m_playbackConfiguration){
			auto fullPath = RecordingConfiguration::getFullRecordingPathForCloudType(playbackConfig->getRecordCloudType(), inputFolder, timeStampFolderNameBuffer);
			playbackConfig->setFullFilePath(fullPath);
		}

	}
		break;
	default:
		break;
	}
}

void PlaybackTabHandler::setPlaybackStatus(bool enable)
{
	m_isPlaybackRunning = enable;
	if (enable){
		startPlayback(m_playbackConfiguration);
		SetDlgItemText(m_hWnd, IDC_PLAY_STOP_BUTTON, L"Stop");
	}
	else{
		stopPlayback();
		
	}
}
void PlaybackTabHandler::playbackStopped()
{
	SetDlgItemText(m_hWnd, IDC_PLAY_STOP_BUTTON, L"Play");
	m_isPlaybackRunning = false;
}

void PlaybackTabHandler::onButtonClicked(WPARAM wParam, LPARAM handle)
{

	switch (LOWORD(wParam))
	{
	case IDC_PLAY_STOP_BUTTON:
		setPlaybackStatus(!m_isPlaybackRunning);
		break;
	case IDC_HD_FACE_CHECKBOX:
		m_playbackConfiguration[HDFace]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD_FACE_CHECKBOX));
		break;
	case IDC_FACE_RAW_DEPTH_CHECKBOX:
		m_playbackConfiguration[FaceRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FACE_RAW_DEPTH_CHECKBOX));
		break;
	case IDC_FULL_RAW_DEPTH_CHECKBOX:
		m_playbackConfiguration[FullDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FULL_RAW_DEPTH_CHECKBOX));
		break;
	case IDC_FULL_RAW_DEPTH_BUTTON_CHOOSE_INPUT:
	{
		WCHAR filePath[MAX_PATH];
		if (WindowsApplication::openFileDialog(filePath, m_hWnd)){
			Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), filePath);
		}
		break;
	}
	case IDC_HDFACE_BUTTON_CHOOSE_INPUT:
	{
		WCHAR filePath[MAX_PATH];
		if (WindowsApplication::openFileDialog(filePath, m_hWnd)){
			Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), filePath);
		}
		break;
	}
	case IDC_FACE_RAW_DEPTH_BUTTON_CHOOSE_INPUT:
	{
		WCHAR filePath[MAX_PATH];
		if (WindowsApplication::openFileDialog(filePath, m_hWnd)){
			Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), filePath);
		}
		break;
	}
	case IDC_BUTTON_CHOOSE_INPUT_DIRECTORY:
	{
		WCHAR szDir[MAX_PATH];
		if (WindowsApplication::openDirectoryDialog(szDir, m_hWnd)){
			DlgDirList(m_hWnd,
				szDir,
				IDC_RECODINGS_LIST_BOX,
				IDC_FILE_PATH_EDIT_BOX,
				DDL_EXCLUSIVE | DDL_READWRITE | DDL_DIRECTORY);
		}
		break;
	}
	default:
		break;
	}
}
void PlaybackTabHandler::onEditBoxeChanged(WPARAM wParam, LPARAM handle)
{

}

void PlaybackTabHandler::playbackConfigurationChanged()
{
	bool isValidConfiguration = true;
	if (auto playbackConfig = m_playbackConfiguration[HDFace]){
		auto firstFile = playbackConfig->getFirstPlaybackFile();
		Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), firstFile);
		//m_playbackConfiguration[HDFace]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD_FACE_CHECKBOX));
		isValidConfiguration &= playbackConfig->isPlaybackConfigurationValid();
	}
	else{
		Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), L"");
		isValidConfiguration = false;
	}
	if (auto playbackConfig = m_playbackConfiguration[FaceRaw]){
		auto firstFile = playbackConfig->getFirstPlaybackFile();
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), firstFile);
		//m_playbackConfiguration[FaceRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FACE_RAW_DEPTH_CHECKBOX));
		isValidConfiguration &= playbackConfig->isPlaybackConfigurationValid();
	}
	else{
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), L"");
		isValidConfiguration = false;
	}
	if (auto playbackConfig = m_playbackConfiguration[FullDepthRaw]){
		auto firstFile = playbackConfig->getFirstPlaybackFile();
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), firstFile);
		//m_playbackConfiguration[FullDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FULL_RAW_DEPTH_CHECKBOX));
		isValidConfiguration &= playbackConfig->isPlaybackConfigurationValid();
	}
	else{
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), L"");
		isValidConfiguration = false;
	}
	Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_STOP_BUTTON), isValidConfiguration);
}

LRESULT CALLBACK PlaybackTabHandler::MessageRouterTab(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	PlaybackTabHandler* pThis = nullptr;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<PlaybackTabHandler*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<PlaybackTabHandler*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProcTab(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}



LRESULT CALLBACK PlaybackTabHandler::DlgProcTab(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
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
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
	{
		processUIMessage(wParam, lParam);
		break;
	}
	case WM_SIZE:
		break;
	case WM_NOTIFY:
		break;

	}

	return FALSE;
}