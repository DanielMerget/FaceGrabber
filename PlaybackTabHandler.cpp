#include "stdafx.h"
#include "PlaybackTabHandler.h"
#include "WindowsAppDialogHelper.h"

PlaybackTabHandler::PlaybackTabHandler() :
	m_playbackConfiguration(RECORD_CLOUD_TYPE_COUNT),
	m_isPlaybackRunning(false),
	m_isSingleThreadedReading(true)
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
	//init playbackconfiguration with recording configuration of recording tab
	for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
		m_playbackConfiguration[i] = std::shared_ptr<PlaybackConfiguration>(new PlaybackConfiguration(*recordingConfiguration[i]));
		m_playbackConfiguration[i]->playbackConfigurationChanged.connect(boost::bind(&PlaybackTabHandler::playbackConfigurationChanged, this));
	}
	//init file path
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
	//set enable/disable according to state of the buttons
	m_playbackConfiguration[HDFace]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_HD_FACE_CHECKBOX));
	m_playbackConfiguration[FaceRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FACE_RAW_DEPTH_CHECKBOX));
	m_playbackConfiguration[FullDepthRaw]->setEnabled(IsDlgButtonChecked(m_hWnd, IDC_FULL_RAW_DEPTH_CHECKBOX));
}


void PlaybackTabHandler::updateReaderStatus(RecordCloudType recordType, std::wstring status)
{
	if (!m_playbackConfiguration[recordType]->isEnabled()){
		return;
	}
	_In_z_ WCHAR* newStatus = &status[0];
	switch (recordType)
	{
	case HDFace:
		SetDlgItemText(m_hWnd, IDC_HDFACE_STATUS_READ, newStatus);
		break;
	case FaceRaw:
		SetDlgItemText(m_hWnd, IDC_FACE_RAW_EDIT_STATUS_READ, newStatus);
		break;
	case FullDepthRaw:
		SetDlgItemText(m_hWnd, IDC_FULL_RAW_DEPTH_STATUS_READ, newStatus);
		break;
	case RECORD_CLOUD_TYPE_COUNT:
		break;
	default:
		break;
	}
}

void PlaybackTabHandler::onCreate()
{
	
	Button_SetCheck(GetDlgItem(m_hWnd, IDC_CHECK_BOX_SINGLE_THREATED_READING), m_isSingleThreadedReading);
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

		for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
			auto fullPath = RecordingConfiguration::getFullRecordingPathForCloudType(static_cast<RecordCloudType>(i), inputFolder, timeStampFolderNameBuffer);
			m_playbackConfiguration[i]->setFullFilePath(fullPath);
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
		startPlayback(m_playbackConfiguration, m_isSingleThreadedReading);
		SetDlgItemText(m_hWnd, IDC_PLAY_STOP_BUTTON, L"Stop");
	}
	else{
		stopPlayback();
		playbackStopped();
	}
}

bool PlaybackTabHandler::isPlaybackRunning()
{
	return m_isPlaybackRunning;
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
	case IDC_CHECK_BOX_SINGLE_THREATED_READING:
		m_isSingleThreadedReading = Button_GetCheck(GetDlgItem(m_hWnd, IDC_CHECK_BOX_SINGLE_THREATED_READING));
		break;
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
		if (WindowsAppDialogHelper::openFileDialog(filePath, m_hWnd)){
			Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), filePath);
		}
		break;
	}
	case IDC_HDFACE_BUTTON_CHOOSE_INPUT:
	{
		WCHAR filePath[MAX_PATH];
		if (WindowsAppDialogHelper::openFileDialog(filePath, m_hWnd)){
			Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), filePath);
		}
		break;
	}
	case IDC_FACE_RAW_DEPTH_BUTTON_CHOOSE_INPUT:
	{
		WCHAR filePath[MAX_PATH];
		if (WindowsAppDialogHelper::openFileDialog(filePath, m_hWnd)){
			Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), filePath);
		}
		break;
	}
	case IDC_BUTTON_CHOOSE_INPUT_DIRECTORY:
	{
		WCHAR szDir[MAX_PATH];
		if (WindowsAppDialogHelper::openDirectoryDialog(szDir, m_hWnd)){
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
	//check if we have valid playback configurations
	//and if any if them is at least enabled
	//set the filename of the first playbackfile and the count of found cloud files
	//as text of the respective label
	bool isValidConfiguration = true;
	bool anyPlaybackConfigurationEnabled = false;
	if (auto playbackConfig = m_playbackConfiguration[HDFace]){
		auto firstFile = playbackConfig->getFirstPlaybackFile();
		Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), firstFile);

		isValidConfiguration &= playbackConfig->isPlaybackConfigurationValid();
		anyPlaybackConfigurationEnabled |= playbackConfig->isEnabled();

		CString numOfFilesStatus;
		numOfFilesStatus.Format(L"Files: %d", playbackConfig->getCloudFilesToPlayCount());
		SetDlgItemText(m_hWnd, IDC_HDFACE_STATUS_READ, numOfFilesStatus);
	}
	else{
		Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX), L"");
		SetDlgItemText(m_hWnd, IDC_HDFACE_STATUS_READ, L"");
		isValidConfiguration = false;
	}
	if (auto playbackConfig = m_playbackConfiguration[FaceRaw]){
		auto firstFile = playbackConfig->getFirstPlaybackFile();
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), firstFile);

		isValidConfiguration &= playbackConfig->isPlaybackConfigurationValid();
		anyPlaybackConfigurationEnabled |= playbackConfig->isEnabled();
		

		CString numOfFilesStatus;
		numOfFilesStatus.Format(L"Files: %d", playbackConfig->getCloudFilesToPlayCount());

		SetDlgItemText(m_hWnd, IDC_FACE_RAW_EDIT_STATUS_READ, numOfFilesStatus);
	}
	else{
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX), L"");
		SetDlgItemText(m_hWnd, IDC_FACE_RAW_EDIT_STATUS_READ, L"");
		isValidConfiguration = false;
	}
	if (auto playbackConfig = m_playbackConfiguration[FullDepthRaw]){
		auto firstFile = playbackConfig->getFirstPlaybackFile();
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), firstFile);

		isValidConfiguration &= playbackConfig->isPlaybackConfigurationValid();
		anyPlaybackConfigurationEnabled |= playbackConfig->isEnabled();

		CString numOfFilesStatus;
		numOfFilesStatus.Format(L"Files: %d", playbackConfig->getCloudFilesToPlayCount());

		SetDlgItemText(m_hWnd, IDC_FULL_RAW_DEPTH_STATUS_READ, numOfFilesStatus);
	}
	else{
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX), L"");
		SetDlgItemText(m_hWnd, IDC_FULL_RAW_DEPTH_STATUS_READ, L"");
		isValidConfiguration = false;
	}
	Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_STOP_BUTTON), isValidConfiguration & anyPlaybackConfigurationEnabled);
}

