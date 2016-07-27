#include "stdafx.h"
#include "PlaybackTabHandler.h"
#include "WindowsAppDialogHelper.h"

PlaybackTabHandler::PlaybackTabHandler() :
	m_playbackConfiguration(RECORD_CLOUD_TYPE_COUNT),
	m_isPlaybackRunning(false),
	m_isPlaybackPaused(false),
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
		// center for playback
		m_playbackConfiguration[i]->setCenteredReading(true);
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
			auto fullPath = RecordingConfiguration::getFullRecordingPathForCloudType(static_cast<RecordCloudType>(i), inputFolder, timeStampFolderNameBuffer,KINECT_VERSION_TYPE_COUNT);
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
		Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_PAUSE_BUTTON), true);
		Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_NEXT_FRAME_BUTTON), true);
		Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_PREV_FRAME_BUTTON), true);
	}
	else{
		stopPlayback();
		playbackStopped();
	}
	setPlaybackPaused(false);
}

void PlaybackTabHandler::setPlaybackPaused(bool enable)
{
	m_isPlaybackPaused = enable;
	if (enable){
		pausePlayback(true);
		SetDlgItemText(m_hWnd, IDC_PLAY_PAUSE_BUTTON, L"Play");
	}
	else{
		pausePlayback(false);
		SetDlgItemText(m_hWnd, IDC_PLAY_PAUSE_BUTTON, L"Pause");
	}
}

bool PlaybackTabHandler::isPlaybackRunning()
{
	return m_isPlaybackRunning;
}

void PlaybackTabHandler::playbackStopped()
{
	SetDlgItemText(m_hWnd, IDC_PLAY_STOP_BUTTON, L"Play");
	Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_PAUSE_BUTTON), false);
	Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_NEXT_FRAME_BUTTON), false);
	Button_Enable(GetDlgItem(m_hWnd, IDC_PLAY_PREV_FRAME_BUTTON), false);
	updatePlaybackSliderPos(SendMessage(GetDlgItem(m_hWnd, IDC_PLAYBACK_SLIDER), TBM_GETRANGEMIN, (WPARAM)TRUE, 0));
	m_isPlaybackRunning = false;
}

void PlaybackTabHandler::updateFPSLimit()
{
	bool isLimited = IsDlgButtonChecked(m_hWnd, IDC_LIMIT_PLAYBACK_FRAMERATE_CHECK);
	Edit_Enable(GetDlgItem(m_hWnd, IDC_LIMIT_PLAYBACK_FRAMRATE_EDIT_BOX), isLimited);

	int fps = 0;

	//if the user has set an limit it is now parsed and overridden
	if (isLimited){
		auto editBoxHandle = GetDlgItem(m_hWnd, IDC_LIMIT_PLAYBACK_FRAMRATE_EDIT_BOX);
		std::vector<wchar_t> buffer(MAX_PATH);
		Edit_GetText(editBoxHandle, buffer.data(), buffer.size());
		fps = _tstoi(buffer.data());
	}

	fpsLimitUpdated(fps);
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
	case IDC_PLAY_PAUSE_BUTTON:
		setPlaybackPaused(!m_isPlaybackPaused);
		break;
	case IDC_PLAY_NEXT_FRAME_BUTTON:
		setPlaybackPaused(true);
		playbackSliderMoved(SendMessage(GetDlgItem(m_hWnd, IDC_PLAYBACK_SLIDER), TBM_GETPOS, (WPARAM)TRUE, 0) + 1);
		break;
	case IDC_PLAY_PREV_FRAME_BUTTON:
		setPlaybackPaused(true);
		playbackSliderMoved(SendMessage(GetDlgItem(m_hWnd, IDC_PLAYBACK_SLIDER), TBM_GETPOS, (WPARAM)TRUE, 0) - 1);
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
	case IDC_LIMIT_PLAYBACK_FRAMERATE_CHECK:
		updateFPSLimit();
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
			for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
				auto fullPath = RecordingConfiguration::getFullRecordingPathForCloudType(static_cast<RecordCloudType>(i), szDir, "",KINECT_VERSION_TYPE_COUNT);
				m_playbackConfiguration[i]->setFullFilePath(fullPath);
			}
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
	HWND editBoxHandle = GetDlgItem(m_hWnd, LOWORD(wParam));
	CString editBoxText;

	Edit_GetText(editBoxHandle, editBoxText.GetBuffer(MAX_PATH), MAX_PATH);
	switch (LOWORD(wParam)){
	case IDC_LIMIT_PLAYBACK_FRAMRATE_EDIT_BOX:
		updateFPSLimit();
		break;
	}
}

void PlaybackTabHandler::onSliderChanged(WPARAM wParam, LPARAM handle)
{
	// trigger playbackSlierMoved only when slider is released?
	int ctrlID = GetDlgCtrlID((HWND)handle);
	switch (ctrlID)
	{
	case IDC_PLAYBACK_SLIDER:
		if (m_isPlaybackRunning){
			playbackSliderMoved(SendMessage((HWND)handle, TBM_GETPOS, (WPARAM)TRUE, 0));
		}
		else
		{
			updatePlaybackSliderPos(SendMessage((HWND)handle, TBM_GETRANGEMIN, (WPARAM)TRUE, 0));
		}
		break;
	default:
		break;
	}
}

void PlaybackTabHandler::updatePlaybackSliderPos(int pos)
{
	HWND slider = GetDlgItem(m_hWnd, IDC_PLAYBACK_SLIDER);
	if (pos >= SendMessage(slider, TBM_GETRANGEMIN, (WPARAM)TRUE, 0) && pos <= SendMessage(slider, TBM_GETRANGEMAX, (WPARAM)TRUE, 0)){
		SendMessage(slider, TBM_SETPOS, (WPARAM)TRUE, pos);
	}
}

void PlaybackTabHandler::updatePlaybackSliderRange(int min, int max)
{
	HWND slider = GetDlgItem(m_hWnd, IDC_PLAYBACK_SLIDER);
	SendMessage(slider, TBM_SETRANGEMIN, (WPARAM)TRUE, min);
	SendMessage(slider, TBM_SETRANGEMAX, (WPARAM)TRUE, max);
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

