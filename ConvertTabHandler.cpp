#include "stdafx.h"
#include "ConvertTabHandler.h"
#include "WindowsAppDialogHelper.h"

ConvertTabHandler::ConvertTabHandler():
	m_playbackConfiguration(new PlaybackConfiguration),
	m_enableColor(true),
	m_recordingConfiguration(new SimpleRecordingConfiguration)
{
	m_playbackConfiguration->setEnabled(true);
}


ConvertTabHandler::~ConvertTabHandler()
{
	m_colorBufferSynchronizer->onApplicationQuit();
	m_nonColorBufferSynchronizer->onApplicationQuit();
	if (m_colorBufferSynchronizerThread.joinable()){
		m_colorBufferSynchronizerThread.join();
	}
	if (m_nonColorBufferSynchronizerThread.joinable()){
		m_nonColorBufferSynchronizerThread.join();
	}
}
void ConvertTabHandler::onCreate()
{

	HWND outputFormatHandle = GetDlgItem(m_hWnd, IDC_COMBO_OUTPUT_FORMAT);

	for (int i = 0; i < RECORD_FILE_FORMAT_COUNT; i++){
		auto fileFormatName = RecordingConfiguration::getFileFormatAsString(static_cast<RecordingFileFormat>(i));
		ComboBox_AddString(outputFormatHandle, fileFormatName);
	}
	m_recordingConfiguration->setRecordFileFormat(PCD);
	ComboBox_SetCurSel(outputFormatHandle, m_recordingConfiguration->getRecordFileFormat());

	m_recordingConfiguration->setFileNameString("Converted_Cloud_Name");
	CString fileName(m_recordingConfiguration->getFileNameString().c_str());
	
	Edit_SetText(GetDlgItem(m_hWnd, IDC_EDIT_BOX_FILE_NAME), fileName.GetBuffer());
	
	Button_SetCheck(GetDlgItem(m_hWnd, IDC_CHECKBOX_COLOR), m_enableColor);

	HWND numOfThreadsToStartHWND = GetDlgItem(m_hWnd, IDC_COMBO_BOX_CONVERT_THREADS);
	
	for (int i = 1; i <= 5; i++){
		CString counter;
		counter.Format(L"%d", i);
		ComboBox_AddString(numOfThreadsToStartHWND, counter);
	}
	m_recordingConfiguration->setThreadCountToStart(5);
	ComboBox_SetCurSel(GetDlgItem(m_hWnd, IDC_COMBO_BOX_CONVERT_THREADS), m_recordingConfiguration->getThreadCountToStart()-1);

	m_playbackConfiguration->playbackConfigurationChanged.connect(boost::bind(&ConvertTabHandler::playbackConfigurationChanged, this));
	//m_recordingConfiguration->recordConfigurationStatusChanged.connect(boost::bind(&ConvertTabHandler::recordingConfigurationChanged, this));

	m_colorBufferSynchronizer = std::shared_ptr<ColorBufferSynchronizer>(new ColorBufferSynchronizer(false));
	m_colorBufferSynchronizerThread = std::thread(&ColorBufferSynchronizer::updateThreadFunc, m_colorBufferSynchronizer);

	m_nonColorBufferSynchronizer = std::shared_ptr<NonColorBufferSynchronizer>(new NonColorBufferSynchronizer(false));
	m_nonColorBufferSynchronizerThread = std::thread(&NonColorBufferSynchronizer::updateThreadFunc, m_nonColorBufferSynchronizer);
}


void ConvertTabHandler::playbackConfigurationChanged()
{
	CString foundFiles;
	foundFiles.Format(L"%d", m_playbackConfiguration->getCloudFilesToPlayCount());
	m_recordingConfiguration->setMaxNumberOfFrames(m_playbackConfiguration->getCloudFilesToPlayCount());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_TEXT_LABEL_FOUND_FILES), foundFiles);
	if (m_playbackConfiguration->isPlaybackConfigurationValid() && m_recordingConfiguration->isRecordConfigurationValid()){
		Button_Enable(GetDlgItem(m_hWnd, IDC_BUTTON_CONVERT), true);
	}
	else{
		Button_Enable(GetDlgItem(m_hWnd, IDC_BUTTON_CONVERT), false);
	}
}

void ConvertTabHandler::recordingConfigurationChanged()
{
	
	if (m_playbackConfiguration->isPlaybackConfigurationValid() && m_recordingConfiguration->isRecordConfigurationValid()){
		Button_Enable(GetDlgItem(m_hWnd, IDC_BUTTON_CONVERT), true);
	}
	else{
		Button_Enable(GetDlgItem(m_hWnd, IDC_BUTTON_CONVERT), false);
	}
}


void ConvertTabHandler::onSelectionChanged(WPARAM wParam, LPARAM handle)
{
	int currentSelection = ComboBox_GetCurSel(GetDlgItem(m_hWnd, LOWORD(wParam)));
	switch (LOWORD(wParam))
	{
	case IDC_COMBO_OUTPUT_FORMAT:
		m_recordingConfiguration->setRecordFileFormat(static_cast<RecordingFileFormat>(currentSelection));
		break;
	case IDC_COMBO_BOX_CONVERT_THREADS:
		m_recordingConfiguration->setThreadCountToStart(currentSelection + 1);
		break;
	default:
		break;
	}
}

void ConvertTabHandler::updateReaderStatus(std::wstring status)
{
	_In_z_ WCHAR* newStatus = &status[0];
	SetDlgItemText(m_hWnd, IDC_TEXT_LABEL_STATUS_READER, newStatus);
}
void ConvertTabHandler::updateWriterStatus(std::wstring status)
{
	_In_z_ WCHAR* newStatus = &status[0];
	SetDlgItemText(m_hWnd, IDC_TEXT_LABEL_STATUS_WRITER, newStatus);
}

void ConvertTabHandler::notifyWriterFinished()
{
	Button_Enable(GetDlgItem(m_hWnd, IDC_BUTTON_CONVERT), true);
	SetDlgItemText(m_hWnd, IDC_TEXT_LABEL_STATUS_READER, L"");
}

void ConvertTabHandler::initColoredConversionPipeline()
{
	m_colorCloudReader = std::shared_ptr<ColoredCloudInputReader>(new ColoredCloudInputReader);
	m_colorBuffer = std::shared_ptr<ColorBuffer>(new ColorBuffer);

	m_colorWriter = std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZRGB>>(new KinectCloudFileWriter<pcl::PointXYZRGB>);

	std::vector< std::shared_ptr<ColorBuffer>> buffers;
	buffers.push_back(m_colorBuffer);

	m_colorBufferSynchronizer->setBuffer(buffers, m_playbackConfiguration->getCloudFilesToPlayCount());


	m_colorBufferSynchronizer->publishSynchronizedData.connect(
		boost::bind(&KinectCloudFileWriter<pcl::PointXYZRGB>::pushCloudsAsync, m_colorWriter, _1));
	m_colorWriter->updateStatus.connect(boost::bind(&ConvertTabHandler::updateWriterStatus, this, _1));

	m_colorWriter->writingWasStopped.connect(boost::bind(&ConvertTabHandler::notifyWriterFinished, this));

	m_colorCloudReader->updateStatus.connect(boost::bind(&ConvertTabHandler::updateReaderStatus, this, _1));

	m_colorCloudReader->setBuffer(m_colorBuffer);
}

void ConvertTabHandler::initNonColoredConversionPipeline()
{
	m_nonColorCloudReader = std::shared_ptr<NonColoredCloudInputReader>(new NonColoredCloudInputReader);
	m_nonColorBuffer = std::shared_ptr<NonColorBuffer>(new NonColorBuffer);

	m_nonColorWriter = std::shared_ptr<KinectCloudFileWriter<pcl::PointXYZ>>(new KinectCloudFileWriter<pcl::PointXYZ>);
	m_nonColorWriter->updateStatus.connect(boost::bind(&ConvertTabHandler::updateWriterStatus, this, _1));
	m_nonColorWriter->writingWasStopped.connect(boost::bind(&ConvertTabHandler::notifyWriterFinished, this));

	std::vector< std::shared_ptr<NonColorBuffer>> buffers;
	buffers.push_back(m_nonColorBuffer);

	m_nonColorBufferSynchronizer->setBuffer(buffers, m_playbackConfiguration->getCloudFilesToPlayCount());

	m_nonColorBufferSynchronizer->publishSynchronizedData.connect(
		boost::bind(&KinectCloudFileWriter<pcl::PointXYZ>::pushCloudsAsync, m_nonColorWriter, _1));

	

	m_nonColorCloudReader->updateStatus.connect(boost::bind(&ConvertTabHandler::updateReaderStatus, this, _1));

	m_nonColorCloudReader->setBuffer(m_nonColorBuffer);
}

void ConvertTabHandler::startFileConversion()
{
	if (m_enableColor){
		if (!m_colorCloudReader){
			initColoredConversionPipeline();
		}
		
		m_colorWriter->setRecordingConfiguration(m_recordingConfiguration);
		m_colorCloudReader->setPlaybackConfiguration(m_playbackConfiguration);
		
		std::async(std::launch::async, &ColoredCloudInputReader::startReading, m_colorCloudReader, true);

		m_colorWriter->startWritingClouds();
	}
	else{
		if (!m_nonColorCloudReader){
			initNonColoredConversionPipeline();
		}
		m_nonColorCloudReader->setPlaybackConfiguration(m_playbackConfiguration);
		m_nonColorWriter->setRecordingConfiguration(m_recordingConfiguration);

		std::async(std::launch::async, &NonColoredCloudInputReader::startReading, m_nonColorCloudReader, true);
		m_nonColorWriter->startWritingClouds();
	}
	Button_Enable(GetDlgItem(m_hWnd, IDC_BUTTON_CONVERT), false);
}

void ConvertTabHandler::onButtonClicked(WPARAM wParam, LPARAM handle)
{
	switch (LOWORD(wParam))
	{	
	case IDC_BUTTON_OUTPUT_FOLDER_CONVERT:
	{
		WCHAR szDir[MAX_PATH];
		if (WindowsAppDialogHelper::openDirectoryDialog(szDir, m_hWnd)){
			SetDlgItemText(m_hWnd, IDC_EDIT_BOX_OUTPUT_FOLDER_CONVERT, szDir);
		}
		break;
	}
	case IDC_BUTTON_INPUT_FOLDER_CONVERT:
		WCHAR szDir[MAX_PATH];
		if (WindowsAppDialogHelper::openDirectoryDialog(szDir, m_hWnd)){
			SetDlgItemText(m_hWnd, IDC_EDIT_BOX_INPUT_FOLDER_CONVERT, szDir);
		}
		break;
	case IDC_CHECKBOX_COLOR:
		m_enableColor = Button_GetCheck(GetDlgItem(m_hWnd, IDC_CHECKBOX_COLOR));
		break;
	case IDC_BUTTON_CONVERT:
		startFileConversion();
		break;
	default:
		break;
	}
}



void ConvertTabHandler::onEditBoxeChanged(WPARAM wParam, LPARAM handle)
{
	HWND editBoxHandle = GetDlgItem(m_hWnd, LOWORD(wParam));

	switch (LOWORD(wParam)){
	case IDC_EDIT_BOX_INPUT_FOLDER_CONVERT:
	{
		CString inputFolder;
		Edit_GetText(GetDlgItem(m_hWnd, IDC_EDIT_BOX_INPUT_FOLDER_CONVERT), inputFolder.GetBuffer(MAX_PATH), MAX_PATH);
		m_playbackConfiguration->setFullFilePath(inputFolder);
		
		break;
	}
	case IDC_EDIT_BOX_OUTPUT_FOLDER_CONVERT:
	{
		CString outputFolder;
		Edit_GetText(GetDlgItem(m_hWnd, IDC_EDIT_BOX_OUTPUT_FOLDER_CONVERT), outputFolder.GetBuffer(MAX_PATH), MAX_PATH);
		CT2CA pszConvertedAnsiString(outputFolder);
		m_recordingConfiguration->setFullRecordingPathString(std::string(pszConvertedAnsiString));
		recordingConfigurationChanged();
		break;
	}
	case IDC_EDIT_BOX_FILE_NAME:
	{
		CString fileName;
		Edit_GetText(GetDlgItem(m_hWnd, IDC_EDIT_BOX_FILE_NAME), fileName.GetBuffer(MAX_PATH), MAX_PATH);
		CT2CA pszConvertedAnsiString(fileName);
		m_recordingConfiguration->setFileNameString(std::string(pszConvertedAnsiString));
		recordingConfigurationChanged();
		break;
	}
	default:
		break;
	}
}
