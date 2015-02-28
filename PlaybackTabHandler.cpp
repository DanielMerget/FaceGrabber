#include "PlaybackTabHandler.h"

PlaybackTabHandler::PlaybackTabHandler()
{
}


PlaybackTabHandler::~PlaybackTabHandler()
{
}

void PlaybackTabHandler::setSharedRecordingConfiguration(SharedRecordingConfiguration recordingConfiguration)
{
	m_recordingConfiguration = recordingConfiguration;
}

void PlaybackTabHandler::onCreate(WPARAM wParam, LPARAM)
{
	auto filePath = m_recordingConfiguration->at(0).getFilePath();
	CString testDir(filePath.c_str());

	DlgDirList(m_hWnd,
		testDir.GetBuffer(0),
		IDC_RECODINGS_LIST_BOX,
		IDC_FILE_PATH_EDIT_BOX,
		DDL_EXCLUSIVE | DDL_READWRITE | DDL_DIRECTORY);

	Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX),			m_recordingConfiguration->at(	HDFace	 ).getFileName());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX),			m_recordingConfiguration->at(	FaceRaw	 ).getFileName());
	Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX),	m_recordingConfiguration->at(FullDepthRaw).getFileName());

}
void PlaybackTabHandler::checkPlayBackPossible()
{

}
#include "WindowsApplication.h"
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
		TCHAR tchBuffer[MAX_PATH];
		DlgDirSelectEx(m_hWnd, tchBuffer, MAX_PATH, IDC_RECODINGS_LIST_BOX);
		Edit_SetText(GetDlgItem(m_hWnd, IDC_HDFACE_EDIT_BOX			), tchBuffer);
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FACE_RAW_EDIT_BOX		), tchBuffer);
		Edit_SetText(GetDlgItem(m_hWnd, IDC_FULL_RAW_DEPTH_EDIT_BOX	), tchBuffer);
	}
		break;
	default:
		break;
	}
}
void PlaybackTabHandler::onButtonClicked(WPARAM wParam, LPARAM handle)
{

	switch (LOWORD(wParam))
	{
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
			for (int i = 0; i < RECORD_CLOUD_TYPE_COUNT; i++){
				m_recordingConfiguration->at(i).setFilePath(szDir);
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