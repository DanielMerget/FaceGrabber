#include "stdafx.h"
#include "WindowsAppDialogHelper.h"
#include <Commdlg.h>


bool WindowsAppDialogHelper::openFileDialog(WCHAR* szDir, HWND handle)
{
	OPENFILENAME ofn;
	ZeroMemory(&ofn, sizeof(ofn));

	// Initialize OPENFILENAME
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = handle;
	ofn.lpstrFile = szDir;

	// Set lpstrFile[0] to '\0' so that GetOpenFileName does not
	// use the contents of szFile to initialize itself.
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrFilter = L"ply\0*.ply\0pcd\0*.pcd\0";
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = NULL;
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = NULL;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	if (GetOpenFileName(&ofn))
	{
		return true;
	}
	return false;
}

bool WindowsAppDialogHelper::openDirectoryDialog(WCHAR* szDir, HWND handle)
{
	BROWSEINFO bInfo;
	bInfo.hwndOwner = handle;
	bInfo.pidlRoot = NULL;
	bInfo.pszDisplayName = szDir; // Address of a buffer to receive the display name of the folder selected by the user
	bInfo.lpszTitle = L"Please, select a output folder"; // Title of the dialog
	bInfo.ulFlags = BIF_USENEWUI;
	bInfo.lpfn = NULL;
	bInfo.lParam = 0;
	bInfo.iImage = -1;

	LPITEMIDLIST lpItem = SHBrowseForFolder(&bInfo);
	if (lpItem != NULL)
	{
		if (SHGetPathFromIDList(lpItem, szDir)){
			return true;
		}
	}
	return false;
}