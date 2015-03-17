#pragma once
#include "stdafx.h"

class WindowsAppDialogHelper
{
public:

	static bool openFileDialog(WCHAR* szDir, HWND handle);
	static bool openDirectoryDialog(WCHAR* szDir, HWND handle);
};

