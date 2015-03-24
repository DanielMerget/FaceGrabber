#pragma once
#include "stdafx.h"

/**
 * \class	WindowsAppDialogHelper
 *
 * \brief	The windows application dialog helper for opening file/directory dialogs.
 */

class WindowsAppDialogHelper
{
public:

	/**
	 * \fn	static bool WindowsAppDialogHelper::openFileDialog(WCHAR* szDir, HWND handle);
	 *
	 * \brief	Opens a file dialog to select a file. Files are filtered by pcd and ply files.
	 *
	 * \param [in,out]	szDir	The resulting file and pre-set path.
	 * \param	handle		 	Handle of the handle.
	 *
	 * \return	true if it succeeds, false if it fails.
	 */

	static bool openFileDialog(WCHAR* szDir, HWND handle);

	/**
	 * \fn	static bool WindowsAppDialogHelper::openDirectoryDialog(WCHAR* szDir, HWND handle);
	 *
	 * \brief	Opens directory dialog.
	 *
	 * \param [in,out]	szDir	The resulting directory and pre-set directory.
	 * \param	handle		 	Handle of the handle.
	 *
	 * \return	true if it succeeds, false if it fails.
	 */
	static bool openDirectoryDialog(WCHAR* szDir, HWND handle);
};

