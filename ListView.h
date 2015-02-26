#pragma once
#include <windows.h>  //include all the basics
#include <tchar.h>    //string and other mapping macros
#include <commctrl.h>

#include <string>
//define an unicode string type alias
typedef std::basic_string<TCHAR> ustring;
class ListView
{
public:
	ListView();
	~ListView();
	int OnCreate(const HWND hwnd, CREATESTRUCT *cs, int x, int y, int width, int height);
	void OnDestroy(const HWND hwnd);
	void OnShow();
	void OnHide();
	HWND CreateListview(const HWND hParent, const HINSTANCE hInst, DWORD dwStyle,
		int x, int y, int height, int width, const int id);
	
	void StartCommonControls(DWORD flags);

	enum {
		IDC_IPADDRESS = 200
	};

	struct LVHandles
	{
		HWND       hListview;
		HIMAGELIST hLargeIcons;
		HIMAGELIST hSmallIcons;
	};
	LVHandles *m_lvh;
};

