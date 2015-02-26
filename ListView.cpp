#include "ListView.h"
#include "resource.h"

ListView::ListView()
{
}


ListView::~ListView()
{
}

void ListView::OnDestroy(const HWND hwnd)
{
	delete m_lvh;
}

int ListView::OnCreate(const HWND hwnd, CREATESTRUCT *cs, int x, int y, int height, int width)
{
	
	StartCommonControls(ICC_LISTVIEW_CLASSES);
	m_lvh = new LVHandles;
	m_lvh->hListview = CreateListview(hwnd, cs->hInstance,
		LVS_LIST, x, y, width, height, IDC_IPADDRESS);
	//
	
	LVITEM lvi = { 0 };

	TCHAR chBuffer[16];
	for (int i = 0; i < 10; ++i)
	{
		lvi.iItem = i;    
		lvi.mask = LVIF_TEXT;
		lvi.pszText = L"lalala";
		lvi.cchTextMax = lstrlen(chBuffer);//length of item label
		ListView_InsertItem(m_lvh->hListview, &lvi);
	}

	return 0;
}

HWND ListView::CreateListview(const HWND hParent, const HINSTANCE hInst, DWORD dwStyle,
	int x, int y, int height, int width, const int id)
{
	dwStyle |= WS_CHILD;
	return CreateWindowEx(0,                  //extended styles
		WC_LISTVIEW,        //control 'class' name
		0,                  //control caption
		dwStyle,            //wnd style
		x,            //position: left
		y,             //position: top
		width,           //width
		height,          //height
		hParent,            //parent window handle
		//control's ID
		reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
		hInst,              //instance
		0);                 //user defined info
}


void ListView::OnShow()
{
	ShowWindow(m_lvh->hListview, SW_SHOW);
}

void ListView::OnHide()
{
	ShowWindow(m_lvh->hListview, SW_HIDE);
}

//=============================================================================
void ListView::StartCommonControls(DWORD flags)
{
	//load the common controls dll, specifying the type of control(s) required 
	INITCOMMONCONTROLSEX iccx;
	iccx.dwSize = sizeof(INITCOMMONCONTROLSEX);
	iccx.dwICC = flags;
	InitCommonControlsEx(&iccx);
}
//=============================================================================

