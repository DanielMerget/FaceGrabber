#include "stdafx.h"
#include "MessageRouterHelper.h"


MessageRouterHelper::MessageRouterHelper()
{
}


MessageRouterHelper::~MessageRouterHelper()
{
}


LRESULT CALLBACK MessageRouterHelper::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	MessageRouterHelper* pThis = nullptr;

	if (WM_INITDIALOG == uMsg)
	{
		pThis = reinterpret_cast<MessageRouterHelper*>(lParam);
		SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
	}
	else
	{
		pThis = reinterpret_cast<MessageRouterHelper*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
	}

	if (pThis)
	{
		return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
	}

	return 0;
}



LRESULT CALLBACK MessageRouterHelper::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(wParam);
	UNREFERENCED_PARAMETER(lParam);

	switch (message)
	{
	case WM_INITDIALOG:
		// Bind application window handle
		m_hWnd = hWnd;
		onCreate();
		break;

		// If the titlebar X is clicked, destroy app
	case WM_CLOSE:
		DestroyWindow(hWnd);
		//m_pclFaceViewer->stopViewer();
		onDestroy();
		break;

	case WM_DESTROY:
		// Quit the main message pump
		//m_listView.OnDestroy(m_hWnd);
		PostQuitMessage(0);
		break;
	case WM_COMMAND:
		processUIMessage(wParam, lParam);
		break;
	case WM_NOTIFY:
		switch (((LPNMHDR)lParam)->code)
		{
		case TCN_SELCHANGE:
		{
			int iPage = TabCtrl_GetCurSel(GetDlgItem(m_hWnd, IDC_TAB2));
			onTabSelected(iPage);
		}
			break;
		}

	default:
		break;

		break;

	}
	return FALSE;
}


void MessageRouterHelper::processUIMessage(WPARAM wParam, LPARAM handle)
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


void MessageRouterHelper::onSelectionChanged(WPARAM wParam, LPARAM handle)
{

}
void MessageRouterHelper::onButtonClicked(WPARAM wParam, LPARAM handle)
{

}
void MessageRouterHelper::onEditBoxeChanged(WPARAM wParam, LPARAM handle)
{

}
void MessageRouterHelper::onTabSelected(int iPage)
{

}
void MessageRouterHelper::onDestroy()
{

}
void MessageRouterHelper::onCreate()
{

}