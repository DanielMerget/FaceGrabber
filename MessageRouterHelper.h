#pragma once
#include "stdafx.h"
#include "resource.h"

class MessageRouterHelper
{
public:
	MessageRouterHelper();
	~MessageRouterHelper();

	/// <summary>
	/// Handles window messages, passes most to the class instance to handle
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	static LRESULT CALLBACK	MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/// <summary>
	/// Handle windows messages for a class instance
	/// </summary>
	/// <param name="hWnd">window message is for</param>
	/// <param name="uMsg">message</param>
	/// <param name="wParam">message data</param>
	/// <param name="lParam">additional message data</param>
	/// <returns>result of message processing</returns>
	LRESULT CALLBACK		DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	void processUIMessage(WPARAM wParam, LPARAM handle);

protected:

	HWND										m_hWnd;

private:
	virtual void onSelectionChanged(WPARAM wParam, LPARAM handle);
	virtual void onButtonClicked(WPARAM wParam, LPARAM handle);
	virtual void onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	virtual void onTabSelected(int iPage);
	virtual void onDestroy();
	virtual void onCreate();
};

