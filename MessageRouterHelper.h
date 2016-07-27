#pragma once
#include "stdafx.h"
#include "resource.h"

/**
 * \class	MessageRouterHelper
 *
 * \brief	A message router helper which filters for the most important events the user
 * 			triggers while using the user interface.
 */
class MessageRouterHelper
{
public:
	MessageRouterHelper();
	~MessageRouterHelper();

	/**
	 * \fn	static LRESULT CALLBACK MessageRouterHelper::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
	 *
	 * \brief	Message router.
	 *
	 * \param	hWnd  	Handle of the window.
	 * \param	uMsg  	The message.
	 * \param	wParam	The wParam field of the message.
	 * \param	lParam	The lParam field of the message.
	 *
	 * \return	A CALLBACK.
	 */

	static LRESULT CALLBACK	MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);


	/**
	 * \fn	LRESULT CALLBACK MessageRouterHelper::DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
	 *
	 * \brief	Handle windows messages for a class instance.
	 *
	 * \param	hWnd  	window handle
	 * \param	uMsg  	message.
	 * \param	wParam	message data.
	 * \param	lParam	additional message data.
	 *
	 * \return	result of message processing.
	 */
	LRESULT CALLBACK		DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/**
	 * \fn	void MessageRouterHelper::processUIMessage(WPARAM wParam, LPARAM handle);
	 *
	 * \brief	Process the user interface message.
	 *
	 * \param	wParam	The wParam field of the message.
	 * \param	handle	The lParam field of the message.
	 */
	void processUIMessage(WPARAM wParam, LPARAM handle);

protected:


	/** \brief	Handle of the window. */
	HWND m_hWnd;

private:
	/**
	* \fn	virtual void MessageRouterHelper::onSliderChanged(WPARAM wParam, LPARAM handle);
	*
	* \brief	Callback for slider changes.
	*
	* \param	wParam	The wParam field of the message.
	* \param	handle	The lParam field of the message.
	*/
	virtual void onSliderChanged(WPARAM wParam, LPARAM handle);

	/**
	 * \fn	virtual void MessageRouterHelper::onSelectionChanged(WPARAM wParam, LPARAM handle);
	 *
	 * \brief	Callback for selection changes.
	 *
	 * \param	wParam	The wParam field of the message.
	 * \param	handle	The lParam field of the message.
	 */
	virtual void onSelectionChanged(WPARAM wParam, LPARAM handle);

	/**
	 * \fn	virtual void MessageRouterHelper::onButtonClicked(WPARAM wParam, LPARAM handle);
	 *
	 * \brief	Callback for the button clicked actions.
	 *
	 * \param	wParam	The wParam field of the message.
	 * \param	handle	The lParam field of the message.
	 */
	virtual void onButtonClicked(WPARAM wParam, LPARAM handle);

	/**
	 * \fn	virtual void MessageRouterHelper::onEditBoxeChanged(WPARAM wParam, LPARAM handle);
	 *
	 * \brief	Callback for the edit box changed action.
	 *
	 * \param	wParam	The wParam field of the message.
	 * \param	handle	The lParam field of the message.
	 */
	virtual void onEditBoxeChanged(WPARAM wParam, LPARAM handle);

	/**
	 * \fn	virtual void MessageRouterHelper::onTabSelected(int iPage);
	 *
	 * \brief	Callback that the an tab was selected.
	 *
	 * \param	iPage	Zero-based index of the now selected tab.
	 */
	virtual void onTabSelected(int iPage);

	/**
	 * \fn	virtual void MessageRouterHelper::onDestroy();
	 *
	 * \brief	Callback that the Window is about to get destroyed.
	 */
	virtual void onDestroy();

	/**
	 * \fn	virtual void MessageRouterHelper::onCreate();
	 *
	 * \brief	Callback that the Windows is about to be created.
	 */
	virtual void onCreate();

	virtual void UpdateStreams(int i);
};

