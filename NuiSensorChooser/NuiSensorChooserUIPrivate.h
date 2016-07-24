//------------------------------------------------------------------------------
// <copyright file="NuiSensorChooserUIPrivate.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <Windows.h>
#include <ShlObj.h>

#include "NuiSensorChooser.h"

//
// The NSC control contains a subclassed icon image control to show current sensor status.
// It also uses a popup control to show more details about current sensor status when mouse
// is hovering over it.
//

#define WM_REFRESH          WM_USER + 100
#define WM_UPDATECONTROL    WM_USER + 101

#define NSC_POPUPWINDOW_CLASSNAME L"NuiSensorChooserPopupWindow"

static const int MaxLoadStringChars = 200;

/// <summary>
/// The appearance setting for  NSC UI control and its corresponding
/// popup window control.
///
/// The NSC UI control only contains a small sensor status image.
///
/// The NSC popup control contains following sub controls:
///     1. Large sensor status image control
///     2. Sensor status description text control
///     3. The help url link and its tooltip controls
///     4. A refresh button control
/// </summary>
class NscControlSetting
{
public:

    /// <summary>
    /// Construct by resources ID
    /// </summary>
    NscControlSetting(
        ChooserStatus eStatus,
        UINT uSmallStatusImageId,
        UINT uLargeStatusImageId,
        UINT uStatusTextId,
        UINT uHelpLinkUrlId,
        UINT uHelpLinkUrlTooltipTextId,
        bool bHelpVisible,
        bool bRefreshVisible
        );

    static const NscControlSetting* GetSetting(DWORD dwStatus);

public:

    // The Nui sensor chooser status that current control setting applys to
    ChooserStatus Status;

    // The small sensor status image for the summary icon control
    HBITMAP SmallStatusImage;

    // The large sensor status image and its text of the NSC popup control
    HBITMAP LargeStatusImage;
    WCHAR   StatusText[MaxLoadStringChars];

    // Help url link and its description text of the NSC popup control
    WCHAR   HelpLinkUrl[MaxLoadStringChars];
    WCHAR   HelpLinkUrlTooltip[MaxLoadStringChars];

    // Indicate if we should hide the help control or refresh control
    // of the NSC popup window control
    bool    HelpControlVisible;
    bool    RefreshControlVisible;

public:

    // All the static resources which will be loaded only once and have whole application
    // life cycle.

    // Static image resources to represent different states of the refresh button
    static HBITMAP RefreshNormalImage;
    static HBITMAP RefreshHoverImage;    // mouse hover over the refresh button
    static HBITMAP RefreshClickImage;    // mouse click on the refresh button

    static HFONT StatusTextFont;
    static HFONT HelpLinkFont;

public:
    // Predefined text colors
    static const COLORREF HelpLinkTextColor = RGB(252, 237, 30);
    static const COLORREF StatusTextColor = RGB(255, 255, 255);

    // The background brush for the popup control
    static HBRUSH BackgroundBrush;

public:

    // The small status image width and height
    static const int SmallStatusImageSize = 40;
};

class NscPopupControl
{
public:

    NscPopupControl();

    /// <summary>
    /// Create a NSC popup control window.
    /// </summary>
    /// <returns>
    /// If the function succeeds, the return value is a handle to the new window.
    /// If the function fails, the return value is NULL. To get extended error information, call GetLastError.
    /// </returns> 
    HWND CreateControl(HWND hWndParent);

    /// <summary>
    /// Change the visibility of the control window.
    /// </summary>
    /// <param name="bShow">true to show the control window, false to hide the window.</param>
    void ShowControlWindow(bool bShow);

private:

    /// <summary>
    /// Register the class for the NSC popup control
    /// </summary>
    /// <returns>The window class atom</returns>
    ATOM RegisterWindowClass();

    /// <summary>
    /// Create the help link tooltip control
    /// </summary>
    void CreateHelpLinkTooltipControl();

    /// <summary>
    /// The NSC popup control will change its appearance based on the
    /// input setting.
    /// </summary>
    void UpdateControlBySetting(const NscControlSetting* pSetting);

    /// <summary>
    /// Update the control window size based on its setting
    /// </summary>
    void UpdateControlWindowPos() { SetControlWindowPos(0); }

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for an instance
    /// </summary>
    LRESULT CALLBACK MessageProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Indentify which child control the mouse is hovering over
    /// </summary>
    enum HoveredChildControl
    {
        /// <summary>
        /// Hovering over help link control
        /// </summary>
        HoverOverHelpLink = 0,

        /// <summary>
        /// Hovering over refresh control
        /// </summary>
        HoverOverRefresh,

        /// <summary>
        /// Not hovering over any interest child control
        /// </summary>
        HoverOverNone
    };

    /// <summary>
    /// Get the hovering control info based on current mouse location
    /// </summary>
    /// <returns>The enum value of the hovering control</returns>
    HoveredChildControl GetHoverLocation(POINTS points);

    /// <summary>
    /// Show or hide help link tooltip control
    /// </summary>
    /// <param name="bShow"> True: show, false: hide. </param>
    inline void ShowHelpLinkTooltip(BOOL bShow);

    /// <summary>
    /// Set the corresponding image of the refresh button for different mouse actions
    /// </summary>
    inline void SetRefreshButtonImage(HBITMAP hBitmap);

    /// <summary>
    /// Get the size and position of the sensor status image control
    /// </summary>
    RECT GetPlacement();

    /// <summary>
    /// Change the size, position and visibility of the control window.
    /// </summary>
    /// <param name="uFlags">The window sizing and positioning flags.</param>
    void SetControlWindowPos(UINT uFlags);

private:

    HWND            m_hwndParent;
    HWND            m_hWnd;

    // The cildren controls in the NSC popup control
    HWND            m_hwndStatusImageControl;
    HWND            m_hwndStatusTextControl;
    HWND            m_hwndRefreshControl;
    HWND            m_hwndHelpLinkControl;
    HWND            m_hwndHelpLinkTooltipControl;

    // Tool info object for the help tooltip control
    TOOLINFOW       m_tiToolInfo;

    const NscControlSetting* m_pSetting;

    // Current mouser hovering location
    HoveredChildControl m_eHoveredControl;

    // The handle to the registered popup window class
    static ATOM PopupAtom;
};

class NscIconControl
{
public:

    NscIconControl(HWND hParent, UINT controlID, const POINT& ptLeftTop);

public:
    /// <summary>
    /// This method will update the corresponding children controls to reflect the sensor status change.
    /// </summary>
    /// <param name="dwStatus"> The current status of the sensor. </param>
    void UpdateSensorStatus(const DWORD dwStatus);

private:

    /// <summary>
    /// Create the sensor status image control and the NSC popup window control
    /// </summary>
    void CreateChildrenControls();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    static LRESULT CALLBACK SubclassMessageRouter(
        HWND hWnd,
        UINT uMsg,
        WPARAM wParam,
        LPARAM lParam,
        UINT_PTR uIdSubclass,
        DWORD_PTR dwRefData
        );

    /// <summary>
    /// Handle windows messages for an instance
    /// </summary>
    LRESULT CALLBACK SubclassMessageProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

private:

    HWND            m_hwndParent;
    UINT            m_controlID;

    // The center top position of the control
    POINT           m_ptCenterTop;

    // The small sensor status image control created by this class
    HWND            m_hwndSmallStatusImageControl;

    // The popup window control instance and its window handle
    HWND            m_hwndPopup;
    NscPopupControl m_popupControl;
};
