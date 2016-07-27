//------------------------------------------------------------------------------
// <copyright file="NuiSensorChooserUIPrivate.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include <Windows.h>
#include <ShellAPI.h>
#include <ShlObj.h>

#include "NuiSensorChooserUI.h"
#include "NuiSensorChooserUIResource.h"
#include "NuiSensorChooserUIPrivate.h"

#pragma comment(lib, "comctl32.lib")

#define IDS_NSCSUBCLASS 1000

HBITMAP NscControlSetting::RefreshNormalImage;
HBITMAP NscControlSetting::RefreshHoverImage;
HBITMAP NscControlSetting::RefreshClickImage;

HFONT  NscControlSetting::HelpLinkFont;

HBRUSH NscControlSetting::BackgroundBrush;

ATOM NscPopupControl::PopupAtom;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Following array contains predefined NSC control appearance setting
// for each different sensor status.

static const NscControlSetting ControlSettings[] = {
    NscControlSetting(
        NuiSensorChooserStatusStarted,
        IDB_ALLSET,
        IDB_ALLSETBIG,
        IDS_MESSAGEALLSET,
        NULL,
        NULL,
        false,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusInitializing,
        IDB_INITIALIZING,
        IDB_INITIALIZINGBIG,
        IDS_MESSAGEINITIALIZING,
        NULL,
        NULL,
        false,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusConflict,
        IDB_ERROR,
        IDB_ERRORBIG,
        IDS_MESSAGECONFLICT,
        IDS_URLCONFLICT,
        IDS_MOREINFORMATIONCONFLICT,
        true,
        true
    ),

    NscControlSetting(
        NuiSensorChooserStatusNotGenuine,
        IDB_ERROR,
        IDB_ERRORBIG,
        IDS_MESSAGENOTGENUINE,
        IDS_URLNOTGENUINE,
        IDS_MOREINFORMATIONNOTGENUINE,
        true,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusNotSupported,
        IDB_ERROR,
        IDB_ERRORBIG,
        IDS_MESSAGENOTSUPPORTED,
        IDS_URLNOTSUPPORTED,
        IDS_MOREINFORMATIONNOTSUPPORTED,
        true,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusError,
        IDB_ERROR,
        IDB_ERRORBIG,
        IDS_MESSAGEERROR,
        IDS_URLERROR,
        IDS_MOREINFORMATIONERROR,
        true,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusInsufficientBandwidth,
        IDB_ERROR,
        IDB_ERRORBIG,
        IDS_MESSAGEINSUFFICIENTBANDWIDTH,
        IDS_URLINSUFFICIENTBANDWIDTH,
        IDS_MOREINFORMATIONINSUFFICIENTBANDWIDTH,
        true,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusNotPowered,
        IDB_ERROR,
        IDB_ERRORBIG,
        IDS_MESSAGENOTPOWERED,
        IDS_URLNOTPOWERED,
        IDS_MOREINFORMATIONNOTPOWERED,
        true,
        false
    ),

    NscControlSetting(
        NuiSensorChooserStatusNoAvailableSensors,
        IDB_NOTAVAILABLESENSOR,
        IDB_NOTAVAILABLESENSORBIG,
        IDS_MESSAGENOAVAILABLESENSORS,
        IDS_URLNOAVAILABLESENSORS,
        IDS_MOREINFORMATIONNOAVAILABLESENSORS,
        true,
        true
    ),
};

static const NscControlSetting NscDefaultSetting(
        NuiSensorChooserStatusNone,
        IDB_NOTAVAILABLESENSOR,
        IDB_NOTAVAILABLESENSORBIG,
        IDS_MESSAGENOAVAILABLESENSORS,
        IDS_URLNOAVAILABLESENSORS,
        IDS_MOREINFORMATIONNOAVAILABLESENSORS,
        false,
        false
    );

/// <summary>
/// Helper function to set the window Z-order
/// </summary>
inline void SetWindowZOrder(HWND hWnd, HWND hWndInsertAfter)
{
    SetWindowPos(hWnd, hWndInsertAfter, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
}

/// <summary>
/// Helper function to convert points type to a point
/// </summary>
inline POINT GetPoint(POINTS points)
{
    POINT pt;

    pt.x = points.x;
    pt.y = points.y;

    return pt;
}

/// <summary>
/// Helper function to calculate the rect width
/// </summary>
inline LONG GetRectWidth(const RECT& rect)
{
    return rect.right - rect.left;
}

/// <summary>
/// Helper function to calculate the rect height
/// </summary>
inline LONG GetRectHeight(const RECT& rect)
{
    return rect.bottom - rect.top;
}

/// <summary>
/// Helper function to check if the input rect is a valid rectangle
/// from geometry perspective
/// </summary>
inline bool IsValidRect(LPRECT lpRect)
{
    if (NULL == lpRect)
    {
        return false;
    }

    return lpRect->left >= 0 && lpRect->right > lpRect->left
        && lpRect->top >= 0 && lpRect->bottom > lpRect->top;
}

/// <summary>
/// Helper function to track mouse hover and leave event
/// </summary>
void TrackMouseHoverAndLeave(HWND hWnd)
{
    TRACKMOUSEEVENT tme;

    tme.cbSize = sizeof(tme);
    tme.dwFlags =  TME_LEAVE | TME_HOVER;
    tme.dwHoverTime = 10;
    tme.hwndTrack = hWnd;

    // Tracking mouse move event to detect if mouse is hovering
    // over help link or refresh button and if mouse is leaving
    // the NSC popup control
    _TrackMouseEvent(&tme);
}

/// <summary>
/// Check if a point is located in the given window
/// Note: The point uses screen coordinates
/// </summary>
BOOL IsPtInWindow(HWND hWnd, const POINT& pt)
{
    if (IsWindowVisible(hWnd))
    {
        RECT rcWindow;
        GetWindowRect(hWnd, &rcWindow);

        return PtInRect(&rcWindow, pt);
    }

    return FALSE;
}

/// <summary>
/// Load static image resource
/// </summary>
inline void EnsureStaticImageLoaded(HBITMAP& hImage, UINT uResId, HINSTANCE hInstance)
{
    if (NULL == hImage)
    {
        hImage = LoadBitmapW(hInstance, MAKEINTRESOURCE(uResId));
    }
}

/// <summary>
/// Create font objects which lives in the whole application lifecycle
/// </summary>
inline void EnsureFontCreated(HFONT& hFont, int cSize, int cWeight, DWORD bUnderline)
{
    if (NULL == hFont)
    {
        hFont = CreateFontW(cSize, 0, 0, 0, cWeight, TRUE, bUnderline, 0,
            ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
            DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
    }
}

/// <summary>
/// Create static brushes
/// </summary>
inline void EnsureBrushCreated(HBRUSH& hBrush, COLORREF color)
{
    if (NULL == hBrush)
    {
        hBrush = CreateSolidBrush(color);
    }
}

NscControlSetting::NscControlSetting(
    ChooserStatus eStatus,
    UINT uSmallStatusImageId,
    UINT uLargeStatusImageId,
    UINT uStatusTextId,
    UINT uHelpLinkUrlId,
    UINT uHelpLinkUrlMessageId,
    bool bHelpVisible,
    bool bRefreshVisible)
    : Status(eStatus)
{
    HINSTANCE hInstance = GetModuleHandle(NULL);

    // Load the small and large status image
    SmallStatusImage = LoadBitmapW(hInstance, MAKEINTRESOURCE(uSmallStatusImageId));
    LargeStatusImage = LoadBitmapW(hInstance, MAKEINTRESOURCE(uLargeStatusImageId));

    // Load the description text for status image
    LoadStringW(hInstance, uStatusTextId, StatusText, ARRAYSIZE(StatusText));

    // Load help url and its description text
    LoadStringW(hInstance, uHelpLinkUrlId, HelpLinkUrl, ARRAYSIZE(HelpLinkUrl));
    LoadStringW(hInstance, uHelpLinkUrlMessageId, HelpLinkUrlTooltip, ARRAYSIZE(HelpLinkUrlTooltip));

    HelpControlVisible = bHelpVisible;
    RefreshControlVisible = bRefreshVisible;

    // Ensure all the static image resources loaded
    EnsureStaticImageLoaded(RefreshNormalImage, IDB_REFRESH, hInstance);
    EnsureStaticImageLoaded(RefreshHoverImage, IDB_REFRESHOVER, hInstance);
    EnsureStaticImageLoaded(RefreshClickImage, IDB_REFRESHCLICK, hInstance);

    // Ensure the helplink font and status text fond created
    EnsureFontCreated(HelpLinkFont, 14, FW_NORMAL, TRUE);

    // Ensure all the brushes created
    EnsureBrushCreated(BackgroundBrush, RGB(80, 80, 80));
}

/// <summary>
/// Get the predefined NSC UI control appearance setting based on the
/// input sensor status.
/// </summary>
const NscControlSetting* NscControlSetting::GetSetting(DWORD dwStatus)
{
    for (int i = 0; i < ARRAY_SIZE(ControlSettings); i ++)
    {
        if ((dwStatus & ControlSettings[i].Status) != 0)
        {
            return &ControlSettings[i];
        }
    }

    return &NscDefaultSetting;
}

NscPopupControl::NscPopupControl()
    : m_hwndParent(NULL)
    , m_hWnd(NULL)
    , m_pSetting(&NscDefaultSetting)
{
    // Ensure the class is registered
    if (NULL == PopupAtom)
    {
        PopupAtom = RegisterWindowClass();
    }
}

/// <summary>
/// Register the class for the NSC popup control
/// </summary>
/// <returns> The window class atom </returns>
ATOM NscPopupControl::RegisterWindowClass()
{
    WNDCLASSEX wcex = {0};

    wcex.cbSize         = sizeof(WNDCLASSEX);
    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.cbWndExtra     = DLGWINDOWEXTRA;
    wcex.lpfnWndProc    = DefDlgProcW;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = GetModuleHandle(0);
    wcex.hIcon          = NULL;
    wcex.hCursor        = LoadCursor(NULL, IDC_ARROW);
    wcex.hbrBackground  = NULL;
    wcex.lpszMenuName   = NULL;
    wcex.lpszClassName  = NSC_POPUPWINDOW_CLASSNAME;

    return RegisterClassExW(&wcex);
}

void NscPopupControl::CreateHelpLinkTooltipControl()
{
    // Create the tooltip of the help link control
    m_hwndHelpLinkTooltipControl = CreateWindowExW(
        WS_EX_TOPMOST,
        TOOLTIPS_CLASS,
        NULL,
        WS_POPUP | TTS_ALWAYSTIP | TTS_NOPREFIX,
        0, 0, 0, 0,
        m_hWnd,
        NULL,
        NULL,
        NULL
        );

    // Create a tool info object and add it to the tooltip control
    ZeroMemory(&m_tiToolInfo, sizeof(m_tiToolInfo));
    m_tiToolInfo.cbSize = sizeof(TOOLINFO);
    m_tiToolInfo.hwnd = m_hwndHelpLinkControl;
    m_tiToolInfo.uFlags = TTF_IDISHWND | TTF_SUBCLASS ;
    m_tiToolInfo.lpszText = L"";
    m_tiToolInfo.uId = (UINT_PTR)m_hwndHelpLinkControl;

    GetClientRect(m_hwndHelpLinkControl, &m_tiToolInfo.rect);

    SendMessageW(m_hwndHelpLinkTooltipControl, TTM_ADDTOOL, 0, (LPARAM)&m_tiToolInfo);
    SendMessageW(m_hwndHelpLinkTooltipControl, TTM_SETDELAYTIME, TTDT_AUTOMATIC, 10);
}

/// <summary>
/// Create a NSC popup window control
/// </summary>
HWND NscPopupControl::CreateControl(HWND hWndParent)
{
    if (NULL != m_hWnd)
    {
        // The control window has already created
        return m_hWnd;
    }

    m_hwndParent = hWndParent;

    m_hWnd = CreateDialogParamW(
        GetModuleHandle(NULL),
        MAKEINTRESOURCE(IDD_NUISENSORCHOOSERUI),
        m_hwndParent,
        (DLGPROC)MessageRouter,
        (LPARAM)(this)
        );

    if (NULL != m_hWnd)
    {
        // Get all the children control window handle, set help link font and create
        // a tooltip which will show up when mouse hovers over the help link
        m_hwndStatusImageControl = GetDlgItem(m_hWnd, IDC_LARGESTATUSIMAGE);
        m_hwndStatusTextControl = GetDlgItem(m_hWnd, IDC_STATUSTEXT);
        m_hwndRefreshControl = GetDlgItem(m_hWnd, IDC_REFRESHBUTTON);
        m_hwndHelpLinkControl = GetDlgItem(m_hWnd, IDC_HELPLINK);

        SendMessageW(m_hwndHelpLinkControl, WM_SETFONT, (WPARAM)NscControlSetting::HelpLinkFont, 0);

        CreateHelpLinkTooltipControl();
    }

    return m_hWnd;
}

/// <summary>
/// Change the visibility of the control window.
/// </summary>
/// <param name="bShow">true to show the control window, false to hide the window.</param>
void NscPopupControl::ShowControlWindow(bool bShow)
{
    SetControlWindowPos(bShow ? SWP_SHOWWINDOW : SWP_HIDEWINDOW);
}

/// <summary>
/// Change the size, position and visibility of the control window.
/// </summary>
/// <param name="uFlags">The window sizing and positioning flags</param>
void NscPopupControl::SetControlWindowPos(UINT uFlags)
{
    // Re-calculate the size and position if needed
    RECT rect = GetPlacement();

    uFlags |= SWP_NOACTIVATE; // Force it to be no activate as it is a just popup window
    SetWindowPos(m_hWnd, HWND_TOP, rect.left, rect.top, GetRectWidth(rect), GetRectHeight(rect), uFlags);
}

/// <summary>
/// Get the size and position of the popup window control
/// </summary>
RECT NscPopupControl::GetPlacement()
{
    RECT rcParent;
    GetWindowRect(m_hwndParent, &rcParent);

    RECT rcPopup;
    GetClientRect(m_hWnd, &rcPopup);

    OffsetRect(
        &rcPopup,
        (rcParent.left + rcParent.right - GetRectWidth(rcPopup)) / 2,
        rcParent.top
        );

    return rcPopup;
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
LRESULT CALLBACK NscPopupControl::MessageRouter(
    HWND hWnd,
    UINT uMsg,
    WPARAM wParam,
    LPARAM lParam
    )
{
    NscPopupControl* pThis = NULL;

    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<NscPopupControl *>(lParam);
        SetWindowLongPtrW(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<NscPopupControl *>(::GetWindowLongPtrW(hWnd, GWLP_USERDATA));
    }

    if (NULL != pThis)
    {
        return pThis->MessageProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for an instance
/// </summary>
LRESULT CALLBACK NscPopupControl::MessageProc(
    HWND hWnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam
    )
{
    switch (message)
    {
    case WM_MOUSEACTIVATE:
        // Does not activate the window, and does not discard the mouse message
        SetWindowLongPtrW(hWnd, DWLP_MSGRESULT, MA_NOACTIVATE);
        return TRUE;

    case WM_CTLCOLORSTATIC:
        {
            // Set the text foreground color and return the background color brush
            HDC hdc = (HDC)wParam;
            SetBkMode(hdc, TRANSPARENT);

            if ((HWND)lParam == m_hwndHelpLinkControl)
            {
                SetTextColor(hdc, NscControlSetting::HelpLinkTextColor);
            }
            else if ((HWND)lParam == m_hwndStatusTextControl)
            {
                SetTextColor(hdc, NscControlSetting::StatusTextColor);
            }
        }

    case WM_CTLCOLORDLG:
        {
            // return the background color brush as the result of
            // WM_CTLCOLORSTATIC and WM_CTLCOLORDLG
            return (LRESULT)NscControlSetting::BackgroundBrush;
        }

    case WM_MOUSEMOVE:
        {
            TrackMouseHoverAndLeave(hWnd);

            if (HoverOverHelpLink == m_eHoveredControl)
            {
                SetCursor(LoadCursorW(NULL, IDC_HAND));
            }
        }
        break;

    case WM_MOUSEHOVER:
        {
            // Since the mouse track event stopped when the WM_MOUSEHOVER message
            // generated, we need set up another one to make sure tooltip fully
            // shows up or dispeared
            TrackMouseHoverAndLeave(hWnd);

            // Get the child control the mouse is currently hovering over
            HoveredChildControl eHoveredControl = GetHoverLocation(MAKEPOINTS(lParam));

            if (m_eHoveredControl == eHoveredControl)
            {
                // If the mouse is still hovering over the same control,
                // we should change nothing
                break;
            }

            m_eHoveredControl = eHoveredControl;

            switch (m_eHoveredControl)
            {
            case HoverOverRefresh:

                // The mouse is hovering over the refresh control
                SetRefreshButtonImage(NscControlSetting::RefreshHoverImage);
                break;

            case HoverOverHelpLink:

                // The mouse is hovering over the help link, we should show the tooltip
                ShowHelpLinkTooltip(TRUE);
                break;

            case HoverOverNone:

                // Reset the refresh hover status and hide the help link tooltip
                SetRefreshButtonImage(NscControlSetting::RefreshNormalImage);
                ShowHelpLinkTooltip(FALSE);
                break;
            }
        }
        break;

    case WM_MOUSELEAVE:
        {
            ShowHelpLinkTooltip(FALSE);
            ShowControlWindow(false);
        }
        break;

    case WM_LBUTTONDOWN:
        {
            if (HoverOverRefresh == m_eHoveredControl)
            {
                // Update the button state when the left mouse button is pressed on the refresh button
                SetRefreshButtonImage(NscControlSetting::RefreshClickImage);
            }
        }
        break;

    case WM_LBUTTONUP:
        {
            switch (m_eHoveredControl)
            {
            case HoverOverRefresh:

                // Since the mouse button up, restore the refresh button image to hover image
                SetRefreshButtonImage(NscControlSetting::RefreshHoverImage);

                // Send message to ask parent window to take refresh actions
                SendMessageW(m_hwndParent, WM_REFRESH, 0, 0);
                break;

            case HoverOverHelpLink:

                // Open the help url when left mouse button clicked
                ShellExecuteW(NULL, NULL, m_pSetting->HelpLinkUrl, NULL, NULL, SW_SHOWNORMAL);

                // Hide the popup window when the help link clicked
                ShowControlWindow(false);
                break;
            }
        }
        break;

    case WM_UPDATECONTROL:
        {
            NscControlSetting * pSetting = reinterpret_cast<NscControlSetting *>(lParam);

            if (NULL != pSetting)
            {
                UpdateControlBySetting(pSetting);
            }
        }
        break;
    }

    return FALSE;
}

/// <summary>
/// Get the hovering control info based on current mouse location
/// </summary>
/// <returns>The enum value of the hovering control</returns>
NscPopupControl::HoveredChildControl NscPopupControl::GetHoverLocation(POINTS points)
{
    POINT ptHot = GetPoint(points);
    ClientToScreen(m_hWnd, &ptHot);

    if (IsPtInWindow(m_hwndRefreshControl, ptHot))
    {
        return HoverOverRefresh;
    }

    if (IsPtInWindow(m_hwndHelpLinkControl, ptHot))
    {
        return HoverOverHelpLink;
    }

    return HoverOverNone;
}

/// <summary>
/// Show or hide help link tooltip control
/// </summary>
/// <param name="bShow"> True: show, false: hide. </param>
void NscPopupControl::ShowHelpLinkTooltip(BOOL bShow)
{
    SendMessageW(m_hwndHelpLinkTooltipControl, TTM_TRACKACTIVATE, (WPARAM)bShow, (LPARAM)&m_tiToolInfo);
}

/// <summary>
/// Update the image of the refresh button based on mouse actions
/// </summary>
void NscPopupControl::SetRefreshButtonImage(HBITMAP hBitmap)
{
    SendMessageW(m_hwndRefreshControl, STM_SETIMAGE, IMAGE_BITMAP, reinterpret_cast<LPARAM>(hBitmap));
}

/// <summary>
/// The NSC popup control will change its appearance based on the input setting
/// </summary>
void NscPopupControl::UpdateControlBySetting(const NscControlSetting* pSetting)
{
    m_pSetting = pSetting;

    // Update the large sensor status image and its description text
    SendMessageW(m_hwndStatusImageControl, STM_SETIMAGE, IMAGE_BITMAP, reinterpret_cast<LPARAM>(m_pSetting->LargeStatusImage));
    SetWindowTextW(m_hwndStatusTextControl, m_pSetting->StatusText);

    // Update the help link tooltip text
    m_tiToolInfo.lpszText = const_cast<LPWSTR>(m_pSetting->HelpLinkUrlTooltip);
    SendMessageW(m_hwndHelpLinkTooltipControl, TTM_SETTOOLINFO, 0, (LPARAM)&m_tiToolInfo);

    // Show or hide help link and refresh button based on current setting
    ShowWindow(m_hwndHelpLinkControl, m_pSetting->HelpControlVisible ? SW_SHOW : SW_HIDE);
    ShowWindow(m_hwndRefreshControl, m_pSetting->RefreshControlVisible ? SW_SHOW : SW_HIDE);

    // Update the popup control window position to reflect all layout changes
    UpdateControlWindowPos();
}

NscIconControl::NscIconControl(
    HWND hParent,
    UINT controlID,
    const POINT& ptCenterTop
    )
    : m_hwndParent(hParent)
    , m_controlID(controlID)
    , m_ptCenterTop(ptCenterTop)
{
    CreateChildrenControls();
}

/// <summary>
/// This method will update its children controls to reflect the sensor status change.
/// </summary>
/// <param name="dwStatus"> The current sensor status. </param>
void NscIconControl::UpdateSensorStatus(const DWORD dwStatus)
{
    auto pSetting = NscControlSetting::GetSetting(dwStatus);

    // Send message to update the small sensor status image
    SendMessageW(m_hwndSmallStatusImageControl, STM_SETIMAGE, IMAGE_BITMAP, reinterpret_cast<LPARAM>(pSetting->SmallStatusImage));

    // Update the NSC popup control based on the setting
    SendMessageW(m_hwndPopup, WM_UPDATECONTROL, NULL, reinterpret_cast<LPARAM>(pSetting));
}

/// <summary>
/// Create the sensor status image control and the NSC popup window control
/// </summary>
void NscIconControl::CreateChildrenControls()
{
    // Create the control that show the status icon
    m_hwndSmallStatusImageControl = CreateWindowExW(
        0,
        L"STATIC",
        NULL,
        SS_BITMAP | WS_CHILD | WS_VISIBLE,
        m_ptCenterTop.x - NscControlSetting::SmallStatusImageSize / 2,
        m_ptCenterTop.y,
        NscControlSetting::SmallStatusImageSize,
        NscControlSetting::SmallStatusImageSize,
        m_hwndParent,
        reinterpret_cast<HMENU>(static_cast<INT_PTR>(m_controlID)),
        NULL,
        this
        );

    SetWindowSubclass(m_hwndSmallStatusImageControl, reinterpret_cast<SUBCLASSPROC>(NscIconControl::SubclassMessageRouter),
        IDS_NSCSUBCLASS, reinterpret_cast<DWORD_PTR>(this));

    // Force the status image is on the top of the parent window
    SetWindowZOrder(m_hwndSmallStatusImageControl, HWND_TOP);

    // Create popup control
    m_hwndPopup = m_popupControl.CreateControl(m_hwndSmallStatusImageControl);

    // Finally, the sensor status image control is ready to show
    ShowWindow(m_hwndSmallStatusImageControl, SW_SHOW);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
LRESULT CALLBACK NscIconControl::SubclassMessageRouter(
    HWND hWnd,
    UINT uMsg,
    WPARAM wParam,
    LPARAM lParam,
    UINT_PTR uIdSubclass,
    DWORD_PTR dwRefData
    )
{
    NscIconControl* pThis = reinterpret_cast<NscIconControl*>(dwRefData);

    if (NULL != pThis)
    {
        pThis->SubclassMessageProc(hWnd, uMsg, wParam, lParam) ;
    }

    return 0;
}

/// <summary>
/// Handle windows messages for an instance
/// </summary>
LRESULT CALLBACK NscIconControl::SubclassMessageProc(
    HWND hWnd,
    UINT uMsg,
    WPARAM wParam,
    LPARAM lParam
    )
{
    switch (uMsg)
    {
    case WM_REFRESH:
        {
            // Send WM_NOTIFY message to the parent window
            NMHDR nmh;
            nmh.code = NSCN_REFRESH;
            nmh.idFrom = GetDlgCtrlID(m_hwndSmallStatusImageControl);
            nmh.hwndFrom = m_hwndSmallStatusImageControl;
            SendMessageW(
                m_hwndParent,
                WM_NOTIFY,
                reinterpret_cast<WPARAM>(m_hwndSmallStatusImageControl),
                reinterpret_cast<LPARAM>(&nmh));
        }
        break;

    case WM_NCHITTEST:
        // Sine mouse is hover on the small status image,
        // try to show the NSC popup
        m_popupControl.ShowControlWindow(true);
        break;
    }

    return DefSubclassProc(hWnd, uMsg, wParam, lParam);
}
