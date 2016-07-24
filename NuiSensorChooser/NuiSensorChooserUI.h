//------------------------------------------------------------------------------
// <copyright file="NuiSensorChooserUI.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <Windows.h>

//
// The NSC control contains a subclassed icon image control to show current sensor status.
// It also uses a popup control to show more details about current sensor status when the mouse
// is hovering over it.
//

#define NSCN_REFRESH        1

class NuiSensorChooserUI
{
public:

    NuiSensorChooserUI(HWND hParent, UINT controlId, const POINT& ptLeftTop);

public:
    /// <summary>
    /// This method will update the corresponding children controls to reflect the sensor status change.
    /// </summary>
    /// <param name="dwStatus"> The current status of the sensor. </param>
    void UpdateSensorStatus(const DWORD dwStatus);

private:

    class NscIconControl* m_pControl;
};
