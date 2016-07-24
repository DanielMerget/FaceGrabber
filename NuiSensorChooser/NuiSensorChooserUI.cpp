//------------------------------------------------------------------------------
// <copyright file="NuiSensorChooserUI.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "NuiSensorChooserUI.h"
#include "NuiSensorChooserUIPrivate.h"

NuiSensorChooserUI::NuiSensorChooserUI(
    HWND hParent,
    UINT controlId,
    const POINT& ptCenterTop
    )
{
    m_pControl = new NscIconControl(hParent, controlId, ptCenterTop);
}

/// <summary>
/// This method will update its children controls to reflect the sensor status change.
/// </summary>
/// <param name="dwStatus"> The current sensor status. </param>
void NuiSensorChooserUI::UpdateSensorStatus(const DWORD dwStatus)
{
    m_pControl->UpdateSensorStatus(dwStatus);
}
