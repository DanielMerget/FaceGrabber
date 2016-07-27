//------------------------------------------------------------------------------
// <copyright file="NuiSensorChooser.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include <assert.h>
#include "NuiSensorChooser.h"

/// <summary>
/// Check whether the specified sensor is available.
/// </summary>
bool IsSensorConflict(_In_ INuiSensor* pNuiSensor)
{
    assert(pNuiSensor != nullptr);

    // Because we can still open a sensor even if it is occupied by other process,
    // we have to explicitly initialize it to check if we can operate it.
    HRESULT hr = pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR);

    if (SUCCEEDED(hr))
    {
        pNuiSensor->NuiShutdown();
        return false;
    }

    return true;
}

/// <summary>
/// Return the current status of the sensor
/// </summary>
ChooserStatus GetStatusFromSensor(_In_ INuiSensor* pNuiSensor, _In_ bool bCheckConflict = false)
{
    assert(pNuiSensor != nullptr);

    ChooserStatus curStatus;

    HRESULT hr = pNuiSensor->NuiStatus();
    switch (hr)
    {
    case S_OK:
        curStatus = NuiSensorChooserStatusStarted;

        if (bCheckConflict)
        {
            // Even if the input sensor is started, we still need to check if it is
            // occupied by other process.
            if (IsSensorConflict(pNuiSensor))
            {
                // other app owns this sensor
                curStatus = NuiSensorChooserStatusConflict;
            }
        }
        break;
    case S_NUI_INITIALIZING:
        curStatus = NuiSensorChooserStatusInitializing;
        break;
    case E_NUI_NOTGENUINE:
        curStatus = NuiSensorChooserStatusNotGenuine;
        break;
    case E_NUI_INSUFFICIENTBANDWIDTH:
        curStatus = NuiSensorChooserStatusInsufficientBandwidth;
        break;
    case E_NUI_NOTSUPPORTED:
        curStatus = NuiSensorChooserStatusNotSupported;
        break;
    case E_NUI_NOTPOWERED:
        curStatus = NuiSensorChooserStatusNotPowered;
        break;
    default:
        curStatus = NuiSensorChooserStatusError;
    }

    return curStatus;
}

/// <summary>
/// Constructor
/// </summary>
NuiSensorChooser::NuiSensorChooser()
    : m_pNuiSensor(nullptr)
    , m_dwLatestStatus(NuiSensorChooserStatusNone)
    , m_bSensorInitialized(false)
{
    TryFindNuiSensor();
}

/// <summary>
/// Destructor
/// </summary>
NuiSensorChooser::~NuiSensorChooser()
{
    UpdateSensorAndStatus(nullptr, NuiSensorChooserStatusNone);
}

/// <summary>
/// This method will decide which type of device change occurs,  and then
/// update the corresponding connected sensor instance and/or its status.
/// </summary>
HRESULT NuiSensorChooser::HandleNuiStatusChanged(_Out_ DWORD* pChangedFlags)
{
    if (nullptr == pChangedFlags)
    {
        return E_POINTER;
    }

    *pChangedFlags = TryFindNuiSensor();
    return S_OK;
}

/// <summary>
/// Get the current connected Nui sensor.
/// </summary>
HRESULT NuiSensorChooser::GetSensor(_In_ DWORD dwFlags, _Out_ INuiSensor** ppNuiSensor)
{
    if (nullptr == ppNuiSensor)
    {
        return E_POINTER;
    }

    *ppNuiSensor = m_pNuiSensor;

    if (nullptr == m_pNuiSensor)
    {
        return S_OK;
    }

    if (m_pNuiSensor->NuiInitializationFlags() != dwFlags)
    {
        // If the input init flag is different from current init flags in the sensor,
        // we will re-initialize the sensor
        HRESULT hr = m_pNuiSensor->NuiInitialize(dwFlags);

        if (FAILED(hr))
        {
            // Set the return point to null and convert all
            // error code to the unique E_FAIL
            *ppNuiSensor = nullptr;
            return E_FAIL;
        }

        // Mark sensor is initialized
        m_bSensorInitialized = true;
    }

    (*ppNuiSensor)->AddRef();

    return S_OK;
}

/// <summary>
/// Retrieve the status of the current connected sensor or the reason why we cannot get a sensor.
/// </summary>
HRESULT NuiSensorChooser::GetStatus(_Out_ DWORD* pStatusFlags) const
{
    if (nullptr == pStatusFlags)
    {
        return E_POINTER;
    }

    *pStatusFlags = m_dwLatestStatus;
    return S_OK;
}

/// <summary>
/// Clients should call this method to resolve the conflicts or find a new sensor
/// when current sensor status is NuiSensorChooserStatusConflict.
/// </summary>
HRESULT NuiSensorChooser::TryResolveConflict(_Out_ DWORD* pChangedFlags)
{
    if (pChangedFlags == nullptr)
    {
        return E_POINTER;
    }

    *pChangedFlags = NUISENSORCHOOSER_NONE_CHANGED_FLAG;

    if ((m_dwLatestStatus & NuiSensorChooserStatusConflict) != 0)
    {
        // Retry to find a sensor
        *pChangedFlags = TryFindNuiSensor();
    }

    return S_OK;
}

/// <summary>
/// The NuiSensorChooser will try to find and connect to the sensor specified by this method.
/// </summary>
HRESULT NuiSensorChooser::SetDesiredSensor(
    _In_opt_z_ const WCHAR* strRequiredSensorId,
    _Out_ DWORD* pChangedFlags)
{
    if (nullptr == pChangedFlags)
    {
        return E_POINTER;
    }

    SetRequiredId(strRequiredSensorId);

    // We either have no sensor or the sensor we have isn't the one they asked for
    *pChangedFlags = TryFindNuiSensor();

    return S_OK;
}

/// <summary>
/// Called when we don't have a sensor or possibly have the wrong sensor, 
/// and we want to see if we can get one
/// </summary>
DWORD NuiSensorChooser::TryFindNuiSensor()
{
    if (IsCurrentSensorValid())
    {
        return NUISENSORCHOOSER_NONE_CHANGED_FLAG;
    }

    // Check if user want to find a specific sensor
    if (HasRequiredSensorId())
    {
        return TryFindRequiredNuiSensor();
    }
    else
    {
        return TryFindAnyOneNuiSensor();
    }
}

/// <summary>
/// Check if current sensor meets the following requirements:
///     1. it is not null
///     2. it is started without error
///     3. if user specified a device id, the current sensor id must equal to it
/// </summary>
bool NuiSensorChooser::IsCurrentSensorValid() const
{
    if (nullptr == m_pNuiSensor)
    {
        return false;
    }

    // Get current sensor status. Sinece we already own this
    // sensor, we do not need to check the conflict status.
    ChooserStatus curStatus =  GetStatusFromSensor(m_pNuiSensor, false);

    if (NuiSensorChooserStatusStarted == curStatus)
    {
        // If current sensor is started, we then check if it is the required
        // sensor specified by users
        return !HasRequiredSensorId() || IsCurrentSensorRequired();
    }

    return false;
}

/// <summary>
/// Find the required sensor and verify if it is operateable
/// </summary>
DWORD NuiSensorChooser::TryFindRequiredNuiSensor()
{
    assert(!m_strRequiredSensorId.empty());

    INuiSensor* pNuiSensor = nullptr;
    HRESULT hr = NuiCreateSensorById(m_strRequiredSensorId.c_str(), &pNuiSensor);

    std::vector<INuiSensor *> vSensors;

    if (SUCCEEDED(hr))
    {
        vSensors.push_back(pNuiSensor);
    }

    return SelectSensorFromCollection(vSensors);
}

/// <summary>
/// Enumerate all sensors and try to find an operateable sensor
/// </summary>
DWORD NuiSensorChooser::TryFindAnyOneNuiSensor()
{
    assert(m_strRequiredSensorId.empty());

    std::vector<INuiSensor *> vSensors;

    int iSensorCount = 0;
    HRESULT hr = NuiGetSensorCount(&iSensorCount);

    if (SUCCEEDED(hr))
    {
        for (int i = 0; i < iSensorCount; ++i)
        {
            INuiSensor* pNuiSensor = nullptr;
            HRESULT hr = NuiCreateSensorByIndex(i, &pNuiSensor);

            if (SUCCEEDED(hr))
            {
                vSensors.push_back(pNuiSensor);
            }
        }
    }

    return SelectSensorFromCollection(vSensors);
}

/// <summary>
/// Update the sensor instance and its status, return
/// if sensor and/or status is changed
/// </summary>
DWORD NuiSensorChooser::SelectSensorFromCollection(_In_ const std::vector<INuiSensor *>& vSensors)
{
    // The status bits are used to save all the error status collected from each sensor.
    // At first, we mark the error bits as NoAvailableSensors if the sensor collection is empty,
    // or just set to None.
    DWORD dwStatusBits = vSensors.size() == 0 ? NuiSensorChooserStatusNoAvailableSensors : NuiSensorChooserStatusNone;
    INuiSensor * pValidSensor = nullptr;

    for (auto it = vSensors.begin(); it != vSensors.end(); ++it)
    {
        // Check if current sensor is started and has no conflict
        ChooserStatus dwStatus = GetStatusFromSensor(*it, true);

        if (NuiSensorChooserStatusStarted == dwStatus)
        {
            // Since sensor is started and no conflict, we treat it as a valid sensor.
            pValidSensor = *it;
            // Reset the status bits to Started state since we find a valid sensor.
            dwStatusBits = NuiSensorChooserStatusStarted;

            break;
        }

        // Combine the sensor error status to the error bits
        dwStatusBits |= dwStatus;
    }

    // Release all the interfaces except the valid one.
    for (auto it = vSensors.begin(); it != vSensors.end(); ++it)
    {
        if (*it != pValidSensor)
        {
            // Release the sensor instance if it is not operateable
            (*it)->Release();
        }
    }

    // Update the internal sensor instance and status. We have
    // two combinations:
    //     1. valid sensor interface pointer, NuiSensorChooserStatusStarted
    //     2. nullptr, Error bits combination
    return UpdateSensorAndStatus(pValidSensor, dwStatusBits);
}

/// <summary>
/// Update the sensor instance and its status, return
/// if sensor and/or status is changed
/// </summary>
DWORD NuiSensorChooser::UpdateSensorAndStatus(
    _In_ INuiSensor* pNuiSensor, 
    _In_ const DWORD dwNewStatus)
{
    DWORD changedFlags = NUISENSORCHOOSER_NONE_CHANGED_FLAG;

    if (dwNewStatus != m_dwLatestStatus)
    {
        m_dwLatestStatus = dwNewStatus;
        changedFlags = NUISENSORCHOOSER_STATUS_CHANGED_FLAG;
    }

    if (m_pNuiSensor != pNuiSensor)
    {
        // Release the old sensor instance first
        if (nullptr != m_pNuiSensor)
        {
            if (m_bSensorInitialized)
            {
                // We should shutdown the sensor if we initialized it
                m_pNuiSensor->NuiShutdown();
            }

            m_pNuiSensor->Release();
        }

        // Set current sensor to the new one and reset the init flag to not initialized
        m_pNuiSensor = pNuiSensor;
        m_bSensorInitialized = false;

        changedFlags = NUISENSORCHOOSER_SENSOR_CHANGED_FLAG;
    }

    return changedFlags;
}

/// <summary>
/// Check if current sensor meets the user requirement
/// </summary>
bool NuiSensorChooser::IsCurrentSensorRequired() const
{
    assert(m_pNuiSensor != nullptr);

    WCHAR *strSensorId = m_pNuiSensor->NuiDeviceConnectionId();

    if (nullptr == strSensorId)
    {
        // If the required d is not empty, the null device id of current sensor
        // cannot match it, so we should return false for this case.
        // If the required id is empty, it means any sensor is ok, so we will
        // return true even if the current sensor id is null.
        return m_strRequiredSensorId.empty();
    }

    // case sensitive comparison
    return m_strRequiredSensorId.compare(strSensorId) == 0;
}

/// <summary>
/// Helper to update the required sensor id
/// </summary>
void NuiSensorChooser::SetRequiredId(_In_opt_z_ const WCHAR* strRequiredSensorId)
{
    if (strRequiredSensorId != nullptr)
    {
        m_strRequiredSensorId = strRequiredSensorId;
    }
    else
    {
        // Since user do not specify a sensor id, we will clear the existing sensor id
        // to indicate that any sensor is acceptable
        m_strRequiredSensorId.clear();
    }
}
