//------------------------------------------------------------------------------
// <copyright file="NuiSensorChooser.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include <Windows.h>

#include <string>
#include <vector>

#include <NuiApi.h>



/// <summary>
/// Sensor change type
/// </summary>
#define NUISENSORCHOOSER_NONE_CHANGED_FLAG      0x00000000
#define NUISENSORCHOOSER_SENSOR_CHANGED_FLAG    0x00000001
#define NUISENSORCHOOSER_STATUS_CHANGED_FLAG    0x00000002

/// <summary>
/// Sensor chooser status
/// </summary>
enum ChooserStatus
{
    /// <summary>
    /// Chooser has not been started or it has been stopped
    /// </summary>
    NuiSensorChooserStatusNone = 0x00000000,

    /// <summary>
    /// Don't have a sensor yet, a sensor is initializing, you may not get it
    /// </summary>
    NuiSensorChooserStatusInitializing = 0x00000001,

    /// <summary>
    /// This NuiSensorChooser has a connected and started sensor.
    /// </summary>
    NuiSensorChooserStatusStarted = 0x00000002,

    /// <summary>
    /// There are no sensors available on the system.  If one shows up
    /// we will try to use it automatically.
    /// </summary>
    NuiSensorChooserStatusNoAvailableSensors = 0x00000004,

    /// <summary>
    /// Available sensor is in use by another application
    /// </summary>
    NuiSensorChooserStatusConflict = 0x00000008,

    /// <summary>
    /// The available sensor is not powered.  If it receives power we
    /// will try to use it automatically.
    /// </summary>
    NuiSensorChooserStatusNotPowered = 0x00000010,

    /// <summary>
    /// There is not enough bandwidth on the USB controller available
    /// for this sensor.
    /// </summary>
    NuiSensorChooserStatusInsufficientBandwidth = 0x00000020,

    /// <summary>
    /// Available sensor is not genuine.
    /// </summary>
    NuiSensorChooserStatusNotGenuine = 0x00000040,

    /// <summary>
    /// Available sensor is not supported
    /// </summary>
    NuiSensorChooserStatusNotSupported = 0x00000080,

    /// <summary>
    /// Available sensor has an error
    /// </summary>
    NuiSensorChooserStatusError = 0x00000100,
};

class NuiSensorChooser
{
public:

    /// <summary>
    /// Constructor
    /// </summary>
    NuiSensorChooser();

    /// <summary>
    /// Destructor
    /// </summary>
    ~NuiSensorChooser();

public:

    /// <summary>
    /// This method will decide which type of device change occurs, and then 
    /// update the corresponding connected sensor instance and/or its status.
    /// </summary>
    /// <param name="pChangedFlags">
    /// The address to the output parameter that receives the device change types,
    /// Possible value includes NUISENSORCHOOSER_NONE_CHANGED_FLAG,
    /// NUISENSORCHOOSER_SENSOR_CHANGED_FLAG, and NUISENSORCHOOSER_STATUS_CHANGED_FLAG.
    /// </param>
    /// <returns>
    /// Returns S_OK if successful; otherwise, returns one of the following error codes:
    /// <list type="table">
    ///    <listheader>
    ///       <term>Error code</term>
    ///       <description>Description</description>
    ///    </listheader>
    ///    <item>
    ///       <term>E_POINTER</term>
    ///       <description>The <paramref name="pChangedFlags"/> parameter is NULL.</description>
    ///    </item>
    /// </list>
    /// </returns> 
    _Check_return_ HRESULT HandleNuiStatusChanged(_Out_ DWORD* pChangedFlags);

    /// <summary>
    /// Get the current connected Nui sensor.
    /// </summary>
    /// <param name="dwFlags">
    /// The flags to initialize the returned sensor.
    /// </param>
    /// <param name="ppNuiSensor">
    /// The address of the pointer that receives the INuiSensor instance of the current connected sensor.
    /// </param>
    /// <returns>
    /// Returns S_OK if successful; otherwise, returns one of the following error codes:
    /// <list type="table">
    ///    <listheader>
    ///       <term>Error code</term>
    ///       <description>Description</description>
    ///    </listheader>
    ///    <item>
    ///       <term>E_POINTER</term>
    ///       <description>The <paramref name="ppNuiSensor"/> parameter is NULL.</description>
    ///    </item>
    ///    <item>
    ///       <term>E_FAILED</term>
    ///       <description>Fail to initialize the Nui sensor.</description>
    ///    </item>
    /// </list>
    /// </returns> 
    _Check_return_ HRESULT GetSensor(_In_ DWORD dwFlags, _Out_ INuiSensor** ppNuiSensor);

    /// <summary>
    /// Clients should call this method to resolve the conflicts or find a new sensor when current
    /// sensor status is SensorConflict.
    /// </summary>
    /// <param name="pChangedFlags">
    /// The address to the output parameter that receives the device change types,
    /// Possible value includes NUISENSORCHOOSER_NONE_CHANGED_FLAG,
    /// NUISENSORCHOOSER_SENSOR_CHANGED_FLAG, and NUISENSORCHOOSER_STATUS_CHANGED_FLAG.
    /// </param>
    /// <returns>
    /// Returns S_OK if successful; otherwise, returns one of the following error codes:
    /// <list type="table">
    ///    <listheader>
    ///       <term>Error code</term>
    ///       <description>Description</description>
    ///    </listheader>
    ///    <item>
    ///       <term>E_POINTER</term>
    ///       <description>The <paramref name="pChangedFlags"/> parameter is NULL.</description>
    ///    </item>
    /// </list>
    /// </returns> 
    _Check_return_ HRESULT TryResolveConflict(_Out_ DWORD* pChangedFlags);

    /// <summary>
    /// Retrieve the status of the current connected sensor or the reason why we cannot get a sensor.
    /// </summary>
    /// <param name="pStatusFlags">
    /// The address of the output parameter that receives the current sensor status.
    /// </param>
    /// <returns>
    /// Returns S_OK if successful; otherwise, returns one of the following error codes:
    /// <list type="table">
    ///    <listheader>
    ///       <term>Error code</term>
    ///       <description>Description</description>
    ///    </listheader>
    ///    <item>
    ///       <term>E_POINTER</term>
    ///       <description>The <paramref name="pStatusFlags"/> parameter is NULL.</description>
    ///    </item>
    /// </list>
    /// </returns> 
    _Check_return_ HRESULT GetStatus(_Out_ DWORD* pStatusFlags) const;

    /// <summary>
    /// The NuiSensorChooser will try to find and connect to the sensor specified by this method.
    /// </summary>
    /// <param name="strRequiredSensorId">
    /// The id string of the sensor which users want to connect. NULL means anyone is Ok.
    /// </param>
    /// <param name="pChangedFlags">
    /// The address to the output parameter that receives the device change types,
    /// possible value includes NUISENSORCHOOSER_NONE_CHANGED_FLAG,
    /// NUISENSORCHOOSER_SENSOR_CHANGED_FLAG, and NUISENSORCHOOSER_STATUS_CHANGED_FLAG.
    /// </param>
    /// <returns>
    /// Returns S_OK if successful; otherwise, returns one of the following error codes:
    /// <list type="table">
    ///    <listheader>
    ///       <term>Error code</term>
    ///       <description>Description</description>
    ///    </listheader>
    ///    <item>
    ///       <term>E_POINTER</term>
    ///       <description>The <paramref name="pChangedFlags"/> parameter is NULL.</description>
    ///    </item>
    ///    <item>
    ///       <term>E_OUTOFMEMORY</term>
    ///       <description>Ran out of memory when set required sensor id string.</description>
    ///    </item>
    /// </list>
    /// </returns>
    _Check_return_ HRESULT SetDesiredSensor(_In_opt_z_ const WCHAR* strRequiredSensorId, _Out_ DWORD* pChangedFlags);

private:

    /// <summary>
    /// Called when we don't have a sensor or possibly have the wrong sensor, 
    /// and we want to see if we can get one
    /// </summary>
    /// <returns>The device change type</returns>
    DWORD TryFindNuiSensor();

    /// <summary>
    /// Find the required sensor and verify if it is operateable
    /// </summary>
    /// <returns>The device change type</returns>
    DWORD TryFindRequiredNuiSensor();

    /// <summary>
    /// Enumerate all sensors and try to find an operateable sensor
    /// </summary>
    /// <returns>The device change type</returns>
    DWORD TryFindAnyOneNuiSensor();

    /// <summary>
    /// Update the sensor instance and its status, return if sensor and/or status is changed
    /// </summary>
    /// <param name="vSensors">
    /// The candidate sensor device list
    /// </param>
    /// <returns>The device change type</returns>
    DWORD SelectSensorFromCollection(_In_ const std::vector<INuiSensor *>& vSensors);

    /// <summary>
    /// Update the sensor instance and its status, return if sensor and/or status is changed
    /// </summary>
    /// <param name="pNuiSensor">
    /// The new selected sensor instance
    /// </param>
    /// <param name="dwNewStatus">
    /// The status of the new selected sensor
    /// </param>
    /// <returns>The device change type</returns>
    DWORD UpdateSensorAndStatus(_In_ INuiSensor* pNuiSensor, _In_ const DWORD dwNewStatus);

    /// <summary>
    /// Helper to update the required sensor id
    /// </summary>
    /// <param name="strRequiredSensorId">
    /// The required sensor Id
    /// </param>
    void SetRequiredId(_In_opt_z_ const WCHAR* strRequiredSensorId);

    /// <summary>
    /// Check whether we need a specific sensor
    /// </summary>
    /// <returns>Return true if the sensor is required, else return false</returns>
    bool HasRequiredSensorId() const { return !m_strRequiredSensorId.empty(); }

    /// <summary>
    /// Check if current sensor meets the user requirement
    /// </summary>
    /// <returns>Return true if current sensor meets user requirement</returns>
    bool IsCurrentSensorRequired() const;

    /// <summary>
    /// Check if current sensor is in operateable state and meets the sensor id requirement
    /// </summary>
    /// <returns>Return true if current sensor is valid</returns>
    bool IsCurrentSensorValid() const;

private:
    INuiSensor*     m_pNuiSensor;
    bool            m_bSensorInitialized;

    DWORD           m_dwLatestStatus;

    std::wstring    m_strRequiredSensorId;
};
