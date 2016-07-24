//------------------------------------------------------------------------------
// <copyright file="NuiImageBuffer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "../stdafx.h"
#include <NuiApi.h>

#define MAX_PLAYER_INDEX    6

enum DEPTH_TREATMENT
{
    CLAMP_UNRELIABLE_DEPTHS,
    TINT_UNRELIABLE_DEPTHS,
    DISPLAY_ALL_DEPTHS,
};

class NuiImageBuffer
{
public:
    /// <summary>
    /// Constructor
    /// </summary>
    NuiImageBuffer();

    /// <summary>
    /// Destructor
    /// </summary>
   ~NuiImageBuffer();

public:
    /// <summary>
    /// Set image size according to image resolution
    /// </summary>
    /// <param name="resolution">Image resolution</param>
    void SetImageSize(NUI_IMAGE_RESOLUTION resolution);

    /// <summary>
    /// Clear buffer
    /// </summary>
    void Clear();

    /// <summary>
    /// Get width of image.
    /// </sumamry>
    /// <returns>Width of image.</returns>
    DWORD GetWidth() const;

    /// <summary>
    /// Get height of image.
    /// </sumamry>
    /// <returns>Width of height.</returns>
    DWORD GetHeight() const;

    /// <suumary>
    /// Get size of buffer.
    /// <summary>
    /// <returns>Size of buffer.</returns>
    DWORD GetBufferSize() const;

    /// <summary>
    /// Return allocated buffer.
    /// </summary>
    /// <returns>
    /// The pointer to the allocated buffer
    /// Return value could be nullptr if the buffer is not allocated
    /// </returns>
    BYTE* GetBuffer() const;

    /// <summary>
    /// Copy color frame image to image buffer
    /// </summary>
    /// <param name="pImage">The pointer to the frame image to copy</param>
    /// <param name="size">Size in bytes to copy</param>
    void CopyRGB(const BYTE* source, UINT size);

    /// <summary>
    /// Copy raw bayer data and convert to RGB image
    /// </summary>
    /// <param name="pImage">The pointer to the frame image to copy</param>
    /// <param name="size">Size in bytes to copy</param>
    void CopyBayer(const BYTE* source, UINT size);

    /// <summary>
    /// Copy and convert infrared frame image to image buffer
    /// </summary>
    /// <param name="pImage">The pointer to the frame image to copy</param>
    /// <param name="size">Size in bytes to copy</param>
    void CopyInfrared(const BYTE* source, UINT size);

    /// <summary>
    /// Copy and convert depth frame image to image buffer
    /// </summary>
    /// <param name="pImage">The pointer to the frame image to copy</param>
    /// <param name="size">Size in bytes to copy</param>
    /// <param name="nearMode">Depth stream range mode</param>
    /// <param name="treatment">Depth treatment mode</param>
    void CopyDepth(const BYTE* source, UINT size, BOOL nearMode, DEPTH_TREATMENT treatment);

	
    /// <summary>
    /// Copy and convert depth frame image to image buffer
    /// </summary>
    /// <param name="pImage">The pointer to the frame image to copy</param>
    /// <param name="size">Size in bytes to copy</param>
    /// <param name="nearMode">Depth stream range mode</param>
    /// <param name="treatment">Depth treatment mode</param>
    void CopyDepth(const USHORT * source, UINT size, BOOL nearMode, DEPTH_TREATMENT treatment);

private:
    /// <summary>
    /// Calculate image width and height according to image resolution enumeration value.
    /// If resolution enumeration is invalid, width and height will be set to zero.
    /// </summary>
    /// <param name="resolution">Enumeration value which indicates the image resolution format</param>
    /// <param name="width">Calculated image width</param>
    /// <param name="height">Calculated image height</param>
    void GetImageSize(NUI_IMAGE_RESOLUTION resolution, DWORD& width, DWORD& height);

    /// <summary>
    /// Set color value
    /// </summary>
    /// <param name="pColor">The pointer to the variable to be set with color</param>
    /// <param name="red">Red component of the color</param>
    /// <param name="green">Green component of the color</parma>
    /// <param name="blue">Blue component of the color</param>
    /// <param name="alpha">Alpha component of the color</param>
    inline void SetColor(UINT* pColor, BYTE red, BYTE green, BYTE blue, BYTE alpha = 255);

    /// <summary>
    /// Calculate intensity of a certain depth
    /// </summary>
    /// <param name="depth">A certain depth</param>
    /// <returns>Intensity calculated from a certain depth</returns>
    BYTE GetIntensity(int depth);

    /// <summary>
    /// Allocate a buffer of size and return it
    /// </summary>
    /// <param name="size">Size of buffer to allocate. Zeor to release buffer memory</param>
    /// <returns>The pointer to the allocated buffer. If size hasn't changed, the previously allocated buffer is returned</returns>
    BYTE* ResetBuffer(UINT size);

private:
 

    bool                m_nearMode;
    DWORD               m_width;
    DWORD               m_height;
    DWORD               m_srcWidth;
    DWORD               m_srcHeight;
    DWORD               m_nSizeInBytes;
    BYTE*               m_pBuffer;
    DEPTH_TREATMENT     m_depthTreatment;
};