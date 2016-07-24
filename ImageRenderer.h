//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// Manages the drawing of image data

#pragma once
#include "stdafx.h"

#include <d2d1.h>
#include <Dwrite.h>
#include <DirectXMath.h>
#include <vector>
#include <Kinect.h>
#include "./NuiImageBuffer.h"

enum ImageRendererBrush
{
    ImageRendererBrushJointTracked = 0,
    ImageRendererBrushJointInferred,
    ImageRendererBrushBoneTracked,
    ImageRendererBrushBoneInferred,
    ImageRendererBrushWhite,
    ImageRendererBrushGray,
    ImageRendererBrushGreen,
    ImageRendererBrushCount
};

enum ImageRendererTextFormat
{
    ImageRendererTextFormatFps = 0,
    ImageRendererTextFormatResolution,
    ImageRendererTextFormatCount
};

class ImageRenderer
{
public:
    /// <summary>
    /// Constructor
    /// </summary>
    ImageRenderer();

    /// <summary>
    /// Destructor
    /// </summary>
    virtual ~ImageRenderer();

    /// <summary>
    /// Set the window to draw to as well as the video format
    /// Implied bits per pixel is 32
    /// </summary>
    /// <param name="hWnd">window to draw to</param>
    /// <param name="pD2DFactory">already created D2D factory object</param>
    /// <param name="sourceWidth">width (in pixels) of image data to be drawn</param>
    /// <param name="sourceHeight">height (in pixels) of image data to be drawn</param>
    /// <param name="sourceStride">length (in bytes) of a single scanline</param>
    /// <returns>indicates success or failure</returns>
    HRESULT initialize(HWND hwnd, ID2D1Factory* pD2DFactory, int sourceWidth, int sourceHeight, int sourceStride);

    /// <summary>
    /// Prepare device to begin drawing
    /// <returns>indicates success or failure</returns>
    /// </summary>
    HRESULT beginDrawing();

    /// <summary>
    /// Ends drawing
    /// <returns>indicates success or failure</returns>
    /// </summary>    
    HRESULT endDrawing();

    /// <summary>
    /// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
    /// </summary>
    /// <param name="pImage">image data in RGBX format</param>
    /// <param name="cbImage">size of image data in bytes</param>
    /// <returns>indicates success or failure</returns>
    HRESULT drawBackground(BYTE* pImage, unsigned long cbImage);


	void drawPoints(const std::vector<ColorSpacePoint> renderPoints);
    /// <summary>
    /// Draws face frame results
    /// </summary>
    /// <param name="iFace">the index of the face frame corresponding to a specific body in the FOV</param>
    /// <param name="pFaceBox">face bounding box</param>
    /// <param name="pFacePoints">face points</param>
    /// <param name="pFaceRotation">face rotation</param>
    /// <param name="pFaceProperties">face properties</param>
    /// <param name="pFaceTextLayout">face result text layout</param>
    void drawFaceFrameResults(int iFace, const RectI* pFaceBox, const PointF* pFacePoints, const Vector4* pFaceRotation, const DetectionResult* pFaceProperties, const D2D1_POINT_2F* pFaceTextLayout);
	   
	/// <summary>
    /// Converts rotation quaternion to Euler angles 
    /// And then maps them to a specified range of values to control the refresh rate
    /// </summary>
    /// <param name="pQuaternion">face rotation quaternion</param>
    /// <param name="pPitch">rotation about the X-axis</param>
    /// <param name="pYaw">rotation about the Y-axis</param>
    /// <param name="pRoll">rotation about the Z-axis</param>
    void extractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll);

	void setSize(int sourceWidth, int sourceHeight, int sourceStride){m_sourceWidth = sourceWidth;m_sourceHeight = sourceHeight;m_sourceStride=sourceStride;}

	// <summary>
	/// Draw frame FPS counter
	/// </summary>
	/// <param name="clientRect">Client area of viewer's window</param>
	void DrawFPS(UINT16 fps);

private:
    /// <summary>
    /// Ensure necessary Direct2d resources are created
    /// </summary>
    /// <returns>indicates success or failure</returns>
    HRESULT ensureResources();

    /// <summary>
    /// Dispose of Direct2d resources 
    /// </summary>
    void discardResources();

    /// <summary>
    /// Validates face bounding box and face points to be within screen space
    /// </summary>
    /// <param name="pFaceBox">the face bounding box</param>
    /// <param name="pFacePoints">the face points</param>
    /// <returns>success or failure</returns>
    bool validateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints);

	D2D1_SIZE_U GetClientSize(HWND hWnd);

	HRESULT EnsureBitmap(const D2D1_SIZE_U& imageSize);
 
	D2D1_RECT_F GetImageRect(const RECT &client);

	template<typename T>
	T ToImageRectPoint(T point,const D2D1_RECT_F& imageRect);

	/// <summary>
	/// Create D2D solid color brushes
	/// </summary>
	void CreateSolidBrushes();

	void CreateSolidBrush(D2D1::ColorF::Enum color, ImageRendererBrush brush);

	
    /// <summary>
    /// Draw a rectangle and fill it
    /// </summary>
    /// <param name="rect">The rectangle to fill</param>
    /// <param name="brush">The index of brush</param>
    void FillRectangle(const D2D1_RECT_F& rect, ImageRendererBrush brush);


	/// <summary>
    /// Draw the circle representing the joint
    /// </summary>
    /// <param name="center">The center of the circle</param>
    /// <param name="radius">The radius of the circle</param>
    /// <param name="brush">Index of brush</param>
    /// <param name="strokeWidth">Width of the line</param>
    /// <param name="strokeStyle">Style of the line</param>
    void DrawCircle(const D2D1_POINT_2F& center, float radius, ImageRendererBrush brush, float strokeWidth = 1.0f, ID2D1StrokeStyle* strokeStyle = nullptr);


	/// <summary>
    /// Draw FPS text
    /// </summary>
    /// <param name="pText">Text to draw</param>
    /// <param name="cch">Count of charaters in text</param>
    /// <param name="rect">The area to draw text</param>
    /// <param name="brush">Index of brush</param>
    /// <param name="format">Text format</param>
    void DrawText(const WCHAR* pText, UINT cch, const D2D1_RECT_F &rect, ImageRendererBrush brush, ImageRendererTextFormat format);

	/// <summary>
    /// Create a specific DWrite text format from parameters
    /// </summary>
    /// <param name="fontFamilyName">Family name of created font</param>
    /// <param name="fontSize">Size of created font</param>
    /// <param name="textAlignment">Alignment of text</param>
    /// <param name="paragraphAlignment">Alignment of paragraph</param>
    /// <param name="ppTextFormat">Created text format</param>
    void CreateTextFormat(const WCHAR* fontFamilyName, FLOAT fontSize, DWRITE_TEXT_ALIGNMENT textAlignment, DWRITE_PARAGRAPH_ALIGNMENT paragraphAlignment, IDWriteTextFormat** ppTextFormat);

	    /// <summary>
    /// Create text formats to draw FPS and image resolution lable
    /// </summary>
    void CreateTextFormats();

    HWND                     m_hWnd;

    // Format information
    UINT                     m_sourceHeight;
    UINT                     m_sourceWidth;
    LONG                     m_sourceStride;

    // Direct2D 
    ID2D1Factory*            m_pD2DFactory;
    ID2D1HwndRenderTarget*   m_pRenderTarget;
    ID2D1Bitmap*             m_pBitmap;
    ID2D1SolidColorBrush*    m_pFaceBrush[BODY_COUNT];

    // DirectWrite
    IDWriteFactory*		     m_pDWriteFactory;
    IDWriteTextFormat*       m_pTextFormat;    

	IDWriteTextFormat*          m_formats[ImageRendererTextFormatCount];
	ID2D1SolidColorBrush*       m_brushes[ImageRendererBrushCount];
};