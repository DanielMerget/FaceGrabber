//------------------------------------------------------------------------------
// <copyright file="ImageRenderer.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <string>
#include "ImageRenderer.h"

using namespace DirectX;

static const float c_FaceBoxThickness = 6.0f;
static const float c_FacePointThickness = 5.0f;
static const float c_FacePointRadius = 0.1f;
static const float c_FacePropertyFontSize = 32.0f;
static const double c_FaceRotationIncrementInDegrees = 5.0f;

static const float c_TextLayoutWidth = 500;
static const float c_TextLayoutHeight = 500;

/// <summary>
/// Constructor
/// </summary>
ImageRenderer::ImageRenderer() : 
    m_hWnd(0),
    m_sourceWidth(0),
    m_sourceHeight(0),
    m_sourceStride(0),
    m_pD2DFactory(nullptr), 
    m_pRenderTarget(nullptr),
    m_pBitmap(0),
    m_pTextFormat(0),
    m_pDWriteFactory(nullptr)
{
    for (int i = 0; i < BODY_COUNT; i++)
    {
        m_pFaceBrush[i] = nullptr;
    }
}

/// <summary>
/// Destructor
/// </summary>
ImageRenderer::~ImageRenderer()
{
    discardResources();
    SafeRelease(m_pTextFormat);
    SafeRelease(m_pDWriteFactory);
    SafeRelease(m_pD2DFactory);
}

/// <summary>
/// Retrieve client area rect and transform into D2D size structure
/// </summary>
/// <param name="hWnd">The handle to window</param>
/// <returns>Client area size in D2D1_SIZE_U structure</returns>
D2D1_SIZE_U ImageRenderer::GetClientSize(HWND hWnd)
{
    RECT rect = {0};
    if (hWnd)
    {
        GetClientRect(hWnd, &rect);
    }

    return D2D1::SizeU(rect.right, rect.bottom);
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::ensureResources()
{
    HRESULT hr = S_OK;

    if (nullptr == m_pRenderTarget)
    {
        //D2D1_SIZE_U size = D2D1::SizeU(m_sourceWidth, m_sourceHeight);
		D2D1_SIZE_U imageSize = D2D1::SizeU(m_sourceWidth, m_sourceHeight);
		
		D2D1_SIZE_U size = GetClientSize(m_hWnd);

        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;
		
		//rtProps.type        = D2D1_RENDER_TARGET_TYPE_DEFAULT;
   	    //rtProps.minLevel    = D2D1_FEATURE_LEVEL_DEFAULT;

        // Create a hWnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(m_hWnd, size),
            &m_pRenderTarget
            );


        if (SUCCEEDED(hr))
        {
            // Create a bitmap that we can copy image data into and then render to the target
            hr = m_pRenderTarget->CreateBitmap(
                imageSize, 
                D2D1::BitmapProperties(D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE)),
                &m_pBitmap 
                );
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateSolidColorBrush((D2D1::ColorF(D2D1::ColorF::Red, 2.0f)), &m_pFaceBrush[0]);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateSolidColorBrush((D2D1::ColorF(D2D1::ColorF::Green, 2.0f)), &m_pFaceBrush[1]);			
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateSolidColorBrush((D2D1::ColorF(D2D1::ColorF::White, 2.0f)), &m_pFaceBrush[2]);			
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateSolidColorBrush((D2D1::ColorF(D2D1::ColorF::Purple, 2.0f)), &m_pFaceBrush[3]);			
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateSolidColorBrush((D2D1::ColorF(D2D1::ColorF::Orange, 2.0f)), &m_pFaceBrush[4]);			
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pRenderTarget->CreateSolidColorBrush((D2D1::ColorF(D2D1::ColorF::Pink, 2.0f)), &m_pFaceBrush[5]);					
        }        
    }

    if (FAILED(hr))
    {
        discardResources();
    }

    return hr;
}

/// <summary>
/// Dispose of Direct2d resources 
/// </summary>
void ImageRenderer::discardResources()
{
    for (int i = 0; i < BODY_COUNT; i++)
    {
        SafeRelease(m_pFaceBrush[i]);
    }
    SafeRelease(m_pRenderTarget);
    SafeRelease(m_pBitmap);
}

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
HRESULT ImageRenderer::initialize(HWND hWnd, ID2D1Factory* pD2DFactory, int sourceWidth, int sourceHeight, int sourceStride)
{
    if (nullptr == pD2DFactory)
    {
        return E_INVALIDARG;
    }

    m_hWnd = hWnd;

    // One factory for the entire application so save a pointer here
    m_pD2DFactory = pD2DFactory;

    m_pD2DFactory->AddRef();

    // Get the frame size
    m_sourceWidth  = sourceWidth;
    m_sourceHeight = sourceHeight;
    m_sourceStride = sourceStride;

    HRESULT hr;
    hr = DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED,
        __uuidof(IDWriteFactory),
        reinterpret_cast<IUnknown**>(&m_pDWriteFactory));

    if (SUCCEEDED(hr))
    {
        hr = m_pDWriteFactory->CreateTextFormat(
            L"Georgia",           
            NULL,               
            DWRITE_FONT_WEIGHT_ULTRA_BLACK,
            DWRITE_FONT_STYLE_NORMAL,
            DWRITE_FONT_STRETCH_NORMAL,
            c_FacePropertyFontSize,
            L"en-us",
            &m_pTextFormat
            );
    }

    if (SUCCEEDED(hr))
    {
        hr = m_pTextFormat->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_LEADING);
    }

    if (SUCCEEDED(hr))
    {
        hr = m_pTextFormat->SetParagraphAlignment(DWRITE_PARAGRAPH_ALIGNMENT_CENTER);
    }

    return hr;
}

/// <summary>
/// Prepare device to begin drawing
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::beginDrawing()
{
    // create the resources for this draw device
    // they will be recreated if previously lost
    HRESULT hr = ensureResources();

    if (SUCCEEDED(hr))
    {
		CreateSolidBrushes();
		CreateTextFormats();

        m_pRenderTarget->BeginDraw();
		m_pRenderTarget->Clear();
    }

    return hr;
}

/// <summary>
/// Ends drawing
/// </summary>    
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::endDrawing()
{
    HRESULT hr;
    hr = m_pRenderTarget->EndDraw();

    // Device lost, need to recreate the render target
    // We'll dispose it now and retry drawing
    if (hr == D2DERR_RECREATE_TARGET)
    {
        hr = S_OK;
        discardResources();
    }

    return hr;
}

/// <summary>
/// Draws a 32 bit per pixel image of previously specified width, height, and stride to the associated hwnd
/// </summary>
/// <param name="pImage">image data in RGBX format</param>
/// <param name="cbImage">size of image data in bytes</param>
/// <returns>indicates success or failure</returns>
HRESULT ImageRenderer::drawBackground(BYTE* pImage, unsigned long cbImage)
{
    HRESULT hr = S_OK;

    // incorrectly sized image data passed in
    if (cbImage < ((m_sourceHeight - 1) * m_sourceStride) + (m_sourceWidth * 4))
    {
        hr = E_INVALIDARG;
    }
	D2D1_SIZE_U imageSize = D2D1::SizeU(m_sourceWidth,m_sourceHeight);
	    // Ensure bitmap is created.
    hr = EnsureBitmap(imageSize);

	    // Get viewer window client rect
    RECT clientRect;
    if (!::GetClientRect(m_hWnd, &clientRect))
    {
        return E_FAIL;
    }
	    // Calculate the area the stream image is to streched to fit
    D2D1_RECT_F imageRect = GetImageRect(clientRect);
    if (SUCCEEDED(hr))
    {
        // Copy the image that was passed in into the direct2d bitmap
        hr = m_pBitmap->CopyFromMemory(NULL, pImage, m_sourceStride);

        if (SUCCEEDED(hr))
        {
            // Draw the bitmap stretched to the size of the window
            m_pRenderTarget->DrawBitmap(m_pBitmap,imageRect);	
        }
    }

    return hr;
}

void ImageRenderer::drawPoints(const std::vector<ColorSpacePoint> renderPoints){
	
    RECT clientRect;
    if (!::GetClientRect(m_hWnd, &clientRect))
    {
        return ;
    }
	    // Calculate the area the stream image is to streched to fit
    D2D1_RECT_F imageRect = GetImageRect(clientRect);


	for (int i = 0; i < renderPoints.size(); i++)
	{
		ColorSpacePoint inImageRectPoints = ToImageRectPoint(renderPoints[i],imageRect);
		//inImageRectPoints.X = renderPoints[i].X * (imageRect.right  - imageRect.left + 1.0f) / m_sourceWidth + imageRect.left;
		//inImageRectPoints.Y = renderPoints[i].Y * (imageRect.bottom - imageRect.top  + 1.0f) / m_sourceHeight + imageRect.top;
		D2D1_ELLIPSE facePoint = D2D1::Ellipse(D2D1::Point2F(inImageRectPoints.X,inImageRectPoints.Y), c_FacePointRadius, c_FacePointRadius); //renderPoints[i].X, renderPoints[i].Y
		m_pRenderTarget->DrawEllipse(facePoint, m_pFaceBrush[0], c_FacePointThickness);
		
	}
}


/// <summary>
/// Draws face frame results
/// </summary>
/// <param name="iFace">the index of the face frame corresponding to a specific body in the FOV</param>
/// <param name="pFaceBox">face bounding box</param>
/// <param name="pFacePoints">face points</param>
/// <param name="pFaceRotation">face rotation</param>
/// <param name="pFaceProperties">face properties</param>
/// <param name="pFaceTextLayout">face result text layout</param>
void ImageRenderer::drawFaceFrameResults(int iFace, const RectI* pFaceBox, const PointF* pFacePoints, const Vector4* pFaceRotation, const DetectionResult* pFaceProperties, const D2D1_POINT_2F* pFaceTextLayout)
{
	RECT clientRect;
    if (!::GetClientRect(m_hWnd, &clientRect))
    {
        return ;
    }

    // draw the face frame results only if the face bounding box is valid
    if (validateFaceBoxAndPoints(pFaceBox, pFacePoints))
    {
        ID2D1SolidColorBrush* brush = m_pFaceBrush[iFace];

        // draw the face bounding box
		D2D1_RECT_F imageRect = GetImageRect(clientRect);
		ColorSpacePoint old_left_top;
		old_left_top.X = (float)pFaceBox->Left;
		old_left_top.Y = (float)pFaceBox->Top;
		
		ColorSpacePoint old_right_bottom;
		old_right_bottom.X = (float)pFaceBox->Right;
		old_right_bottom.Y = (float)pFaceBox->Bottom;
		

		ColorSpacePoint left_top = ToImageRectPoint(old_left_top,imageRect);
		ColorSpacePoint righ_bottom = ToImageRectPoint(old_right_bottom,imageRect);
		

		/*
        D2D1_RECT_F faceBox = D2D1::RectF(static_cast<FLOAT>(pFaceBox->Left), 
            static_cast<FLOAT>(pFaceBox->Top),
            static_cast<FLOAT>(pFaceBox->Right),
            static_cast<FLOAT>(pFaceBox->Bottom));
        m_pRenderTarget->DrawRectangle(faceBox, brush, c_FaceBoxThickness);*/
				
        D2D1_RECT_F faceBox = D2D1::RectF(left_top.X, left_top.Y,righ_bottom.X,righ_bottom.Y);
        m_pRenderTarget->DrawRectangle(faceBox, brush, c_FaceBoxThickness);

        // draw each face point
        for (int i = 0; i < FacePointType::FacePointType_Count; i++)
        {
			PointF fp = ToImageRectPoint(pFacePoints[i],imageRect);
            D2D1_ELLIPSE facePoint= D2D1::Ellipse(D2D1::Point2F(fp.X, fp.Y), c_FacePointRadius, c_FacePointRadius);
            m_pRenderTarget->DrawEllipse(facePoint, brush, c_FacePointThickness);
        }

        std::wstring faceText = L"";
#if 0
        // extract each face property information and store it is faceText
        for (int iProperty = 0; iProperty < FaceProperty::FaceProperty_Count; iProperty++)
        {
            switch (iProperty)
            {
            case FaceProperty::FaceProperty_Happy:
                faceText += L"Happy :";
                break;
            case FaceProperty::FaceProperty_Engaged:
                faceText += L"Engaged :";
                break;
            case FaceProperty::FaceProperty_LeftEyeClosed:
                faceText += L"LeftEyeClosed :";
                break;
            case FaceProperty::FaceProperty_RightEyeClosed:
                faceText += L"RightEyeClosed :";
                break;
            case FaceProperty::FaceProperty_LookingAway:
                faceText += L"LookingAway :";
                break;
            case FaceProperty::FaceProperty_MouthMoved:
                faceText += L"MouthMoved :";
                break;
            case FaceProperty::FaceProperty_MouthOpen: 
                faceText += L"MouthOpen :";
                break;
            case FaceProperty::FaceProperty_WearingGlasses:
                faceText += L"WearingGlasses :";
                break;
            default:
                break;
            }

            switch (pFaceProperties[iProperty]) 
            {
            case DetectionResult::DetectionResult_Unknown:
                faceText += L" UnKnown";
                break;
            case DetectionResult::DetectionResult_Yes:
                faceText += L" Yes";
                break;

            // consider a "maybe" as a "no" to restrict 
            // the detection result refresh rate
            case DetectionResult::DetectionResult_No:
            case DetectionResult::DetectionResult_Maybe:
                faceText += L" No";
                break;
            default:
                break;
            }

            faceText += L"\n";
        }
#endif
        // extract face rotation in degrees as Euler angles
        int pitch, yaw, roll;
        extractFaceRotationInDegrees(pFaceRotation, &pitch, &yaw, &roll);

        faceText += L"FaceYaw : " + std::to_wstring(yaw) + L"\n";
        faceText += L"FacePitch : " + std::to_wstring(pitch) + L"\n";
        faceText += L"FaceRoll : " + std::to_wstring(roll) + L"\n";

		ColorSpacePoint tmp;
		tmp.X = pFaceTextLayout->x;
		tmp.Y = pFaceTextLayout->y;
		ColorSpacePoint textLayOut = ToImageRectPoint(tmp,imageRect);
		

		/*
        D2D1_RECT_F layoutRect = D2D1::RectF(textLayOut.X, textLayOut.Y, 
            textLayOut.X + c_TextLayoutWidth, 
            textLayOut.Y + c_TextLayoutHeight);    
		*/
		D2D1_RECT_F layoutRect = D2D1::RectF(textLayOut.X, textLayOut.Y, 
			textLayOut.X + c_TextLayoutWidth*(clientRect.right-clientRect.left)/m_sourceWidth,
            textLayOut.Y + c_TextLayoutHeight*(clientRect.bottom-clientRect.top)/m_sourceHeight); 
        // render the face property and face rotation information

        m_pRenderTarget->DrawTextW(faceText.c_str(), 
            static_cast<UINT32>(faceText.length()), 
            m_pTextFormat, 
            layoutRect, 
            brush);
    }
}

/// <summary>
/// Validates face bounding box and face points to be within screen space
/// </summary>
/// <param name="pFaceBox">the face bounding box</param>
/// <param name="pFacePoints">the face points</param>
/// <returns>success or failure</returns>
bool ImageRenderer::validateFaceBoxAndPoints(const RectI* pFaceBox, const PointF* pFacePoints)
{
    bool isFaceValid = false;

    if (pFaceBox != nullptr)
    {
        INT32 screenWidth = m_sourceWidth;
        INT32 screenHeight = m_sourceHeight;

        INT32 width = pFaceBox->Right - pFaceBox->Left;
        INT32 height = pFaceBox->Bottom - pFaceBox->Top;

        // check if we have a valid rectangle within the bounds of the screen space
        isFaceValid = width > 0 && 
            height > 0 && 
            pFaceBox->Right <= screenWidth && 
            pFaceBox->Bottom <= screenHeight;

        if (isFaceValid)
        {
            for (int i = 0; i < FacePointType::FacePointType_Count; i++)
            {
                // check if we have a valid face point within the bounds of the screen space                        
                bool isFacePointValid = pFacePoints[i].X > 0.0f &&
                    pFacePoints[i].Y > 0.0f &&
                    pFacePoints[i].X < m_sourceWidth &&
                    pFacePoints[i].Y < m_sourceHeight;

                if (!isFacePointValid)
                {
                    isFaceValid = false;
                    break;
                }
            }
        }
    }

    return isFaceValid;
}


/// <summary>
/// Converts rotation quaternion to Euler angles 
/// And then maps them to a specified range of values to control the refresh rate
/// </summary>
/// <param name="pQuaternion">face rotation quaternion</param>
/// <param name="pPitch">rotation about the X-axis</param>
/// <param name="pYaw">rotation about the Y-axis</param>
/// <param name="pRoll">rotation about the Z-axis</param>
void ImageRenderer::extractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll)
{
    double x = pQuaternion->x;
    double y = pQuaternion->y;
    double z = pQuaternion->z;
    double w = pQuaternion->w;

    // convert face rotation quaternion to Euler angles in degrees		
    double dPitch, dYaw, dRoll;
    dPitch = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0;
    dYaw = asin(2 * (w * y - x * z)) / M_PI * 180.0;
    dRoll = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0;

    // clamp rotation values in degrees to a specified range of values to control the refresh rate
    double increment = c_FaceRotationIncrementInDegrees; 
    *pPitch = static_cast<int>(floor((dPitch + increment/2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * increment);
    *pYaw = static_cast<int>(floor((dYaw + increment/2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * increment);
    *pRoll = static_cast<int>(floor((dRoll + increment/2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * increment);
}

/// <summary>
/// Ensure bitmap is created
/// </summary>
/// <param name=imageSize>Size of bit map</param>
/// <returns>Indicates success or failure</returns>
HRESULT ImageRenderer::EnsureBitmap(const D2D1_SIZE_U& imageSize)
{
    if (m_pBitmap)
    {
        D2D1_SIZE_U size = m_pBitmap->GetPixelSize();
        if (size.width == imageSize.width && size.height == imageSize.height)
        {
            return S_OK;
        }
    }

    SafeRelease(m_pBitmap);

    if (m_pRenderTarget)
    {
        // Create a bitmap that we can copy image data into it and then render to the target.
        return m_pRenderTarget->CreateBitmap(imageSize,
                                             D2D1::BitmapProperties(D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE)),
                                             &m_pBitmap);
    }

    return E_FAIL;
}


D2D1_RECT_F ImageRenderer::GetImageRect(const RECT &client)
{
    D2D1_RECT_F imageRect = D2D1::RectF();

	float ratio  = static_cast<float>(m_sourceWidth) / static_cast<float>(m_sourceHeight);
    float width  = static_cast<float>(client.right);
    float height = width / ratio;

    if (height > client.bottom)
    {
        height = static_cast<float>(client.bottom);
        width  = height * ratio;
    }

    imageRect.left   = (client.right  - width  + 1) / 2.0f;
    imageRect.top    = (client.bottom - height + 1) / 2.0f;
    imageRect.right  = imageRect.left + width;
    imageRect.bottom = imageRect.top  + height;
    

    return imageRect;
}

template<typename T>
T ImageRenderer::ToImageRectPoint(T point,const D2D1_RECT_F& imageRect)
{
   

    FLOAT resultX, resultY;
	resultX = point.X * (imageRect.right  - imageRect.left + 1.0f) / m_sourceWidth + imageRect.left;
	resultY = point.Y * (imageRect.bottom - imageRect.top  + 1.0f) / m_sourceHeight + imageRect.top;
	T result;
	result.X = resultX;
	result.Y = resultY;
    return result;
}

// <summary>
/// Draw frame FPS counter
/// </summary>
/// <param name="clientRect">Client area of viewer's window</param>
void ImageRenderer::DrawFPS(UINT16 fps)
{
	RECT clientRect;
    if (!::GetClientRect(m_hWnd, &clientRect))
    {
        return;
    }

    // Get rectangle position and size
    D2D1_RECT_F rect = D2D1::RectF((FLOAT)clientRect.right - 50.0f, 0.0f, (FLOAT)clientRect.right, (FLOAT)clientRect.top + 50.0f);

    // Fill rectangle
    FillRectangle(rect, ImageRendererBrushGray);

    // Draw a while circle
    D2D1_POINT_2F center = D2D1::Point2F((rect.right + rect.left) / 2.0f, (rect.bottom + rect.top) / 2.0f);
    DrawCircle(center, 23.0f, ImageRendererBrushWhite, 4.0f);
    
    // Draw FPS text
    WCHAR text[256];
    swprintf_s(text, sizeof(text) / sizeof(WCHAR) - 1, L"%d", fps);
    UINT cch = (UINT)wcsnlen_s(text, 256);
    DrawText(text, cch, rect, ImageRendererBrushWhite, ImageRendererTextFormatFps);
}


/// <summary>
/// Create D2D solid color brushes
/// </summary>
void ImageRenderer::CreateSolidBrushes()
{
    // Create solid brushes to draw skeleton, fps & resolution text
    CreateSolidBrush(D2D1::ColorF::LightGreen,  ImageRendererBrushJointTracked);
    CreateSolidBrush(D2D1::ColorF::Yellow,      ImageRendererBrushJointInferred);
    CreateSolidBrush(D2D1::ColorF::Green,       ImageRendererBrushBoneTracked);
    CreateSolidBrush(D2D1::ColorF::Gray,        ImageRendererBrushBoneInferred);
    CreateSolidBrush(D2D1::ColorF::White,       ImageRendererBrushWhite);
    CreateSolidBrush(D2D1::ColorF::Gray,        ImageRendererBrushGray);
    CreateSolidBrush(D2D1::ColorF::Green,       ImageRendererBrushGreen);
}


/// <summary>
/// Create a specific D2D solid color brush from parameters
/// </summary>
/// <param name="color">The color used by brush</param>
/// <param name="brush">The index of brush</param>
void ImageRenderer::CreateSolidBrush(D2D1::ColorF::Enum color, ImageRendererBrush brush)
{
    m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(color, 1.0f), &m_brushes[brush]);
}


/// <summary>
/// Draw a rectangle and fill it
/// </summary>
/// <param name="rect">The rectangle to fill</param>
/// <param name="brush">The index of brush</param>
void ImageRenderer::FillRectangle(const D2D1_RECT_F& rect, ImageRendererBrush brush)
{
    // Validate brush index
    if (brush < 0 || brush >= ImageRendererBrushCount)
    {
        return;
    }

    // Select brush
    ID2D1SolidColorBrush* pBrush = m_brushes[brush];

    // Draw rectangle
    m_pRenderTarget->FillRectangle(rect, pBrush);
}


/// <summary>
/// Draw the circle representing the joint
/// </summary>
/// <param name="center">The center of the circle</param>
/// <param name="radius">The radius of the circle</param>
/// <param name="brush">Index of brush</param>
/// <param name="strokeWidth">Width of the line</param>
/// <param name="strokeStyle">Style of the line</param>
void ImageRenderer::DrawCircle(const D2D1_POINT_2F& center, float radius, ImageRendererBrush brush, float strokeWidth, ID2D1StrokeStyle* strokeStyle)
{
    // Validate brush index
    if (brush < 0 || brush >= ImageRendererBrushCount)
    {
        return;
    }

    // Select created brush
    ID2D1SolidColorBrush* pBrush = m_brushes[brush];

    // Create ellipse with same radius on both long and short axis. Identical to the circle
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(center, radius, radius);

    // Draw ellipse
    m_pRenderTarget->DrawEllipse(ellipse, pBrush, strokeWidth, strokeStyle);
}


/// <summary>
/// Draw FPS text
/// </summary>
/// <param name="pText">Text to draw</param>
/// <param name="cch">Count of charaters in text</param>
/// <param name="rect">The area to draw text</param>
/// <param name="brush">Index of brush</param>
/// <param name="format">Text format</param>
void ImageRenderer::DrawText(const WCHAR* pText, UINT cch, const D2D1_RECT_F &rect, ImageRendererBrush brush, ImageRendererTextFormat format)
{
    // Validate pointer to text
    if (!pText)
    {
        return;
    }

    // Validate brush index
    if (brush < 0 || brush >= ImageRendererBrushCount)
    {
        return;
    }

    // Validate text format index
    if (format < 0 || format >= ImageRendererTextFormatCount)
    {
        return;
    }

    // Select created text format
    IDWriteTextFormat* pFormat = m_formats[format];

    // Select created brush
    ID2D1SolidColorBrush* pBrush = m_brushes[brush];

    // Draw text to rectangle area
    m_pRenderTarget->DrawText(pText, cch, pFormat, rect, pBrush);
}


/// <summary>
/// Create text formats to draw FPS and image resolution lable
/// </summary>
void ImageRenderer::CreateTextFormats()
{
    CreateTextFormat(L"Segoe UI", 25.0f, DWRITE_TEXT_ALIGNMENT_CENTER,  DWRITE_PARAGRAPH_ALIGNMENT_CENTER, &m_formats[ImageRendererTextFormatFps]);        // FPS text format
    CreateTextFormat(L"Segoe UI", 12.0f, DWRITE_TEXT_ALIGNMENT_LEADING, DWRITE_PARAGRAPH_ALIGNMENT_NEAR,   &m_formats[ImageRendererTextFormatResolution]); // Resolution text format
}

/// <summary>
/// Create a specific DWrite text format from parameters
/// </summary>
/// <param name="fontFamilyName">Family name of created font</param>
/// <param name="fontSize">Size of created font</param>
/// <param name="textAlignment">Alignment of text</param>
/// <param name="paragraphAlignment">Alignment of paragraph</param>
/// <param name="ppTextFormat">Created text format</param>
void ImageRenderer::CreateTextFormat(const WCHAR* fontFamilyName, FLOAT fontSize, DWRITE_TEXT_ALIGNMENT textAlignment, DWRITE_PARAGRAPH_ALIGNMENT paragraphAlignment, IDWriteTextFormat** ppTextFormat)
{
    HRESULT hr = m_pDWriteFactory->CreateTextFormat(fontFamilyName, nullptr, DWRITE_FONT_WEIGHT_NORMAL, DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, fontSize, L"", ppTextFormat);
    if (SUCCEEDED(hr))
    {
        (*ppTextFormat)->SetTextAlignment(textAlignment);
        (*ppTextFormat)->SetParagraphAlignment(paragraphAlignment);
    }
}

