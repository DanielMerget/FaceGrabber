#pragma once
#include <Kinect.h>
#include <Kinect.Face.h>
#include <limits>
class OutputStreamsUpdaterStragedy
{
public:
	OutputStreamsUpdaterStragedy();
	~OutputStreamsUpdaterStragedy();

	bool isFloatValueInfinity(float value)
	{
		return std::numeric_limits < float >::infinity() == value;
	}

	virtual HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace, RGBQUAD* colorBuffer, UINT16* depthBuffer) = 0;

	void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

protected:
	// Coordinate mapper
	ICoordinateMapper*			m_pCoordinateMapper;
	int							m_depthWidth;
	int							m_depthHeight;
	int							m_colorWidth;
	int							m_colorHeight;
};

