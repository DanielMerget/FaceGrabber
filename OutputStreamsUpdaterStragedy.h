#pragma once
#include "stdafx.h"

#include <Kinect.h>
#include <Kinect.Face.h>
#include <limits>
class OutputStreamsUpdaterStragedy
{
public:
	OutputStreamsUpdaterStragedy();
	~OutputStreamsUpdaterStragedy();

	bool isValidCamSpacePoint(CameraSpacePoint camSpacePoint){

		return (camSpacePoint.X >= - FLT_MAX && camSpacePoint.X <= FLT_MAX)
			&& (camSpacePoint.Y >= - FLT_MAX && camSpacePoint.Y <= FLT_MAX)
			&& (camSpacePoint.Z >= - FLT_MAX && camSpacePoint.Z <= FLT_MAX);
	}

	bool isValidDepthPoint(DepthSpacePoint depthPoint){
		return (depthPoint.X > 0 && depthPoint.X < m_depthWidth) && (depthPoint.Y > 0 && depthPoint.Y < m_depthWidth);
	}
	bool isFloatValueInfinity(float value)
	{
		return std::numeric_limits < float >::infinity() == value;
	}

	virtual HRESULT updateOutputStreams(IFaceModel* faceModel, IFaceAlignment* faceAlignment, int bufferSize, CameraSpacePoint* detectedHDFacePointsCamSpace,
		ColorSpacePoint* detectedHDFacePointsColorSpace) = 0;

	virtual void initialize(ICoordinateMapper* m_pCoordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

	virtual void startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer) = 0;
	virtual void stopFaceCollection() = 0;
protected:
	// Coordinate mapper
	ICoordinateMapper*			m_pCoordinateMapper;
	int							m_depthWidth;
	int							m_depthHeight;
	int							m_colorWidth;
	int							m_colorHeight;
	
};

