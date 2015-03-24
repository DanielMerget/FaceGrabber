#include "stdafx.h"
#include "OutputStreamsUpdaterStragedy.h"


OutputStreamsUpdaterStragedy::OutputStreamsUpdaterStragedy() :
	m_pCoordinateMapper(nullptr),
	m_depthWidth (-1),
	m_depthHeight(-1),
	m_colorWidth (-1),
	m_colorHeight(-1)
{
}


OutputStreamsUpdaterStragedy::~OutputStreamsUpdaterStragedy()
{
}


void OutputStreamsUpdaterStragedy::initialize(ICoordinateMapper* coordinateMapper, int depthWidth, int depthHeight, int colorWidth, int colorHeight)
{
	m_pCoordinateMapper = coordinateMapper;
	m_depthWidth = depthWidth;
	m_depthHeight = depthHeight;
	m_colorHeight = colorHeight;
	m_colorWidth = colorWidth;
}