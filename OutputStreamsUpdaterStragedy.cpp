#include "OutputStreamsUpdaterStragedy.h"


OutputStreamsUpdaterStragedy::OutputStreamsUpdaterStragedy()
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