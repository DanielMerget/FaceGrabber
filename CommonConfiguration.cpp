
#include "stdafx.h"
#include "CommonConfiguration.h"

CommonConfiguration::CommonConfiguration()
{

}

CommonConfiguration::~CommonConfiguration()
{

}

RecordingShowOpt CommonConfiguration::getShowOpt()
{
	return m_ShowOpt;
}


void CommonConfiguration::setShowOpt(RecordingShowOpt ShowOpt)
{
	m_ShowOpt = ShowOpt;
}


void CommonConfiguration::setFacePointsShowOpt(FacePointsShowOpt FacePointsShowOpt)
{
	m_FacePointsShowOpt = FacePointsShowOpt;
}


FacePointsShowOpt CommonConfiguration::getFacePointsShowOpt()
{
	return m_FacePointsShowOpt;
}

void CommonConfiguration::setThreadCountToStart(int threadsCount)
{
	m_threadsCount = threadsCount;
}

int CommonConfiguration::getThreadCountToStart()
{
	return m_threadsCount;
}

void CommonConfiguration::setKeepBGEnabled(bool enabled)
{
	m_keepBGEnabled = enabled;
}

bool CommonConfiguration::isKeepBGEnabled()
{
	return m_keepBGEnabled;
}

CString CommonConfiguration::getShowOptAsString(RecordingShowOpt ShowOpt)
{
	switch (ShowOpt)
	{
	case Color_Raw:
		return CString(L"Raw Color");
	case Depth_Raw:
		return CString(L"Raw Depth");
	case Color_Body:
		return CString(L"Color Body");
	case Depth_Body:
		return CString(L"Depth Body");
	case Infrared_Raw:
		return CString(L"Infrared");
	case RECORD_SHOW_OPT_COUNT:
		return CString(L"ERROR");
	default:
		return CString(L"UNKNOWN_Show Opt");
		break;
	}
}
