#pragma once
#include "stdafx.h"

#include <boost/signals2.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <limits>

/**
 * \class	ColoredOutputStreamUpdater
 *
 * \brief	The ColoredOutputStreamUpdater takes the buffers and face information provided by the
 * 			updateOutputStreams, extracts the HDFace, FaceRaw and FullRawDepth point clouds and publishes
 * 			them via cloudsUpdated or cloudUpdated. Beforehand the ColoredOutputStreamUpdater has to be
 * 			initialied with the coordinatemapper provided by the kinect and information about the resolution
 * 			of the update-streams.
 */
class KinectV1OutPutStreamUpdater
{
public:

	KinectV1OutPutStreamUpdater();
	~KinectV1OutPutStreamUpdater();

	
	/** \brief	The images updated */
	boost::signals2::signal<void(boost::shared_ptr<cv::Mat>)> imageUpdated[3]; // 1 -> color 2 ->depth 3->aligned depth

	/**
	 * \fn	void ColoredOutputStreamUpdater::startFaceCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer);
	 *
	 * \brief	Starts collecting point clouds of all tracked users.
	 * \param [in]	colorBuffer the current color Buffer
	 * \param [in]	depthBuffer the current depth Buffer
	 */
	void startKinectV1DataCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer,UINT16* alignedDepthBuffer, int depthWidth, int depthHeight, int colorWidth, int colorHeight);

	/**
	 * \fn	void ColoredOutputStreamUpdater::stopFaceCollection();
	 *
	 * \brief	Stops the collection of the faces, runs the conversion of the entire depth buffer if neccesary and
	 * 			 calls the signals.
	 */
	void stopKinectV1DataCollection();

	/**
	* \fn	void RecordTabHandler::setCeterEnabled(bool enable);
	*
	* \brief	Enables or disables the centering of recorded Clouds.
	*
	* \param	enable	true to enable, false to disable.
	*/
	//void setCeterEnabled(bool enable);



private:


	/** \brief	true to enable, false to disable the centering. */
	//bool m_centerEnabled;

	/** \brief	Buffer for color data. */
	RGBQUAD* m_colorBuffer;

	/** \brief	Buffer for infrared data. */
	RGBQUAD*  m_depthImageBuffer;

	/** \brief	Buffer for depth data. */
	UINT16*	m_depthBuffer;

	UINT16*	m_alignedDepthBuffer;
	
	/** \brief	Width of the depth frame. */
	int							m_depthWidth;

	/** \brief	Height of the depth frame. */
	int							m_depthHeight;

	/** \brief	Width of the color frame. */
	int							m_colorWidth;

	/** \brief	Height of the color frame. */
	int							m_colorHeight;



};
