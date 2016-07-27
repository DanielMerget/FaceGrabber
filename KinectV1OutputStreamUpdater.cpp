
#include "KinectV1OutputStreamUpdater.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

KinectV1OutPutStreamUpdater::KinectV1OutPutStreamUpdater()
{
	m_colorBuffer = nullptr;
	m_depthBuffer = nullptr;
	m_alignedDepthBuffer = nullptr;

}
KinectV1OutPutStreamUpdater::~KinectV1OutPutStreamUpdater()
{
}

void KinectV1OutPutStreamUpdater::startKinectV1DataCollection(RGBQUAD* colorBuffer, UINT16* depthBuffer,UINT16* alignedDepthBuffer, int depthWidth, int depthHeight, int colorWidth, int colorHeight)
{

	m_colorBuffer = colorBuffer;
	m_depthBuffer = depthBuffer;
	m_alignedDepthBuffer = alignedDepthBuffer;
	
	m_depthWidth = depthWidth;
	m_depthHeight = depthHeight;
	m_colorWidth = colorWidth;
	m_colorHeight = colorHeight;
}

void KinectV1OutPutStreamUpdater::stopKinectV1DataCollection()
{	
	static int pushCounts = 0;
		//KinectColorRaw
	if (!imageUpdated[0].empty()){
		// update writer
		cv::Mat m_colorImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_colorBuffer,cv::Mat::AUTO_STEP); //, ,cv::Mat::AUTO_STEP

		boost::shared_ptr<cv::Mat> m_colorImagePtr(new cv::Mat());
		*m_colorImagePtr = m_colorImage.clone();
		cv::cvtColor(m_colorImage,*m_colorImagePtr, CV_BGRA2BGR);
				
		
		/*
		cv::Mat imageCopy;
		imageCopy = m_colorImagePtr->clone();
		cv::cvtColor(imageCopy, *m_colorImagePtr, CV_BGRA2BGR);
		cv::imshow("UpdateColor",m_colorImage);
		//m_colorImage.convertTo(imageF_8UC3, CV_8UC3, 255);
		cv::cvtColor(*m_colorImagePtr,imageCopy, CV_BGRA2BGR);
		cv::imwrite("E:\\temp\\init.png", imageCopy);
		*/
		//Sleep(500);

		imageUpdated[0](m_colorImagePtr);
		
		pushCounts++;
		
	}

	//KinectDepthRaw
	if (!imageUpdated[1].empty()){
		// update writer
		cv::Mat m_depthImage = cv::Mat(m_depthHeight, m_depthWidth, CV_16UC1, m_depthBuffer, cv::Mat::AUTO_STEP);
		boost::shared_ptr<cv::Mat> m_depthImagePtr(new cv::Mat());
		*m_depthImagePtr = m_depthImage.clone();
		imageUpdated[1](m_depthImagePtr);
#if 0
		//cv::imshow("UpdateDepth",m_depthImage);

		//smooth algorithm
		//Mat depthMat(height, width, CV_16UC1, depth); // from kinect
		cv::Mat depthf(m_depthHeight, m_depthWidth, CV_8UC1);

		m_depthImage.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		//imshow("original-depth", depthf);

		const unsigned char noDepth = 0; // change to 255, if values no depth uses max value
		cv::Mat temp, temp2;

		// 1 step - downsize for performance, use a smaller version of depth image
		cv::Mat small_depthf; 
		resize(depthf, small_depthf, cv::Size(), 0.2, 0.2);
		
		// 2 step - inpaint only the masked "unknown" pixels
		cv::inpaint(depthf, (depthf == noDepth), temp, 5.0, cv::INPAINT_TELEA);
		
		// 3 step - upscale to original size and replace inpainted regions in original depth image
		resize(temp, temp2, depthf.size());
		temp2.copyTo(depthf, (depthf == noDepth)); // add to the original signal

		//imshow("depth-inpaint", depthf); 

		*m_depthImagePtr = depthf.clone();
#endif 
		
	}


	//KinectV1DeptIntensityImage
	if (!imageUpdated[2].empty()){
		// update writer
		//cv::Mat m_alignedDepthImage = cv::Mat(m_colorHeight, m_colorWidth, CV_8UC4, m_alignedDepthBuffer, cv::Mat::AUTO_STEP);   
		cv::Mat m_alignedDepthImage = cv::Mat(m_colorHeight, m_colorWidth,  CV_16UC1, m_alignedDepthBuffer, cv::Mat::AUTO_STEP);   
		boost::shared_ptr<cv::Mat> m_alignedDepthImagePtr(new cv::Mat());
		*m_alignedDepthImagePtr = m_alignedDepthImage.clone();
		imageUpdated[2](m_alignedDepthImagePtr);
	}



	//m_isValidFaceFrame = false;
	m_colorBuffer = nullptr;
	m_depthBuffer = nullptr;
	m_alignedDepthBuffer = nullptr;
}

