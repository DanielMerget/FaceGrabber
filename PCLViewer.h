#pragma once
#include "stdafx.h"
#undef max
#undef min
#include <pcl/visualization/cloud_viewer.h>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <boost/signals.hpp>


/**
 * \class	PCLViewer
 *
 * \brief	The PCL viewer is able to render Point Clouds using the PCLVisualizer. The PCLViewer
 * 			extends the PCLVisualizer by enabling multi-threaded updating of the point clouds.
 * 			Furthermore, dynamic switching between colored and non-colored clouds, as well as
 * 			changing the amount of viewports is supported.
 */

class PCLViewer
{
public:

	/**
	 * \fn	PCLViewer::PCLViewer(int cloudCount, std::string viewerName = "PCLViewer");
	 *
	 * \brief	Constructor.
	 *
	 * \param	cloudCount	Number of clouds to show.
	 * \param	viewerName	Name of the viewer to be displayed.
	 */

	PCLViewer(int cloudCount, std::string viewerName = "PCLViewer");

	/**
	 * \fn	PCLViewer::~PCLViewer();
	 *
	 * \brief	Destructor.
	 */
	~PCLViewer();

	/**
	 * \fn	void PCLViewer::updateColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);
	 *
	 * \brief	Updates the rendered cloud with the provided colored clouds. The indices of the list are matched
	 * 			to the created viewports. The number of clouds in the list must be specified with setNumOfClouds
	 * 			beforehand.
	 *
	 * \param	clouds	The clouds to be updated.
	 */
	void updateColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

	/**
	 * \fn	void PCLViewer::updateNonColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds);
	 *
	 * \brief	Updates the rendered cloud with the provided non-colored clouds. The indices of the list are matched
	 * 			to the created viewports. The number of clouds in the list must be specified with setNumOfClouds
	 * 			beforehand.
	 * 			
	 * \param	clouds	The clouds.
	 */
	void updateNonColoredClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds);

	/**
	 * \fn	void PCLViewer::stopViewer();
	 *
	 * \brief	Stops the viewer.
	 */
	void stopViewer();

	/**
	 * \fn	void PCLViewer::useColoredCloud(bool useColored);
	 *
	 * \brief	Specifies whether the colored or non-colored clouds are rendered.
	 *
	 * \param	useColored	true if use colored.
	 */
	void useColoredCloud(bool useColored);


	/**
	 * \fn	void PCLViewer::setNumOfClouds(int numOfClouds);
	 *
	 * \brief	Sets number of clouds to be rendered/expected and creates the 
	 * 			viewports if necessary.
	 *
	 * \param	numOfClouds	Number of clouds.
	 */
	void setNumOfClouds(int numOfClouds);
private:

	/**
	 * \fn	void PCLViewer::updateLoop();
	 *
	 * \brief	Update loop of the viewer. This method runs in the PCLViewer thread. It creates
	 * 			the PCLVisualizer and updates the buffered point clouds and adjusts the amount of
	 * 			viewports if neccessary.
	 */
	void updateLoop();

	/**
	 * \fn	void PCLViewer::updateColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);
	 *
	 * \brief	Updates the colored cloud. Used to dynamically switching between colored and non-colored clouds
	 * 			by using the signal/slot pattern.
	 *
	 * \param	cloudIndex	Zero-based index of the cloud to be updated.
	 * \param	cloudID   	Identifier for the cloud.
	 * \param	viewer	  	The viewer in which the cloud should be updated.
	 */
	void updateColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);

	/**
	 * \fn	void PCLViewer::updateNonColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);
	 *
	 * \brief	Updates the non-colored cloud. Used to dynamically switching between colored and non-colored clouds
	 * 			by using the signal/slot pattern.
	 *
	 * \param	cloudIndex	Zero-based index of the cloud to be updated.
	 * \param	cloudID   	Identifier for the cloud.
	 * \param	viewer	  	The viewer in which the cloud should be updated.
	 */
	void updateNonColoredCloud(int cloudIndex, std::string cloudID, pcl::visualization::PCLVisualizer::Ptr viewer);

	/**
	 * \fn	void PCLViewer::createViewPortsForViewer(pcl::visualization::PCLVisualizer::Ptr viewer);
	 *
	 * \brief	Creates view ports for viewer and sets a standard background for each viewport.
	 *
	 * \param	viewer	The viewer for which the viewports are created.
	 */
	void createViewPortsForViewer(pcl::visualization::PCLVisualizer::Ptr viewer);

	/**
	 * \fn	void PCLViewer::matchPointCloudsToViewPorts(pcl::visualization::PCLVisualizer::Ptr viewer);
	 *
	 * \brief	Match point clouds to view ports. The first time clouds have to be added to the viewer.
	 * 			Afterwards, they can be updated using the updateNonColoredCloud/updateColoredCloud
	 * 			methods.
	 *
	 * \param	viewer	The viewer for which the current buffered point clouds are added to.
	 */
	void matchPointCloudsToViewPorts(pcl::visualization::PCLVisualizer::Ptr viewer);


	/** \brief	signal connected with the according cloud-updater method. */
	boost::signals2::signal<void(int cloudIndex, std::string, pcl::visualization::PCLVisualizer::Ptr)> updateCurrentCloudWithIndexAndIdentifier;


	/** \brief	true if the PCLViewer is currently s running. */
	bool m_isRunning;


	/** \brief	Name of the viewer. */
	std::string m_viewerName;


	/** \brief	The buffered colored clouds. */
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>		m_coloredClouds;


	/** \brief	The buffered non colored clouds. */
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>		m_nonColoredClouds;


	/** \brief	true if amount of view ports has configuration changed. */
	bool m_viewPortConfigurationChanged;


	/** \brief	The view port configuration changed mutex. */
	std::mutex m_viewPortConfigurationChangedMutex;


	
	/** \brief	The view port identifier. */
	std::vector<int> m_viewPorts;


	/** \brief	The cloud identifier. */
	std::vector<std::string> m_cloudIDs;


	/** \brief	The update thread. */
	std::thread m_updateThread;


	/** \brief	The cloud buffer-mutex. */
	std::mutex m_cloudMutex;


	/** \brief	Number of shown clouds. */
	int m_cloudCount;

	/** \brief	The use colored cloud mutex. */
	std::mutex m_useColoredCloudMutex;

	/** \brief	true to render colored cloud. */
	bool m_useColoredCloud;

	/** \brief	true if clouds in the buffere were updated. */
	bool m_cloudsUpdated;

	/** \brief	The buffered clouds updated condition variable. */
	std::condition_variable m_cloudUpdate;
};

