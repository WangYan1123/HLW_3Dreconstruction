#ifndef PPF
#define PPF

#include "HPR.h"
#include <mutex>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>

class CustomVisualizer
{
public:
	bool ready = true;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	CustomVisualizer() :viewer(pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"))) { viewer->getRenderWindow()->GlobalWarningDisplayOff(); }
	void init();
};

class DescriptorPPF
{
private:
	//IO
	std::string model_filename_;
	std::string type = "PPF";

	//Algorithm params
	double t_sampling = 0.04;
	float samp_rad;
	float norm_rad;
	double Lvoxel;
	float angle_discretization_step = 12.0f / 180.0f * float(M_PI);
	float distance_discretization_step = 0.005f;
	int scene_reference_point_sampling_rate = 10;
	int scene_referred_point_sampling_rate = 5;
	int icp_max_iter_ = 100;
	float icp_corr_distance_ = 0.01f;

	// Model 
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_sampling = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr model = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud <pcl::PointNormal>::Ptr model_keypoints_with_normals = pcl::PointCloud <pcl::PointNormal>::Ptr(new pcl::PointCloud <pcl::PointNormal>());
	
public:
	CustomVisualizer customViewer;
	bool loadModel();
};

#endif
