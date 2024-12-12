#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <limits>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloudT;
typedef pcl::PointCloud<pcl::SHOT352> SHOTCloudT;

class Cole {
public:
    double ComputeResolution(const PointCloud::ConstPtr& cloud);
    void UniformScale(float originResolution, PointCloud::Ptr cloud, float targetResolution);
    PointCloud::Ptr DownsamplePointCloud(const PointCloud::Ptr& input_cloud, float leaf_size);


};
