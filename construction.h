#pragma once
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>  
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <open3d/Open3D.h>

float calculateDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    float mr = 0;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    std::vector<int> pointIdx(2);
    std::vector<float> pointDst(2);
    kdtree.setInputCloud(cloud);

#pragma omp parallel for reduction(+:mr)
    for (int i = 0; i < cloud->points.size(); i++) {
        if (!std::isfinite(cloud->points[i].x)) continue;
        if (kdtree.nearestKSearch(cloud->points[i], 2, pointIdx, pointDst) == 2) {
            mr += sqrt(pointDst[1]); // 直接使用返回的平方距离
        }
    }

    return mr / cloud->points.size();
}