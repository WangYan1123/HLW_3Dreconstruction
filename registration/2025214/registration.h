#pragma once
#if defined(REG_LIBRARY)
#define REG_EXPORT __declspec(dllexport)
#else
#define REG_EXPORT __declspec(dllimport)
#endif

// 标准库头文件
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>


// 第三方库头文件
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <open3d/Open3D.h>
#include <vtkOutputWindow.h>

// PCL头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/convolution_3d.h>  // 高斯滤波

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/features/shot_omp.h>

#include <pcl/keypoints/iss_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>




using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloudT;
typedef pcl::PointCloud<pcl::SHOT352> SHOTCloudT;

using namespace Eigen;
using namespace std;

struct CloudProcessResult {
    PointCloud::Ptr keypoints;
    SHOTCloudT::Ptr shot;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh;
    float resolution;
};

struct AlignmentResult {
    PointCloud::Ptr aligned_cloud;
    float rmse;
    Eigen::Matrix4f transformation;
};

CloudProcessResult load_pcloud(PointCloud::Ptr cloud);


AlignmentResult UseICP(PointCloud::Ptr cloud_src,
    PointCloud::Ptr cloud_tgt,
    const Eigen::Matrix4f& initial_guess,
    float max_correspondence_distance,
    int maximum_iterations,
    double transformation_epsilon,
    double euclidean_fitness_epsilon);

PointCloud::Ptr Ransac_corr(const PointCloud::Ptr& cloud_src, const PointCloud::Ptr& cloud_tgt);

//模板函数的实现与声明需要在同一个头文件中
template <typename FeatureT>
AlignmentResult Ransac(const PointCloud::Ptr& cloud_src,
    const PointCloud::Ptr& cloud_tgt,
    const typename pcl::PointCloud<FeatureT>::Ptr& model_features,
    const typename pcl::PointCloud<FeatureT>::Ptr& scene_features,
    float min_sample_distance,//最小采样距离，避免过小
    int number_of_samples,//通常设置2-3，采样点数越多计算量越大，但结果可能越准确
    int correspondence_randomness //通常设置15-20，随机点的点数
)
{

    // 创建 SampleConsensusInitialAlignment 对象
    pcl::SampleConsensusInitialAlignment<PointT, PointT, FeatureT> scia;
    scia.setInputSource(cloud_src);
    scia.setInputTarget(cloud_tgt);
    scia.setSourceFeatures(model_features);
    scia.setTargetFeatures(scene_features);
    scia.setCorrespondenceRandomness(correspondence_randomness);//选择随机特征对时，使用的邻居点数越大，匹配随机性越大
    scia.setMinSampleDistance(min_sample_distance);
    scia.setNumberOfSamples(number_of_samples);//每次迭代中使用的采样点数
    // 对齐点云
    PointCloud::Ptr sac_result(new PointCloud);
    scia.align(*sac_result);



    // 返回对齐结果和变换矩阵
    AlignmentResult result;
    result.aligned_cloud = sac_result;
    result.transformation = scia.getFinalTransformation();
    return result;
}



