#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <memory>
#include <time.h>

#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/FastGlobalRegistration.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace open3d;
typedef geometry::PointCloud PointCloudT;

typedef std::shared_ptr<PointCloudT> PointCloudTPtr;
typedef pipelines::registration::Feature FeatureT;
typedef std::shared_ptr<FeatureT> FeatureTPtr;
typedef pipelines::registration::RegistrationResult regResult;

class regO3d {
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr open3dToPcl(const PointCloudTPtr& o3dCloud);
    PointCloudTPtr pclToOpen3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static float computeCloudR(const PointCloudTPtr& cloud);
    /**
    * @brief 预处理，执行降采样，法向量估计，特征描述。
    *
    * @param imageCloud in
    * @param spaceCloud in
    * @param leaf 栅格大小
    * @return std::tuple<PointCloudTPtr,PointCloudTPtr,FeatureTPtr,FeatureTPtr> 返回智能指针
    */
    static std::tuple<PointCloudTPtr, PointCloudTPtr, FeatureTPtr, FeatureTPtr>
        preprocessPointCloud(PointCloudTPtr imageCloud, PointCloudTPtr spaceCloud, double& resolution);

    /**
   * @brief 剪枝RANSAC，消除误匹配并完成初始变换
   *
   * @param imageCloudDown input
   * @param spaceCloudDown in
   * @param imageCloud_fpfh in
   * @param spaceCloud_fpfh in
   * @param leaf 栅格大小
   * @return regResult 初始变换结果
   */
    static regResult executeGlobalRegistration(PointCloudTPtr imageCloudDown, PointCloudTPtr spaceCloudDown,
        FeatureTPtr imageCloud_fpfh, FeatureTPtr spaceCloud_fpfh,
        double& leaf);

    /**
     * @brief 使用ICP优化变换，注：在原始点云上优化
     *
     * @param imageCloud in
     * @param spaceCloud in
     * @param transInit in
     * @param leaf 栅格大小
     * @return regResult final变换结果
     */
    static regResult optimizeUsingICP(PointCloudTPtr imageCloud, PointCloudTPtr spaceCloud,
        const Eigen::Matrix4d& transInit, double& leaf);
    /**
     * @brief 点云配准主函数
     * 
     * @param transToWorld in 标定得到的相机到世界坐标系，4X4变换矩阵
     * @param transFinal out 4X4变换矩阵
     * @return int 状态值，负数是报错
     */
    static int regO3d::globalRegistrationWithICP(PointCloudTPtr oricloud, PointCloudTPtr tarcloud);
};