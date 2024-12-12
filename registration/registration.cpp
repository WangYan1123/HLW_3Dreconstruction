#include "registration.h"
#include "construction.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr open3dToPcl(const PointCloudTPtr& o3dCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclCloud->points.resize(o3dCloud->points_.size());
    for (size_t i = 0; i < o3dCloud->points_.size(); ++i) {
        pclCloud->points[i].x = o3dCloud->points_[i](0);
        pclCloud->points[i].y = o3dCloud->points_[i](1);
        pclCloud->points[i].z = o3dCloud->points_[i](2);
    }
    return pclCloud;
}

PointCloudTPtr pclToOpen3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    PointCloudTPtr open3dCloud(new PointCloudT);
    open3dCloud->points_.resize(cloud->points.size());
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        open3dCloud->points_[i](0) = cloud->points[i].x;
        open3dCloud->points_[i](1) = cloud->points[i].y;
        open3dCloud->points_[i](2) = cloud->points[i].z;
    }
    return open3dCloud;
}


float regO3d::computeCloudR(const PointCloudTPtr& cloud) {
    if (cloud->points_.empty()) {
        return 0.0f; 
    }

    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*cloud);
    std::vector<float> nearest_distances;

    for (size_t i = 0; i < cloud->points_.size(); ++i) {
        std::vector<int> indices(2);  
        std::vector<double> distances(2);
        int k = kdtree.SearchKNN(cloud->points_[i], 2, indices, distances);
        if (k > 1) {
            nearest_distances.push_back(std::sqrt(distances[1]));
        }
    }

    if (!nearest_distances.empty()) {
        std::sort(nearest_distances.begin(), nearest_distances.end());
        size_t mid = nearest_distances.size() / 2;
        return nearest_distances[mid]; // 返回中位数距离
    }
    return 0.0f; // 如果未计算到任何距离，返回 0
}


std::tuple<PointCloudTPtr, PointCloudTPtr, FeatureTPtr, FeatureTPtr>
regO3d::preprocessPointCloud(PointCloudTPtr oriCloud, PointCloudTPtr tarCloud,double& leaf) {
    visualization::DrawGeometries({oriCloud, tarCloud}, "input");
    utility::Timer time;

    std::cout << ">>> down sample ...." << std::endl;
    time.Start();
    auto oriCloudDown = oriCloud->VoxelDownSample(leaf);
    auto tarCloudDown = tarCloud->VoxelDownSample(leaf);
    time.Stop();
    std::cout << "oriCloud from " << oriCloud->points_.size() << " to " << oriCloudDown->points_.size() << std::endl;
    std::cout << "tarCloud from " << tarCloud->points_.size() << " to " << tarCloudDown->points_.size() << std::endl;
    std::cout << "down sample takes:" << time.GetDurationInMillisecond() << "ms" << std::endl;

    float resolution=regO3d::computeCloudR(oriCloudDown);

    auto oriResult = oriCloudDown->RemoveRadiusOutliers(10, resolution * 2, false);
    auto oriRadius = std::get<0>(oriResult);
    auto tarResult = tarCloudDown->RemoveRadiusOutliers(10, resolution * 2, false);
    auto tarRadius = std::get<0>(tarResult);

    std::cout << ">>> Estimating normals ...." << std::endl;
    time.Start();
    oriCloudDown->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(resolution * 2, 30));
    tarCloudDown->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(resolution * 2, 30));
    time.Stop();
    std::cout << "Estimating normals takes:" << time.GetDurationInMillisecond() << "ms" << std::endl;

    std::cout << ">>> Estimating features ...." << std::endl;
    time.Start();
    auto oriCloud_fpfh = pipelines::registration::ComputeFPFHFeature(
        *oriCloudDown, open3d::geometry::KDTreeSearchParamHybrid(resolution * 5, 100));
    auto tarCloud_fpfh = pipelines::registration::ComputeFPFHFeature(
        *tarCloudDown, open3d::geometry::KDTreeSearchParamHybrid(resolution * 5, 100));
    time.Stop();
    std::cout << "Estimating features takes:" << time.GetDurationInMillisecond() << "ms" << std::endl;
    return std::make_tuple(oriCloudDown, tarCloudDown, oriCloud_fpfh, tarCloud_fpfh);
}


void VisualizeRegistration(const open3d::geometry::PointCloud& source,
    const open3d::geometry::PointCloud& target,
    const Eigen::Matrix4d& Transformation) {
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
        new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    target_ptr->PaintUniformColor({ 1, 0, 0 });
    source_transformed_ptr->PaintUniformColor({ 0, 1, 0 });
    visualization::DrawGeometries({ source_transformed_ptr, target_ptr },
        "Registration result");
}

regResult executeFastGlobalRegistration(PointCloudTPtr imageCloudDown, PointCloudTPtr spaceCloudDown,
    FeatureTPtr imageCloud_fpfh, FeatureTPtr spaceCloud_fpfh,
    double& leaf) {
    double distanceThreshold = leaf * 8;

    regResult registration_result;
    utility::Timer time;
    std::cout << ">>> global registration ...." << std::endl;
    auto option = pipelines::registration::FastGlobalRegistrationOption();
    option.maximum_correspondence_distance_ = distanceThreshold;
    // std::cout<<imageCloudDown->points_.size()<<std::endl;
    time.Start();
    registration_result = pipelines::registration::FastGlobalRegistrationBasedOnFeatureMatching(
        *imageCloudDown, *spaceCloudDown, *imageCloud_fpfh, *spaceCloud_fpfh, option);
    time.Stop();
    std::cout << "global registration takes:" << time.GetDurationInMillisecond() << "ms" << std::endl;
    // 这里的fitness是inlier ratio (# of inlier correspondences / # of all correspondences)，
    // 反应的是特征描述的能力，真实匹配占总匹配的数量；并非评价配准
    std::cout << "fitness:" << registration_result.fitness_ << " RMSE:" << registration_result.inlier_rmse_
        << " correspondence_set size:" << registration_result.correspondence_set_.size() << std::endl;
     std::cout << registration_result.transformation_<< std::endl;
    VisualizeRegistration(*imageCloudDown, *spaceCloudDown,
                          registration_result.transformation_);
    return registration_result;
}

regResult regO3d::optimizeUsingICP(PointCloudTPtr imageCloud, PointCloudTPtr spaceCloud,
    const Eigen::Matrix4d& transInit, double& leaf) {

    utility::Timer time;
    std::cout << ">>> icp optimize ...." << std::endl;
    double distance = leaf * 3;
    time.Start();
    regResult registration_result = pipelines::registration::RegistrationICP(*imageCloud, *spaceCloud, distance, transInit,
        pipelines::registration::TransformationEstimationPointToPoint(false),
        pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 10));
    time.Stop();
    std::cout << "icp optimize takes:" << time.GetDurationInMillisecond() << "ms" << std::endl;
    std::cout << "fitness:" << registration_result.fitness_ << " RMSE:" << registration_result.inlier_rmse_
        << " correspondence_set size:" << registration_result.correspondence_set_.size() << std::endl;
    VisualizeRegistration(*imageCloud, *spaceCloud,
        registration_result.transformation_);
    return registration_result;
}

int regO3d::globalRegistrationWithICP(PointCloudTPtr oriCloud, PointCloudTPtr tarCloud) {
    PointCloudTPtr oriCloudDown, tarCloudDown;
    FeatureTPtr oriCloud_fpfh, tarCloud_fpfh; 

    double leaf = 2.5;
    std::tie(oriCloudDown, tarCloudDown, oriCloud_fpfh, tarCloud_fpfh)= regO3d::preprocessPointCloud(oriCloud, tarCloud,leaf);

    double resolution = regO3d::computeCloudR(oriCloudDown);
    std::cout << "resolution:" << resolution << std::endl;
    regResult resInit = executeFastGlobalRegistration(oriCloudDown, tarCloudDown, oriCloud_fpfh, tarCloud_fpfh, resolution);
    regResult resFinal = optimizeUsingICP(oriCloud, tarCloud, resInit.transformation_, resolution);
    std::cout << ">>> trans matrix from CT to world: " << std::endl;
    std::cout << resFinal.transformation_<< std::endl;
    return 0;
}


//int main()
//{
//    //PointCloudTPtr oriCloud(new PointCloudT), tarCloud(new PointCloudT);
//    //io::ReadPointCloud(std::string(REG_DATA_PATH) + "frontCloud.pcd", *oriCloud);
//    //io::ReadPointCloud(std::string(REG_DATA_PATH) + "sideCloud1.pcd", *tarCloud);
//    // 指定文件路径
//    std::string Path = "D://wy-project//3Dconstruction//data//deskCloud.pcd";
//    std::string modelPath = "D://wy-project//3Dconstruction//data//model.pcd";
//    // 读取点云
//    auto oriCloud = open3d::io::CreatePointCloudFromFile(Path);
//    auto tarCloud = open3d::io::CreatePointCloudFromFile(modelPath);
//    if (oriCloud == nullptr || tarCloud == nullptr) {
//        std::cerr << "Failed to read point cloud  " << std::endl;
//    }
//    else {
//        std::cout << "Successfully read point cloud  " << std::endl;
//    }
//    if (oriCloud->points_.size() == 0 || tarCloud->points_.size() == 0) return -1;
//    std::cout << "imageCloud.size():" << oriCloud->points_.size() << std::endl;
//    std::cout << "spaceCloud.size():" << tarCloud->points_.size() << std::endl;
//
//    regO3d::globalRegistrationWithICP(oriCloud, tarCloud);
//    return 0;
//}