#define _USE_MATH_DEFINES  // 让 MSVC 支持 M_PI
#include <cmath>
#include <open3d/Open3D.h>
#include <iostream>
#include <vector>

using namespace open3d;
using namespace open3d::geometry;
using namespace open3d::pipelines::registration;

double ComputeResolution(const std::shared_ptr<open3d::geometry::PointCloud>& cloud) {
    if (cloud->points_.empty()) return 0.0;

    double total_dist = 0.0;
    int count = 0;

    open3d::geometry::KDTreeFlann kdtree(*cloud);
    for (const auto& point : cloud->points_) {
        std::vector<int> indices;
        std::vector<double> distances;
        if (kdtree.SearchKNN(point, 2, indices, distances) > 1) {
            total_dist += sqrt(distances[1]);
            count++;
        }
    }
    return (count > 0) ? (total_dist / count) : 0.0;
}

void VisualizeRegistration(const open3d::geometry::PointCloud& source,
    const open3d::geometry::PointCloud& target,
    const Eigen::Matrix4d& transformation) {

    auto source_transformed = std::make_shared<open3d::geometry::PointCloud>(source);
    source_transformed->Transform(transformation);

    source_transformed->PaintUniformColor(Eigen::Vector3d(0, 0, 1)); // 蓝色
    auto target_copy = std::make_shared<open3d::geometry::PointCloud>(target);
    target_copy->PaintUniformColor(Eigen::Vector3d(1, 0, 0)); // 红色

    open3d::visualization::DrawGeometries({ source_transformed, target_copy }, "Registration Result");
}

Eigen::Matrix4d ComputeFGRRegistration(const std::shared_ptr<PointCloud>& source_down, const std::shared_ptr<PointCloud>& target_down, float search_radius) {
    float distance_threshold = search_radius * 1.5;
    int max_iterations = 64;
    int max_tuples = 1000;

    std::cout << "compute FPFH description" << std::endl;
    std::shared_ptr<Feature> source_fpfh, target_fpfh;
    source_down->EstimateNormals(KDTreeSearchParamHybrid(2 * search_radius, 30));
    source_fpfh = ComputeFPFHFeature(*source_down, KDTreeSearchParamHybrid(5 * search_radius, 100));
    target_down->EstimateNormals(KDTreeSearchParamHybrid(2 * search_radius, 30));
    target_fpfh = ComputeFPFHFeature(*target_down, KDTreeSearchParamHybrid(5 * search_radius, 100));

    std::cout << "FGR registration" << std::endl;
    RegistrationResult registration_result = FastGlobalRegistrationBasedOnFeatureMatching(
        *source_down, *target_down, *source_fpfh, *target_fpfh,
        FastGlobalRegistrationOption(
            1.4, true, true, distance_threshold, max_iterations, 0.95, max_tuples));

    return registration_result.transformation_;
}

std::shared_ptr<PointCloud> Ransac_corr(const std::shared_ptr<PointCloud>& cloud_src, const std::shared_ptr<PointCloud>& cloud_tgt) {
    auto kd_tree = std::make_shared<geometry::KDTreeFlann>(*cloud_tgt);
    std::vector<std::pair<int, int>> correspondences;

    for (size_t i = 0; i < cloud_src->points_.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<double> distances(1);
        if (kd_tree->SearchKNN(cloud_src->points_[i], 1, indices, distances) > 0) {
            if (distances[0] < 7.0) {
                correspondences.emplace_back(i, indices[0]);
            }
        }
    }

    auto result = std::make_shared<PointCloud>();
    for (const auto& corr : correspondences) {
        result->points_.push_back(cloud_src->points_[corr.first]);
    }

    auto cloud_src_colored = std::make_shared<PointCloud>(*cloud_src);
    cloud_src_colored->PaintUniformColor(Eigen::Vector3d(0, 1, 0));  // 绿色
    auto cloud_tgt_colored = std::make_shared<PointCloud>(*cloud_tgt);
    cloud_tgt_colored->PaintUniformColor(Eigen::Vector3d(1, 0, 0));  // 红色

    std::shared_ptr<geometry::LineSet> line_set = std::make_shared<geometry::LineSet>();
    for (const auto& corr : correspondences) {
        Eigen::Vector3d src_pt = cloud_src->points_[corr.first];
        Eigen::Vector3d tgt_pt = cloud_tgt->points_[corr.second];
        line_set->points_.push_back(src_pt);
        line_set->points_.push_back(tgt_pt);
        line_set->lines_.push_back(Eigen::Vector2i(line_set->points_.size() - 2, line_set->points_.size() - 1));
        line_set->colors_.push_back(Eigen::Vector3d(0, 0, 1));  // 蓝色
    }

    visualization::DrawGeometries({ cloud_src_colored, cloud_tgt_colored, line_set }, "Point Cloud Correspondences");

    return result;
}

