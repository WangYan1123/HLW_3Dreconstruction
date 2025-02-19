#include <registration.h>

int main() {

    std::string path_source = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel2_ThreeComponents.pcd";
    std::string path_target = "D://wy-project//3Dconstruction//data//outside_point//outside_TestModel2.pcd";
    std::string source_part1 = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel2_ThreeComponents_part1.pcd";
    std::string source_part2 = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel2_ThreeComponents_part2.pcd";
    std::string source_part3 = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel2_ThreeComponents_part3.pcd";

    //  -----------------------------加载场景点云 -----------------------------
    std::cout << ">>>>>>>>>>    load source  " << std::endl;
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(path_source, *source)) {
        std::cerr << "Failed to load source point cloud: " << path_source << std::endl;
        return -1;
    }
    double oriSourceR = ComputeResolution(source);
    std::cout << "oriSceneR: " << oriSourceR << std::endl;

    // ----------------------------- 加载模型点云 -------------------------------------
    std::cout << std::endl << ">>>>>>>>>>    load model  " << std::endl;
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(path_target, *target)) {
        std::cerr << "Failed to load model point cloud: " << path_target << std::endl;
        return -1;
    }
    double oriModelR = ComputeResolution(target);
    std::cout << "oriModelR: " << oriModelR << std::endl;

    // ----------------------------- 加载部件点云 -------------------------------------
    auto part1 = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(source_part1, *part1)) {
        std::cerr << "Failed to load model point cloud: " << path_target << std::endl;
        return -1;
    }
   
    auto part2 = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(source_part2, *part2)) {
        std::cerr << "Failed to load model point cloud: " << path_target << std::endl;
        return -1;
    }

    auto part3 = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(source_part3, *part3)) {
        std::cerr << "Failed to load model point cloud: " << path_target << std::endl;
        return -1;
    }
  
    // -----------------------------旋转点云--------------------------------------
    Eigen::Matrix4d transInit = Eigen::Matrix4d::Identity();
    double theta = M_PI / 2;
    transInit(0, 0) = cos(theta);
    transInit(0, 2) = sin(theta);
    transInit(2, 0) = -sin(theta);
    transInit(2, 2) = cos(theta);
    target->Transform(transInit);

    //----------------------------- 统一分辨率-----------------------------
    double target_res = std::max(oriSourceR, oriModelR) * 10;
    auto source_down = source->VoxelDownSample(target_res);
    double oriSourceR_down = ComputeResolution(source_down);
    std::cout << "downsamSourceR: " << oriSourceR_down << std::endl;
    std::cout << "cloud_scene_downsample size: " << source_down->points_.size() << std::endl;

    auto target_down = target->VoxelDownSample(target_res);
    double oriModelR_down = ComputeResolution(target_down);
    std::cout << "downsamModelR: " << oriModelR_down << std::endl;
    std::cout << "target_downsample size: " << target_down->points_.size() << std::endl;

    Eigen::Matrix4d trans = ComputeFGRRegistration(target_down, source_down, target_res);

    // ------------------------------结果可视化-----------------------------------
    VisualizeRegistration(*target, *source, trans);

    auto target_trans = std::make_shared<open3d::geometry::PointCloud>(*target);
    target_trans->Transform(trans);
    auto extractPart1 = Ransac_corr(part1, target_trans);
    auto extractPart2 = Ransac_corr(part2, target_trans);
    auto extractPart3 = Ransac_corr(part3, target_trans);

    visualization::DrawGeometries({ extractPart1 }, "extractPart1");
    visualization::DrawGeometries({ extractPart2 }, "extractPart2");
    visualization::DrawGeometries({ extractPart3 }, "extractPart3");

    return 0;
}
