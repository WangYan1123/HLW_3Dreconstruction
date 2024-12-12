#include "ColeProcess.h"

double ComputeResolution(const PointCloud::ConstPtr& cloud) {
    size_t totalPoints = cloud->size();
    size_t sampleSize = static_cast<size_t>(totalPoints * 0.001);  // 计算要采样的点数，0.1% 的点

    if (sampleSize == 0 || totalPoints < 2) {
        return std::numeric_limits<double>::infinity();
    }

    // 随机生成样本点的索引
    std::vector<size_t> sampleIndices(sampleSize);
    std::random_device randomDevice;
    std::mt19937 generator(randomDevice());
    std::uniform_int_distribution<size_t> distribution(0, totalPoints - 1);

    for (size_t i = 0; i < sampleSize; ++i) {
        sampleIndices[i] = distribution(generator);
    }

    pcl::search::KdTree<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    double totalDistance = 0.0;
    int validPoints = 0;

    for (size_t i = 0; i < sampleSize; ++i) {
        size_t index = sampleIndices[i];

        if (!std::isfinite((*cloud)[index].x)) {
            continue;  // 跳过无效的点
        }

        std::vector<int> indices(2);
        std::vector<float> sqrDistances(2);
        int nres = kdtree.nearestKSearch(index, 2, indices, sqrDistances);

        if (nres == 2) {
            totalDistance += sqrt(sqrDistances[1]);
            ++validPoints;
        }
    }

    return (validPoints > 0) ? (totalDistance / validPoints) : std::numeric_limits<double>::infinity();
}

void UniformScale(float originResolution, PointCloud::Ptr cloud, float targetResolution) {

    if (originResolution == std::numeric_limits<double>::infinity()) {
        std::cerr << "ERROR, there are no valid points in the point cloud。" << std::endl;
        return;
    }

    float scaleFactor = targetResolution / static_cast<float>(originResolution);

    for (auto& point : cloud->points) {
        point.x *= scaleFactor;
        point.y *= scaleFactor;
        point.z *= scaleFactor;
    }
}

PointCloud::Ptr DownsamplePointCloud(const PointCloud::Ptr& input_cloud, float leaf_size) {
    PointCloud::Ptr downsampledCloud(new PointCloud);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input_cloud);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*downsampledCloud);
    return downsampledCloud;
}

void savePointCloudToXYZ(const PointCloud::Ptr& cloud, const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < cloud->size(); ++i) {
        const PointT& point = (*cloud)[i];
        file << point.x << " " << point.y << " " << point.z << "\n";
    }

    file.close();
}


int main()
{
    std::string path = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel3_BentFSS model.pcd";
    PointCloud::Ptr cloud(new PointCloud);

    if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) {
        PCL_ERROR("Couldn't read file testCloud.pcd\n");
        return -1; 
    }
    else {
        std::cout << "Successfully loaded " << cloud->points.size()
            << " points from " << path << std::endl;
    }

    float Resolution = ComputeResolution(cloud);
    std::cout << "testCloud2 Resolution:" << Resolution << std::endl;

    PointCloud::Ptr DownTest = DownsamplePointCloud(cloud, Resolution*5);
    std::cout << "cloud point size:" << DownTest->points.size() << std::endl;

    float Resolution2 = ComputeResolution(DownTest);
    std::cout << "testCloud Resolution:" << Resolution2<<std::endl;
    UniformScale(Resolution2, DownTest, 0.02);

    float Resolution3 = ComputeResolution(DownTest);
    std::cout << "testCloud Resolution:" << Resolution3 << std::endl;

    //savePointCloudToXYZ(DownTest,"TestModel1_Almond1000mm.xyz");
    pcl::io::savePCDFileASCII("TestModel3_BentFSS model_process.pcd", *DownTest);




}