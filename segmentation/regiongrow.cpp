//#include "regiongrow.h"
//#include "construction.h"
//#include "ColeProcess.h"
//
//
//PointCloud::Ptr downsamplePointCloud(const PointCloud::Ptr& input_cloud, float leaf_size) {
//    PointCloud::Ptr downsampled_cloud(new PointCloud);
//    pcl::VoxelGrid<PointT> voxel_grid_filter;
//    voxel_grid_filter.setInputCloud(input_cloud);
//    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
//    voxel_grid_filter.filter(*downsampled_cloud);
//    return downsampled_cloud;
//}
//
//int  main(int argc, char** argv)
//{
//   /* std::string ply_file_path = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel5_B21_0516.PLY";
//
//    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//    if (pcl::io::loadPLYFile<PointT>(ply_file_path, *cloud) == -1) {
//        PCL_ERROR("Couldn't read .ply file.\n");
//        return -1;
//    }
//    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;
//
//    pcl::PointCloud<PointT>::Ptr sampled_cloud(new pcl::PointCloud<PointT>);
//    pcl::UniformSampling<PointT> uniform_sampling;
//    uniform_sampling.setInputCloud(cloud);
//    uniform_sampling.setRadiusSearch(0.001);  
//    uniform_sampling.filter(*sampled_cloud);
//
//    std::cout << "Sampling completed. Sampled cloud size: " << sampled_cloud->points.size() << std::endl;
//
//    pcl::visualization::PCLVisualizer viewer1("Sampled Cloud Viewer");
//    viewer1.addPointCloud(sampled_cloud, "sampled cloud");
//    while (!viewer1.wasStopped()) {
//        viewer1.spinOnce(100);
//    }*/
//
//    PointCloud::Ptr cloud(new PointCloud);
//    std::string path = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel2_ThreeComponents.pcd";
//    if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) {
//        PCL_ERROR("Couldn't read file %s\n", path.c_str());
//        return -1;
//    }
//    cout << "points:" << cloud->points.size() << endl;
//
//    cloud = downsamplePointCloud(cloud,1.5);
//    cout << "points:" << cloud->points.size() << endl;
//
//    std::cout << "normal estimating" << endl;
//    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod(tree);
//    normal_estimator.setInputCloud(cloud);
//    normal_estimator.setKSearch(50);
//    normal_estimator.compute(*normals);
//  
//    std::cout << "region growing " << endl;
//    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//    reg.setMinClusterSize(1);  
//    reg.setMaxClusterSize(1000000); 
//    reg.setSearchMethod(tree);    
//    reg.setNumberOfNeighbours(50);  
//    reg.setInputCloud(cloud);  
//    //reg.setIndices (indices);
//    reg.setInputNormals(normals);   
//    reg.setSmoothnessThreshold(4.5 / 180.0 * M_PI);  
//    reg.setCurvatureThreshold(2);    
//
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract(clusters);
//
//    std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
//    std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
//
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
//    pcl::visualization::CloudViewer viewer("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//    while (!viewer.wasStopped())
//    {
//    }
//
//    // Save the colored point cloud to a file
//    pcl::io::savePCDFile("colored_clusters.pcd", *colored_cloud);
//    std::cout << "Saved colored point cloud to 'colored_clusters.pcd'" << std::endl;
//
//    return (0);
//}