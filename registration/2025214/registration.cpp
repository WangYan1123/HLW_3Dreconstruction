#include"registration.h"
#include "ColeProcess.h"

// ------------------------------ISS关键点提取-----------------------------
PointCloud::Ptr extractISSKeypoints(const PointCloud::Ptr& cloud, float pointcloud_distance) {
    pcl::ISSKeypoint3D<PointT, PointT> issDetector;
    issDetector.setSearchMethod(pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>()));
    issDetector.setSalientRadius(6 * pointcloud_distance);   // 设置用于计算协方差矩阵的球邻域半径
    issDetector.setNonMaxRadius(4 * pointcloud_distance);     // 设置非极大值抑制应用算法的半径
    issDetector.setThreshold21(0.975);               // 设定第二个和第一个特征值之比的上限
    issDetector.setThreshold32(0.975);       // 设定第三个和第二个特征值之比的上限
    issDetector.setMinNeighbors(4);                    // 在应用非极大值抑制算法时，设置必须找到的最小邻居数
    issDetector.setInputCloud(cloud);
    PointCloud::Ptr keypoints(new PointCloud());
    issDetector.compute(*keypoints);
    return keypoints;
}

void cloudEdgeDetection_3D(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_in,
    int k_neighbors, float distance_threshold,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_Edges, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_Inliers)
{
    cloud_Edges->clear();
    cloud_Inliers->clear();
    pcl::PointIndices::Ptr Edges_indices(new pcl::PointIndices());
    pcl::PointIndices::Ptr Inliers_indices(new pcl::PointIndices());
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_in);
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        pcl::PointXYZ search_point = cloud_in->points[i];
        // 搜索邻域点
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        if (kdtree.nearestKSearch(search_point, k_neighbors, point_indices, point_distances) > 0) {
            Eigen::Vector3f sum_neighbors(0.0f, 0.0f, 0.0f);
            for (const auto& idx : point_indices) {
                const auto& neighbor = cloud_in->points[idx];
                sum_neighbors += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z);
            }
            Eigen::Vector3f k_norm = sum_neighbors / point_indices.size();
            float distance = (Eigen::Vector3f(search_point.x, search_point.y, search_point.z) - k_norm).norm();

            // 判断是否为边缘点
            if (distance > distance_threshold) {
                Edges_indices->indices.push_back(static_cast<int>(i));  
                cloud_Edges->points.push_back(search_point); 
            }
            else {
                Inliers_indices->indices.push_back(static_cast<int>(i)); 
                cloud_Inliers->points.push_back(search_point); 
            }
        }
    }
}

PointCloud::Ptr mergePointClouds(PointCloud::Ptr cloud1, PointCloud::Ptr cloud2) {
    pcl::PointCloud< PointT>::Ptr merged_cloud(new pcl::PointCloud< PointT>);
    *merged_cloud = *cloud1;
    *merged_cloud += *cloud2;
    return merged_cloud;
}

PointCloud::Ptr extract_keypoints(const PointCloud::Ptr& cloud, float pointcloud_Resolution) {
    PointCloud::Ptr scene_Edges_test = PointCloud::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    PointCloud::Ptr scene_Inliers_test = PointCloud::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloudEdgeDetection_3D(cloud, 50, pointcloud_Resolution, scene_Edges_test, scene_Inliers_test);
    return  scene_Edges_test;
}

NormalCloudT::Ptr computeNormals(const PointCloud::Ptr& input_cloud, float radius_search) {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius_search);
    NormalCloudT::Ptr normals(new NormalCloudT);
    ne.compute(*normals);
    return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(const PointCloud::Ptr& input_cloud,
    const NormalCloudT::Ptr& input_normals,
    float radius_search) {
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(input_cloud);
    fpfh.setInputNormals(input_normals);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(radius_search);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.compute(*fpfhs);
    return fpfhs;
}

pcl::PointCloud<pcl::ReferenceFrame>::Ptr computeLRF(PointCloud::Ptr cloud, NormalCloudT::Ptr normals, double radius) {
    pcl::SHOTLocalReferenceFrameEstimation<PointT, pcl::ReferenceFrame> lrf_estimation;
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf(new pcl::PointCloud<pcl::ReferenceFrame>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    lrf_estimation.setRadiusSearch(radius);
    lrf_estimation.setInputCloud(cloud);
    lrf_estimation.setSearchMethod(tree);
    lrf_estimation.compute(*lrf);
    return lrf;
}

SHOTCloudT::Ptr computeSHOTFeatures(PointCloud::Ptr cloud, NormalCloudT::Ptr normals, pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, double radius) {
    pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352> shot;
    SHOTCloudT::Ptr shot_features(new SHOTCloudT);
    shot.setNumberOfThreads(4);
    shot.setInputCloud(cloud);
    shot.setInputNormals(normals);
    shot.setRadiusSearch(radius);
    shot.setInputReferenceFrames(lrf);
    shot.compute(*shot_features);
    return shot_features;
}
 
CloudProcessResult load_pcloud(PointCloud::Ptr cloud) {
    CloudProcessResult Cloud;

    Cloud.resolution = Cole::ComputeResolution(cloud);
    std::cout << "load pt resolution: " << Cloud.resolution << std::endl;

    Cloud.keypoints = extract_keypoints(cloud, Cloud.resolution);

    std::cout << "computeNormals & FPFH" << std::endl;
    NormalCloudT::Ptr cloud_src_normals = computeNormals(Cloud.keypoints, Cloud.resolution * 10);
    Cloud.fpfh = computeFPFH(Cloud.keypoints, cloud_src_normals, Cloud.resolution * 10);

    std::cout << "computeNormals & SHOT" << std::endl;
    NormalCloudT::Ptr cloud_src_normals_SHOT = computeNormals(Cloud.keypoints, Cloud.resolution * 15);
    auto model_lrf = computeLRF(Cloud.keypoints, cloud_src_normals_SHOT, Cloud.resolution * 20);
    Cloud.shot = computeSHOTFeatures(Cloud.keypoints, cloud_src_normals_SHOT, model_lrf, Cloud.resolution * 20);

    return Cloud;
}

AlignmentResult UseICP(PointCloud::Ptr cloud_src,
    PointCloud::Ptr cloud_tgt,
    const Eigen::Matrix4f& initial_guess,
    float max_correspondence_distance,//1.0
    int maximum_iterations,//300
    double transformation_epsilon,// 1e-5
    double euclidean_fitness_epsilon //1.0
) {
    PointCloud::Ptr icp_result(new PointCloud);
    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setMaximumIterations(maximum_iterations);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    icp.align(*icp_result, initial_guess);

    Eigen::Matrix4f final_transformation = icp.getFinalTransformation();
    bool has_converged = icp.hasConverged();
    double fitness_score = icp.getFitnessScore();

    AlignmentResult result;
    result.aligned_cloud = icp_result;
    result.transformation = final_transformation;
    result.rmse = icp.getFitnessScore();
    return result;

}


PointCloud::Ptr Ransac_corr(const PointCloud::Ptr& cloud_src, const PointCloud::Ptr& cloud_tgt) {
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud_src);
    core.setInputTarget(cloud_tgt);

    boost::shared_ptr<pcl::Correspondences> correspondence_all(new pcl::Correspondences);
    core.determineCorrespondences(*correspondence_all, 7);

    PointCloud::Ptr result(new PointCloud);
    for (const auto& corr : *correspondence_all) {
        result->points.push_back(cloud_src->points[corr.index_query]);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("\u663e\u793a\u70b9\u4e91"));
    viewer->setBackgroundColor(255, 255, 255);

    //Eigen::Affine3f transform_tgt = Eigen::Affine3f::Identity();
    //transform_tgt.translation() << 100.0, 100.0, 100.0;

    //PointCloud::Ptr translated_cloud_tgt(new PointCloud);
    //pcl::transformPointCloud(*cloud_tgt, *translated_cloud_tgt, transform_tgt);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_tgt, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_tgt, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(cloud_src, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_src, input_color, "input cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input cloud");

    viewer->addCorrespondences<pcl::PointXYZ>(cloud_src, cloud_tgt, *correspondence_all, "correspondence");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return result;
}


int main()
{
    std::string depth_path_model = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel4_TurboEngine.pcd";
    std::string depth_path_scene = "D://wy-project//3Dconstruction//data//outside_point//outside_TestModel4.pcd";

    //
    cout << ">>>>>>>>>>    load scene  " << endl;
    PointCloud::Ptr cloud_scene(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(depth_path_scene, *cloud_scene) == -1) {
        PCL_ERROR("Couldn't read file %s \n", depth_path_scene.c_str());
    }
    float oriSceneR = Cole::ComputeResolution(cloud_scene);
    cout << "oriSceneR: " << oriSceneR << endl;

    cout << endl << ">>>>>>>>>>    load model  " << endl;
    PointCloud::Ptr cloud_model(new PointCloud);
    if (pcl::io::loadPCDFile<PointT>(depth_path_model, *cloud_model) == -1) {
        PCL_ERROR("Couldn't read file %s \n", depth_path_model.c_str());
    }
    float oriModelR = Cole::ComputeResolution(cloud_model);
    cout << "oriModelR: " << oriModelR << endl;

    //统一分辨率
    double target_res = std::max(oriSceneR, oriModelR);
    PointCloud::Ptr cloud_scene_downsample = Cole::DownsamplePointCloud(cloud_scene, target_res*10);
    cout << "cloud_scene_downsample size: " << cloud_scene_downsample->size() << endl;

    PointCloud::Ptr cloud_model_downsample = Cole::DownsamplePointCloud(cloud_model, target_res*10);
    cout << "cloud_model_downsample size: " << cloud_model_downsample->size() << endl;


    //特征计算
    CloudProcessResult scene = load_pcloud(cloud_scene_downsample);
    Cole::visualizePointCloud(scene.keypoints, "scene keypoints");
    cout << "scene keypoints size: " <<scene.keypoints->size() << endl;

    vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_scene_downsample, *cloud_scene_downsample, indices);
    Cole::visualizePointCloud(cloud_scene_downsample,"cloud_scene_downsample");

    CloudProcessResult model = load_pcloud(cloud_model_downsample);
    Cole::visualizePointCloud(model.keypoints, "model keypoints");
    cout << "model keypoints size: " << model.keypoints->size() << endl;
    Cole::visualizePointCloud(cloud_model_downsample, "cloud_model_downsample");

    if (!model.keypoints || model.keypoints->empty()) {
        std::cerr << "model.keypoints is null or empty!" << std::endl;
    }
    else {
        model.keypoints->width = model.keypoints->points.size();
        model.keypoints->height = 1;
        if (pcl::io::savePCDFileASCII("model51.pcd", *model.keypoints) == 0) {
            std::cout << "Successfully saved model51.pcd (" << model.keypoints->points.size() << " points)." << std::endl;
        }
        else {
            std::cerr << "Failed to save model51.pcd" << std::endl;
        }
    }

    if (!scene.keypoints || scene.keypoints->empty()) {
        std::cerr << "scene.keypoints is null or empty!" << std::endl;
    }
    else {
        scene.keypoints->width = scene.keypoints->points.size();
        scene.keypoints->height = 1;
        if (pcl::io::savePCDFileASCII("model52.pcd", *scene.keypoints) == 0) {
            std::cout << "Successfully saved model52.pcd (" << scene.keypoints->points.size() << " points)." << std::endl;
        }
        else {
            std::cerr << "Failed to save model52.pcd" << std::endl;
        }
    }


    //FPFH的sac+icp
    cout << endl<< "FPFH SAC  registration" << endl;
    AlignmentResult FPFHresult = Ransac<pcl::FPFHSignature33>(model.keypoints, scene.keypoints, model.fpfh, scene.fpfh, model.resolution, 6, 20);//1，2，20
    Cole::visualizePCCor(model.keypoints, scene.keypoints, FPFHresult.transformation,"FPFH transformation");

    cout << "rmse icp SHOT: " << FPFHresult.rmse << endl;
    cout << "ICP RESULT: " << endl << FPFHresult.transformation << endl;

    AlignmentResult ICP_FPFH = UseICP(cloud_model_downsample, cloud_scene_downsample, FPFHresult.transformation, model.resolution * 5, 100, 1e-4, 1e-4);//1.0,300,1e-5,1.0
    Cole::visualizePCCor(cloud_model, cloud_scene, ICP_FPFH.transformation, "FPFH ICP transformation");

    cout <<endl<< "rmse icp fpfh: " << ICP_FPFH.rmse << endl;
    cout << "ICP RESULT:" << endl << ICP_FPFH.transformation << endl;

    //// 最近邻点
    //PointCloud::Ptr transf(new PointCloud);
    //pcl::transformPointCloud(*cloud_model, *transf, ICP_FPFH.transformation);

    //PointCloud::Ptr transf_part1(new PointCloud);
    //PointCloud::Ptr transf_part2(new PointCloud);
    //PointCloud::Ptr transf_part3(new PointCloud);
    /*pcl::transformPointCloud(*part1, *transf_part1, ICP_FPFH.transformation);
    pcl::transformPointCloud(*part2, *transf_part2, ICP_FPFH.transformation);
    pcl::transformPointCloud(*part3, *transf_part3, ICP_FPFH.transformation);*/
    //savePointCloudToFile(transf_part1,"transf_part1.pcd");
    //cout << "successfully saved" << transf_part1->points.size() << endl;
    //savePointCloudToFile(transf_part2, "transf_part2.pcd");
    //cout << "successfully saved" << transf_part2->points.size() << endl;
    //savePointCloudToFile(transf_part3, "transf_part3.pcd");
    //cout << "successfully saved" << transf_part3->points.size() << endl;*/

    //Ransac_corr(cloud_model,cloud_scene);
    //pcl::transformPointCloud(*cloud_model, *transf, ICP_FPFH.transformation);

    //SHOT sac+icp
    AlignmentResult SHOTresult = Ransac<pcl::SHOT352>(model.keypoints, scene.keypoints, model.shot, scene.shot, model.resolution, 5, 20);//1,2,20
    Cole::visualizePCCor(model.keypoints, scene.keypoints, SHOTresult.transformation,"SHOT transformation");

    cout << "rmse icp SHOT: " << SHOTresult.rmse << endl;
    cout << "ICP RESULT: " << endl << SHOTresult.transformation << endl;

    AlignmentResult ICP_SHOT = UseICP(cloud_model_downsample, cloud_scene_downsample, SHOTresult.transformation, model.resolution * 5, 200, 1e-4, 1e-4);//1.0,300,1e-5,1.0
    Cole::visualizePCCor(cloud_model, cloud_scene, ICP_SHOT.transformation, "SHOT ICP transformation");

    cout << endl<< "rmse icp SHOT: " << ICP_SHOT.rmse << endl;
    cout << "ICP RESULT: " << endl << ICP_SHOT.transformation << endl;

    
    /*std::cout << FPFHresult.transformation << endl;
    savePointCloudToFile(FPFHresult.aligned_cloud, "cloud_fpfh_sac.pcd");
    std::cout << ICP_FPFH.transformation << endl;
    savePointCloudToFile(ICP_FPFH.aligned_cloud, "cloud_fpfh_icp.pcd");
    std::cout << SHOTresult.transformation << endl;
     
      savePointCloudToFile(SHOTresult.aligned_cloud, "cloud_shot_sac.pcd");
    cout << SHOTresult.transformation << endl;
    savePointCloudToFile(ICP_SHOT.aligned_cloud, "cloud_shot_icp.pcd");
    cout << ICP_SHOT.transformation << endl;*/
}  
