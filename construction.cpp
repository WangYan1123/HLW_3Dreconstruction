#include "construction.h"   

// 可视化点云
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	if (cloud == nullptr || cloud->empty()) {
		std::cerr << "点云为空，无法进行可视化！" << std::endl;
		return;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("点云可视化"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(
		pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters(); 

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
}


int main()
{
	////读取pcd
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //   std::string path = "C://wy_workspace//TuYang//data//tytest_2//model//rentou//CloudCrop.pcd";
	//if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
	//	PCL_ERROR("Couldn't read file %s\n", path.c_str());
	//	return -1;
	//}


	////pcl读取ply
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::string path = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel5_B21_0516.PLY";
	//if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) == -1) {
	//	PCL_ERROR("Couldn't read file %s\n", path.c_str());
	//	return -1;
	//}

	//visualizePointCloud(cloud);

	////open3d读取PLY
	//std::shared_ptr<open3d::geometry::PointCloud> cloud = std::make_shared<open3d::geometry::PointCloud>();
	//if (!open3d::io::ReadPointCloud(path, *cloud)) {
	//	std::cerr << "无法读取 PLY 文件，请检查路径是否正确: " <<path << std::endl;
	//	return -1;
	//}
	//std::cout << "成功读取 PLY 文件: " <<path << std::endl;
	//// 可视化点云
	//open3d::visualization::Visualizer visualizer;
	//visualizer.CreateVisualizerWindow("点云可视化", 800, 600);
	//visualizer.AddGeometry(cloud);
	//visualizer.Run(); // 启动可视化窗口
	//visualizer.DestroyVisualizerWindow();

	std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();
	std::string input_ply_file = "D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel5_B21_0516.PLY";

	if (!open3d::io::ReadTriangleMesh(input_ply_file, *mesh)) {
		std::cerr << "无法读取 PLY 文件，请检查路径是否正确: " << input_ply_file << std::endl;
		return -1;
	}
	std::cout << "成功读取 PLY 文件: " << input_ply_file << std::endl;

	if (mesh->vertices_.empty()) {
		std::cerr << "网格没有顶点数据！" << std::endl;
		return -1;
	}

	size_t num_vertices = mesh->vertices_.size();
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0.0, 1.0); 

	for (size_t i = 0; i < num_vertices; ++i) {

		Eigen::Vector3d color(dis(gen), dis(gen), dis(gen));  // RGB 随机颜色值
		mesh->vertex_colors_.push_back(color);  // 为顶点赋颜色
	}

	// 可视化三角网格
	open3d::visualization::Visualizer visualizer;
	visualizer.CreateVisualizerWindow("三角网格可视化", 800, 600);
	visualizer.AddGeometry(mesh);
	visualizer.Run();  // 启动可视化窗口
	visualizer.DestroyVisualizerWindow();


	//double distance = computeCloudResolution(cloud);
	//std::cout << "distance:" << distance << std::endl;
	//// 统计滤波
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisOutlierRemoval;
	//statisOutlierRemoval.setInputCloud(cloud);
	//statisOutlierRemoval.setMeanK(10);
	//statisOutlierRemoval.setStddevMulThresh(3.0);
	//statisOutlierRemoval.filter(*cloud_filtered);
	//pcl::io::savePCDFile("cloud_filtered.pcd", *cloud_filtered);
	//std::cout << "cloud_filtered" << std::endl;

	////// 对点云重采样  
	////pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>);
	////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointXYZ>);
	////pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	////mls.setComputeNormals(true);
	////mls.setInputCloud(cloud_filtered);
	////mls.setPolynomialOrder(2);
	////mls.setSearchMethod(treeSampling);
	////mls.setSearchRadius(0.01f);
	////mls.process(*cloud_mls);
	////pcl::io::savePCDFile("cloud_mls.pcd", *cloud_mls);

	//// 法线估计
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud(cloud_filtered);
	//n.setInputCloud(cloud_filtered);
	//n.setSearchMethod(tree);
	//n.setKSearch(10);
	//n.compute(*normals);
	//std::cout << "compute normals" << std::endl;

	////连接字段
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_with_normals);

	//// 贪心投影三角化
	//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象
	//pcl::PolygonMesh triangles; //存储最终三角化的网络模型
	//gp3.setSearchRadius(distance*3);  //设置搜索时的半径，也就是KNN的球半径
	//gp3.setMu(distance *4);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	//gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100
	//gp3.setMinimumAngle(0); //设置三角化后得到的三角形内角的最小的角度
	//gp3.setMaximumAngle(M_PI); //设置三角化后得到的三角形内角的最大角度
	//gp3.setMaximumSurfaceAngle(M_PI/4); //设置某点法线方向偏离样本点法线的最大角度，如果超过，连接时不考虑该点
	//gp3.setNormalConsistency(true);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查
	//gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	//gp3.setSearchMethod(tree2);   //设置搜索方式
	//gp3.reconstruct(triangles);  //重建提取三角化

	//pcl::io::savePLYFile("bunny.ply", triangles);
	//std::cout << "PolygonMesh over" << std::endl;

	return 0;
}
