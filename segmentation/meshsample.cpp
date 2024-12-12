//#include <iostream>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//using namespace std;
//using namespace pcl;
//int main()
//{
//	/*+++++++++++++++++++++++++单视角点云获取+++++++++++++++++++++++++++++++*/
//	//读取CAD模型
//	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
//	reader->SetFileName("D://wy-project//3Dconstruction//data//Models for Test 20241128 HLW(1)//TestModel4_TurboEngine.stl");
//	reader->Update();
//	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
//	polydata = reader->GetOutput();
//	polydata->GetNumberOfPoints();
//
//	//***单视角点云获取
//	//主要是renderViewTesselatedSphere的参数设定
//	//输入
//	float resx = 256;   //显示视点图窗的X大小  分辨率，值多大，采集的点越多
//	float resy = resx;  //显示视点图窗的Y大小
//	std::vector<pcl::PointCloud<pcl::PointXYZ>, \
//		Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > views_xyz;// 各视点点云对应的XYZ信息
//
//	//输出
//	std::vector<Eigen::Matrix4f, \
//		Eigen::aligned_allocator<Eigen::Matrix4f> > poses;// 从目标坐标变换到视点相机坐标
//	std::vector<float> entropies;//0-1之间，视点看到模型的百分比
//
//	//输入
//	int tesselation_level = 1;//表示在角度下的细分数
//	float view_angle = 90;//虚拟相机的视场
//	float radius_sphere = 1;//radius_sphere半径
//	bool use_vertices = FALSE;//是否采用tessellated icosahedron 的vertices
//
//
//	//PCLVisualizer 显示
//	pcl::visualization::PCLVisualizer vis;
//	vis.addModelFromPolyData(polydata, "mesh", 0);
//	vis.setRepresentationToSurfaceForAllActors();
//	vis.renderViewTesselatedSphere(resx, resy, views_xyz, poses, entropies, \
//		tesselation_level, view_angle, radius_sphere, use_vertices);//显示个角度点云
//
//	//保存
//	for (int i = 0; i < views_xyz.size(); i++)
//	{
//		pcl::PointCloud<pcl::PointXYZ> views_cloud;
//		pcl::transformPointCloud<pcl::PointXYZ>(views_xyz[i]/*输入点云*/, views_cloud/*输出点云*/, poses[i]/*刚性变换*/);
//
//		std::stringstream ss;
//		ss << "cloud_view_" << i << ".ply";
//		pcl::io::savePLYFile(ss.str(), views_cloud);
//	}
//
//	//显示原STL文件
//	while (!vis.wasStopped())
//	{
//		vis.spinOnce();
//	}
//}