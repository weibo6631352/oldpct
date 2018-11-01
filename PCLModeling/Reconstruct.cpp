#include "Reconstruct.h"
#include <QRendView.h>
#include "dvprint.h"

//#define UseCGAL
#define UsePCL
// #define UseLVR
// #define UseMESHLAB

void Reconstruct::BuildSceneMesh(PointCloudT::Ptr cloud)
{
#if defined UseCGAL
    CGALReconstruct(cloud);
#elif defined UsePCL
    // PCLPoissonReconstruct(cloud); //松柏重建
    // PclCubeReconstruct(cloud); //移动立方体重建
    PclGp3Reconstruct(cloud); //贪婪三角建模
#elif defined UseLVR
    LVReconstruct(cloud);
#elif defined UseMESHLAB
    MESHLABeconstruct(cloud);
#endif //

}

#ifdef UseCGAL
#include <CGAL/trace.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/facets_in_complex_2_to_triangle_mesh.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/pca_estimate_normals.h>
#include <vector>
#include <CGAL/IO/STL_writer.h>
#include <fstream>
#include <CGAL\Eigen_solver_traits.h>
#include <Eigen\src\IterativeLinearSolvers\ConjugateGradient.h>
#include <CGAL\Eigen_matrix.h>
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <ostream>
// Types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Point_with_normal_3<Kernel> Point_with_normal;
typedef Kernel::Sphere_3 Sphere;
typedef std::vector<Point_with_normal> PointList;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;
typedef Kernel::Vector_3 Vector;

// Point with normal vector stored in a std::pair.
// typedef std::pair<Point, Vector> PointVectorPair;
// typedef std::vector<PointVectorPair> PointList;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

void mesh2stl(Polyhedron& P)
{
    const char* oname = "out";
    boost::shared_ptr<std::ofstream> p_out(new std::ofstream(oname));
    *p_out << "solid " << oname << std::endl;
    auto vi = P.vertices_begin();
    Point p = vi->point();
    double minx = p.x();
    double miny = p.y();
    double minz = p.z();
    for (; vi != P.vertices_end(); ++vi) {
        p = vi->point();
        if (p.x() < minx)
            minx = p.x();
        if (p.y() < miny)
            miny = p.y();
        if (p.z() < minz)
            minz = p.z();
    }
    // translate into positive octant
    Vector trans(-minx, -miny, -minz);
    for (auto i = P.vertices_begin(); i != P.vertices_end(); ++i) {
        i->point() = i->point() + trans;
    }
    // write triangles
    for (auto i = P.facets_begin(); i != P.facets_end(); ++i) {
        auto h = i->halfedge();
        if (h->next()->next()->next() != h) {
            return;
        }
        Point p = h->vertex()->point();
        Point q = h->next()->vertex()->point();
        Point r = h->next()->next()->vertex()->point();
        // compute normal
        Vector n = CGAL::cross_product(q - p, r - p);
        Vector norm = n / std::sqrt(n * n);
        *p_out << "    facet normal " << norm << std::endl;
        *p_out << "      outer loop " << std::endl;
        *p_out << "        vertex " << p << std::endl;
        *p_out << "        vertex " << q << std::endl;
        *p_out << "        vertex " << r << std::endl;
        *p_out << "      endloop " << std::endl;
        *p_out << "    endfacet " << std::endl;
    }

    *p_out << "endsolid " << oname << std::endl;
    p_out->flush();
    p_out->close();
}
void Reconstruct::CGALReconstruct(PointCloudT::Ptr cloud)
{
    unsigned int nb_neighbors_pca_normals = 16;
    // Poisson options
    FT sm_angle = 20.0; // Min triangle angle in degrees.
    FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
    FT sm_distance = 0.5; // Surface Approximation error w.r.t. point set average spacing.
    // Reads the point set file in points[].
    // Note: read_xyz_points_and_normals() requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    PointList points;
    for (int i = 0; i < cloud->size(); ++i)
    {
        PointT &pt = cloud->at(i);
        Point_with_normal cgpt;
        Vector v3(pt.x, pt.y, pt.z);
        cgpt += v3;
        points.push_back(cgpt);
    }
    CGAL::pca_estimate_normals<Concurrency_tag>(points,
        nb_neighbors_pca_normals,
        CGAL::parameters::normal_map(CGAL::make_normal_of_point_with_normal_map(PointList::value_type())));


    //     std::ifstream stream("data/kitten.xyz");
    //     if (!stream ||
    //         !CGAL::read_xyz_points(
    //         stream,
    //         std::back_inserter(points),
    //         CGAL::parameters::normal_map(CGAL::make_normal_of_point_with_normal_map(PointList::value_type()))))
    //     {
    //         std::cerr << "Error: cannot read file data/kitten.xyz" << std::endl;
    //         return ;
    //     }

    // Creates implicit function from the read points using the default solver.
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    Poisson_reconstruction_function function(points.begin(), points.end(),
        CGAL::make_normal_of_point_with_normal_map(PointList::value_type()));
    // Computes the Poisson indicator function f()
    // at each vertex of the triangulation.
    if (!function.compute_implicit_function())
        return;
    // Computes average spacing
    FT average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 16 /* knn = 1 ring */);
    // Gets one point inside the implicit surface
    // and computes implicit function bounding sphere radius.
    Point inner_point = function.get_inner_point();
    Sphere bsphere = function.bounding_sphere();
    FT radius = std::sqrt(bsphere.squared_radius());
    // Defines the implicit surface: requires defining a
    // conservative bounding sphere centered at inner point.
    FT sm_sphere_radius = 5.0 * radius;
    
    
    FT sm_dichotomy_error = sm_distance*average_spacing / 1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
        Sphere(inner_point, sm_sphere_radius*sm_sphere_radius),
        sm_dichotomy_error / sm_sphere_radius);
    dd("2");
    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
        sm_radius*average_spacing,  // Max triangle size
        sm_distance*average_spacing); // Approximation error
    // Generates surface mesh with manifold option
    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
        surface,                              // implicit surface
        criteria,                             // meshing criteria
        CGAL::Manifold_with_boundary_tag());  // require manifold mesh
    dd("3");
    if (tr.number_of_vertices() == 0)
        return;
    // saves reconstructed surface mesh
    std::ofstream out("kitten_poisson-20-30-0.375.off");
    CGAL::set_binary_mode(out);
    Polyhedron output_mesh;
    dd("4");
    CGAL::facets_in_complex_2_to_triangle_mesh(c2t3, output_mesh);
    dd("5");
    CGAL::write_STL<Polyhedron>(output_mesh, out);
    //mesh2stl(output_mesh);
    dd("6");
    out << output_mesh;
    // computes the approximation error of the reconstruction
    double max_dist =
        CGAL::Polygon_mesh_processing::approximate_max_distance_to_point_set(output_mesh,
        points,
        4000);
    std::cout << "Max distance to point_set: " << max_dist << std::endl;
}
#endif //UseCGAL


#ifdef UsePCL
//点的类型的头文件
#include <pcl/point_types.h>
//点云文件IO（pcd文件和ply文件）
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//kd树
#include <pcl/kdtree/kdtree_flann.h>
//特征提取
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
//重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
//可视化
#include <pcl/visualization/pcl_visualizer.h>
//多线程
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
void Reconstruct::PCLPoissonReconstruct(PointCloudT::Ptr color_cloud)
{
    if (!color_cloud)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(color_cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
    {
        cloud->at(i).x = color_cloud->at(i).x;
        cloud->at(i).y = color_cloud->at(i).y;
        cloud->at(i).z = color_cloud->at(i).z;
    }
    // 计算法向量
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //法向量点云对象指针
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(16);
    n.compute(*normals); //计算法线，结果存储在normals中

    //将点云和法线放到一起
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    //创建Poisson对象，并设置参数
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
    pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
    //pn.setIndices();

    //设置搜索方法和输入点云
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;
    //执行重构
    pn.performReconstruction(mesh);

    //保存网格图
    pcl::io::savePLYFile("result.ply", mesh);

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCLPoissonReconstruct"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);
    viewer->addPolygonMesh(mesh, "my");
    viewer->addCoordinateSystem(50.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void Reconstruct::PclCubeReconstruct(PointCloudT::Ptr color_cloud)
{
    if (!color_cloud)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(color_cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
    {
        cloud->at(i).x = color_cloud->at(i).x;
        cloud->at(i).y = color_cloud->at(i).y;
        cloud->at(i).z = color_cloud->at(i).z;
    }
    // 估计法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(16);
    n.compute(*normals); //计算法线，结果存储在normals中
    //* normals 不能同时包含点的法向量和表面的曲率

    //将点云和法线放到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals


    //创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    //初始化MarchingCubes对象，并设置参数
    pcl::MarchingCubes<pcl::PointNormal> *mc;
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal>();
    /*
    if (hoppe_or_rbf == 0)
    mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
    else
    {
    mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
    (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
    }
    */

    //创建多变形网格，用于存储结果
    pcl::PolygonMesh mesh;

    //设置MarchingCubes对象的参数
    mc->setIsoLevel(0.0f);
    mc->setGridResolution(100, 100, 100);
    mc->setPercentageExtendGrid(0.0f);

    //设置搜索方法
    mc->setInputCloud(cloud_with_normals);

    //执行重构，结果保存在mesh中
    mc->reconstruct(mesh);

    //保存网格图
    pcl::io::savePLYFile("result.ply", mesh);

    // 显示结果图
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cube"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5); //设置背景
    viewer->addPolygonMesh(mesh, "my"); //设置显示的网格
    viewer->addCoordinateSystem(1.0); //设置坐标系
    viewer->initCameraParameters();
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}

#include <pcl/surface/gp3.h>
void Reconstruct::PclGp3Reconstruct(PointCloudT::Ptr color_cloud)
{
    if (!color_cloud)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(color_cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
    {
        cloud->at(i).x = color_cloud->at(i).x;
        cloud->at(i).y = color_cloud->at(i).y;
        cloud->at(i).z = color_cloud->at(i).z;
    }


    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(16);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(3);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("triangles"));
    viewer->setBackgroundColor(0.5, 0.5, 0.5);

    viewer->addPolygonMesh(triangles, "triangles");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

#endif