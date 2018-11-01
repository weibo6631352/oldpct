#include "QResampling.h"
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件
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
#include <QDoubleValidator>
#include <QRendView.h>
#include <PCManage.h>

QResampling::QResampling(QWidget *parent)
    : QSubDialogBase(parent)
{
    ui.setupUi(this);
    ui.lineEdit->setValidator(new QDoubleValidator(this));
}

QResampling::~QResampling()
{
}



void QResampling::PclMlsReconstruct(double k/* = 5*/, PointCloudT::Ptr color_cloud /*= nullptr*/)
{
    QRendView* ins = QRendView::MainRendView();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (!color_cloud)
    {
        color_cloud = PCManage::ins().cloud_;
    }
    
    cloud->resize(color_cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
    {
        cloud->at(i).x = color_cloud->at(i).x;
        cloud->at(i).y = color_cloud->at(i).y;
        cloud->at(i).z = color_cloud->at(i).z;
    }


    // 创建 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // 定义最小二乘实现的对象mls
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(k);
    mls.setUpsamplingRadius(2);

    // Reconstruct
    mls.process(mls_points);

    color_cloud->clear();
    color_cloud->resize(mls_points.size());
    for (int i = 0; i < color_cloud->size(); ++i)
    {
        color_cloud->at(i).x = mls_points.at(i).x;
        color_cloud->at(i).y = mls_points.at(i).y;
        color_cloud->at(i).z = mls_points.at(i).z;
    }

    // Save output
    //pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
    ins->UpdateView();
}

void QResampling::on_click()
{
    PclMlsReconstruct(ui.lineEdit->text().toDouble());
    accept();
}
