#include "ProjectionXyPlane.h"
#include "QRendView.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

ProjectionXyPlane::ProjectionXyPlane()
{
}


ProjectionXyPlane::~ProjectionXyPlane()
{
}

void ProjectionXyPlane::Fun()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;      //创建投影滤波对象
	proj.setModelType(pcl::SACMODEL_PLANE);      //设置对象对应的投影模型
	proj.setInputCloud(cloud);                    //设置输入点云
	proj.setModelCoefficients(coefficients);     //设置模型对应的系数
	proj.filter(*cloud);                //执行投影滤波存储结果cloud_projected
	ins->UpdateView();
}

void ProjectionXyPlane::Fun(PointCloudT::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;      //创建投影滤波对象
    proj.setModelType(pcl::SACMODEL_PLANE);      //设置对象对应的投影模型
    proj.setInputCloud(cloud);                    //设置输入点云
    proj.setModelCoefficients(coefficients);     //设置模型对应的系数
    proj.filter(*cloud);                //执行投影滤波存储结果cloud_projected
}

