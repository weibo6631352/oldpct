#include "QQuyuzengzhangColor.h"
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/features/normal_3d_omp.h>
#include "QRendView.h"
#include "PCManage.h"
#include "dvprint.h"

QQuyuzengzhangColor::QQuyuzengzhangColor(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);
	ui.lineEdit->setValidator(new QDoubleValidator(this));
	ui.lineEdit_2->setValidator(new QDoubleValidator(this));
	ui.lineEdit_3->setValidator(new QDoubleValidator(this));
	ui.lineEdit_4->setValidator(new QIntValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit->setText(QString::fromLocal8Bit(  pt.get<std::string>("区域增长color.距离阀值").c_str()));
	ui.lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长color.点色差阀值").c_str()));
	ui.lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长color.类色差阀值").c_str()));
	ui.lineEdit_4->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长color.最小聚类数量").c_str()));
}

QQuyuzengzhangColor::~QQuyuzengzhangColor()
{
}

void QQuyuzengzhangColor::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;

	std::string       distanceThreshold = ui.lineEdit->text().toLocal8Bit().data();
	std::string       pointColorThreshold = ui.lineEdit_2->text().toLocal8Bit().data();
	std::string       regionColorThreshold = ui.lineEdit_3->text().toLocal8Bit().data();
	std::string       minClusterSize = ui.lineEdit_4->text().toLocal8Bit().data();

	pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	//	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::RegionGrowingRGB<PointT> reg;//创建基于颜色的区域生长分割类的对象
	reg.setInputCloud(cloud);//设置分割原始点云
	//	reg.setIndices(indices);//设置分割点云序列索引
	reg.setSearchMethod(tree);//设置搜索方法，最近临搜索
	reg.setDistanceThreshold(/*10*/atof(distanceThreshold.c_str()));//设置距离阈值，小于该值的视为邻域点
	reg.setPointColorThreshold(/*8*/atof(pointColorThreshold.c_str()));//设置点之间的色差阈值，小于该值的视为一个聚类
	reg.setRegionColorThreshold(/*15*/atof(regionColorThreshold.c_str()));//设置聚类之间的色差阈值，小于该值的应用合并算法，合并为同一个聚类
	reg.setMinClusterSize(/*200*/atof(minClusterSize.c_str()));//设置聚类中点的数量下限，如果点数量少于该值，应用合并算法，合并到最近临的一个聚类

// 	std::vector <pcl::PointIndices> clusters;
// 	reg.extract(clusters);//应用分割算法，提取聚类
	


	reg.extract(PCManage::ins().jlClusters_);
    PointCloudT::Ptr colored_cloud = reg.getColoredCloud();//对聚类随机赋一颜色，不同的颜色为不同的聚类
	pcl::copyPointCloud(*colored_cloud, *cloud);


	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("区域增长color.距离阀值", distanceThreshold);
	pt.put<std::string>("区域增长color.点色差阀值", pointColorThreshold);
	pt.put<std::string>("区域增长color.类色差阀值", regionColorThreshold);
	pt.put<std::string>("区域增长color.最小聚类数量", minClusterSize);
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  

	ins->UpdateView();
	this->accept();
}

void QQuyuzengzhangColor::Fun(PointCloudT::Ptr cloud, PointCloudT::Ptr colorCloud /*= nullptr*/)
{
	if (!cloud)
	{
		cloud = PCManage::ins().cloud_;
	}

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	float       distanceThreshold = pt.get<float>("区域增长color.距离阀值");
	float       pointColorThreshold = pt.get<float>("区域增长color.点色差阀值");
	float       regionColorThreshold = pt.get<float>("区域增长color.类色差阀值");
	float       minClusterSize = pt.get<float>("区域增长color.最小聚类数量");

	pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	//	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::RegionGrowingRGB<PointT> reg;//创建基于颜色的区域生长分割类的对象
	reg.setInputCloud(cloud);//设置分割原始点云
	//	reg.setIndices(indices);//设置分割点云序列索引
	reg.setSearchMethod(tree);//设置搜索方法，最近临搜索
	reg.setDistanceThreshold(/*10*/distanceThreshold);//设置距离阈值，小于该值的视为邻域点
	reg.setPointColorThreshold(/*8*/pointColorThreshold);//设置点之间的色差阈值，小于该值的视为一个聚类
	reg.setRegionColorThreshold(/*15*/regionColorThreshold);//设置聚类之间的色差阈值，小于该值的应用合并算法，合并为同一个聚类
	reg.setMinClusterSize(/*200*/minClusterSize);//设置聚类中点的数量下限，如果点数量少于该值，应用合并算法，合并到最近临的一个聚类

// 	std::vector <pcl::PointIndices> clusters;
// 	reg.extract(clusters);//应用分割算法，提取聚类
	

	reg.extract(PCManage::ins().jlClusters_);

    if (colorCloud)
    {
        PointCloudT::Ptr colored_cloud = reg.getColoredCloud();//对聚类随机赋一颜色，不同的颜色为不同的聚类
        pcl::copyPointCloud(*colored_cloud, *colorCloud);
    }
//     else
//     {
//         PointCloudT::Ptr colored_cloud = reg.getColoredCloud();//对聚类随机赋一颜色，不同的颜色为不同的聚类
//         pcl::copyPointCloud(*colored_cloud, *cloud);
//     }
	
}