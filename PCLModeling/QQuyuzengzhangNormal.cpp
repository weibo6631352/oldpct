#include "QQuyuzengzhangNormal.h"
#include <pcl/search/search.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include "QRendView.h"
#include "PCManage.h"

QQuyuzengzhangNormal::QQuyuzengzhangNormal(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);
	ui.lineEdit->setValidator(new QDoubleValidator(this));
	ui.lineEdit_2->setValidator(new QIntValidator(this));
	ui.lineEdit_3->setValidator(new QIntValidator(this));
	ui.lineEdit_4->setValidator(new QIntValidator(this));
	ui.lineEdit_5->setValidator(new QDoubleValidator(this));
	ui.lineEdit_6->setValidator(new QDoubleValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>(  "区域增长normal.kdtree计算法线K值").c_str()));
	ui.lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长normal.最小簇数量"        ).c_str()));
	ui.lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长normal.最大簇数量"         ).c_str()));
	ui.lineEdit_4->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长normal.邻近数量"           ).c_str()));
	ui.lineEdit_5->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长normal.平滑阀值"           ).c_str()));
	ui.lineEdit_6->setText(QString::fromLocal8Bit(pt.get<std::string>("区域增长normal.曲率阀值"           ).c_str()));
}

QQuyuzengzhangNormal::~QQuyuzengzhangNormal()
{
}

void QQuyuzengzhangNormal::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud_ = PCManage::ins().cloud_;

	//超体聚类 参数依次是粒子距离、晶核距离、颜色容差、  
	std::string       kdK = ui.lineEdit->text().toLocal8Bit().data();
	std::string       minCount = ui.lineEdit_2->text().toLocal8Bit().data();
	std::string       maxCount = ui.lineEdit_3->text().toLocal8Bit().data();
	std::string       knum = ui.lineEdit_4->text().toLocal8Bit().data();
	std::string      smoothness = ui.lineEdit_5->text().toLocal8Bit().data();
	std::string		Curvature = ui.lineEdit_6->text().toLocal8Bit().data();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_, *cloud);


	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	//normal_estimator.setKSearch(atoi(kdK.c_str()));
	normal_estimator.setRadiusSearch(atof(kdK.c_str()));
	normal_estimator.compute(*normals);



	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(atoi(minCount.c_str()));
	reg.setMaxClusterSize(atoi(maxCount.c_str()));
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(atoi(knum.c_str()));
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(/*3.0 / 180.0 * M_PI*/atof(smoothness.c_str()));
	reg.setCurvatureThreshold(/*1*/atof(Curvature.c_str()));
    
//     std::vector <pcl::PointIndices> clusters;
//   	reg.extract(clusters);



	
	reg.extract(PCManage::ins().jlClusters_);
    PointCloudT::Ptr colored_cloud = reg.getColoredCloud();
	pcl::copyPointCloud(*colored_cloud, *cloud_);


	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("区域增长normal.kdtree计算法线K值",kdK);
	pt.put<std::string>("区域增长normal.最小簇数量"       ,minCount);
	pt.put<std::string>("区域增长normal.最大簇数量"       ,maxCount);
	pt.put<std::string>("区域增长normal.邻近数量"         , knum);
	pt.put<std::string>("区域增长normal.平滑阀值"     ,    smoothness);
	pt.put<std::string>("区域增长normal.曲率阀值"         , Curvature);
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  


	ins->UpdateView();
	this->accept();
}

void QQuyuzengzhangNormal::Fun(PointCloudT::Ptr cloud_, PointCloudT::Ptr colorCloud /*= nullptr*/)
{
	if (!cloud_)
		cloud_ = PCManage::ins().cloud_;

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	float      kdK = pt.get<float>("区域增长normal.kdtree计算法线K值");
	int       minCount = pt.get<int>("区域增长normal.最小簇数量");
	int       maxCount = pt.get<int>("区域增长normal.最大簇数量");
	int      knum = pt.get<int>("区域增长normal.邻近数量");
	float      smoothness = pt.get<float>("区域增长normal.平滑阀值");
	float		Curvature = pt.get<float>("区域增长normal.曲率阀值");
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_, *cloud);


	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setRadiusSearch(kdK);
	normal_estimator.compute(*normals);


	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(minCount);
	reg.setMaxClusterSize(maxCount);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(knum);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(/*3.0 / 180.0 * M_PI*/smoothness);
	reg.setCurvatureThreshold(/*1*/Curvature);

 //	std::vector <pcl::PointIndices> clusters;
    reg.extract(PCManage::ins().jlClusters_);


    PointCloudT::Ptr colored_cloud = reg.getColoredCloud();
	//reg.extract(PCManage::ins().jlClusters_);
    if (colorCloud)
    {
        pcl::copyPointCloud(*colored_cloud, *colorCloud);
    }
    else
    {
        pcl::copyPointCloud(*colored_cloud, *cloud_);
    }
}
