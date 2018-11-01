#include "QOushifenge.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include "QRendView.h"
#include "PCManage.h"

QOushifenge::QOushifenge(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);
	ui.lineEdit->setValidator(new QDoubleValidator(this));
	ui.lineEdit_2->setValidator(new QIntValidator(this));
	ui.lineEdit_3->setValidator(new QIntValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit->setText(QString::fromLocal8Bit(  pt.get<std::string>("欧氏分割.临近半径").c_str()));
	ui.lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("欧氏分割.最小聚类点数目").c_str()));
	ui.lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("欧氏分割.最大聚类点数目").c_str()));
}

QOushifenge::~QOushifenge()
{
}

void QOushifenge::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;

	std::string       clusterTolerance = ui.lineEdit->text().toLocal8Bit().data();
	std::string       minClusterSize = ui.lineEdit_2->text().toLocal8Bit().data();
	std::string       maxClusterSize = ui.lineEdit_3->text().toLocal8Bit().data();

	

	// 讀取文件
	PointCloudT::Ptr add_cloud(new PointCloudT);

	// 下采樣，體素葉子大小為0.01
	pcl::VoxelGrid<PointT> vg;
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*
	pcl::PCDWriter writer;


	// 建立用於提取搜尋方法的kdtree樹物件
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;   //歐式聚類物件
	//ec.setClusterTolerance(0.02);                     // 設定近鄰搜尋的搜尋半徑為2cm
	ec.setClusterTolerance(/*2*/atof(clusterTolerance.c_str()));                     // 設定近鄰搜尋的搜尋半徑為2cm
	ec.setMinClusterSize(atoi(/*100*/minClusterSize.c_str()));                 //設定一個聚類需要的最少的點數目為100
	ec.setMaxClusterSize(/*2500000*/atoi(maxClusterSize.c_str()));               //設定一個聚類需要的最大點數目為25000
	ec.setSearchMethod(tree);                    //設定點雲的搜尋機制
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);           //從點雲中提取聚類，並將點雲索引儲存在cluster_indices中
	PCManage::ins().jlClusters_ = cluster_indices;
	//迭代訪問點雲索引cluster_indices,直到分割處所有聚類
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{ //迭代容器中的點雲的索引，並且分開儲存索引的點雲
		PointCloudT::Ptr cloud_cluster(new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			//設定儲存點雲的屬性問題
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		// 		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<PointT>(ss.str(), *cloud_cluster, false); //*

		//————————————以上就是實現所有的聚類的步驟，並且儲存了————————————————————————————//
		//以下就是我為了回答網友提問解決視覺化除了平面以後的視覺化的程式碼也就兩行   
		uint8_t r = rand() % 256;
		uint8_t g = rand() % 256;
		uint8_t b = rand() % 256;
		for (int i = 0; i < cloud_cluster->size(); ++i)
		{
			cloud_cluster->points[i].r = r;
			cloud_cluster->points[i].g = g;
			cloud_cluster->points[i].b = b;
		}

		*add_cloud += *cloud_cluster;


		//pcl::io::savePCDFileASCII("add_cloud.pcd", *add_cloud);
	}
	pcl::copyPointCloud(*add_cloud, *cloud);



	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("欧氏分割.临近半径", clusterTolerance);
	pt.put<std::string>("欧氏分割.最小聚类点数目", minClusterSize);
	pt.put<std::string>("欧氏分割.最大聚类点数目", maxClusterSize);
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  


	ins->UpdateView();
	this->accept();
}

void QOushifenge::Fun(PointCloudT::Ptr cloud)
{
	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	std::string       clusterTolerance = pt.get<std::string>("欧氏分割.临近半径");
	std::string       minClusterSize = pt.get<std::string>("欧氏分割.最小聚类点数目");
	std::string       maxClusterSize = pt.get<std::string>("欧氏分割.最大聚类点数目");



	// 讀取文件
	PointCloudT::Ptr add_cloud(new PointCloudT);

	// 建立用於提取搜尋方法的kdtree樹物件
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;   //歐式聚類物件
	ec.setClusterTolerance(/*2*/atof(clusterTolerance.c_str()));                     // 設定近鄰搜尋的搜尋半徑為2cm
	ec.setMinClusterSize(atoi(/*100*/minClusterSize.c_str()));                 //設定一個聚類需要的最少的點數目為100
	ec.setMaxClusterSize(/*2500000*/atoi(maxClusterSize.c_str()));               //設定一個聚類需要的最大點數目為25000
	ec.setSearchMethod(tree);                    //設定點雲的搜尋機制
    ec.setInputCloud(cloud);
	ec.extract(cluster_indices);           //從點雲中提取聚類，並將點雲索引儲存在cluster_indices中


	PCManage::ins().jlClusters_ = cluster_indices;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud->at(*pit).r = r;
            cloud->at(*pit).g = g;
            cloud->at(*pit).b = b;
        }
    }
}

void QOushifenge::Fun(PointCloudT::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
    std::string       clusterTolerance = pt.get<std::string>("欧氏分割.临近半径");
    std::string       minClusterSize = pt.get<std::string>("欧氏分割.最小聚类点数目");
    std::string       maxClusterSize = pt.get<std::string>("欧氏分割.最大聚类点数目");

    // 讀取文件
    PointCloudT::Ptr add_cloud(new PointCloudT);
    // 建立用於提取搜尋方法的kdtree樹物件
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;   //歐式聚類物件
    ec.setClusterTolerance(/*2*/atof(clusterTolerance.c_str()));                     // 設定近鄰搜尋的搜尋半徑為2cm
    ec.setMinClusterSize(atoi(/*100*/minClusterSize.c_str()));                 //設定一個聚類需要的最少的點數目為100
    ec.setMaxClusterSize(/*2500000*/atoi(maxClusterSize.c_str()));               //設定一個聚類需要的最大點數目為25000
    ec.setSearchMethod(tree);                    //設定點雲的搜尋機制
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);           //從點雲中提取聚類，並將點雲索引儲存在cluster_indices中
}

void QOushifenge::Fun(PointCloudT::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices, double k)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
    std::string       clusterTolerance = pt.get<std::string>("欧氏分割.临近半径");
    std::string       minClusterSize = pt.get<std::string>("欧氏分割.最小聚类点数目");
    std::string       maxClusterSize = pt.get<std::string>("欧氏分割.最大聚类点数目");

    // 讀取文件
    PointCloudT::Ptr add_cloud(new PointCloudT);
    // 建立用於提取搜尋方法的kdtree樹物件
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;   //歐式聚類物件
    ec.setClusterTolerance(k);                     // 設定近鄰搜尋的搜尋半徑為2cm
    ec.setMinClusterSize(atoi(/*100*/minClusterSize.c_str()));                 //設定一個聚類需要的最少的點數目為100
    ec.setMaxClusterSize(/*2500000*/atoi(maxClusterSize.c_str()));               //設定一個聚類需要的最大點數目為25000
    ec.setSearchMethod(tree);                    //設定點雲的搜尋機制
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);           //從點雲中提取聚類，並將點雲索引儲存在cluster_indices中
}
