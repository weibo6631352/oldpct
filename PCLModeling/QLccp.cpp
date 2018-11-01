#include "QLccp.h"
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include "QRendView.h"
#include "PCManage.h"
#define Random(x) (rand() % x)  

QLccp::QLccp(QWidget *parent)
: QSubDialogBase(parent)
{
	ui.setupUi(this);

	ui.min_lineEdit_4->setValidator(new QDoubleValidator(this));
	ui.max_lineEdit_4->setValidator(new QDoubleValidator(this));
	ui.min_lineEdit_3->setValidator(new QDoubleValidator(this));
	ui.max_lineEdit_3->setValidator(new QDoubleValidator(this));
	ui.min_lineEdit_2->setValidator(new QDoubleValidator(this));
	ui.max_lineEdit_2->setValidator(new QIntValidator(this));
	ui.min_lineEdit->setValidator(new QDoubleValidator(this));
	ui.max_lineEdit->setValidator(new QDoubleValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.min_lineEdit_4->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.粒子距离").c_str()));
	ui.max_lineEdit_4->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.晶核距离").c_str()));
	ui.min_lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.颜色容差").c_str()));
	ui.max_lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.凹度容差阀值").c_str()));
	ui.min_lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.平滑阀值").c_str()));
	ui.max_lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.最小分割数量").c_str()));
	ui.min_lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.空间权重").c_str()));
	ui.max_lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>("lccp.向量权重").c_str()));

}

QLccp::~QLccp()
{
}


void QLccp::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;

	//输入点云  
	PointCloudT::Ptr input_cloud_ptr(new PointCloudT);
	pcl::copyPointCloud(*cloud, *input_cloud_ptr);

	//超体聚类 参数依次是粒子距离、晶核距离、颜色容差、  
	std::string       voxel_resolution = ui.min_lineEdit_4->text().toLocal8Bit().data();
	std::string       seed_resolution = ui.max_lineEdit_4->text().toLocal8Bit().data();
	std::string       color_importance = ui.min_lineEdit_3->text().toLocal8Bit().data();
	std::string       concavity_tolerance_threshold = ui.max_lineEdit_3->text().toLocal8Bit().data();
	std::string       smoothness_threshold = ui.min_lineEdit_2->text().toLocal8Bit().data();
	std::string		min_segment_size = ui.max_lineEdit_2->text().toLocal8Bit().data();
	std::string   spatial_importance = ui.min_lineEdit->text().toLocal8Bit().data();
	std::string   normal_importance = ui.max_lineEdit->text().toLocal8Bit().data();

	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	unsigned int k_factor = 0;

	//voxel_resolution is the resolution (in meters) of voxels used、seed_resolution is the average size (in meters) of resulting supervoxels    
	pcl::SupervoxelClustering<PointT> super(atof(voxel_resolution.c_str()), atof(seed_resolution.c_str()));
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(input_cloud_ptr);
	//Set the importance of color for supervoxels.   
	super.setColorImportance(atof(color_importance.c_str()));
	//Set the importance of spatial distance for supervoxels.  
	super.setSpatialImportance(atof(spatial_importance.c_str()));
	//Set the importance of scalar normal product for supervoxels.   
	super.setNormalImportance(atof(normal_importance.c_str()));
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);

	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);

	//LCCP分割  

	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(atof(concavity_tolerance_threshold.c_str()));
	lccp.setSmoothnessCheck(true, atof(voxel_resolution.c_str()), atof(seed_resolution.c_str()), atof(smoothness_threshold.c_str()));
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(atoi(min_segment_size.c_str()));
	lccp.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList  sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

	ofstream outFile2("分割后合并3.txt", std::ios_base::out);
	for (int i = 0; i < lccp_labeled_cloud->size(); i++) {
		outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t" << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << endl;
	}

	int label_max2 = 0;
	for (int i = 0; i< lccp_labeled_cloud->size(); i++) {
		if (lccp_labeled_cloud->points[i].label>label_max2)
			label_max2 = lccp_labeled_cloud->points[i].label;
	}
	PointCloudT::Ptr ColoredCloud2(new PointCloudT);
	ColoredCloud2->height = 1;
	ColoredCloud2->width = lccp_labeled_cloud->size();
	ColoredCloud2->resize(lccp_labeled_cloud->size());
	for (int i = 0; i < label_max2; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		for (int j = 0; j < lccp_labeled_cloud->size(); j++) {
			if (lccp_labeled_cloud->points[j].label == i) {
				ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				ColoredCloud2->points[j].r = color_R;
				ColoredCloud2->points[j].g = color_G;
				ColoredCloud2->points[j].b = color_B;
			}
		}
	}
	pcl::copyPointCloud(*ColoredCloud2, *cloud);


	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("lccp.粒子距离", voxel_resolution);
	pt.put<std::string>("lccp.晶核距离", seed_resolution);
	pt.put<std::string>("lccp.颜色容差", color_importance);
	pt.put<std::string>("lccp.凹度容差阀值", concavity_tolerance_threshold);
	pt.put<std::string>("lccp.平滑阀值", smoothness_threshold);
	pt.put<std::string>("lccp.最小分割数量", min_segment_size);
	pt.put<std::string>("lccp.空间权重", spatial_importance);
	pt.put<std::string>("lccp.向量权重", normal_importance);


	
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  


	ins->UpdateView();
	this->accept();
}
