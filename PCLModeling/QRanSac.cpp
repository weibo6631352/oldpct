#include "QRanSac.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "QRendView.h"
#include "PCManage.h"
#include <pcl/search/kdtree.h>

QRanSac::QRanSac(QWidget *parent)
: QSubDialogBase(parent)
{
	ui.setupUi(this);
	cloud_ = boost::make_shared<PointCloudT>();

	ui.min_lineEdit->setValidator(new QIntValidator(this));
	ui.max_lineEdit->setValidator(new QDoubleValidator(this));
	ui.max_lineEdit_2->setValidator(new QDoubleValidator(this));
	QStringList items;
	items << QStringLiteral("SACMODEL_PLANE")
		<< QStringLiteral("SACMODEL_LINE")
		<< QStringLiteral("SACMODEL_CIRCLE2D")
		<< QStringLiteral("SACMODEL_CIRCLE3D")
		<< QStringLiteral("SACMODEL_SPHERE")
		<< QStringLiteral("SACMODEL_CYLINDER")
		<< QStringLiteral("SACMODEL_CONE")
		<< QStringLiteral("SACMODEL_TORUS")
		<< QStringLiteral("SACMODEL_PARALLEL_LINE")
		<< QStringLiteral("SACMODEL_PERPENDICULAR_PLANE")
		<< QStringLiteral("SACMODEL_PARALLEL_LINES")
		<< QStringLiteral("SACMODEL_NORMAL_PLANE")
		<< QStringLiteral("SACMODEL_NORMAL_SPHERE")
		<< QStringLiteral("SACMODEL_REGISTRATION")
		<< QStringLiteral("SACMODEL_REGISTRATION_2D")
		<< QStringLiteral("SACMODEL_PARALLEL_PLANE")
		<< QStringLiteral("SACMODEL_NORMAL_PARALLEL_PLANE")
		<< QStringLiteral("SACMODEL_STICK");
	ui.comboBox->addItems(items);
	
	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.min_lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>("ransac.最大迭代次数").c_str()));
	ui.max_lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>("ransac.查询点到目标模型的距离阈值").c_str()));
	ui.max_lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("ransac.至少一个样本不包含离群点的概率").c_str()));
	ui.comboBox->setCurrentIndex(pt.get<int>("ransac.图元类型"));
}

QRanSac::~QRanSac()
{
}


void QRanSac::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;

	int curindex = ui.comboBox->currentIndex();
	int maxIterations = atoi(ui.min_lineEdit->text().toLocal8Bit().data());
	double distanceThreshold = atof(ui.max_lineEdit->text().toLocal8Bit().data());
	double probability = atof(ui.max_lineEdit_2->text().toLocal8Bit().data());

	
	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建一个分割器
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory-设置目标几何形状
	seg.setModelType(/*pcl::SACMODEL_CIRCLE3D*/curindex);
	//分割方法：随机采样法
	seg.setMethodType(pcl::SAC_RANSAC);
	//最大迭代次数（默认值为50）
	seg.setMaxIterations(maxIterations);
	//查询点到目标模型的距离阈值
	seg.setDistanceThreshold(distanceThreshold);
	//至少一个样本不包含离群点的概率（默认值为0.99）。
//	seg.setProbability(probability);
// 	seg.setAxis(Eigen::Vector3f(0, 1, 0));
// 	seg.setEpsAngle(10.0f * (M_PI / 180.0f));//轴向约束
// 	pcl::SACSegmentation<PointT>::SearchPtr searcher(new pcl::search::KdTree<PointT>);
// 	seg.setSamplesMaxDist(3, searcher);
//	seg.setRadiusLimits(1000, (std::numeric_limits<double>::max)());
	//输入点云
	seg.setInputCloud(cloud);
	//分割点云
	seg.segment(*inliers, *coefficients);
	//pcl::copyPointCloud(*cloud, *inliers, *cloud);


	PointCloudT tempCloud;
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.filter(tempCloud);
	//pcl::copyPointCloud(tempCloud, *ui.widget->cloud_);
	*cloud_ += tempCloud;
	// 提取非地面点
	extract.setNegative(true);
	extract.filter(*cloud);

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<int>("ransac.最大迭代次数", maxIterations);
	pt.put<double>("ransac.查询点到目标模型的距离阈值", distanceThreshold);
	pt.put<double>("ransac.至少一个样本不包含离群点的概率", probability);
	pt.put<int>("ransac.图元类型", curindex);



    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  

	ui.widget->UpdateView(cloud_);
	ins->UpdateView();
	//this->accept();
}
