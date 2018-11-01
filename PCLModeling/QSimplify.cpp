#include "QSimplify.h"
#include <QRendView.h>
#include <pcl/filters/uniform_sampling.h>   //均匀采样
#include "PCManage.h"

QSimplify::QSimplify(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);
	ui.lineEdit->setValidator(new QDoubleValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>("点云简化.网格大小").c_str()));
}

QSimplify::~QSimplify()
{
}

void QSimplify::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;

	std::string   gridSize = ui.lineEdit->text().toLocal8Bit().data();

    double leaf = atof(gridSize.c_str());

    //均匀采样点云并提取关键点      体素下采样，重心代替
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);  //输入点云
    uniform_sampling.setRadiusSearch(leaf);   //设置半径 
    uniform_sampling.filter(*cloud);   //滤波

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("点云简化.网格大小", gridSize);
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  


	ins->UpdateView();
	this->accept();
}

void QSimplify::Fun(PointCloudT::Ptr cloud)
{
	if (!cloud)
		cloud = PCManage::ins().cloud_;

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	std::string   gridSize = pt.get<std::string>("点云简化.网格大小");


    double leaf = atof(gridSize.c_str());

    //均匀采样点云并提取关键点      体素下采样，重心代替
    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);  //输入点云
    uniform_sampling.setRadiusSearch(leaf);   //设置半径 
    uniform_sampling.filter(*cloud);   //滤波
}
