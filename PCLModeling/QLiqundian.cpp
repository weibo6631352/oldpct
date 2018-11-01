#include "QLiqundian.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include "QRendView.h"

QLiqundian::QLiqundian(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);

	ui.lineEdit_2->setValidator(new QIntValidator(this));
	ui.lineEdit_3->setValidator(new QDoubleValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("离群点.k").c_str()));
	ui.lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("离群点.平均距离").c_str()));
}

QLiqundian::~QLiqundian()
{
}

void QLiqundian::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	PointT minPt, maxPt;

	std::string k = ui.lineEdit_2->text().toLocal8Bit().data();
	std::string avgDistance = ui.lineEdit_3->text().toLocal8Bit().data();

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(atoi(k.c_str()));                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(atof(avgDistance.c_str()));                      //设置判断是否为离群点的阀值
	sor.filter(*cloud);                    //存储


	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("离群点.k", k);  // 写字段  
	pt.put<std::string>("离群点.平均距离", avgDistance);  // 写字段  
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  

	ins->UpdateView();
	this->accept();
}

void QLiqundian::Fun(PointCloudT::Ptr srcCloud /*= nullptr*/)
{
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	if (nullptr != srcCloud)
	{
		cloud = srcCloud;
	}
	
	PointT minPt, maxPt;

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	std::string k = pt.get<std::string>("离群点.k");
	std::string avgDistance = pt.get<std::string>("离群点.平均距离");

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;   //创建滤波器对象
	sor.setInputCloud(cloud);                           //设置待滤波的点云
	sor.setMeanK(atoi(k.c_str()));                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(atof(avgDistance.c_str()));                      //设置判断是否为离群点的阀值
	sor.filter(*cloud);                    //存储

}
