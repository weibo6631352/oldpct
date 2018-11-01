#include "QGround.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include "QRendView.h"
#include "PCManage.h"
#include <pcl/common/common.h>
#include "dvprint.h"

QGround::QGround(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);

	ui.min_lineEdit_2->setValidator(new QIntValidator(this));
	ui.max_lineEdit_2->setValidator(new QDoubleValidator(this));
	ui.max_lineEdit->setValidator(new QDoubleValidator(this));
    ui.max_lineEdit_3->setValidator(new QDoubleValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.min_lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("地面过滤.窗口大小").c_str()));
	ui.max_lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("地面过滤.斜率值").c_str()));
    ui.max_lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("地面过滤.最小高度").c_str()));
	ui.max_lineEdit->setText(QString::fromLocal8Bit(  pt.get<std::string>("地面过滤.最大高度").c_str()));
	ui.radioButton->setChecked(pt.get<bool>("地面过滤.排除"));
	ui.checkBox->setChecked(   pt.get<bool>("地面过滤.是否使用高度"));
}

QGround::~QGround()
{
}

void QGround::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	PointT minPt, maxPt;

	std::string windowsize = ui.min_lineEdit_2->text().toLocal8Bit().data();
	std::string slope = ui.max_lineEdit_2->text().toLocal8Bit().data();
	std::string maxlDistance = ui.max_lineEdit->text().toLocal8Bit().data();
    std::string minlDistance = ui.max_lineEdit_3->text().toLocal8Bit().data();
	bool checked = ui.radioButton->isChecked();
	bool useHeight = ui.checkBox->isChecked();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndicesPtr ground(new pcl::PointIndices);

	//	生成形态滤波器
	pcl::ProgressiveMorphologicalFilter<PointT> pmf;
	pmf.setInputCloud(cloud);
	//设置窗的大小
	pmf.setMaxWindowSize(atoi(windowsize.c_str()));
	// 设置计算高度阈值的斜率值
	pmf.setSlope(atof(slope.c_str()));

	if (useHeight)
	{
// 		// 设置初始高度参数被认为是地面点
		pmf.setInitialDistance(atof(minlDistance.c_str()));

		// 设置被认为是地面点的最大高度
        pmf.setMaxDistance(atof(maxlDistance.c_str()));
	}


	

	//提取地面
	pmf.extract(ground->indices);

	// 从标号到点云
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground);
	// 提取非地面点
	extract.setNegative(checked);
	extract.filter(*cloud);




	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<std::string>("地面过滤.窗口大小", windowsize);  // 写字段  
	pt.put<std::string>("地面过滤.斜率值", slope);  // 写字段  
	pt.put<std::string>("地面过滤.最大高度", maxlDistance);  // 写字段  
    pt.put<std::string>("地面过滤.最小高度", minlDistance);  // 写字段  
	pt.put<bool>("地面过滤.排除", checked);  // 写字段  
	pt.put<bool>("地面过滤.是否使用高度", useHeight);  // 写字段  
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  


	ins->UpdateView();
	this->accept();
}

void QGround::Fun(PointCloudT::Ptr cloud, PointCloudT &ground)
{
	if (!cloud)
		cloud = PCManage::ins().cloud_;

	PointT minPt, maxPt;


	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	std::string windowsize = pt.get<std::string>("地面过滤.窗口大小");
	std::string slope = pt.get<std::string>("地面过滤.斜率值");
	std::string maxlDistance = pt.get<std::string>("地面过滤.最大高度");
    std::string minlDistance = pt.get<std::string>("地面过滤.最小高度");
	//bool checked = pt.get<bool>("地面过滤.排除");
	bool useHeight = pt.get<bool>("地面过滤.是否使用高度");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointIndicesPtr groundindices(new pcl::PointIndices);

	//	生成形态滤波器
	pcl::ProgressiveMorphologicalFilter<PointT> pmf;
	pmf.setInputCloud(cloud);
	//设置窗的大小
	pmf.setMaxWindowSize(atoi(windowsize.c_str()));
	// 设置计算高度阈值的斜率值
	pmf.setSlope(atof(slope.c_str()));

	if (useHeight)
	{

		// 设置初始高度参数被认为是地面点
		pmf.setInitialDistance(atof(minlDistance.c_str()));

		// 设置被认为是地面点的最大高度
        pmf.setMaxDistance(atof(maxlDistance.c_str()));
	}

	//提取地面
    pmf.extract(groundindices->indices);

	// 从标号到点云
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
    extract.setIndices(groundindices);
    extract.filter(ground);
	// 提取非地面点
	extract.setNegative(true);
	extract.filter(*cloud);

    //dd("地面点个数=%d", ground->size());
}

// 将点云存入二维数组
void PutPointCloud2Arr(PointCloudT &cloud, std::vector<std::vector<pcl::PointIndices>> &pointsArr, int row, int col, PointT min, PointT max, double gridSize)
{
    // 填充网格
    for (int i = 0; i < cloud.size(); ++i)
    {
        PointT &pt = cloud.at(i);
        pointsArr[(int)((pt.x - min.x) / gridSize)][(int)((pt.y - min.y) / gridSize)].indices.push_back(i);
    }
}

 