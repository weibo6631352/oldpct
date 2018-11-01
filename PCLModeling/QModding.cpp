#include "QModding.h"
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include "QRendView.h"
#include "PCManage.h"

QModding::QModding(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);
	ui.lineEdit->setValidator(new QIntValidator(this));
	ui.lineEdit_2->setValidator(new QDoubleValidator(this));
	ui.lineEdit_3->setValidator(new QDoubleValidator(this));
	ui.lineEdit_4->setValidator(new QIntValidator(this));
	ui.lineEdit_5->setValidator(new QDoubleValidator(this));
	ui.lineEdit_6->setValidator(new QDoubleValidator(this));
	ui.lineEdit_7->setValidator(new QDoubleValidator(this));

	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit  ->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.kd树k值").c_str()));
	ui.lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.搜索半径").c_str()));
	ui.lineEdit_3->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.加权因子").c_str()));
	ui.lineEdit_4->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.临近点阈值").c_str()));
	ui.lineEdit_5->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.面角度").c_str()));
	ui.lineEdit_6->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.三角形最小角度").c_str()));
	ui.lineEdit_7->setText(QString::fromLocal8Bit(pt.get<std::string>("生成模型.三角形最大角度").c_str()));
}

QModding::~QModding()
{
}

void QModding::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;


	//设定结晶参数
	int KSearch =/* 0.008f*/atof(ui.lineEdit->text().toLocal8Bit().data());
	float SearchRadius = /*0.1f*/atof(ui.lineEdit_2->text().toLocal8Bit().data());
	float Mu = /*0.2f*/atof(ui.lineEdit_3->text().toLocal8Bit().data());
	int MaximumNearestNeighbors = /*0.4f*/atof(ui.lineEdit_4->text().toLocal8Bit().data());
	float MaximumSurfaceAngle = /*1.0f*/atof(ui.lineEdit_5->text().toLocal8Bit().data());
	float MinimumAngle = /*1.0f*/atof(ui.lineEdit_6->text().toLocal8Bit().data());
	float MaximumAngle = /*1.0f*/atof(ui.lineEdit_7->text().toLocal8Bit().data());

	// 计算法向*
	pcl::NormalEstimation<PointT, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(/*20*/KSearch);	//20
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	// 搜索半径的设定（这个参数必须由用户指定），它决定的重建后三角形的大小0.025
	gp3.setSearchRadius(/*0.025*/SearchRadius);

	// Set typical values for the parameters
	// mu是个加权因子，对于每个参考点，其映射所选球的半径由mu与离参考点最近点的距离乘积所决定，这样就很好解决了点云密度不均匀的问题，mu一般取值为2.5-3。
	gp3.setMu(/*2.5*/Mu);
	// 临近点阈值设定。一般为80-100。
	gp3.setMaximumNearestNeighbors(/*100*/MaximumNearestNeighbors);

	// 两点的法向量角度差大于此值，这两点将不会连接成三角形，这个就恰恰和点云局部平滑的约束呼应，如果一个点是尖锐点那么它将不会和周围的点组成三角形，
	// 其实这个也有效的滤掉了一些离群点。这个值一般为45度。
	gp3.setMaximumSurfaceAngle(/*M_PI / 4*/MaximumSurfaceAngle); // 45 degrees
	// 三角形最大、最小角度的阈值。
	gp3.setMinimumAngle(/*M_PI / 18*/MinimumAngle); // 10 degrees
	gp3.setMaximumAngle(/*2 * M_PI / 3*/MaximumAngle); // 120 degrees
	//输入的法向量是否连续变化的。这个一般来讲是false，除非输入的点云是全局光滑的（比如说一个球）。
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	ins->viewer_->addPolygonMesh(triangles, "triangles");  //设置所要显示的网格对象
	//设置网格模型显示模式
	//viewer_->setRepresentationToSurfaceForAllActors(); //网格模型以面片形式显示  
	//viewer_->setRepresentationToPointsForAllActors(); //网格模型以点形式显示  
	ins->viewer_->setRepresentationToWireframeForAllActors();  //网格模型以线框图模式显示
	ins->viewer_->resetCamera();



	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<int>("生成模型.kd树k值", KSearch);
	pt.put<float>("生成模型.搜索半径", SearchRadius);
	pt.put<float>("生成模型.加权因子", Mu);
	pt.put<int>("生成模型.临近点阈值", MaximumNearestNeighbors);
	pt.put<float>("生成模型.面角度", MaximumSurfaceAngle);
	pt.put<float>("生成模型.三角形最小角度", MinimumAngle);
	pt.put<float>("生成模型.三角形最大角度", MaximumAngle);
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  

// 	pcl::io::saveVTKFile("mesh.vtk", triangles);
// 	pcl::io::savePLYFile("mesh.ply", triangles);
	//ins->UpdateView();
	this->accept();
}
