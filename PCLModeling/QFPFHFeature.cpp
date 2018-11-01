#include "QFPFHFeature.h"
#include "QRendView.h"

#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h> //包含fpfh加速计算的omp(多核并行计算)
#include "dvprint.h"
#include "PCManage.h"
#include "QRendView.h"

#include <QMessageBox>
#include <QString>
#include <QStringList>
#include <QFileDialog>
#include <QComboBox>


#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>  
#include <vtkPointsProjectedHull.h>
#include <vtkPolyLine.h>
#include <vtkRenderWindow.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>
#include <boost/thread/thread.hpp>


using namespace std;
typedef pcl::PointCloud<PointT> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::PFHSignature125> pfhFeature;

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);
    std::vector< int > indices;
    if (event.getPointsIndices(indices) == -1)
        return;

    PointCloudT::Ptr cloud = PCManage::ins().cloud_;
    for (int i = 0; i < indices.size(); ++i)
    {
        clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
    }
    *(PointCloudT*)args = *clicked_points_3d;
}


QFPFHFeature::QFPFHFeature(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);
	cloud_ = boost::make_shared<PointCloudT>();
	ui.lineEdit->setValidator(new QDoubleValidator(this));
	ui.lineEdit_2->setValidator(new QIntValidator(this));

	QStringList items;
	items << QStringLiteral("PFH特征")
		<< QStringLiteral("FPFH特征");

	ui.comboBox->addItems(items);
	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  
	ui.lineEdit->setText(QString::fromLocal8Bit(pt.get<std::string>("fpfh.邻域半径").c_str()));
	ui.lineEdit_2->setText(QString::fromLocal8Bit(pt.get<std::string>("fpfh.邻域数量").c_str()));
	ui.comboBox->setCurrentIndex(pt.get<int>("fpfh.特征类型"));
	
	

//	connect(ui.comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(comboxChange(int)));
	connect(ui.pushButton_2, &QPushButton::clicked, [this](){
		ui.lineEdit_3->setText( QFileDialog::getOpenFileName(
			this,
			QStringLiteral("待识别对象"),
			NULL,// ".",
			QStringLiteral("*.pcd")));
	});


    // Add point picking callback to viewer:  
    
    
    QRendView::MainRendView()->viewer_->registerAreaPickingCallback(pp_callback, (void*)&selectPoints_);
}

void QFPFHFeature::comboxChange(int index)
{
	if (0 == index)
	{
		ui.lineEdit->setEnabled(true);
		ui.lineEdit_2->setEnabled(false);
	}
	else
	{
		ui.lineEdit->setEnabled(false);
		ui.lineEdit_2->setEnabled(true);
	}
}


QFPFHFeature::~QFPFHFeature()
{
}


fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<PointT>::Ptr tree, double kdis)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
/*	est_normal.setKSearch(10);*/
	est_normal.setRadiusSearch(kdis);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4); //指定4核计算
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
//	est_fpfh.setKSearch(10);
	est_fpfh.setRadiusSearch(kdis);
	est_fpfh.compute(*fpfh);

	return fpfh;
}

pfhFeature::Ptr compute_pfh_feature(pointcloud::Ptr input_cloud, pcl::search::KdTree<PointT>::Ptr tree, double kdis)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	//est_normal.setKSearch(10);
	est_normal.setRadiusSearch(kdis);
	est_normal.compute(*point_normal);


	pcl::PFHEstimation<PointT, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(input_cloud);
	pfh.setInputNormals(point_normal);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	//pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
	pfh.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	//pfh.setRadiusSearch(0.05);
	pfh.setRadiusSearch(kdis);

	// Compute the features
	pfh.compute(*pfhs);

	return pfhs;
}

template <typename T,typename Type, typename K>
void computer_align(pointcloud::Ptr const source, pointcloud::Ptr const target, pointcloud::Ptr const align, pcl::IndicesPtr indices, K kval, Type source_fpfh, Type target_fpfh)
{
	//对齐(占用了大部分运行时间)
	//pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	pcl::SampleConsensusInitialAlignment<PointT, PointT, T> sac_ia;
	sac_ia.setInputSource(source);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target);
	sac_ia.setTargetFeatures(target_fpfh);
	
	//  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
//	sac_ia.setCorrespondenceRandomness(kval); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.setMaxCorrespondenceDistance(0.5);
	sac_ia.align(*align);
	*indices = *sac_ia.getIndices();
}

void QFPFHFeature::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;




	//设定结晶参数
	float kdis =/* 0.008f*/atof(ui.lineEdit->text().toLocal8Bit().data());
	int knum = /*0.1f*/atoi(ui.lineEdit_2->text().toLocal8Bit().data());

	QString fileName = ui.lineEdit_3->text();



	clock_t start, end, time;
	start = clock();
// 	pointcloud::Ptr source(new pointcloud);
//     pcl::io::loadPCDFile<PointT>(fileName.toLocal8Bit().data(), *source);
    pointcloud::Ptr source(selectPoints_.makeShared());
    dd("%d",source->size());

	pointcloud::Ptr target(new pointcloud);
	//pcl::copyPointCloud(*cloud, *target);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pointcloud::Ptr align(new pointcloud);
	pcl::IndicesPtr indices(new std::vector<int>);
	int curindex = ui.comboBox->currentIndex();

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	for (int i = 0; i < source->size(); ++i)
	{
		double p[3] = { source->at(i).x, source->at(i).y, source->at(i).z };
		points->InsertNextPoint(p);
	}

	vtkSmartPointer<vtkParametricSpline> spline =
		vtkSmartPointer<vtkParametricSpline>::New();
	spline->SetPoints(points);
	vtkSmartPointer<vtkParametricFunctionSource> functionSource =
		vtkSmartPointer<vtkParametricFunctionSource>::New();
	functionSource->SetParametricFunction(spline);
	functionSource->Update();
	vtkPoints* points1 = functionSource->GetOutput()->GetPoints();
	source->resize(0);
	source->clear();
	for (int i = 0; i < points1->GetNumberOfPoints(); ++i)
	{
		PointT pt;
		pt.x = points1->GetPoint(i)[0];
		pt.y = points1->GetPoint(i)[1];
		pt.z = points1->GetPoint(i)[2];
		pt.r = 255;
		pt.g = 0;
		pt.b = 255;
		source->push_back(pt);
	}

	*target = *cloud;
	if (0 == curindex)
	{
		pfhFeature::Ptr source_fpfh = compute_pfh_feature(source, tree, kdis);
		pfhFeature::Ptr target_fpfh = compute_pfh_feature(target, tree, kdis);
		computer_align<pcl::PFHSignature125, pfhFeature::Ptr, float>(source, target, align, indices, knum, source_fpfh, target_fpfh);
	}
	else if (1 == curindex)
	{
		fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree, kdis);
		fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree, kdis);

		computer_align<pcl::FPFHSignature33, fpfhFeature::Ptr, int>(source, target, align, indices, knum, source_fpfh, target_fpfh);
	}
	else
	{
		return;
	}


	pointcloud::Ptr temp(new pointcloud);
	pcl::ExtractIndices<PointT> extract1;
	extract1.setInputCloud(cloud);
	extract1.setIndices(indices);
	extract1.setNegative(true);
	extract1.filter(*cloud);

	if (align->size() == 0 || indices->size() == 0)
	{
		QMessageBox::information(this, QString::number(align->size()), QString::number(indices->size()));
	}
	*cloud_ += *align;
	//*cloud_ += *source;

	//*cloud += *cloud_;
	boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);
	pt.put<float>("fpfh.邻域半径", kdis);
	pt.put<int>("fpfh.邻域数量", knum);
	pt.put<int>("fpfh.特征类型", curindex);
    boost::property_tree::ini_parser::write_ini((QApplication::applicationDirPath() + QStringLiteral("/user.ini")).toLocal8Bit().data(), pt);  // 打开读文件  

	ui.widget->UpdateView(cloud_);
	ins->UpdateView();
	//this->accept();
}

