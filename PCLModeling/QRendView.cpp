#include "QRendView.h"
#include "vtkRenderWindow.h"
#include <pcl/common/common.h>
#include <QCoreApplication>
#include "dvprint.h"

QRendView* QRendView::mainRendView_ = nullptr;
QRendView::QRendView(QWidget *parent)
: QVTKWidget(parent)
{
	if (!mainRendView_)
		mainRendView_ = this;
	viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>("viewer", false);
	viewer_->setBackgroundColor(1, 1, 1);
	SetRenderWindow(viewer_->getRenderWindow());
	QVTKInteractor* Interactor = GetInteractor();
	viewer_->setupInteractor(Interactor, viewer_->getRenderWindow());
}

QRendView::~QRendView()
{
	// 这里很坑，你得替它删，不然这里就内存泄漏了
	viewer_->removeAllPointClouds();
	viewer_->removeAllShapes();
}


vtkRenderWindow* QRendView::GetRendWindow()
{
	return viewer_->getRenderWindow();
}

void QRendView::UpdateView(PointCloudT::Ptr cloud, bool removePre /*= true*/)
{
	if (cloud == nullptr)
		cloud = PCManage::ins().cloud_;

	if (removePre)
	{
		//viewer_->removeAllShapes();
	}
    viewer_->removePointCloud("cloud");
    viewer_->removeText3D("aabbBoxmin");
    viewer_->removeText3D("aabbBoxmax");
	
	viewer_->addPointCloud(cloud);
	viewer_->resetCamera();

	PointT min, max;
	getMinMax3D(*cloud, min, max);
	std::stringstream minstr, maxstr, counts;
    counts << "counts: " << cloud->size();
	minstr << std::fixed << setprecision(2) << "min: " << min.x << " " << min.y << " " << min.z;
	maxstr << std::fixed << setprecision(2) << "max: " << max.x << " " << max.y << " " << max.z;
    viewer_->addText(counts.str(), 5, 30, 1, 0, 1, std::string("counts"));
    viewer_->addText(minstr.str(), 5, 20, 1, 0, 1, std::string("aabbBoxmin"));
    viewer_->addText(maxstr.str(), 5, 10, 1, 0, 1, std::string("aabbBoxmax"));

	update();
    QCoreApplication::processEvents();
}

QRendView* QRendView::MainRendView()
{
	return mainRendView_;
}
