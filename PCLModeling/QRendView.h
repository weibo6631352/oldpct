#pragma once

#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "PCManage.h"

class QRendView : public QVTKWidget
{
	Q_OBJECT

public:
	QRendView(QWidget *parent = Q_NULLPTR);
	~QRendView();

	static QRendView* MainRendView();
	static QRendView* mainRendView_;

	void UpdateView(PointCloudT::Ptr cloud = nullptr, bool removePre = true);

	vtkRenderWindow* GetRendWindow();

	pcl::visualization::PCLVisualizer::Ptr viewer_;
};
