#include "PointHeightColor.h"
#include "QRendView.h"
#include <pcl/common/common.h>

PointHeightColor::PointHeightColor()
{
}


PointHeightColor::~PointHeightColor()
{
}

void PointHeightColor::Fun()
{
	QRendView* ins = QRendView::MainRendView();
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	PointT min;
	PointT max;
	pcl::getMinMax3D(*cloud, min, max);

	uchar staC[3] = { 0, 255, 255 };
	uchar endC[3] = { 255, 0, 0 };
	short subR = endC[0] - staC[0];
	short subG = endC[1] - staC[1];
	short subB = endC[2] - staC[2];


	double heig = max.z - min.z;
	for (int i = 0; i < cloud->size(); ++i)
	{
		PointT &pt = cloud->at(i);
		double l = (pt.z - min.z) / heig;
		pt.r = (subR)* (l)+staC[0];
		pt.b = (subG)* (l)+staC[1];
		pt.g = (subB)* (l)+staC[2];
	}

	ins->UpdateView();
}
