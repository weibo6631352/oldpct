#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <boost/make_shared.hpp>
#include <vector>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PCManage
{
public:
	PointCloudT::Ptr cloud_;			//点云数据
	std::vector <pcl::PointIndices> jlClusters_;	//聚类结果

	static PCManage &ins();
private:
	PCManage();
};

