#pragma once
/**
*  三维重建类                                             
**/
#include "PCManage.h"

#ifndef IN
#define IN
#endif

class Reconstruct
{
public:
    static void BuildSceneMesh(IN PointCloudT::Ptr cloud);

    // pcl 松柏重建
    static void PCLPoissonReconstruct(PointCloudT::Ptr color_cloud);

    // pcl 三维立方体算法重建
    static void PclCubeReconstruct(PointCloudT::Ptr color_cloud);

    // pcl 贪婪投影三角化算法
    static void PclGp3Reconstruct(PointCloudT::Ptr color_cloud);

    // cgal 三角网算法
    static void CGALReconstruct(PointCloudT::Ptr cloud);
};


