#pragma once
#include "MathGeoLibFwd.h"
#include "Math/float3.h"
#include <boost/shared_ptr.hpp>
#include "PCManage.h"

#ifndef IN
#define IN
#endif

// 计算中心点
PointT GetCenter(const PointCloudT cloud);

// vec转pointt
PointT Vec2PointT(const vec& pt);

//pointt转vec
vec PointT2Vec(const PointT& pt);

// 转换到局部坐标系，因为高斯坐标系的数值太大，无法进行obb包围盒运算或者拟合运算
void Transitionlocal(PointCloudT &cloud, IN vec const *centerptr = nullptr);

// xyz文件转为las文件
void Xyz2Las(const std::string & infile, const std::string &outfile);