#include "PCManage.h"

 PCManage::PCManage()
 {
 	cloud_ = boost::make_shared<PointCloudT>();
 }

 PCManage& PCManage::ins()
 {
 	static PCManage pcm;
 	return pcm;
 }
