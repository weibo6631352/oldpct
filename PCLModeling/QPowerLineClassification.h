#pragma once

#include <QSubDialogBase.h>
#include "ui_QPowerLineClassification.h"
#include <PCManage.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "LeastSquare.h"
#include "MathGeoLibFwd.h"
#include "Math/float3.h"

double GetDistance3d(PointT &pt1, PointT &pt2);
double GetDistance2d(PointT &pt1, PointT &pt2);
double GetDistance2d(double x1, double y1, double x2, double y2);
double GetDistance2d(const vec &pt1, const vec &pt2);
typedef struct
{
    enum Asix{X=0, Y=1, Z=2};
    vec sta;
    vec end;
    vec v;
    Asix maxAsix;
    std::vector<vec> pts;
    std::vector<vec> fit_pts;
    Fit fit;
    Fit fit_z;
} LineInfo;
typedef struct
{
    PointT cen;
    double radiu;
    std::string id;
    std::string description;
}CollisionBall;

struct CloudInfo
{
    vec min, max, center;
    int count;

    CloudInfo() :count(0){};
    void setMinMax(const PointT &omin, PointT &omax)
    {
        min.x = omin.x;
        min.y = omin.y;
        min.z = omin.z;
        max.x = omax.x;
        max.y = omax.y;
        max.z = omax.z;

        center = (min + max) / 2;
    }
};
vec PointT2Vec(const PointT &ptt);
PointT Vec2PointT(const vec &ptv);

class QPowerLineClassification : public QSubDialogBase
{
	Q_OBJECT

public:
	QPowerLineClassification(QWidget *parent = Q_NULLPTR);
	~QPowerLineClassification();
	

	void InitData();//初始化数据
    void SaveSetting();//保存设置

private:
	PointCloudT::Ptr cloud_;//维护独立的点云数据
    CloudInfo info_;                //当前点云信息

    PointCloudT::Ptr ground_;//地面点
    std::vector<PointCloudT::Ptr> groundObjects_;//地物数据
    std::vector<PointCloudT::Ptr> towers_;//铁塔数据
    std::vector<PointCloudT::Ptr> lines_;//线性数据
    std::vector<boost::shared_ptr<LineInfo>> lineInfos_;//线数据信息，比如起终点和向量
    std::vector<CollisionBall> balls_;//碰撞球形标注
    std::set<vtkProp *> line_models_;
    
    
	Ui::QPowerLineClassification ui;//界面

    // ransac方法提取圆弧
	//void RanSacArc();

    void InitCloudInfo();

	// 是否符合电力线的特点
    bool LikePowerLine(LineInfo &line);

    // 是否符铁塔的特点
    bool LikeTower(PointCloudT &cloud);

    // 检查法向
	bool CheckNormal(PointCloudT &cloud);

    // 在xy平面是否是直线
	bool IsXyPlaneLine(PointCloudT &cloud);

    // 删除高密度类
	void RemoveHighDensityClass();

    // 删除aabb包围盒异常的
	void RemoveAABBUnusual();

    // 将点云存入xy平面对应的二维数组
    void PutPointCloud2Arr(PointCloudT::Ptr cloud, std::vector<std::vector<pcl::PointIndices::Ptr>> &pointsArr, int row, int col, PointT min, PointT max, double gridSize);

    // 将点云存入xy平面对应的二维数组
    void PutPointCloud2Arr(PointCloudT::Ptr cloud, std::vector<std::vector<pcl::PointIndices::Ptr>> &pointsArr, PointCloudT::Ptr ground, std::vector<std::vector<pcl::PointIndices::Ptr>> &groundArr, int row, int col, PointT min, PointT max, double gridSize);
    
    // 是否高密度过高
	bool IsHighDensity(PointCloudT &cloud, float density, float leafSize, float neighborNum);

    // 应用聚类后的点云
	void UpdateJLClusters2Cloud();

    // aabb包围盒是否异常
	bool IsAABBUnusual(PointCloudT &cloud, double aabbLimit);

    //obb包围盒是否异常
    bool IsOBBUnusual(PointCloudT &cloud, double obbLimit = 100);

    // 聚类中是否包含离群点
    bool IsHaveLiQunDian(PointCloudT::Ptr cloud);

    // 提取铁塔类型
	void RemoveLowerClass();

    // 生成管子
    void AddTube(const std::vector<vec> &pts, const float raidu);
    void AddTube(const boost::shared_ptr<LineInfo> &line, const float radiu);

    // 生成多段线
    void AddVtkLines(const boost::shared_ptr<LineInfo> &line);

    // 是否过矮的类型
	bool IsLowerClass(PointCloudT &cloud, pcl::KdTreeFLANN<PointT> &ground_kdtree, double groundLower, double groundLnterval, double aabbHeight);

    // 是否过矮的类型
    bool IsLowerClass(PointCloudT &cloud, pcl::KdTreeFLANN<PointT> &ground_kdtree, double groundLower);
    // 获取obb包围盒
	vec GetObbBox(PointCloudT &cloud);

    // 空间范围查找
    void DistanceSerach(double cenX, double cenY, double cenZ, double dis, std::vector<int> &indices);

    // 空间绕z轴查找
	void SerachVecRangePoints(PointT pt, double dis, pcl::PointIndices::Ptr vecPoints, pcl::PointIndices::Ptr vecrangePoint);

    // 空间绕z轴查找
    void SerachVecRangePoints(PointT pt, double dis, double minz, double maxz, pcl::PointIndices::Ptr vecrangePoint);

    // 去除重复的
    void NoRepeat(pcl::PointIndices::Ptr vecrangePoint);

    // 去除重复的
    void NoRepeat(std::vector<uint>& indices);

    // 显示过近检测结果
    void ShowTooNearCheck();

    // 标注过近检查线
    void LabelCrashLine(PointCloudT::Ptr crashPoint, double K, double gap);

    // 显示电力线结果
    void ShowTowerLines();

    // 地物误判恢复
    void RecoverGroundObject();

    // 获取电力线的两个端点
    void CalcLineInfo(PointCloudT::Ptr cloud, LineInfo &info);

    // 电力线合并
    void LineConnect();

    // 两条线是否接近
    bool IsConnectLine(const LineInfo &l1, const LineInfo &l2);

    // 向量是否一致
    bool IsSimilarityVec(vec &v1, vec &v2, double error = 0.15);

    // 向量是否一致
    bool IsSimilarityVec(vec &sta, vec &end, vec& sumvec);

    // 获取两点的向量，最大维度的大减小
    vec GetMaxAxisVec(vec &vpt1, vec& vpt2, bool bSwap = false);

    // 获取两点的向量，最大维度的大减小
    vec GetMaxAxisVec(const LineInfo &info);

    // 获取最大轴0x,1y
    int GetXyMaxAsix(vec &sta, vec& end);

    // 交换两个点
    void swapvec(vec &v1, vec &v2);

    // 判断是否是铁塔
    bool IsTower(PointCloudT::Ptr cloud, std::vector<uint> indices);

    // 计算此类的z跨度
    float GetClustersHeight(const std::vector<uint> &clusters, const PointCloudT &cloud_);

    // 地面误判恢复
    void RecoverGround(PointCloudT::Ptr ground, PointCloudT::Ptr outCloud);

    // 从删除view里的指定点云
    void RemoveViewClouds(QString name);

    // 从view里添加指定点云
    void SetShowClassNosyns(QString classname, bool show = true);

    // 从view里添加指定点云
    void SetShowClass(std::string classname, bool checked = true);

    // 从view里添加指定点云
    void UpdateShowClass(std::string classname);

    // 从view里删除所有碰撞球
    void RemoveViewBalls();

    // 从view里删除所有线模型
    void RemoveLineModels();

    // 刷新view
    void UpdateView();

    // 清除显示所有
    void ClearShowClass();

    // 地面过滤高程直方图法
    void ExtractGroundOnElevationHistogram();

    public slots:
    // 一键识别按钮
	void OnApply();

    // 收缩或者展开参数设置页面
	void OnGroupBoxCheck(bool cheched);
    
    // 地物提取
    void GroundObjectFilter();

    // 铁塔提取
	void ExtractTower();

    // 电力线提取
    void ExtractPowerLine();

    // 更新主界面视图
	void UpdateMainView();

    // 碰撞检查
    void TooNearCheck();
    void TooNearCheck2();

    // 地物粗提取
    void RoughExtractGroundObject();

    // 区域增长normal方法和color方法
    void QuyuzengzhangNormalAndColor();

    // 指定显示的内容
    void ShowSelectClass(bool checked);

    // 提取地面
    void ExtractGround();

    // 导出当前视图
    void ExportCurView();

    void Simplify();

    // 使用opencv寻找直线 
    void UseOpencvFineLine();

};
