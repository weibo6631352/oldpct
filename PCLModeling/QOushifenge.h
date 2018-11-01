#pragma once

#include <QSubDialogBase.h>
#include "ui_QOushifenge.h"
#include <PCManage.h>

class QOushifenge : public QSubDialogBase
{
	Q_OBJECT

public:
	QOushifenge(QWidget *parent = Q_NULLPTR);
	~QOushifenge();
    static void Fun(PointCloudT::Ptr cloud);
    static void Fun(PointCloudT::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices);
    static void Fun(PointCloudT::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices, double k);
private:
	Ui::QOushifenge ui;
public slots:
void OnApply();
};
