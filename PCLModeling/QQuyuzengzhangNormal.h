#pragma once

#include <QSubDialogBase.h>
#include "ui_QQuyuzengzhangNormal.h"
#include "PCManage.h"

class QQuyuzengzhangNormal : public QSubDialogBase
{
	Q_OBJECT

public:
	QQuyuzengzhangNormal(QWidget *parent = Q_NULLPTR);
	~QQuyuzengzhangNormal();
    static void Fun(PointCloudT::Ptr cloud_ = nullptr, PointCloudT::Ptr colorCloud = nullptr);
private:
	Ui::QQuyuzengzhangNormal ui;

public slots:
void OnApply();
};
