#pragma once

#include <QSubDialogBase.h>
#include "ui_QQuyuzengzhangColor.h"
#include "PCManage.h"
//https://www.cnblogs.com/ironstark/p/4998037.html
class QQuyuzengzhangColor : public QSubDialogBase
{
	Q_OBJECT

public:
	QQuyuzengzhangColor(QWidget *parent = Q_NULLPTR);
	~QQuyuzengzhangColor();
    static void Fun(PointCloudT::Ptr cloud = nullptr, PointCloudT::Ptr colorCloud = nullptr);
private:
	Ui::QQuyuzengzhangColor ui;

public slots:
void OnApply();

};
