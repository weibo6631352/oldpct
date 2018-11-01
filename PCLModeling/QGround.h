#pragma once

#include <QSubDialogBase.h>
#include "ui_QGround.h"
#include "PCManage.h"

//http://www.echojb.com/cloud/2017/06/22/438253.html

class QGround : public QSubDialogBase
{
	Q_OBJECT

public:
	QGround(QWidget *parent = Q_NULLPTR);
	~QGround();
    static  void Fun(PointCloudT::Ptr cloud, PointCloudT &ground);
private:
	Ui::QGround ui;
public slots:
void OnApply();

};
