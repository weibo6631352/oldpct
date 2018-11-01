#pragma once

#include <QSubDialogBase.h>
#include "ui_QSimplify.h"
#include "PCManage.h"

class QSimplify : public QSubDialogBase
{
	Q_OBJECT

public:
	QSimplify(QWidget *parent = Q_NULLPTR);
	~QSimplify();
	static void Fun(PointCloudT::Ptr cloud = nullptr);
private:
	Ui::QSimplify ui;
public slots:
void OnApply();
};
