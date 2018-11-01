#pragma once

#include <QSubDialogBase.h>
#include "ui_QLiqundian.h"
#include "PCManage.h"

class QLiqundian : public QSubDialogBase
{
	Q_OBJECT

public:
	QLiqundian(QWidget *parent = Q_NULLPTR);
	~QLiqundian();
	
	static void Fun(PointCloudT::Ptr srcCloud = nullptr);
private:
	Ui::QLiqundian ui;

public slots:
void OnApply();
};
