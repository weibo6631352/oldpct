#pragma once

#include <QSubDialogBase.h>
#include "ui_QRanSac.h"
#include <QRendView.h>

class QRanSac : public QSubDialogBase
{
	Q_OBJECT

public:
	QRanSac(QWidget *parent = Q_NULLPTR);
	~QRanSac();
	
private:
	Ui::QRanSac ui;
	PointCloudT::Ptr cloud_;
public slots:
	void OnApply();
};
