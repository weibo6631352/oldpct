#pragma once

#include <QSubDialogBase.h>
#include "ui_QFPFHFeature.h"

class QFPFHFeature : public QSubDialogBase
{
	Q_OBJECT

public:
	QFPFHFeature(QWidget *parent = Q_NULLPTR);
	~QFPFHFeature();

    
	PointCloudT::Ptr cloud_;
    PointCloudT selectPoints_;
public slots:
	void OnApply();
	void comboxChange(int index);
private:
	Ui::QFPFHFeature ui;
};
