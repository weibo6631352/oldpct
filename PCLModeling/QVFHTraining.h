#pragma once

#include <QSubDialogBase.h>
#include "ui_QVFHTraining.h"

class QVFHTraining : public QSubDialogBase
{
	Q_OBJECT

public:
	QVFHTraining(QWidget *parent = Q_NULLPTR);
	~QVFHTraining();

private:
	Ui::QVFHTraining ui;

	public slots:
	void OnApply();
	void OnBuild();
	void OnCreateVfh();
};
