#pragma once

#include "ui_QBilateralFilter.h"
#include <QSubDialogBase.h>

class QBilateralFilter : public QSubDialogBase
{
	Q_OBJECT

public:
	QBilateralFilter(QWidget *parent = Q_NULLPTR);
	~QBilateralFilter();

private:
	Ui::QBilateralFilter ui;
public slots:
void OnApply();

};
