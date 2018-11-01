#pragma once

#include <QSubDialogBase.h>
#include "ui_QZhiTongXYZ.h"

class QZhiTongXYZ : public QSubDialogBase
{
	Q_OBJECT

public:
	QZhiTongXYZ(QWidget *parent = Q_NULLPTR);
	~QZhiTongXYZ();

private:
	Ui::QZhiTongXYZ ui;
public slots:
void OnApply();
};
