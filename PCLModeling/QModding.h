#pragma once

#include <QSubDialogBase.h>
#include "ui_QModding.h"

class QModding : public QSubDialogBase
{
	Q_OBJECT

public:
	QModding(QWidget *parent = Q_NULLPTR);
	~QModding();
	
private:
	Ui::QModding ui;
public slots:
void OnApply();
};
