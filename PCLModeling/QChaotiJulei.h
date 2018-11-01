#pragma once

#include <QSubDialogBase.h>
#include "ui_QChaotiJulei.h"

class QChaotiJulei : public QSubDialogBase
{
	Q_OBJECT

public:
	QChaotiJulei(QWidget *parent = Q_NULLPTR);
	~QChaotiJulei();
	
private:
	Ui::QChaotiJulei ui;

public slots:
void OnApply();
};
