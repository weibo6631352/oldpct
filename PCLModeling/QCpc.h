#pragma once

#include <QSubDialogBase.h>
#include "ui_QCpc.h"

class QCpc : public QSubDialogBase
{
	Q_OBJECT

public:
	QCpc(QWidget *parent = Q_NULLPTR);
	~QCpc();
	
private:
	Ui::QCpc ui;

public slots:
void OnApply();
};
