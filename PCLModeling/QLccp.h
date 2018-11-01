#pragma once

#include <QSubDialogBase.h>
#include "ui_QLccp.h"

class QLccp : public QSubDialogBase
{
	Q_OBJECT

public:
	QLccp(QWidget *parent = Q_NULLPTR);
	~QLccp();
	
private:
	Ui::QLccp ui;

public slots:
void OnApply();
};
