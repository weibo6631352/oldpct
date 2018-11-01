#pragma once

#include <QSubDialogBase.h>
#include "ui_QAligningObjectTemplates.h"

class QAligningObjectTemplates : public QSubDialogBase
{
	Q_OBJECT

public:
	QAligningObjectTemplates(QWidget *parent = Q_NULLPTR);
	~QAligningObjectTemplates();

private:
	Ui::QAligningObjectTemplates ui;

	public slots:
	void OnApply();
};
