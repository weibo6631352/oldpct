#pragma once

#include <QSubDialogBase.h>
#include "ui_QCorrespondenceGrouping.h"

class QCorrespondenceGrouping : public QSubDialogBase
{
	Q_OBJECT

public:
	QCorrespondenceGrouping(QWidget *parent = Q_NULLPTR);
	~QCorrespondenceGrouping();

private:
	Ui::QCorrespondenceGrouping ui;

	public slots:
	void OnApply();
};
