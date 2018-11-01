#pragma once
#include "cloudEditorWidget.h"
#include <QDialog>
class CloudEditorWidgetEx :
	 public QDialog
{
public:
	CloudEditorWidgetEx(QWidget *parent = 0);
	~CloudEditorWidgetEx();

	CloudEditorWidget *editer_;

};

