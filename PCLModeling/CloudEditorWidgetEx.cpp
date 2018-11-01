#include "CloudEditorWidgetEx.h"
#include <QVBoxLayout>



CloudEditorWidgetEx::CloudEditorWidgetEx(QWidget *parent /*= 0*/)
:QDialog(parent)
{
    setWindowTitle(QStringLiteral("±à¼­"));
	QVBoxLayout *layout = new QVBoxLayout(this);
	editer_ = new CloudEditorWidget();
	layout->addWidget(editer_);
	resize(800, 600);
}


CloudEditorWidgetEx::~CloudEditorWidgetEx()
{
}
