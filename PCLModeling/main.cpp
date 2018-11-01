#include "MainWindow.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

    // 设置vtk警告窗口不显示
    vtkOutputWindow::SetGlobalWarningDisplay(0);

    // 显示主窗口
	MainWindow w;
	w.show();
	
	return a.exec();
}
