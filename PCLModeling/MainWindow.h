#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "QRendView.h"
#include <QtGui>
#include <QMainWindow>
#include <QActionGroup>
#include <QSpinBox>
#include <QSlider>
#include <QMessageBox>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <localTypes.h>
#include <QPushButton>
#include <CloudEditorWidgetEx.h>

/**
* 主界面框架类
* @author 王伟博
* 承载了菜单栏、工具栏、主视图、事件交互
*/
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);
	~MainWindow();
	
    // 加载xyz文件
    void LoadXyzFile(const std::string &file);

    // 加载pcd文件
	void loadFilePCD(const std::string &filename);
	
    // 加载las文件
    void loadFileLAS(const std::string &filename);

    // 加载ply点云文件
    void loadPlyFile(const std::string & file);
	
    // 写las文件
    void WriteLasFile(const QString& file);
	
    // 写pcd文件
    void WritePcdFile(const QString& file);
 	
    // 写xyz文件
    void WriteXyzFile(const QString& file);

    // 写ply文件
    void WritePlyFile(const QString & file);


private:
    void Offset_center();

 public slots:
 // 打开文件
 void OpenFile();

 // 写文件
 void WriteFile();
 
 // 关于
 void about();

 // 帮助
 void help();


private:
    // 连接信号槽
    void InitUiConnect();

    Ui::MainWindow ui;

    // 编辑按钮
    QPushButton* menu_cornerbtn_;
};
