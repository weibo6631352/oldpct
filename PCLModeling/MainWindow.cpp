#include "MainWindow.h"
#include <lasreader.hpp>
#include <laswriter.hpp>
#include <string.h>
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>  
#include <pcl/io/vtk_io.h> 
#include "QRendView.h"
#include <QFileDialog>
#include <QZhiTongXYZ.h>
#include <QLiqundian.h>
#include <QBilateralFilter.h>
#include <QGround.h>
#include <QLccp.h>
#include <QCpc.h>
#include <QRanSac.h>
#include <QQuyuzengzhangNormal.h>
#include <QQuyuzengzhangColor.h>
#include <QOushifenge.h>
#include <QChaotiJulei.h>
#include <QModding.h>
#include <QPowerLineClassification.h>
#include <QSimplify.h>
#include <algorithm>
#include <cloudEditorWidget.h>
#include <localTypes.h>
#include <PCManage.h>
#include <QFPFHFeature.h>
#include <QAligningObjectTemplates.h>
#include <QVFHTraining.h>
#include <QCorrespondenceGrouping.h>
#include <ProjectionXyPlane.h>
#include "PointHeightColor.h"
#include "ConvexBudding.h"
#include "QResampling.h"
#include "dvprint.h"
#include "Reconstruct.h"
#include "PublicFuncs.h"
#include <CGAL/IO/PLY_writer.h>
#include <CGAL/IO/PLY_reader.h>
#include <CGAL/Point_3.h>
#include <CGAL/Simple_cartesian.h>


MainWindow::MainWindow(QWidget *parent)
{
	ui.setupUi(this);
	menu_cornerbtn_ = new QPushButton(QStringLiteral("编辑"),ui.menuBar);
	connect(menu_cornerbtn_, &QPushButton::clicked, [this](){

		CloudEditorWidgetEx editer(this);
		editer.editer_->setCloud(*PCManage::ins().cloud_);
		editer.exec();
		*PCManage::ins().cloud_ = editer.editer_->getCloud();
		ui.centralWidget->UpdateView();
	});
	ui.menuBar->setCornerWidget(menu_cornerbtn_);
	InitUiConnect();
}

MainWindow::~MainWindow()
{

}

void
MainWindow::about()
{
	QMessageBox::about(this, QStringLiteral("点云建模"),
		QStringLiteral("点云建模\n"));
}

void
MainWindow::help()
{
	QMessageBox::about(this, QStringLiteral("编辑点云快捷键"),
		QStringLiteral("浏览模式\n") +
		QStringLiteral("  Drag:\t\t绕中心点旋转\n") +
		QStringLiteral("  Alt Drag:\t\tz轴移动 Z\n") +
		QStringLiteral("  Ctrl Drag:\t\t平面拖动\n") +
		QStringLiteral("  Shift Drag:\t\t缩放\n") +
		QStringLiteral("\n") +
		QStringLiteral("Selection Transform Mode\n") +
		QStringLiteral("  Drag:\t\tRotate about centeroid\n") +
		QStringLiteral("  Alt Drag:\t\tTranslate Z\n") +
		QStringLiteral("  Ctrl Drag:\t\tPan\n") +
		QStringLiteral("\n") +
		QStringLiteral("Mouse Picking\n") +
		QStringLiteral("  Left Click:\t\tSelect Point\n") +
		QStringLiteral("  Ctrl Left Click:\tDeselect Point\n") +
		QStringLiteral("  Shift Left Click:\tAppend to Selection\n") +
		QStringLiteral("\n") +
		QStringLiteral("2D Picking (Rubberband)\n") +
		QStringLiteral("  Drag:\t\t选择面\n") +
		QStringLiteral("  Ctrl Drag:\t\t挖洞面\n") +
		QStringLiteral("  Shift Drag:\t\t多选面\n") +
		QStringLiteral("\n") +
		QStringLiteral("Shortcut Keys\n") +
		QStringLiteral("  1:\t\tColor Points White\n") +
		QStringLiteral("  2:\t\tUse ColorWheel X\n") +
		QStringLiteral("  3:\t\tUse ColorWheel Y\n") +
		QStringLiteral("  4:\t\tUse ColorWheel Z\n") +
		QStringLiteral("  5:\t\tUse RGB Color\n") +
		QStringLiteral("  Ctrl C:\t\tCopy Selection\n") +
		QStringLiteral("  Ctrl X:\t\tCut Selection\n") +
		QStringLiteral("  Ctrl V:\t\t浏览模式\n") +
		QStringLiteral("  Ctrl Z:\t\t撤销\n") +
		QStringLiteral("  V:\t\tView Mode\n") +
		QStringLiteral("  T:\t\tSelection Transform Mode已选变换模式\n") +
		QStringLiteral("  E:\t\tPoint Selection Mode\n") +
		QStringLiteral("  S:\t\t2D 浏览模式\n") +
		QStringLiteral("  Del:\t\tDelete Selection\n") +
		QStringLiteral("  +:\t\tIncrease Point Size\n") +
		QStringLiteral("  -:\t\tDecrease Point Size\n") +
		QStringLiteral("  Ctrl +:\t\tInc. Selection Point Size\n") +
		QStringLiteral("  Ctrl -:\t\tDec. Selection Point Size\n") +
		QStringLiteral("  Esc:\t\tCancel Selection\n") 
		);
}

void MainWindow::InitUiConnect()
{
 	connect(ui.open, &QAction::triggered, this, &MainWindow::OpenFile);
 	connect(ui.write, &QAction::triggered, this, &MainWindow::WriteFile);
	connect(ui.zhitongXYZ, &QAction::triggered,[this](){QZhiTongXYZ dlg(this);dlg.exec(); });
	connect(ui.liqundian, &QAction::triggered, [this](){QLiqundian dlg(this); dlg.exec(); });
	connect(ui.shuangbianlvbo, &QAction::triggered, [this](){QBilateralFilter dlg(this); dlg.exec(); });
	connect(ui.dimian, &QAction::triggered, [this](){QGround dlg(this); dlg.exec(); });
	connect(ui.lccp, &QAction::triggered, [this](){QLccp dlg(this); dlg.exec(); });
	connect(ui.cpc, &QAction::triggered, [this](){QCpc dlg(this); dlg.exec(); });
	connect(ui.ranSaC, &QAction::triggered, [this](){QRanSac *dlg = new QRanSac(this); dlg->setAttribute(Qt::WA_DeleteOnClose); /*dlg->setModal(true);*/ dlg->show();  });
	connect(ui.quyuzengzhangNormal, &QAction::triggered, [this](){QQuyuzengzhangNormal dlg(this); dlg.exec(); });
	connect(ui.quyuzengzhangColor, &QAction::triggered, [this](){QQuyuzengzhangColor dlg(this); dlg.exec(); });
	connect(ui.oushifenge, &QAction::triggered, [this](){QOushifenge dlg(this); dlg.exec(); });
	connect(ui.chaotijulei, &QAction::triggered, [this](){QChaotiJulei dlg(this); dlg.exec(); });
	connect(ui.modeling, &QAction::triggered, [this](){QModding dlg(this); dlg.exec(); });
	connect(ui.powerline, &QAction::triggered, [this](){/*QPowerLineClassification dlg(this); dlg.exec(); */QPowerLineClassification *dlg = new QPowerLineClassification(this); dlg->setAttribute(Qt::WA_DeleteOnClose); /*dlg->setModal(true);*/ dlg->show(); });
	connect(ui.simplify, &QAction::triggered, [this](){QSimplify dlg(this); dlg.exec(); });
	connect(ui.editerhelp, &QAction::triggered, this, &MainWindow::help);
	connect(ui.about, &QAction::triggered, this, &MainWindow::about);
	connect(ui.FPFH, &QAction::triggered, [this](){QFPFHFeature *dlg = new QFPFHFeature(this); dlg->setAttribute(Qt::WA_DeleteOnClose); /*dlg->setModal(true);*/ dlg->show(); });
	connect(ui.qAligningObjectTemplates, &QAction::triggered, [this](){QAligningObjectTemplates dlg(this); dlg.exec(); });
	connect(ui.VFH_6DOF, &QAction::triggered, [this](){QVFHTraining dlg(this); dlg.exec(); });
	connect(ui.CorrespondenceGrouping3D, &QAction::triggered, [this](){QCorrespondenceGrouping dlg(this); dlg.exec(); });
	connect(ui.xyPlane, &QAction::triggered, [this](){ProjectionXyPlane::Fun(); });
	connect(ui.heightColor, &QAction::triggered, [this](){PointHeightColor::Fun(); });
    connect(ui.aobao, &QAction::triggered, [this](){ConvexBudding::Fun(); });
    connect(ui.offset_center, &QAction::triggered, this, &MainWindow::Offset_center);
    connect(ui.mls_simple, &QAction::triggered, [this](){QResampling dlg(this); dlg.exec();  });
    connect(ui.build_trian_model, &QAction::triggered, [](){Reconstruct::BuildSceneMesh(PCManage::ins().cloud_); });
}

void MainWindow::Offset_center()
{
    if (PCManage::ins().cloud_)
    {
        Transitionlocal(*PCManage::ins().cloud_);
        auto view = QRendView::MainRendView();
        if (view)
            QRendView::MainRendView()->UpdateView();
    }
}

void MainWindow::OpenFile()
{
	QString filename;
	filename = QFileDialog::getOpenFileName(this,
		QStringLiteral("选择文件"),
		"",
		QStringLiteral("点云 (*.las *.pcd *.xyz *.pts *.ply)")); //选择路径  

	if (filename.isEmpty())
	{
		return;
	}
	else
	{
		if (filename.toLower().endsWith(".las"))
		{
			loadFileLAS(filename.toLocal8Bit().data());
		}
		else if (filename.toLower().endsWith(".pcd"))
		{
			loadFilePCD(filename.toLocal8Bit().data());
		}
        else if (filename.toLower().endsWith(".xyz") || filename.toLower().endsWith(".pts"))
        {
            LoadXyzFile(filename.toLocal8Bit().data());
            //QString outfile = filename;
            //Xyz2Las(filename.toLocal8Bit().data(), outfile.replace(outfile.mid(outfile.lastIndexOf(QStringLiteral("."))), QStringLiteral(".las")).toLocal8Bit().data());
        }
        else if (filename.toLower().endsWith("ply"))
        {
            loadPlyFile(filename.toLocal8Bit().data());
        }
		else
		{
			return;
		}

        dd("%s是%s点云", filename.toLocal8Bit().data(), PCManage::ins().cloud_->height <= 1 ? "无序" : "有序");
		update();
	}
}

void MainWindow::WriteFile()
{
	QString filename;
	filename = QFileDialog::getSaveFileName(this,
		QStringLiteral("保存文件"), "", QStringLiteral("Las (*.las);;Pcd (*.pcd);;Xyz (*.xyz);;Ply (*.ply)"));

	if (filename.isEmpty())
	{
		return;
	}
	else
	{
		if (filename.toLower().endsWith(".las"))
		{
			WriteLasFile(filename);
		}
		else if (filename.toLower().endsWith(".pcd"))
		{
			WritePcdFile(filename);
		}
		else if (filename.toLower().endsWith(".xyz"))
		{
			WriteXyzFile(filename);
		}
        else if (filename.toLower().endsWith(".ply"))
        {
            WritePlyFile(filename);
        }
		else
		{

		}
	}
}

void MainWindow::WriteLasFile(const QString& file)
{
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	LASwriteOpener laswriteopener;
	laswriteopener.set_file_name(file.toLocal8Bit().data());
	if (!laswriteopener.active())
	{
		return;
	}

	LASheader lasheader;
    lasheader.x_offset = 0;
    lasheader.y_offset = 0;
    lasheader.z_offset = 0.0;
    lasheader.point_data_format = 3;	//1是xyzrgb		,颜色设置不上，未找到原因暂时屏蔽掉
    lasheader.point_data_record_length = 34;
    // init point 

	LASpoint laspoint;
	laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

	// open laswriter

	LASwriter* laswriter = laswriteopener.open(&lasheader);
	if (laswriter == 0)
	{
		fprintf(stderr, "ERROR: could not open laswriter\n");
		return;
	}
	for (auto it = cloud->begin(); it != cloud->end(); ++it)
	{
		laspoint.set_x(it->x);
		laspoint.set_y(it->y);
		laspoint.set_z(it->z);
        laspoint.have_rgb = true;
        laspoint.set_R(it->r*256);
        laspoint.set_G(it->g*256);
        laspoint.set_B(it->b*256);
		laswriter->write_point(&laspoint);
	}

	// update the header
	laswriter->update_header(&lasheader, TRUE);
	laswriter->close();
	delete laswriter;
}

void MainWindow::WritePcdFile(const QString& file)
{
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	if (cloud->size())
		pcl::io::savePCDFile(file.toLocal8Bit().data(), *cloud);
}

void MainWindow::WriteXyzFile(const QString& file)
{
	PointCloudT::Ptr cloud = PCManage::ins().cloud_;
	ofstream os(file.toLocal8Bit().data());

	if (os.is_open())
	{
		for (auto it = cloud->begin(); it != cloud->end(); ++it)
		{
            os << it->x << " " << it->y << " " << it->z << " " << (int)(it->r) << " " << (int)(it->g) << " " << (int)(it->b) << endl;
		}
		os.close();
	}
}

void MainWindow::LoadXyzFile(const std::string & file)
{
    PointCloudT::Ptr cloud = PCManage::ins().cloud_;
    cloud->clear();
    ifstream is(file);
    std::string line;
    if (is.is_open())
    {
        PointT pt; 
        float  x, y, z;
        uchar r, g, b;
        int scanfCount = 0;
        while (std::getline(is, line))
        {
            scanfCount = sscanf(line.c_str(), "%f %f %f %d %d %d", &x, &y, &z, &r, &g, &b);
            if (6 == scanfCount)
            {
                pt.r = r;
                pt.g = g;
                pt.b = b;
            }
            else if (3 == scanfCount)
            {
                pt.r = 0;
                pt.g = 0;
                pt.b = 0;
            }
            else
            {
                continue;
            }

            pt.x = x;
            pt.y = y;
            pt.z = z;
            cloud->push_back(pt);
        }
        is.close();
    }
    ui.centralWidget->UpdateView();
}

void MainWindow::loadFilePCD(const std::string &filename)
{
	QRendView* ins = ui.centralWidget;
	if (pcl::io::loadPCDFile<Point3D>(filename, *PCManage::ins().cloud_) == -1)
		throw;
	ins->UpdateView();
}

void MainWindow::loadFileLAS(const std::string &filename)
{
	QRendView* ins = ui.centralWidget;
	PclCloudPtr pcl_cloud_ptr;
	Cloud3D tmp;
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(filename.c_str());
	if (!lasreadopener.active())
	{
		return;
	}
	LASreader* lasreader = lasreadopener.open();
	tmp.resize(lasreader->header.number_of_point_records);
	size_t step = 0;
	while (lasreader->read_point())
	{
		tmp.at(step).x = lasreader->point.get_x();
		tmp.at(step).y = lasreader->point.get_y();
		tmp.at(step).z = lasreader->point.get_z();

        tmp.at(step).r = lasreader->point.get_R() / (256);
        tmp.at(step).g = lasreader->point.get_G() / (256);
        tmp.at(step).b = lasreader->point.get_B() / (256);
		++step;
	}
	lasreader->close();
    delete lasreader;
	pcl::copyPointCloud(tmp, *PCManage::ins().cloud_);
	ins->UpdateView();
}

void MainWindow::loadPlyFile(const std::string & file)
{
    QRendView* ins = ui.centralWidget;
    std::ifstream in(file);


//     typedef CGAL::Simple_cartesian<double> Kernel;
//     typedef Kernel::Point_3 Point;

    std::vector<CGAL::Simple_cartesian<double>::Point_3> pts;
    if (!in
        || !(CGAL::read_ply_points(in, std::back_inserter(pts))))
    {
        return;
    }
    in.close();
    PointCloudT::Ptr cloud = PCManage::ins().cloud_;
    cloud->clear();
    cloud->resize(pts.size());
    for (int i = 0; i < pts.size(); ++i)
    {
        cloud->at(i).x = pts[i].x();
        cloud->at(i).y = pts[i].y();
        cloud->at(i).z = pts[i].z();
     }
    
    ins->UpdateView();
}


void MainWindow::WritePlyFile(const QString & file)
{
    PointCloudT::Ptr cloud = PCManage::ins().cloud_;


    std::ofstream f(file.toLocal8Bit().data());
    if (f.is_open())
    {
        f << "ply" << std::endl
            << "format ascii 1.0" << std::endl
            << "element vertex " << cloud->size() << std::endl
            << "property float x" << std::endl
            << "property float y" << std::endl
            << "property float z" << std::endl
            << "property uchar red" << std::endl
            << "property uchar green" << std::endl
            << "property uchar blue" << std::endl
            << "end_header" << std::endl;

        for (std::size_t i = 0; i < cloud->size(); ++i)
        {
            f << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " ";
            f << "0 0 0" << std::endl;
        }
        f.close();
    }
}
