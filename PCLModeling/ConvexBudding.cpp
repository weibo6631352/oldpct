#include "ConvexBudding.h"

#include "dvprint.h"
#include "PCManage.h"
#include "QRendView.h"

#include <QMessageBox>
#include <QString>
#include <QStringList>
#include <QFileDialog>
#include <QComboBox>


#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>  
#include <vtkPointsProjectedHull.h>
#include <vtkPolyLine.h>
#include <vtkRenderWindow.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>

#include <list>

ConvexBudding::ConvexBudding()
{
}


ConvexBudding::~ConvexBudding()
{
}

void ConvexBudding::Fun()
{
    QRendView* ins = QRendView::MainRendView();
    PointCloudT::Ptr cloud = PCManage::ins().cloud_;
   


    vtkSmartPointer<vtkPoints> point0 =
        vtkSmartPointer<vtkPoints>::New();
    for (int i = 0; i < cloud->size(); ++i)
    {
        PointT &pt = cloud->at(i);
        point0->InsertNextPoint(pt.x, pt.y, pt.z);
    }

    //[6] --提取轮廓凸包  
    vtkSmartPointer<vtkPointsProjectedHull> hull =
        vtkSmartPointer<vtkPointsProjectedHull>::New();
    hull->DeepCopy(point0);
    int zSize = hull->GetSizeCCWHullZ();

    double *pts = new double[zSize * 3];
    hull->GetCCWHullZ(pts, zSize);

    vtkSmartPointer<vtkPoints> zHullPoints =
        vtkSmartPointer<vtkPoints>::New();
    for (int i = 0; i < zSize; ++i)
    {
        double xval = pts[3 * i];
        double yval = pts[3 * i + 1];
        double zval = pts[3 * i + 2];
//         qDebug() << "(x,z)value of point" << i << ":("
//             << xval << yval << ")";
        zHullPoints->InsertNextPoint(xval, yval, zval);
    }
    // Insert the first point again to close the loop  
    zHullPoints->InsertNextPoint(pts[0], pts[1], pts[2]);
    //[6]  

    //[7] --画出凸包  
    // Display the x hull  
    vtkSmartPointer<vtkPolyLine> zPolyLine =
        vtkSmartPointer<vtkPolyLine>::New();
    zPolyLine->GetPointIds()->SetNumberOfIds(zHullPoints->GetNumberOfPoints());

    for (vtkIdType i = 0; i < zHullPoints->GetNumberOfPoints(); i++)
    {
        zPolyLine->GetPointIds()->SetId(i, i);
    }

    // Create a cell array to store the lines in and add the lines to it  
    vtkSmartPointer<vtkCellArray> cells =
        vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(zPolyLine);

    // Create a polydata to store everything in  
    vtkSmartPointer<vtkPolyData> polyData =
        vtkSmartPointer<vtkPolyData>::New();

    // Add the points to the dataset  
    polyData->SetPoints(zHullPoints);

    // Add the lines to the dataset  
    polyData->SetLines(cells);
    //[7]  
    vtkSmartPointer<vtkPolyDataMapper> zHullMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    zHullMapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> zHullActor =
        vtkSmartPointer<vtkActor>::New();
    zHullActor->SetMapper(zHullMapper);
    zHullActor->GetProperty()->SetLineWidth(1);
    zHullActor->GetProperty()->SetColor(1, 0, 0);

    vtkSmartPointer<vtkRenderer> renderer =
        vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(zHullActor);
    renderer->SetBackground(1.0, 1.0, 1.0);

    vtkSmartPointer<vtkRenderWindow> renderWindow =
        vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindow->SetSize(640, 320);
    renderWindow->Render();
    renderWindow->SetWindowName("contour extract");
    renderWindow->Render();

    renderWindowInteractor->Start();



    ins->UpdateView();
}
