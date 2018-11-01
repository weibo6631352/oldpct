#pragma once

#include <QSubDialogBase.h>
#include "ui_QResampling.h"
#include "PCManage.h"

class QResampling : public QSubDialogBase
{
    Q_OBJECT

public:
    QResampling(QWidget *parent = Q_NULLPTR);
    ~QResampling();
    static void PclMlsReconstruct(double k = 5, PointCloudT::Ptr color_cloud = nullptr);

public slots:
    void on_click();
private:
    Ui::QResampling ui;
};
