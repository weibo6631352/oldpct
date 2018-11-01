#pragma once

#include <QDialog>
#include "ui_QSubDialogBase.h"
#include <boost/property_tree/ptree.hpp>    
#include <boost/property_tree/ini_parser.hpp>    
// #include "boost/property_tree/json_parser.hpp"
// #include "boost/property_tree/xml_parser.hpp"

class QSubDialogBase : public QDialog
{
	Q_OBJECT

public:
	QSubDialogBase(QWidget *parent = Q_NULLPTR);
	~QSubDialogBase();


private:
	Ui::QSubDialogBase ui;
};
