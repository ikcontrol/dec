/**
 * Date: December 2020
 *
 * Author: Diego Rodríguez <diego.rodriguez@ikerlan.es>
 *
 * Description: 
 * This *.hpp file contains an implementation for the managemente of visual
 * functionality based on a QT User Graphic Interface.
 **/
#ifndef app_gui_MAIN_WINDOW_H
#define app_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtWidgets/QMainWindow>
#include <QStringListModel>
#include <QThread>
#include <QTimer>
#include <QString>
#include <QMessageBox>

/* Other usefull required libraries */
#include <stdlib.h>
#include <string>

/* Headers to obtain IP Address */
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/*****************************************************************************
** Namespace
*****************************************************************************/
namespace app_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  /* Configuration Methods */
  void closeEvent(QCloseEvent *event);              // Overloaded function
//  void ReadSettings();                              // Load up qt program settings at startup
//  void WriteSettings();                             // Save qt program settings when closing

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
  void on_actionAbout_triggered();
  void on_pbTestButton_clicked(bool check);

  /******************************************
  ** Manual connections
  *******************************************/
  void timerPeriodicAction();                
//    void moveLeft();                                 // Manual connection duo to an explicit use of the method


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  QStringListModel *logging_model;
  QTimer *ros_periodic_timer;

  /* Other interesting variables */
  std::string sIPAddress;

  /* Private Methods Linked with Public Q_SlOTS */
  void showButtonTestMessage();

private Q_SLOTS:
  // void on_pbReconnect_clicked();
};

}  // namespace app_gui

#endif // app_gui_MAIN_WINDOW_H