/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/app_gui/main_window.hpp"
#include <QtWidgets>
#include <QMessageBox>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace app_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
/* Constructor of the class */
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

//    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /* Starting a timer object to periodically update the robot status of the task */
  ros_periodic_timer = new QTimer(this);
  connect(ros_periodic_timer, SIGNAL(timeout()),this,SLOT(timerPeriodicAction()));
  ros_periodic_timer->start(10000);            // Time meassured in milliseconds

  /* Initialization of ROS Node */
  if (!qnode.init()) {
    printf("Ros node cannot be started ...\n");
    QMessageBox::critical(this,tr("ROS CONECTION ERROR"),tr("The node cannot be initialized because no ROS Master was found. Please, start a ROS master and click on 'Reconect' button to try reconnection"));
    ui.leROSMasterIP->setText(QString::fromUtf8(" "));
    // ui.pbReconnect->setEnabled(true);
  }
  else {
    printf("Ros node was started ...\n");
    /* Obtaining mastr URI from ROS */
    ui.leROSMasterIP->setText(QString::fromUtf8(qnode.getsROSMasterURI().c_str()));
    // ui.pbReconnect->setEnabled(false);
  }

  /******************************
   ** Obtaining IP Address of the Local Machine
  *******************************/
  struct ifaddrs *ifAddr=nullptr, *ifa=nullptr;
  int iAuxCnt = 0;
  getifaddrs(&ifAddr);
  for (ifa = ifAddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa ->ifa_addr->sa_family==AF_INET) { // is a valid IP4 Address
        char addressBuffer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr, addressBuffer, INET_ADDRSTRLEN);
        iAuxCnt++;
        if (iAuxCnt == 2){
          iAuxCnt = 0;
          sIPAddress = addressBuffer;
          ui.leLocalIP->setText(QString::fromUtf8(sIPAddress.c_str()));
        }
     }
  }
  if (ifAddr!=nullptr)
     freeifaddrs(ifAddr);         // Fre IfAddr pointer

	/*********************
	** Logging
	**********************/
//	ui.view_logging->setModel(qnode.loggingModel());
//  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
  ** Auto Start
  **********************/
//    if ( ui.checkbox_remember_settings->isChecked() ) {
//    }

  /*************************
   ** Button test - explicit way
   ************************/
//   QObject::connect(ui.button_left,SIGNAL(clicked()),this,SLOT(moveLeft()));
}

/* Destructor of the class */
MainWindow::~MainWindow() {
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::on_pbTestButton_clicked(bool check) {
  showButtonTestMessage();
}

void MainWindow::showButtonTestMessage() {          // Private method implemented to do an action when on_button_test_clicked
  QMessageBox msgBox;

  msgBox.setText("Button test ...");

  msgBox.exec();
  close();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::timerPeriodicAction() {
  printf("Timer time has finished!");
}

//void MainWindow::moveLeft() {
//  logging_model = qnode.loggingModel();
//  logging_model->insertRows(logging_model->rowCount(), 1);
//  std::stringstream logging_model_msg;
//  logging_model_msg << "move to left ...";
//  QVariant new_row(QString(logging_model_msg.str().c_str()));
//  logging_model->setData(logging_model->index(logging_model->rowCount()-1), new_row);

//  std::cout << logging_model->rowCount() << std::endl;
//  std::cout << logging_model_msg.str().c_str() << std::endl;
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Ikerlan S. Coop.</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
//	WriteSettings();
	QMainWindow::closeEvent(event);
}

//void MainWindow::ReadSettings() {
//    QSettings settings("Qt-Ros Package", "user_interface");
//    restoreGeometry(settings.value("geometry").toByteArray());
//    restoreState(settings.value("windowState").toByteArray());
//    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
//    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
//    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
////    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	//ui.line_edit_topic->setEnabled(false);
//    }
//}

//void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "user_interface");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
////    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
//}

/*****************************************************************************************************************************
 ************************************************    [PRIVATE Q_SLOTS] *******************************************************
 ****************************************************************************************************************************/
// void MainWindow::on_pbReconnect_clicked()
// {
//   printf("Trying to reconnect ...\n");
//   /* Initialization of ROS Node */
//   if (!qnode.init()) {
//     printf("Ros node cannot be started ...\n");
//     QMessageBox::critical(this,tr("ROS CONECTION ERROR"),tr("The node cannot be initialized because no ROS Master was found. Please, start a ROS master and click on 'Reconect' button to try reconnection"));
//     ui.leROSMasterIP->setText(QString::fromUtf8(" "));
//     ui.pbReconnect->setEnabled(true);
//     ui.leLocalIP->setText(QString::fromUtf8(" "));
//   }
//   else {
//     printf("Ros node was started ...\n");
//     ui.leROSMasterIP->setText(QString::fromUtf8(qnode.getsROSMasterURI().c_str()));
//     ui.pbReconnect->setEnabled(false);
//   }
// }


}  // namespace app_gui
