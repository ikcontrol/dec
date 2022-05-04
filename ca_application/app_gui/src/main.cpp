/**
 * Date: December 2020
 *
 * Author: Diego Rodríguez <diego.rodríguez@ikerlan.es>
 * 
 * Descripticon:
 * Main script for a gui interface
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QApplication>
#include "../include/app_gui/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    app_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}