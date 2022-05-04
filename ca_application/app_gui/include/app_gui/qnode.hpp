/**
 * Date: December 2012
 * 
 * Author: Diego Rodríguez <diego.rodriguez@ikerlan.es>
 * 
 * Description: 
 * This *.hpp class defines the header file of a ROS Communication based object.
 * 
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef app_gui_QNODE_HPP_
#define app_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace app_gui {

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
           Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

  /* Getters of the ROS listened topics */
  std::string getsROSMasterURI();

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  QStringListModel logging_model;
  std::string sROSMasterURI;

  /* Private callback methods */
};

}  // namespace app_gui

#endif /* app_gui_QNODE_HPP_ */
