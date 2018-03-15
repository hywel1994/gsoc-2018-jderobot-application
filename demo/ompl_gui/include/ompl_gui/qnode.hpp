/**
 * @file /include/ompl_gui/qnode.hpp
 *
 * @brief Ros ompl gui!
 *
 * @date march 2018
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ompl_gui_QNODE_HPP_
#define ompl_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ompl_gui {

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
  void setPose(double start_x, double start_y,double start_z, double goal_x, double goal_y, double goal_z);
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

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

  double startX;
  double startY;
  double startZ;
  double goalX;
  double goalY;
  double goalZ;

  ros::Publisher start_pose_publisher;
  ros::Publisher goal_pose_publisher;
  ros::Publisher init_ompl_publisher;
    QStringListModel logging_model;
};

}  // namespace ompl_gui

#endif /* ompl_gui_QNODE_HPP_ */
