/**
 * @file /src/qnode.cpp
 *
 * @brief Ros ompl gui!
 *
 * @date march 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ompl_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ompl_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"ompl_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  start_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("start_pose", 1000);
  goal_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
  init_ompl_publisher = n.advertise<std_msgs::Int8>("init_ompl",1000);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"ompl_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  start_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("start_pose", 1000);
  goal_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
  init_ompl_publisher = n.advertise<std_msgs::Int8>("init_ompl",1000);
	start();
	return true;
}

void QNode::setPose(double start_x, double start_y, double start_z, double goal_x, double goal_y, double goal_z){
  startX = start_x;
  startY = start_y;
  startZ = start_z;
  goalX = goal_x;
  goalY = goal_y;
  goalZ = goal_z;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	while ( ros::ok() ) {

    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose;
    std_msgs::Int8 init_ompl;

    start_pose.pose.position.x = startX;
    start_pose.pose.position.y = startY;
    start_pose.pose.position.z = startZ;
    goal_pose.pose.position.x = goalX;
    goal_pose.pose.position.y = goalY;
    goal_pose.pose.position.z = goalZ;
    init_ompl.data = 1;

    start_pose_publisher.publish(start_pose);
    goal_pose_publisher.publish(goal_pose);
    init_ompl_publisher.publish(init_ompl);
    log(Info,std::string("I sent: pose"));
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace ompl_gui
