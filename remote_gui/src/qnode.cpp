/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/remote_gui/qnode.hpp"
#include <QStringListModel> //add

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace remote_gui {

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
	ros::init(init_argc,init_argv,"remote_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
 // image_transport::ImageTransport it(n);

	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    buttom_publisher = n.advertise<std_msgs::String>("cmd",1000);
    chatter_subscriber = n.subscribe("cmd", 1000, &QNode::Callback, this);
   // image_sub = it.subscribe("/usb_cam/image_raw",1000,&QNode::myCallback_img,this);

    //chatter_subscriber = n.subscribe("chatter"，1000，＆QNode ::Callback,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"remote_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
  //image_transport::ImageTransport it(n);
	// Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    buttom_publisher = n.advertise<std_msgs::String>("cmd",1000);
    chatter_subscriber = n.subscribe("cmd", 1000, &QNode::Callback, this);
    //image_sub = it.subscribe("/usb_cam/image_raw",1000,&QNode::myCallback_img,this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Successfull Connection!";
    msg.data = ss.str();
    chatter_publisher.publish(msg);
    log(Info,msg.data);
	while ( ros::ok() ) {
/*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
*/
		ros::spinOnce();
		loop_rate.sleep();
//		++count;
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

    void QNode::log_sub( const LogLevel &level, const std::string &msg) {
        logging_model_sub.insertRows(logging_model_sub.rowCount(),1);
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
        logging_model_sub.setData(logging_model_sub.index(logging_model_sub.rowCount()-1),new_row);
        Q_EMIT loggingUpdated_sub(); // used to readjust the scrollbar
    }

    void QNode::Callback(const std_msgs::StringConstPtr &submsg)
    {
        log_sub(Info,std::string("sub:") + submsg->data.c_str());
    }

    void QNode::sent_cmd()
    {
        if(ros::ok())
        {
            std_msgs::String msg;
            std::stringstream ss;
            ss << "Send image";
            msg.data = ss.str();
            buttom_publisher.publish(msg);
            log(Info,std::string("I sent:")+msg.data);
            ros::spinOnce();
        }
    }

/*
    void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
    {
      std_msgs::String imgmsg;
      std::stringstream ss;
      ss << "a new image";
      imgmsg.data = ss.str();
      //buttom_publisher.publish(msg);

      try
      {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img = cv_ptr->image;
        image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
        //ROS_INFO("I'm setting picture in remote_gui callback function!");
        log_sub(Info,std::string("I got:")+imgmsg.data);
        Q_EMIT loggingCamera();
        //ROS_INFO("I'm  function!");
      }

      catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
      }
      */
}  // namespace remote_gui
