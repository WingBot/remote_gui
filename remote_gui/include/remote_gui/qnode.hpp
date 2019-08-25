/**
 * @file /include/remote_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef remote_gui_QNODE_HPP_
#define remote_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <opencv-3.3.1-dev/opencv2/core.hpp>
#include <opencv-3.3.1-dev/opencv2/highgui/highgui.hpp>
#include <opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <QImage>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace remote_gui {

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
  QStringListModel* loggingModel_sub() { return &logging_model_sub; }
  void log_sub( const LogLevel &level, const std::string &msg);
  void Callback(const std_msgs::StringConstPtr&submsg);
  void sent_cmd();
  //void myCallback_img(const sensor_msgs::ImageConstPtr& msg);
  //QImage image;



Q_SIGNALS:
  void loggingUpdated();
  void loggingUpdated_sub();
  void rosShutdown();
  //void loggingCamera();

private:
	int init_argc;
	char** init_argv;
  cv::Mat img;
	ros::Publisher chatter_publisher;
  ros::Publisher buttom_publisher;
  ros::Subscriber chatter_subscriber;
  image_transport::Subscriber image_sub;
  QStringListModel logging_model;
  QStringListModel logging_model_sub;
  QStringListModel logging_model_img;
};

}  // namespace remote_gui

#endif /* remote_gui_QNODE_HPP_ */
