/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/remote_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace remote_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    /*********************cmd button*********************/
    QObject::connect(ui.sent_cmd, SIGNAL(clicked()), this, SLOT(pub_cmd()));
    QObject::connect(ui.OpenCamera, SIGNAL(clicked()), this, SLOT(Open_Camera()));
    QObject::connect(ui.LocalMaster, SIGNAL(clicked()), this, SLOT(Local_Master()));
    QObject::connect(ui.GeditBashrc, SIGNAL(clicked()), this, SLOT(Gedit_Bashrc()));
    ReadSettings();
  setWindowIcon(QIcon(":/images/logo.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.view_logging_sub->setModel(qnode.loggingModel_sub());
    QObject::connect(&qnode, SIGNAL(loggingUpdated_sub()), this, SLOT(updateLoggingView_sub()));

  //  QObject::connect(&qnode, SIGNAL(loggingCamera()), this, SLOT(updateLogcamera()));

    ui.dock_status->show();

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
  //  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateLoggingView_sub() {
        ui.view_logging_sub->scrollToBottom();
}
/*
void MainWindow::displayCamera(const QImage &image) {
  qimage_mutex_.lock();
  qimage_ = image.copy();
  ui.CameraLabel->setPixmap(QPixmap::fromImage(qimage_));
  ui.CameraLabel->resize(ui.CameraLabel->pixmap()->size());
  qimage_mutex_.unlock();
}


void MainWindow::updateLogcamera()
{
    MainWindow::displayCamera(qnode.image); //Video display is not smooth
}
*/
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright GIIM & Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "remote_gui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://arsl.local:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.233")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "remote_gui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::pub_cmd()
{
    qnode.sent_cmd();
}
void MainWindow::Open_Camera()
{
      system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch'&");
}

void MainWindow::Gedit_Bashrc()
{
      QString prog = "gedit";
      QStringList arg;
      arg <<"/home/zech/.bashrc";
      QProcess *proc = new QProcess;
      //proc->setStandardOutputFile("./output.txt");
      proc->start(prog, arg);
      proc->waitForFinished();
}

void MainWindow::Local_Master()
{
      system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash;roscore' ");
      /*QString prog2 = "gnome-terminal";
      QStringList arg2;
      arg2 <<"-x bash -c 'roscore'";
      QProcess *proc2 = new QProcess;
      //proc->setStandardOutputFile("./output.txt");
      proc2->start(prog2, arg2);
      proc2->waitForFinished();
      */
}
}  // namespace remote_gui


