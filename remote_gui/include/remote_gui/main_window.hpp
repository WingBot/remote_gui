/**
 * @file /include/remote_gui/main_window.hpp
 *
 * @brief Qt based gui for remote_gui.
 *
 * @date November 2010
 **/
#ifndef remote_gui_MAIN_WINDOW_H
#define remote_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>
#include <QMutex>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace remote_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
        void updateLoggingView(); // no idea why this can't connect automatically
        void updateLoggingView_sub(); // no idea why this can't connect automatically
        void pub_cmd();
        //void updateLogcamera();
       // void displayCamera(const QImage& image);
        void Open_Camera();
        void Gedit_Bashrc();
        void Local_Master();
/*private slots:
        void on_OpenCamera_clicked();
   */
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  //QImage qimage_;
  //mutable QMutex qimage_mutex_;
};

}  // namespace remote_gui

#endif // remote_gui_MAIN_WINDOW_H
