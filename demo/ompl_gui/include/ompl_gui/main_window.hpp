/**
 * @file /include/ompl_gui/main_window.hpp
 *
 * @brief Qt based gui for ompl_gui.
 *
 * @date march 2018
 **/
#ifndef ompl_gui_MAIN_WINDOW_H
#define ompl_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <string>
#include <iostream>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ompl_gui {

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

  void setPose();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

  void on_button_update_clicked(bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace ompl_gui

#endif // ompl_gui_MAIN_WINDOW_H
