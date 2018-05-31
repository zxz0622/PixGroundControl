#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "autopilot_interface.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Serial_Port *serial_port;
    Autopilot_Interface *autopilot;
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
