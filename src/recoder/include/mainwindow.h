#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "userdef.h"
#include "ROSThread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void RosInit(ros::NodeHandle &n);

private slots:
    void on_pushButton_save_path_pressed();
    void on_pushButton_start_pressed();
    void on_pushButton_stop_pressed();

private:
    Ui::MainWindow *ui;

    QMutex m_mutex;
    ROSThread *m_ros_thread;

    QString m_data_save_path;

    QList<QString> m_table_topic_name;
    QList<QString> m_table_topic_cnt;

    QTimer *m_timer;
    void OnTimerCallback();

//signals:



};

#endif // MAINWINDOW_H
