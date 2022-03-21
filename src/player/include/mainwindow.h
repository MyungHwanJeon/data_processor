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
    void on_pushButton_load_path_pressed();
    void on_pushButton_play_pressed();
    void on_pushButton_pause_pressed();

    void on_horizontalSlider_pos_valueChanged(int value);
    void on_horizontalSlider_pos_sliderPressed();
    void on_horizontalSlider_pos_sliderReleased();

    void on_doubleSpinBox_speed_valueChanged(double arg1);

    void SetStamp(quint64 stamp);


private:
    Ui::MainWindow *ui;

    QMutex m_mutex;
    ROSThread *m_ros_thread;

    QString m_data_load_path;
    int m_slider_value;
    bool m_slider_checker;

    double m_play_speed;


//signals:



};

#endif // MAINWINDOW_H
