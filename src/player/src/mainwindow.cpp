#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_ros_thread = new ROSThread(this, &m_mutex);
    ui->setupUi(this);
    m_ros_thread->start();


    ui->pushButton_play->setEnabled(false);
    ui->pushButton_pause->setEnabled(false);

    ui->doubleSpinBox_speed->setRange(0.01,20.0);
    ui->doubleSpinBox_speed->setValue(1.0);
    ui->doubleSpinBox_speed->setSingleStep(0.01);
    m_play_speed = 1.0;

    ui->horizontalSlider_pos->setRange(0,10000);
    ui->horizontalSlider_pos->setValue(0);
    m_slider_value = 0;
    m_slider_checker = false;

    connect(m_ros_thread, SIGNAL(StampShow(quint64)), this, SLOT(SetStamp(quint64)));


}

MainWindow::~MainWindow()
{
    delete ui;
    m_ros_thread->quit();
    if(!m_ros_thread->wait(500))
    {
        m_ros_thread->terminate();
        m_ros_thread->wait();
    }
}

void MainWindow::RosInit(ros::NodeHandle &n)
{
    m_ros_thread->ros_initialize(n);    
}

void MainWindow::on_pushButton_load_path_pressed()
{
    ui->pushButton_load_path->setEnabled(false);

    QFileDialog dialog;
    m_data_load_path = dialog.getExistingDirectory();
    ui->textEdit_load_path->setText(m_data_load_path);
    m_ros_thread->m_data_load_path = m_data_load_path.toUtf8().constData();

    m_ros_thread->m_play_flag = false;
    m_ros_thread->m_pause_flag = false;

    ui->pushButton_play->setEnabled(true);
    ui->pushButton_pause->setEnabled(false);

    m_ros_thread->ready();

    ui->pushButton_load_path->setEnabled(true);
}

void MainWindow::on_pushButton_play_pressed()
{
    ui->pushButton_play->setEnabled(false);
    ui->pushButton_pause->setEnabled(true);

    m_ros_thread->m_play_flag = true;
    m_ros_thread->m_pause_flag = false;

    m_ros_thread->m_data_stamp_data.active = true;

}

void MainWindow::on_pushButton_pause_pressed()
{
    ui->pushButton_play->setEnabled(false);
    ui->pushButton_pause->setEnabled(false);

    m_ros_thread->m_play_flag = false;
    m_ros_thread->m_pause_flag = true;

    ui->pushButton_play->setEnabled(true);
    ui->pushButton_pause->setEnabled(false);
}

void MainWindow::on_horizontalSlider_pos_valueChanged(int value)
{
    if (value < 1)    m_slider_value = 1;
    else              m_slider_value = value;  
}

void MainWindow::on_horizontalSlider_pos_sliderPressed()
{
    m_slider_checker = true;
}

void MainWindow::on_horizontalSlider_pos_sliderReleased()
{
    m_ros_thread->ResetProcessStamp(m_slider_value);
    m_slider_checker = false;
}

void MainWindow::on_doubleSpinBox_speed_valueChanged(double arg1)
{
    m_play_speed = arg1;
    m_ros_thread->m_play_speed = m_play_speed;
}

void MainWindow::SetStamp(quint64 stamp)
{
    //set slide bar
    if(m_slider_checker == false)
    {
        ui->horizontalSlider_pos->setValue(static_cast<int>(static_cast<float>(stamp - m_ros_thread->m_initial_data_stamp)/static_cast<float>(m_ros_thread->m_last_data_stamp - m_ros_thread->m_initial_data_stamp)*10000));
    }
}
