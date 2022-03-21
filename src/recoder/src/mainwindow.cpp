#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->pushButton_start->setEnabled(true);
    ui->pushButton_stop->setEnabled(false);

    ui->tableWidget->setColumnWidth(0, 250);
    ui->tableWidget->setColumnWidth(1, 100);

    ui->tableWidget->setItem(0, 0, new  QTableWidgetItem("Franka States"));
    ui->tableWidget->setItem(1, 0, new  QTableWidgetItem("Joint States"));
    ui->tableWidget->setItem(2, 0, new  QTableWidgetItem("Camera Info"));
    ui->tableWidget->setItem(3, 0, new  QTableWidgetItem("Camera Color"));
    ui->tableWidget->setItem(4, 0, new  QTableWidgetItem("Camera Depth"));
    ui->tableWidget->setItem(5, 0, new  QTableWidgetItem("Detection 2D"));
    ui->tableWidget->setItem(6, 0, new  QTableWidgetItem("Data Stamp"));

    ui->tableWidget->setItem(0, 1, new  QTableWidgetItem("0"));
    ui->tableWidget->setItem(1, 1, new  QTableWidgetItem("0"));
    ui->tableWidget->setItem(2, 1, new  QTableWidgetItem("0"));
    ui->tableWidget->setItem(3, 1, new  QTableWidgetItem("0"));
    ui->tableWidget->setItem(4, 1, new  QTableWidgetItem("0"));
    ui->tableWidget->setItem(5, 1, new  QTableWidgetItem("0"));
    ui->tableWidget->setItem(6, 1, new  QTableWidgetItem("0"));

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::OnTimerCallback);
    m_timer->start(50);

    m_ros_thread = new ROSThread(this, &m_mutex);
    m_ros_thread->start();

}

MainWindow::~MainWindow()
{
    if (m_timer->isActive())
            m_timer->stop();


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

void MainWindow::on_pushButton_save_path_pressed()
{
    QFileDialog dialog;
    m_data_save_path = dialog.getExistingDirectory();
    ui->textEdit_save_path->setText(m_data_save_path);
    m_ros_thread->m_data_save_path = m_data_save_path.toUtf8().constData();
}

void MainWindow::on_pushButton_start_pressed()
{
    ui->pushButton_start->setEnabled(false);
    ui->pushButton_stop->setEnabled(true);

    mkdir((m_data_save_path.toUtf8().constData() + std::string("/camera/")).c_str(), 0777);
    mkdir((m_data_save_path.toUtf8().constData() + std::string("/camera/color/")).c_str(), 0777);
    mkdir((m_data_save_path.toUtf8().constData() + std::string("/camera/depth/")).c_str(), 0777);
    m_ros_thread->m_data_stamp_file.open((m_ros_thread->m_data_save_path + std::string("/data_stamp.csv")));
    m_ros_thread->m_franka_states_file.open((m_ros_thread->m_data_save_path + std::string("/franka_states.csv")));
    m_ros_thread->m_franka_joint_states_file.open((m_ros_thread->m_data_save_path + std::string("/franka_joint_states.csv")));
    m_ros_thread->m_camera_info_file.open((m_ros_thread->m_data_save_path + std::string("/camera_info.csv")));

    mkdir((m_data_save_path.toUtf8().constData() + std::string("/detection_result/")).c_str(), 0777);
    m_ros_thread->m_detection_result_file.open((m_ros_thread->m_data_save_path + std::string("/detection_result/detection_result.csv")));
    mkdir((m_data_save_path.toUtf8().constData() + std::string("/detection_result/mask/")).c_str(), 0777);

    usleep(1000);

    m_ros_thread->m_franka_states_data.active = true;
    m_ros_thread->m_franka_joint_states_data.active = true;
    m_ros_thread->m_camera_info_data.active = true;
    m_ros_thread->m_camera_color_data.active = true;
    m_ros_thread->m_camera_depth_data.active = true;
    m_ros_thread->m_detection_result_data.active = true;

    m_ros_thread->m_franka_states_data.cnt = 0;
    m_ros_thread->m_franka_joint_states_data.cnt = 0;
    m_ros_thread->m_camera_info_data.cnt = 0;
    m_ros_thread->m_camera_color_data.cnt = 0;
    m_ros_thread->m_camera_depth_data.cnt = 0;
    m_ros_thread->m_detection_result_data.cnt = 0;

    m_ros_thread->m_data_stamp_cnt = 0;
}

void MainWindow::on_pushButton_stop_pressed()
{
    ui->pushButton_start->setEnabled(false);
    ui->pushButton_stop->setEnabled(false);

    m_ros_thread->m_franka_states_data.active = false;
    m_ros_thread->m_franka_joint_states_data.active = false;
    m_ros_thread->m_camera_info_data.active = false;
    m_ros_thread->m_camera_color_data.active = false;
    m_ros_thread->m_camera_depth_data.active = false;
    m_ros_thread->m_detection_result_data.active = false;

    while (!m_ros_thread->m_franka_states_data.data_queue.empty() ||
           !m_ros_thread->m_franka_joint_states_data.data_queue.empty() ||
           !m_ros_thread->m_camera_info_data.data_queue.empty() ||
           !m_ros_thread->m_camera_color_data.data_queue.empty() ||
           !m_ros_thread->m_camera_depth_data.data_queue.empty() ||
           !m_ros_thread->m_detection_result_data.data_queue.empty())
    {
        usleep(1000000);
    }

    usleep(1000);

    for(auto iter = m_ros_thread->m_data_stamp.begin() ; iter != m_ros_thread->m_data_stamp.end() ; iter ++)
    {
        m_ros_thread->m_data_stamp_file << setprecision(20) << iter->first<< "," << iter->second << "\n";
    }
    m_ros_thread->m_data_stamp.clear();

    usleep(1000);

    m_ros_thread->m_data_stamp_file.close();
    m_ros_thread->m_franka_states_file.close();
    m_ros_thread->m_franka_joint_states_file.close();
    m_ros_thread->m_camera_info_file.close();
    m_ros_thread->m_detection_result_file.close();

    ui->pushButton_start->setEnabled(true);
    ui->pushButton_stop->setEnabled(false);

}

void MainWindow::OnTimerCallback()
{
    ui->tableWidget->setItem(0, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_franka_states_data.cnt).c_str()));
    ui->tableWidget->setItem(1, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_franka_joint_states_data.cnt).c_str()));
    ui->tableWidget->setItem(2, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_camera_info_data.cnt).c_str()));
    ui->tableWidget->setItem(3, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_camera_color_data.cnt).c_str()));
    ui->tableWidget->setItem(4, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_camera_depth_data.cnt).c_str()));
    ui->tableWidget->setItem(5, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_detection_result_data.cnt).c_str()));
    ui->tableWidget->setItem(6, 1, new  QTableWidgetItem(std::to_string(m_ros_thread->m_data_stamp_cnt).c_str()));
}
