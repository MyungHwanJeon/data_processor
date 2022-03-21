#ifndef _ROS_THREAD_H_
#define _ROS_THREAD_H_

#include "userdef.h"

using namespace std;

class ROSThread : public QThread
{
    Q_OBJECT

public:
    explicit ROSThread(QObject *parent = 0, QMutex *th_mutex = 0);
    ~ROSThread();   

    QMutex *m_mutex;
    QMutex *m_total_file_mutex;


    DataThread<int64_t, franka_rpm_msgs::FrankaState> m_franka_states_data;
    DataThread<int64_t, sensor_msgs::JointState> m_franka_joint_states_data;
    DataThread<int64_t, sensor_msgs::CameraInfo> m_camera_info_data;
//    DataThread<std::string, cv::Mat> m_camera_color_data;
    DataThread<int64_t, sensor_msgs::Image> m_camera_color_data;
//    DataThread<std::string, cv::Mat> m_camera_depth_data;
    DataThread<int64_t, sensor_msgs::Image> m_camera_depth_data;
    DataThread<int64_t, vision_msgs::Detection2DArray> m_detection_result_data;

    multimap<int64_t, string> m_data_stamp;
    long m_data_stamp_cnt = 0;

    std::mutex m_mutex_common;

    bool m_thread_run;

    std::string m_data_save_path;
    std::ofstream m_data_stamp_file;
    std::ofstream m_franka_states_file;
    std::ofstream m_franka_joint_states_file;
    std::ofstream m_camera_info_file;
    std::ofstream m_detection_result_file;


    void ros_initialize(ros::NodeHandle &n_);
    void run();
    ros::NodeHandle nh;

    ros::Subscriber m_franka_states_sub;
    ros::Subscriber m_franka_states_full_sub;
    ros::Subscriber m_franka_joint_states_sub;

    ros::Subscriber m_camera_info_sub;
    ros::Subscriber m_camera_color_sub;
    ros::Subscriber m_camera_depth_sub;

    ros::Subscriber m_detection_results_sub;

    //callback function
    void FrankaStatesCallback(const franka_rpm_msgs::FrankaStateConstPtr& msg);
    void FrankaJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);

    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void CameraColorCallback(const sensor_msgs::ImageConstPtr& msg);
    void CameraDepthCallback(const sensor_msgs::ImageConstPtr& msg);

    void DetectionResultCallback(const vision_msgs::Detection2DArrayConstPtr& msg);


    //publish    

    void FrankaStatesSave();
    void FrankaJointStatesSave();

    void CameraInfoSave();
    void CameraColorSave();
    void CameraDepthSave();

    void DetectionResultSave();

};

#endif
