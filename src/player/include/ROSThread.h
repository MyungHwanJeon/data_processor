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
    std::mutex m_mutex_common;

    ros::Timer m_timer;
    void TimerCallback(const ros::TimerEvent&);

    bool m_play_flag;
    bool m_pause_flag;
    bool m_stop_skip_flag;
    bool m_loop_flag;
    double m_play_speed;

    DataThread<int64_t> m_data_stamp_data;
    DataThread<int64_t> m_franka_states_data;
    DataThread<int64_t> m_franka_joint_states_data;
    DataThread<int64_t> m_camera_info_data;
    DataThread<int64_t> m_camera_color_data;
    DataThread<int64_t> m_camera_depth_data;
    DataThread<int64_t> m_detection_result_data;

    multimap<int64_t, string> m_data_stamp;
    map<int64_t, int64_t> m_stop_period;
    int64_t m_initial_data_stamp;
    int64_t m_last_data_stamp;
    int64_t m_processed_stamp;
    int64_t m_pre_timer_stamp;
    int64_t m_prev_clock_stamp;
    bool m_reset_process_stamp_flag;
    long m_data_stamp_cnt = 0;

    map<int64_t, franka_rpm_msgs::FrankaState> m_franka_states_data_list;
    map<int64_t, sensor_msgs::JointState> m_franka_joint_states_data_list;
    map<int64_t, sensor_msgs::CameraInfo> m_camera_info_data_list;
    map<int64_t, vision_msgs::Detection2DArray> m_detection_result_data_list;

    vector<string> m_camera_color_file_list;
    vector<string> m_camera_depth_file_list;
    vector<string> m_detection_result_mask_file_list;

    void ros_initialize(ros::NodeHandle &n_);
    void run();
    ros::NodeHandle nh;

    ros::Publisher m_clock_pub;

    ros::Publisher m_franka_states_pub;
    ros::Publisher m_franka_states_full_pub;
    ros::Publisher m_franka_joint_states_pub;

    ros::Publisher m_camera_info_pub;
    ros::Publisher m_camera_color_pub;
    ros::Publisher m_camera_depth_pub;

    ros::Publisher m_detection_results_pub;

    std::string m_data_load_path;
    void ready();

    int m_stamp_show_count;

    //publish    
    void DataStampThread();

    void FrankaStatesPublish();
    void FrankaJointStatesPublish();

    void CameraInfoPublish();
    void CameraColorPublish();
    void CameraDepthPublish();

    void DetectionResultPublish();

    void ResetProcessStamp(int position);
    int GetDirList(string dir, std::vector<std::string> &files);
    std::vector<std::string> split(std::string str, char delimiter);

signals:
    void StampShow(quint64 stamp);
};

#endif
