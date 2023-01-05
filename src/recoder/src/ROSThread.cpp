#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), m_mutex(th_mutex)
{
    m_total_file_mutex = new QMutex;

    m_thread_run = true;
    m_data_stamp.clear();
    m_data_stamp_cnt = 0;

    m_franka_states_data.cnt = 0;
    m_franka_joint_states_data.cnt = 0;
    m_camera_info_data.cnt = 0;
    m_camera_color_data.cnt = 0;
    m_camera_depth_data.cnt = 0;
    m_detection_result_data.cnt = 0;

    m_franka_states_data.active = false;
    m_franka_joint_states_data.active = false;
    m_camera_info_data.active = false;
    m_camera_color_data.active = false;
    m_camera_depth_data.active = false;
    m_detection_result_data.active = false;

    m_franka_states_data.thd = std::thread(&ROSThread::FrankaStatesSave, this);
    m_franka_joint_states_data.thd = std::thread(&ROSThread::FrankaJointStatesSave, this);
    m_camera_info_data.thd = std::thread(&ROSThread::CameraInfoSave, this);
    m_camera_color_data.thd = std::thread(&ROSThread::CameraColorSave, this);
    m_camera_depth_data.thd = std::thread(&ROSThread::CameraDepthSave, this);
    m_detection_result_data.thd = std::thread(&ROSThread::DetectionResultSave, this);
}
ROSThread::~ROSThread()
{
    m_franka_states_data.active = false;
    m_franka_joint_states_data.active = false;
    m_camera_info_data.active = false;
    m_camera_color_data.active = false;
    m_camera_depth_data.active = false;
    m_detection_result_data.active = false;

    m_thread_run = false;

    m_franka_states_data.cv.notify_all();
    m_franka_joint_states_data.cv.notify_all();
    m_camera_info_data.cv.notify_all();
    m_camera_color_data.cv.notify_all();
    m_camera_depth_data.cv.notify_all();
    m_detection_result_data.cv.notify_all();

    usleep(1000000);

    m_franka_states_data.thd.join();
    m_franka_joint_states_data.thd.join();
    m_camera_info_data.thd.join();
    m_camera_color_data.thd.join();
    m_camera_depth_data.thd.join();
    m_detection_result_data.thd.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
    nh = n;

    m_franka_states_sub = nh.subscribe<franka_rpm_msgs::FrankaState>("/franka_rpm/franka_states", 1000, boost::bind(&ROSThread::FrankaStatesCallback, this, _1));
    m_franka_joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/franka_rpm/joint_states", 1000, boost::bind(&ROSThread::FrankaJointStatesCallback, this, _1));

    m_camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/pose_estimation/PrimA6D/camera_info", 1000, boost::bind(&ROSThread::CameraInfoCallback, this, _1));
    m_camera_color_sub = nh.subscribe<sensor_msgs::Image>("/pose_estimation/PrimA6D/color_raw", 1000, boost::bind(&ROSThread::CameraColorCallback, this, _1));
    m_camera_depth_sub = nh.subscribe<sensor_msgs::Image>("/pose_estimation/PrimA6D/depth_raw", 1000, boost::bind(&ROSThread::CameraDepthCallback, this, _1));

    m_detection_results_sub = nh.subscribe<vision_msgs::Detection2DArray>("/pose_estimation/PrimA6D/detection2D_array", 1000, boost::bind(&ROSThread::DetectionResultCallback, this, _1));
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::FrankaStatesCallback(const franka_rpm_msgs::FrankaStateConstPtr& msg)
{
    if (m_franka_states_data.active)
    {
        int64_t msg_stamp = msg->header.stamp.toNSec();

        m_total_file_mutex->lock();
        m_data_stamp_cnt++;
        m_total_file_mutex->unlock();

        m_franka_states_data.push(make_pair(msg_stamp, *msg));
        m_franka_states_data.cv.notify_all();
    }
}

void ROSThread::FrankaJointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    if (m_franka_joint_states_data.active)
    {
        int64_t msg_stamp = msg->header.stamp.toNSec();

        m_total_file_mutex->lock();
        m_data_stamp_cnt++;
        m_total_file_mutex->unlock();

        m_franka_joint_states_data.push(make_pair(msg_stamp, *msg));
        m_franka_joint_states_data.cv.notify_all();
    }
}

void ROSThread::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    if (m_camera_info_data.active)
    {
        int64_t msg_stamp = msg->header.stamp.toNSec();

        m_total_file_mutex->lock();
        m_data_stamp_cnt++;
        m_total_file_mutex->unlock();

        m_camera_info_data.push(make_pair(msg_stamp, *msg));
        m_camera_info_data.cv.notify_all();
    }
}

void ROSThread::CameraColorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (m_camera_color_data.active)
    {
        int64_t msg_stamp = msg->header.stamp.toNSec();

        m_total_file_mutex->lock();
        m_data_stamp_cnt++;
        m_total_file_mutex->unlock();

        m_camera_color_data.push(make_pair(msg_stamp, *msg));
        m_camera_color_data.cv.notify_all();
    }
}

void ROSThread::CameraDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (m_camera_depth_data.active)
    {
        int64_t msg_stamp = msg->header.stamp.toNSec();

        m_total_file_mutex->lock();
        m_data_stamp_cnt++;
        m_total_file_mutex->unlock();

        m_camera_depth_data.push(make_pair(msg_stamp, *msg));
        m_camera_depth_data.cv.notify_all();
    }
}

void ROSThread::DetectionResultCallback(const vision_msgs::Detection2DArrayConstPtr& msg)
{
    if (m_detection_result_data.active)
    {
        int64_t msg_stamp = msg->header.stamp.toNSec();

        m_total_file_mutex->lock();
        m_data_stamp_cnt++;
        m_total_file_mutex->unlock();

        m_detection_result_data.push(make_pair(msg_stamp, *msg));
        m_detection_result_data.cv.notify_all();
    }
}

void ROSThread::FrankaStatesSave()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_franka_states_data.mtx);
        m_franka_states_data.cv.wait(ul);
        if (m_thread_run == false)
        {
            m_mutex_common.lock();
            cout << "Franka States save thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_franka_states_data.data_queue.empty())
        {
            auto data = m_franka_states_data.pop();

            m_total_file_mutex->lock();            
            m_data_stamp.insert(multimap<int64_t, string>::value_type(data.first, "franka_states"));
            m_total_file_mutex->unlock();

            m_franka_states_file << setprecision(20) << data.first << setprecision(15);
            for(int i = 0 ; i < 16 ; i ++) m_franka_states_file << "," << data.second.O_T_EE[i];
            for(int i = 0 ; i < 16 ; i ++) m_franka_states_file << "," << data.second.O_T_CAM[i];
            for(int i = 0 ; i < 16 ; i ++) m_franka_states_file << "," << data.second.O_T_GRIPPER[i];
            for(int i = 0 ; i < 16 ; i ++) m_franka_states_file << "," << data.second.EE_T_CAM[i];
            for(int i = 0 ; i < 16 ; i ++) m_franka_states_file << "," << data.second.EE_T_GRIPPER[i];
            for(int i = 0 ; i < 16 ; i ++) m_franka_states_file << "," << data.second.CAM_T_GRIPPER[i];
            m_franka_states_file << "\n";

        }
    }
}

void ROSThread::FrankaJointStatesSave()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_franka_joint_states_data.mtx);
        m_franka_joint_states_data.cv.wait(ul);
        if (m_thread_run == false)
        {
            m_mutex_common.lock();
            cout << "Franka Joint States save thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_franka_joint_states_data.data_queue.empty())
        {
            auto data = m_franka_joint_states_data.pop();

            m_total_file_mutex->lock();            
            m_data_stamp.insert(multimap<int64_t, string>::value_type(data.first, "franka_joint_states"));
            m_total_file_mutex->unlock();

            m_franka_joint_states_file << setprecision(20) << data.first << setprecision(15);
            for(int i = 0 ; i < data.second.name.size() ; i ++) m_franka_joint_states_file << "," << data.second.name[i];
            for(int i = 0 ; i < data.second.position.size() ; i ++) m_franka_joint_states_file << "," << data.second.position[i];
            for(int i = 0 ; i < data.second.velocity.size() ; i ++) m_franka_joint_states_file << "," << data.second.velocity[i];
            for(int i = 0 ; i < data.second.effort.size() ; i ++) m_franka_joint_states_file << "," << data.second.effort[i];
            m_franka_joint_states_file << "\n";
        }
    }
}

void ROSThread::CameraInfoSave()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_camera_info_data.mtx);
        m_camera_info_data.cv.wait(ul);
        if (m_thread_run == false)
        {
            m_mutex_common.lock();
            cout << "Camera Info save thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_camera_info_data.data_queue.empty())
        {
            auto data = m_camera_info_data.pop();

            m_total_file_mutex->lock();            
            m_data_stamp.insert(multimap<int64_t, string>::value_type(data.first, "camera_info"));
            m_total_file_mutex->unlock();

            m_camera_info_file << setprecision(20) << data.first << setprecision(15);
            m_camera_info_file << "," << data.second.height << "," << data.second.width;
            m_camera_info_file << "," << data.second.distortion_model;
            for(int i = 0 ; i < data.second.D.size() ; i ++) m_camera_info_file << "," << data.second.D[i];
            for(int i = 0 ; i < 9 ; i ++) m_camera_info_file << "," << data.second.K[i];
            for(int i = 0 ; i < 9 ; i ++) m_camera_info_file << "," << data.second.R[i];
            for(int i = 0 ; i < 12 ; i ++) m_camera_info_file << "," << data.second.P[i];
            m_camera_info_file << "," << data.second.binning_x << "," << data.second.binning_y;
            m_camera_info_file << "\n";
        }
    }
}

void ROSThread::CameraColorSave()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_camera_color_data.mtx);
        m_camera_color_data.cv.wait(ul);
        if (m_thread_run == false)
        {
            m_mutex_common.lock();
            cout << "Camera Color save thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_camera_color_data.data_queue.empty())
        {
            auto data = m_camera_color_data.pop();

            m_total_file_mutex->lock();            
            m_data_stamp.insert(multimap<int64_t, string>::value_type(data.first, "camera_color"));
            m_total_file_mutex->unlock();

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(data.second, data.second.encoding);
                cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2RGB);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);
            cv::imwrite(m_data_save_path + string("/camera/color/") + to_string(data.first) + string( ".png" ), cv_ptr->image, compression_params);
        }
    }
}

void ROSThread::CameraDepthSave()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_camera_depth_data.mtx);
        m_camera_depth_data.cv.wait(ul);
        if (m_thread_run == false)
        {
            m_mutex_common.lock();
            cout << "Camera Depth save thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_camera_depth_data.data_queue.empty())
        {
            auto data = m_camera_depth_data.pop();

            m_total_file_mutex->lock();            
            m_data_stamp.insert(multimap<int64_t, string>::value_type(data.first, "camera_depth"));
            m_total_file_mutex->unlock();

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(data.second, data.second.encoding);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);
            cv::imwrite(m_data_save_path + string("/camera/depth/") + to_string(data.first) + string( ".png" ), cv_ptr->image, compression_params);
        }
    }
}

void ROSThread::DetectionResultSave()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_detection_result_data.mtx);
        m_detection_result_data.cv.wait(ul);
        if (m_thread_run == false)
        {
            m_mutex_common.lock();
            cout << "Detection Result save thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_detection_result_data.data_queue.empty())
        {
            auto data = m_detection_result_data.pop();

            m_total_file_mutex->lock();            
            m_data_stamp.insert(multimap<int64_t, string>::value_type(data.first, "detection_result"));
            m_total_file_mutex->unlock();

            m_detection_result_file << setprecision(20) << data.first << setprecision(15);
            m_detection_result_file << "," << data.second.detections.size();
            for(int i = 0 ; i < data.second.detections.size() ; i ++)
            {
                m_detection_result_file << "," << data.second.detections[i].results[0].id;
                m_detection_result_file << "," << data.second.detections[i].results[0].score;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.position.x;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.position.y;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.position.z;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.orientation.x;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.orientation.y;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.orientation.z;
                m_detection_result_file << "," << data.second.detections[i].results[0].pose.pose.orientation.w;
                for(int j = 0 ; j < 36 ; j ++)
                {
                    m_detection_result_file << "," << data.second.detections[i].results[0].pose.covariance[j];
                }
                m_detection_result_file << "," << data.second.detections[i].bbox.center.x;
                m_detection_result_file << "," << data.second.detections[i].bbox.center.y;
                m_detection_result_file << "," << data.second.detections[i].bbox.center.theta;
                m_detection_result_file << "," << data.second.detections[i].bbox.size_x;
                m_detection_result_file << "," << data.second.detections[i].bbox.size_y;

                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(data.second.detections[i].source_img, data.second.detections[i].source_img.encoding);
                }
                catch (cv_bridge::Exception &e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }

                vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
                compression_params.push_back(0);
                cv::imwrite(m_data_save_path + string("/detection_result/mask/") + to_string(data.second.detections[i].results[0].id) + string( "_" ) + to_string(data.first) + string( ".png" ), cv_ptr->image, compression_params);
            }
            m_detection_result_file << "\n";
        }
    }
}
