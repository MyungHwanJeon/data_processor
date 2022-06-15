#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), m_mutex(th_mutex)
{
    m_total_file_mutex = new QMutex;

    m_play_flag = false;
    m_pause_flag = false;

    m_loop_flag = false;

    m_play_speed = 1.0;

    m_data_stamp.clear();
    m_data_stamp_cnt = 0;

    m_stamp_show_count = 0;

    // m_stop_skip_flag = false;

    m_camera_color_prev_idx = 0;
    m_camera_depth_prev_idx = 0;
    m_detection_result_prev_idx = 0;

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
}
ROSThread::~ROSThread()
{
    m_data_stamp_data.active = false;
    m_franka_states_data.active = false;
    m_franka_joint_states_data.active = false;
    m_camera_info_data.active = false;
    m_camera_color_data.active = false;
    m_camera_depth_data.active = false;
    m_detection_result_data.active = false;    

    m_data_stamp_data.cv.notify_all();
    m_franka_states_data.cv.notify_all();
    m_franka_joint_states_data.cv.notify_all();
    m_camera_info_data.cv.notify_all();
    m_camera_color_data.cv.notify_all();
    m_camera_depth_data.cv.notify_all();
    m_detection_result_data.cv.notify_all();

    if(m_data_stamp_data.thd.joinable()) m_data_stamp_data.thd.detach();
    if(m_franka_states_data.thd.joinable()) m_franka_states_data.thd.detach();
    if(m_franka_joint_states_data.thd.joinable()) m_franka_joint_states_data.thd.detach();
    if(m_camera_info_data.thd.joinable()) m_camera_info_data.thd.detach();
    if(m_camera_color_data.thd.joinable()) m_camera_color_data.thd.detach();
    if(m_camera_depth_data.thd.joinable()) m_camera_depth_data.thd.detach();
    if(m_detection_result_data.thd.joinable()) m_detection_result_data.thd.detach();

    while(m_data_stamp_data.thd.joinable())    m_data_stamp_data.thd.join();
    while(m_franka_states_data.thd.joinable())    m_franka_states_data.thd.join();
    while(m_franka_joint_states_data.thd.joinable())    m_franka_joint_states_data.thd.join();
    while(m_camera_info_data.thd.joinable())    m_camera_info_data.thd.join();
    while(m_camera_color_data.thd.joinable())    m_camera_color_data.thd.join();
    while(m_camera_depth_data.thd.joinable())    m_camera_depth_data.thd.join();
    while(m_detection_result_data.thd.joinable())    m_detection_result_data.thd.join();
}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
    nh = n;

    m_pre_timer_stamp = ros::Time::now().toNSec();
    m_timer = nh.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

    m_clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    m_franka_states_pub = nh.advertise<franka_rpm_msgs::FrankaState>("/franka_rpm/franka_states", 1000);
    m_franka_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/franka_rpm/joint_states", 1000);

    m_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/pose_estimation/PrimA6D/camera_info", 1000);
    m_camera_color_pub = nh.advertise<sensor_msgs::Image>("/pose_estimation/PrimA6D/color_raw", 1000);
    m_camera_depth_pub = nh.advertise<sensor_msgs::Image>("/pose_estimation/PrimA6D/depth_raw", 1000);

    m_detection_results_pub = nh.advertise<vision_msgs::Detection2DArray>("/pose_estimation/PrimA6D/detection2D_array", 1000);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::ready()
{
    m_data_stamp_data.active = false;
    m_franka_states_data.active = false;
    m_franka_joint_states_data.active = false;
    m_camera_info_data.active = false;
    m_camera_color_data.active = false;
    m_camera_depth_data.active = false;
    m_detection_result_data.active = false;

    m_data_stamp_data.cv.notify_all();
    m_franka_states_data.cv.notify_all();
    m_franka_joint_states_data.cv.notify_all();
    m_camera_info_data.cv.notify_all();
    m_camera_color_data.cv.notify_all();
    m_camera_depth_data.cv.notify_all();
    m_detection_result_data.cv.notify_all();

    if(m_data_stamp_data.thd.joinable()) m_data_stamp_data.thd.detach();
    if(m_franka_states_data.thd.joinable()) m_franka_states_data.thd.detach();
    if(m_franka_joint_states_data.thd.joinable()) m_franka_joint_states_data.thd.detach();
    if(m_camera_info_data.thd.joinable()) m_camera_info_data.thd.detach();
    if(m_camera_color_data.thd.joinable()) m_camera_color_data.thd.detach();
    if(m_camera_depth_data.thd.joinable()) m_camera_depth_data.thd.detach();
    if(m_detection_result_data.thd.joinable()) m_detection_result_data.thd.detach();

    while(m_data_stamp_data.thd.joinable()) m_data_stamp_data.thd.join();    
    while(m_franka_states_data.thd.joinable())    m_franka_states_data.thd.join();
    while(m_franka_joint_states_data.thd.joinable())    m_franka_joint_states_data.thd.join();
    while(m_camera_info_data.thd.joinable())    m_camera_info_data.thd.join();
    while(m_camera_color_data.thd.joinable())    m_camera_color_data.thd.join();
    while(m_camera_depth_data.thd.joinable())    m_camera_depth_data.thd.join();
    while(m_detection_result_data.thd.joinable())    m_detection_result_data.thd.join();

    ResetProcessStamp(1);    

    ifstream fs;
    int64_t stamp;
    std::string line;

    fs.open((m_data_load_path + "/data_stamp.csv").c_str());
    if(!fs.good())
    {
        cout << "Please check the file path. The input path is wrong (data_stamp.csv not exist)" << endl;
        return;
    }
    fs.close();

    // load data stamp
    fs.open((m_data_load_path + "/data_stamp.csv").c_str());
    m_data_stamp.clear();
    while (!fs.eof())
    {
        getline(fs, line);
        if (line.size() < 1)    break;

        std::vector<std::string> splited = split(line, ',');

        m_data_stamp.insert(multimap<int64_t, string>::value_type(std::stol(splited[0])+1, splited[1]));
    }

    // long time_franka_state_prev = 0;
    // long time_franka_state_next = 0;
    // long time_detection_result = 0;   
    // bool flag_detection_result = false; 
    // while (!fs.eof())
    // {
    //     getline(fs, line);
    //     if (line.size() < 1)    break;

    //     std::vector<std::string> splited = split(line, ',');

    //     if (splited[1].compare("detection_result") == 0)
    //     {            
    //         time_detection_result = stol(splited[0]);
    //         flag_detection_result = true;
    //     }
    //     else if(splited[1].compare("franka_states") == 0)
    //     {
    //         if (!flag_detection_result)
    //         {
    //             time_franka_state_prev = std::stol(splited[0]);
    //         }
    //         else
    //         {
    //             time_franka_state_next = std::stol(splited[0]);

    //             long prev_time_diff = abs(time_franka_state_prev - time_detection_result);
    //             long next_time_diff = abs(time_franka_state_next - time_detection_result);

    //             if (prev_time_diff >= next_time_diff)
    //             {
    //                 m_data_stamp.insert(multimap<int64_t, string>::value_type(time_detection_result, "detection_result"));
    //                 m_data_stamp.insert(multimap<int64_t, string>::value_type(time_franka_state_next, "franka_states"));
    //             }
    //             else
    //             {
    //                 m_data_stamp.insert(multimap<int64_t, string>::value_type(time_detection_result, "detection_result"));
    //                 m_data_stamp.insert(multimap<int64_t, string>::value_type(time_franka_state_prev, "franka_states"));
    //             }

    //             flag_detection_result = false;
    //         }
    //     }
    //     else
    //     {
    //         m_data_stamp.insert(multimap<int64_t, string>::value_type(std::stol(splited[0]), splited[1]));
    //     }        
    // }

    m_initial_data_stamp = m_data_stamp.begin()->first - 1;
    m_last_data_stamp = prev(m_data_stamp.end(),1)->first - 1;

    cout << "Stamp data are loaded" << endl;
    fs.close();

    // load franka states
    fs.open((m_data_load_path+"/franka_states.csv").c_str());
    franka_rpm_msgs::FrankaState frank_state;
    m_franka_states_data_list.clear();
    while (!fs.eof())
    {
        getline(fs, line);
        if (line.size() < 1)    break;

        std::vector<std::string> splited = split(line, ',');

        stamp = std::stol(splited[0])+1;
        frank_state.header.stamp.fromNSec(stamp);

        for (int i=0; i<16; i++)    frank_state.O_T_EE[i] = std::stod(splited[1+i]);
        for (int i=0; i<16; i++)    frank_state.O_T_CAM[i] = std::stod(splited[1+16+i]);
        for (int i=0; i<16; i++)    frank_state.O_T_GRIPPER[i] = std::stod(splited[1+16*2+i]);
        for (int i=0; i<16; i++)    frank_state.EE_T_CAM[i] = std::stod(splited[1+16*3+i]);
        for (int i=0; i<16; i++)    frank_state.EE_T_GRIPPER[i] = std::stod(splited[1+16*4+i]);
        for (int i=0; i<16; i++)    frank_state.CAM_T_GRIPPER[i] = std::stod(splited[1+16*5+i]);

        m_franka_states_data_list[stamp] = frank_state;
    }    
    cout << "franka states are loaded" << endl;
    fs.close();

    // load franka joint states
    fs.open((m_data_load_path+"/franka_joint_states.csv").c_str());
    sensor_msgs::JointState joint_state;
    m_franka_joint_states_data_list.clear();
    while (!fs.eof())
    {
        getline(fs, line);
        if (line.size() < 1)    break;

        std::vector<std::string> splited = split(line, ',');

        stamp = std::stol(splited[0])+1;
        joint_state.header.stamp.fromNSec(stamp);

        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.velocity.resize(7);
        joint_state.effort.resize(7);

        for (int i=0; i<7; i++) joint_state.name[i] = splited[1+i];
        for (int i=0; i<7; i++) joint_state.position[i] = std::stod(splited[1+7+i]);
        for (int i=0; i<7; i++) joint_state.velocity[i] = std::stod(splited[1+7*2+i]);
        for (int i=0; i<7; i++) joint_state.effort[i] = std::stod(splited[1+7*3+i]);

        m_franka_joint_states_data_list[stamp] = joint_state;
    }          
    cout << "franka joint states are loaded" << endl;
    fs.close();

    // load camera info
    fs.open((m_data_load_path+"/camera_info.csv").c_str());
    sensor_msgs::CameraInfo camera_info;
    m_camera_info_data_list.clear();
    while (!fs.eof())
    {
        getline(fs, line);
        if (line.size() < 1)    break;

        std::vector<std::string> splited = split(line, ',');

        stamp = std::stol(splited[0])+1;
        camera_info.header.stamp.fromNSec(stamp);

        camera_info.width = std::stoi(splited[1]);
        camera_info.height = std::stoi(splited[2]);
        camera_info.distortion_model = splited[3];
        camera_info.D.resize(5);        
        for (int i=0; i<5; i++) camera_info.D[i] = std::stod(splited[4+i]);        
        for (int i=0; i<9; i++) camera_info.K[i] = std::stod(splited[4+5+i]);        
        for (int i=0; i<9; i++) camera_info.R[i] = std::stod(splited[4+5+9+i]);        
        for (int i=0; i<12; i++) camera_info.P[i] = std::stod(splited[4+5+9+9+i]);        
        camera_info.binning_x = std::stoi(splited[4+5+9+9+12]);
        camera_info.binning_y = std::stoi(splited[4+5+9+9+12+1]);

        m_camera_info_data_list[stamp] = camera_info;

    }
    cout << "camera info are loaded" << endl;
    fs.close();

    // load detection result
    fs.open((m_data_load_path+"/detection_result/detection_result.csv").c_str());
    vision_msgs::Detection2DArray detection2darray;
    m_detection_result_data_list.clear();
    while (!fs.eof())
    {
        getline(fs, line);
        if (line.size() < 1)    break;

        std::vector<std::string> splited = split(line, ',');

        stamp = std::stol(splited[0])+1;
        detection2darray.header.stamp.fromNSec(stamp);

        detection2darray.detections.resize(std::stoi(splited[1]));
        for (int i=0; i<std::stoi(splited[1]); i++)
        {
            vision_msgs::Detection2D detection2d;
            detection2d.results.resize(1);

            detection2d.header.stamp.fromNSec(stamp);

            detection2d.bbox.center.x = std::stod(splited[2+50*i+9+36+0]);
            detection2d.bbox.center.y = std::stod(splited[2+50*i+9+36+1]);
            detection2d.bbox.center.theta = std::stod(splited[2+50*i+9+36+2]);
            detection2d.bbox.size_x = std::stod(splited[2+50*i+9+36+3]);
            detection2d.bbox.size_y = std::stod(splited[2+50*i+9+36+4]);

            vision_msgs::ObjectHypothesisWithPose hyp;

            hyp.id = std::stoi(splited[2+50*i+0]);
            hyp.score = std::stod(splited[2+50*i+1]);
            hyp.pose.pose.position.x = std::stod(splited[2+50*i+2]);
            hyp.pose.pose.position.y = std::stod(splited[2+50*i+3]);
            hyp.pose.pose.position.z = std::stod(splited[2+50*i+4]);
            hyp.pose.pose.orientation.x = std::stod(splited[2+50*i+5]);
            hyp.pose.pose.orientation.y = std::stod(splited[2+50*i+6]);
            hyp.pose.pose.orientation.z = std::stod(splited[2+50*i+7]);
            hyp.pose.pose.orientation.w = std::stod(splited[2+50*i+8]);
            for(int j = 0 ; j < 36 ; j ++)    hyp.pose.covariance[j] = std::stod(splited[2+50*i+9+j]);

            detection2d.results[0] = hyp;
            detection2darray.detections[i] = detection2d;
        }

        m_detection_result_data_list[stamp] = detection2darray;
    }
    fs.close();
    cout << "datection result are loaded" << endl;

    // detection result time list
    m_detection_result_time_list.clear();
    for(auto iter = m_data_stamp.begin() ; iter != m_data_stamp.end() ; iter ++)
    {        
        if(iter->second.compare("detection_result") == 0)   m_detection_result_time_list.push_back(iter->first);        
    }    

    // load image file list
    m_camera_color_file_list.clear();
    m_camera_depth_file_list.clear();
    m_detection_result_mask_file_list.clear();

    GetDirList(m_data_load_path + "/camera/color/", m_camera_color_file_list);
    GetDirList(m_data_load_path + "/camera/depth/", m_camera_depth_file_list);
    GetDirList(m_data_load_path + "/detection_result/mask/", m_detection_result_mask_file_list);    

    m_data_stamp_data.active = true;
    m_franka_states_data.active = true;
    m_franka_joint_states_data.active = true;
    m_camera_info_data.active = true;
    m_camera_color_data.active = true;
    m_camera_depth_data.active = true;
    m_detection_result_data.active = true;

    m_data_stamp_data.thd = std::thread(&ROSThread::DataStampThread,this);

    m_franka_states_data.thd = std::thread(&ROSThread::FrankaStatesPublish, this);
    m_franka_joint_states_data.thd = std::thread(&ROSThread::FrankaJointStatesPublish, this);
    m_camera_info_data.thd = std::thread(&ROSThread::CameraInfoPublish, this);
    m_camera_color_data.thd = std::thread(&ROSThread::CameraColorPublish, this);
    m_camera_depth_data.thd = std::thread(&ROSThread::CameraDepthPublish, this);
    m_detection_result_data.thd = std::thread(&ROSThread::DetectionResultPublish, this);
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(m_play_flag == true && m_pause_flag == false)
    {
        m_processed_stamp += static_cast<int64_t>(static_cast<double>(current_stamp - m_pre_timer_stamp) * m_play_speed);
    }
    m_pre_timer_stamp = current_stamp;

    if(m_play_flag == false)
    {
    //    m_processed_stamp = 0; //reset
    //    m_prev_clock_stamp = 0;
    }
}

void ROSThread::DataStampThread()
{
    auto stop_region_iter = m_stop_period.begin();

    for(auto iter = m_data_stamp.begin() ; iter != m_data_stamp.end() ; iter ++)
    {
        auto stamp = iter->first;

        while((stamp > (m_initial_data_stamp + m_processed_stamp))&&(m_data_stamp_data.active == true))
        {
            if(m_processed_stamp == 0)
            {
                iter = m_data_stamp.begin();
                stop_region_iter = m_stop_period.begin();
                stamp = iter->first;
            }

            usleep(1);
            if(m_reset_process_stamp_flag == true) break;
            //wait for data publish
        }

        if(m_reset_process_stamp_flag == true)
        {
            auto target_stamp = m_processed_stamp + m_initial_data_stamp;
            //set iter
            iter = m_data_stamp.lower_bound(target_stamp);
            iter = prev(iter,1);
            //set stop region order
            auto new_stamp = iter->first;
            stop_region_iter = m_stop_period.upper_bound(new_stamp);

            m_reset_process_stamp_flag = false;
            continue;
        }

        //check whether stop region or not
        if(stamp == stop_region_iter->first)
        {
            if(m_stop_skip_flag == true)
            {
                cout << "Skip stop section!!" << endl;
                iter = m_data_stamp.find(stop_region_iter->second);  //find stop region end
                iter = prev(iter,1);
                m_processed_stamp = stop_region_iter->second - m_initial_data_stamp;
            }
            stop_region_iter++;
            if(m_stop_skip_flag == true)
            {
                continue;
            }
        }

        if(m_data_stamp_data.active == false)
            return;

        if(iter->second.compare("franka_states") == 0)
        {
            m_franka_states_data.push(stamp);
            m_franka_states_data.cv.notify_all();
        }
        if(iter->second.compare("franka_joint_states") == 0)
        {
            m_franka_joint_states_data.push(stamp);
            m_franka_joint_states_data.cv.notify_all();
        }
        if(iter->second.compare("camera_info") == 0)
        {
            m_camera_info_data.push(stamp);
            m_camera_info_data.cv.notify_all();
        }
        if(iter->second.compare("camera_color") == 0)
        {
            m_camera_color_data.push(stamp);
            m_camera_color_data.cv.notify_all();
        }
        if(iter->second.compare("camera_depth") == 0)
        {
            m_camera_depth_data.push(stamp);
            m_camera_depth_data.cv.notify_all();
        }
        if(iter->second.compare("detection_result") == 0)
        {
            m_detection_result_data.push(stamp);
            m_detection_result_data.cv.notify_all();
        }

        m_stamp_show_count++;
        if(m_stamp_show_count > 10)
        {
            m_stamp_show_count = 0;
            emit StampShow(stamp);
        }

        // if(m_prev_clock_stamp == 0 || (stamp - m_prev_clock_stamp) > 10000000)
        if(m_prev_clock_stamp == 0 || (stamp - m_prev_clock_stamp) > 1000000)
        {
            rosgraph_msgs::Clock clock;

            clock.clock.fromNSec(stamp);
            m_clock_pub.publish(clock);
            m_prev_clock_stamp = stamp;
        }

        if(m_loop_flag == true && iter == prev(m_data_stamp.end(),1))
        {
            iter = m_data_stamp.begin();
            stop_region_iter = m_stop_period.begin();
            m_processed_stamp = 0;
        }
        if(m_loop_flag == false && iter == prev(m_data_stamp.end(),1))
        {
            m_play_flag = false;
            while(!m_play_flag)
            {
                iter = m_data_stamp.begin();
                stop_region_iter = m_stop_period.begin();
                m_processed_stamp = 0;
                usleep(10000);
            }
        }
    }
    cout << "Data publish complete" << endl;
}

void ROSThread::FrankaStatesPublish()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_franka_states_data.mtx);
        m_franka_states_data.cv.wait(ul);
        if (m_franka_states_data.active == false)
        {
            m_mutex_common.lock();
            cout << "Franka States Publish thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_franka_states_data.data_queue.empty())
        {
            auto data = m_franka_states_data.pop();
            m_franka_states_pub.publish(m_franka_states_data_list[data]);
        }
    }
}

void ROSThread::FrankaJointStatesPublish()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_franka_joint_states_data.mtx);
        m_franka_joint_states_data.cv.wait(ul);
        if (m_franka_joint_states_data.active == false)
        {
            m_mutex_common.lock();
            cout << "Franka Joint States Publish thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_franka_joint_states_data.data_queue.empty())
        {
            auto data = m_franka_joint_states_data.pop();
            m_franka_joint_states_pub.publish(m_franka_joint_states_data_list[data]);
        }
    }
}

void ROSThread::CameraInfoPublish()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_camera_info_data.mtx);
        m_camera_info_data.cv.wait(ul);
        if (m_camera_info_data.active == false)
        {
            m_mutex_common.lock();
            cout << "Camera Info Publish thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_camera_info_data.data_queue.empty())
        {
            auto data = m_camera_info_data.pop();
            m_camera_info_pub.publish(m_camera_info_data_list[data]);
        }
    }
}

void ROSThread::CameraColorPublish()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_camera_color_data.mtx);
        m_camera_color_data.cv.wait(ul);
        if (m_camera_color_data.active == false)
        {
            m_mutex_common.lock();
            cout << "Camera Color Publish thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_camera_color_data.data_queue.empty())
        {
            auto data = m_camera_color_data.pop();

            if(m_camera_color_file_list.size() == 0) continue;            

            if (to_string(data - 1)+".png" == m_camera_color_next.first && !m_camera_color_next.second.empty())
            {                
                cv_bridge::CvImage msg;
                msg.header.stamp.fromNSec(data);
                msg.header.frame_id = "camera_color";
                msg.encoding = sensor_msgs::image_encodings::RGB8;
                msg.image = m_camera_color_next.second;

                m_camera_color_pub.publish(msg.toImageMsg());           
            }
            else
            {
                std::string img_path = m_data_load_path + "/camera/color/" + to_string(data - 1) + ".png";
                cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
                cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

                cv_bridge::CvImage msg;
                msg.header.stamp.fromNSec(data);
                msg.header.frame_id = "camera_color";
                msg.encoding = sensor_msgs::image_encodings::RGB8;
                msg.image = img;

                m_camera_color_pub.publish(msg.toImageMsg());

                m_camera_color_prev_idx = 0;                            
            }

            int current_img_index = find(next(m_camera_color_file_list.begin(), max(0, m_camera_color_prev_idx - 10)), m_camera_color_file_list.end(), to_string(data - 1)+".png") - m_camera_color_file_list.begin();
            if (current_img_index < m_camera_color_file_list.size() - 2)
            {                
                std::string img_path = m_data_load_path + "/camera/color/" + m_camera_color_file_list[current_img_index + 1];
                cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
                cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

                if (!img.empty())
                {
                    m_camera_color_next = make_pair(m_camera_color_file_list[current_img_index + 1], img);                     
                }
            }
            m_camera_color_prev_idx = current_img_index;                       
        }
    }
}

void ROSThread::CameraDepthPublish()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_camera_depth_data.mtx);
        m_camera_depth_data.cv.wait(ul);
        if (m_camera_depth_data.active == false)
        {
            m_mutex_common.lock();
            cout << "Camera Depth Publish thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_camera_depth_data.data_queue.empty())
        {
            auto data = m_camera_depth_data.pop();

            if (m_camera_depth_file_list.size() == 0)   continue;

            if (to_string(data - 1) + ".png" == m_camera_depth_next.first && !m_camera_depth_next.second.empty())
            {
                cv_bridge::CvImage msg;
                msg.header.stamp.fromNSec(data);
                msg.header.frame_id = "camera_depth";
                msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                msg.image = m_camera_depth_next.second;

                m_camera_depth_pub.publish(msg.toImageMsg());                                
            }
            else
            {
                std::string img_path = m_data_load_path + "/camera/depth/" + to_string(data - 1) + ".png";
                cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_ANYDEPTH);

                cv_bridge::CvImage msg;
                msg.header.stamp.fromNSec(data);
                msg.header.frame_id = "camera_depth";
                msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                msg.image = img;

                m_camera_depth_pub.publish(msg.toImageMsg());

                m_camera_depth_prev_idx = 0;                
            }

            int current_img_index = find(next(m_camera_depth_file_list.begin(), max(0, m_camera_depth_prev_idx - 10)), m_camera_depth_file_list.end(), to_string(data - 1) + ".png") - m_camera_depth_file_list.begin();
            if (current_img_index < m_camera_depth_file_list.size() - 2)
            {                
                std::string img_path = m_data_load_path + "/camera/depth/" + m_camera_depth_file_list[current_img_index + 1];
                cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_ANYDEPTH);                

                if (!img.empty())
                {
                    m_camera_depth_next = make_pair(m_camera_depth_file_list[current_img_index + 1], img);
                }
            }
            m_camera_depth_prev_idx = current_img_index;
        }
    }
}

void ROSThread::DetectionResultPublish()
{
    while (1)
    {
        std::unique_lock<std::mutex> ul(m_detection_result_data.mtx);
        m_detection_result_data.cv.wait(ul);
        if (m_detection_result_data.active == false)
        {
            m_mutex_common.lock();
            cout << "Detection Result Publish thread finish" << endl;
            m_mutex_common.unlock();
            return;
        }
        ul.unlock();

        while (!m_detection_result_data.data_queue.empty())
        {
            auto data = m_detection_result_data.pop();

            int n_obj = m_detection_result_data_list[data].detections.size();
            if (m_detection_mask_next.size() != 0 && n_obj == m_detection_mask_next.size())
            {                
                for (int i = 0; i < n_obj; i++)
                {
                    cv_bridge::CvImage msg;
                    msg.header.stamp.fromNSec(data);
                    msg.header.frame_id = "object mask";
                    msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
                    msg.image = m_detection_mask_next[i].second;

                    msg.toImageMsg(m_detection_result_data_list[data].detections[i].source_img);
                }
                m_detection_results_pub.publish(m_detection_result_data_list[data]);
            }
            else
            {
                for (int i = 0; i < n_obj; i++)
                {
                    std::string img_path = m_data_load_path + "/detection_result/mask/" + to_string(m_detection_result_data_list[data].detections[i].results[0].id) + "_" + to_string(data-1) + ".png";
                    cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);

                    cv_bridge::CvImage msg;
                    msg.header.stamp.fromNSec(data);
                    msg.header.frame_id = "object mask";
                    msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
                    msg.image = img;

                    msg.toImageMsg(m_detection_result_data_list[data].detections[i].source_img);
                }
                m_detection_results_pub.publish(m_detection_result_data_list[data]);

                m_detection_result_prev_idx = 0;
            }

            m_detection_mask_next.clear();
            int current_stamp_idx = find(next(m_detection_result_time_list.begin(), max(0, m_detection_result_prev_idx - 10)), m_detection_result_time_list.end(), data) - m_detection_result_time_list.begin();
            int64_t next_stamp = m_detection_result_time_list[current_stamp_idx + 1];
            n_obj = m_detection_result_data_list[next_stamp].detections.size();
            for (int i = 0; i < n_obj; i++)
            {
                std::string img_path = m_data_load_path + "/detection_result/mask/" + to_string(m_detection_result_data_list[next_stamp].detections[i].results[0].id) + "_" + to_string(next_stamp-1)+".png";
                cv::Mat img = cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);

                m_detection_mask_next.push_back(make_pair(img_path, img));
            }
            m_detection_result_prev_idx = current_stamp_idx;
        }
    }
}

void ROSThread::ResetProcessStamp(int position)
{
    if(position > 0 && position < 10000)
    {
        m_processed_stamp = static_cast<int64_t>(static_cast<float>(m_last_data_stamp - m_initial_data_stamp)*static_cast<float>(position)/static_cast<float>(10000));
        m_reset_process_stamp_flag = true;
    }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{
    vector<string> tmp_files;
    struct dirent **namelist;
    int n;
    n = scandir(dir.c_str(),&namelist, 0 , alphasort);
    if (n < 0)
        perror("scandir");
    else
    {
        while (n--)
        {
            if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != "..")
            {
                tmp_files.push_back(string(namelist[n]->d_name));
            }
            free(namelist[n]);
        }
        free(namelist);
    }

    for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++)
    {
        files.push_back(*iter);
    }
    return 0;
}

std::vector<std::string> ROSThread::split(std::string input, char delimiter)
{
    vector<string> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter))
    {
        answer.push_back(temp);
    }

    return answer;
}
