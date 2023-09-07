#ifndef USERDEF_H
#define USERDEF_H

#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>
#include <string.h>
#include <queue>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <rosbag/bag.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include "franka_rpm_msgs/FrankaState.h"


#include <QMainWindow>
#include <QThread>
#include <QVector>
#include <QMutex>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QProcess>
#include <QThread>
#include <QPixmap>
#include <QDateTime>
#include <QReadLocker>
#include <QPainter>
#include <QLabel>
#include <QObject>
#include <QErrorMessage>
#include <QCloseEvent>
#include <QInputDialog>
#include <QApplication>
#include <QMutexLocker>
//#include <QTimer>

#include <signal.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>



template <typename T>
struct DataThread
{
    std::mutex mtx;
    std::queue<T> data_queue;
    std::condition_variable cv;
    std::thread thd;
    long cnt;
    bool active;

    DataThread() : active(false), cnt(0){}

    void push(T data)
    {
        mtx.lock();
        data_queue.push(data);
        cnt++;
        mtx.unlock();
    }

    T pop()
    {
        T result;
        mtx.lock();
        result = data_queue.front();
        data_queue.pop();
        mtx.unlock();
        return result;
    }

    void clear()
    {
        mtx.lock();
        while (!data_queue.empty())
        {
            data_queue.pop();
        }
        mtx.unlock();
    }
};

#endif // USERDEF_H
