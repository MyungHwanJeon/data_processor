#include "mainwindow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "recoder");
    ros::NodeHandle nh;

    QApplication a(argc, argv);
    MainWindow w;
    w.RosInit(nh);
    w.show();

    return a.exec();
}
