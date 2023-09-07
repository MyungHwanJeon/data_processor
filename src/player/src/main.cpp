#include "mainwindow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "player");
    ros::NodeHandle nh;

    QApplication a(argc, argv);
    MainWindow w;    
    w.show();

    return a.exec();
}
