cmake_minimum_required(VERSION 2.8.3)
project(recoder)

set (CMAKE_BUILD_TYPE release)

find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  vision_msgs
  message_generation
  rosbag
  image_transport
  cv_bridge
  dynamic_reconfigure
  pcl_ros
  pcl_conversions
  pcl_msgs
  franka_rpm_msgs
)

find_package(Eigen3 REQUIRED)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
#set (CMAKE_PREFIX_PATH /opt/Qt5.6.1/5.6/gcc_64/lib/cmake)

find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    recoder
  CATKIN_DEPENDS 
    roscpp rospy 
    std_msgs 
    geometry_msgs 
    vision_msgs
    message_runtime 
    image_transport 
    cv_bridge 
    dynamic_reconfigure 
    pcl_ros
    pcl_conversions  
    pcl_msgs
    franka_rpm_msgs

  DEPENDS 
    Eigen
)


set (SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set (INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set (Recoder_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set (Recoder_QTLib_hdr ${INCLUDE_DIR}/mainwindow.h ${INCLUDE_DIR}/ROSThread.h ${INCLUDE_DIR}/userdef.h)
set (Recoder_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (Recoder_QTBin_src ${SRC_DIR}/main.cpp)



#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${recoder_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}  
  ${PROJECT_BINARY_DIR}
  include
  ${Eigen_INCLUDE_DIRS}
)


qt5_wrap_cpp(Recoder_QTLib_hdr_moc ${Recoder_QTLib_hdr})
qt5_wrap_ui (Recoder_QTLib_ui_moc  ${Recoder_QTLib_ui})
#qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)



###########
## Build ##
###########

add_definitions(-std=c++11)

#add_definitions(${Qt5Widgets_DEFINITIONS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_library(Recoder_QTLib SHARED
 ${Recoder_QTLib_src}
 ${Recoder_QTLib_hdr_moc}
 ${Recoder_QTLib_ui_moc}
)

target_link_libraries (Recoder_QTLib Qt5::Widgets Qt5::Gui Qt5::OpenGL)

add_executable(recoder ${Recoder_QTLib_src} ${Recoder_QTBin_src} ${SHADER_RSC_ADDED})     

#add_dependencies(recoder ${PROJECT_NAME}_gencfg)
add_dependencies(recoder ${catkin_EXPORTED_TARGETS})
add_dependencies(Recoder_QTLib ${catkin_EXPORTED_TARGETS})

target_link_libraries(recoder

  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  Recoder_QTLib
  GL
  ${BOOST_CUSTOM_LIBS}
  ${Eigen_LIBRARIES}
)



