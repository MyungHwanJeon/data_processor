cmake_minimum_required(VERSION 2.8.3)
project(player)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE release)
endif ()



find_package(catkin REQUIRED cmake_modules
  COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  vision_msgs
  rosgraph_msgs
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
set(CMAKE_AUTOMOC ON)

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
    player
  CATKIN_DEPENDS 
    roscpp rospy 
    std_msgs 
    geometry_msgs 
    vision_msgs
    rosgraph_msgs
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

set (player_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set (player_QTLib_hdr ${INCLUDE_DIR}/mainwindow.h ${INCLUDE_DIR}/ROSThread.h ${INCLUDE_DIR}/userdef.h)
set (player_QTLib_ui  ${SRC_DIR}/mainwindow.ui)
set (player_QTBin_src ${SRC_DIR}/main.cpp)



#find_package(GLEW REQUIRED)
#find_package(GLUT REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${player_INCLUDE_DIRS}
  ${SRC_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}  
  ${PROJECT_BINARY_DIR}
  include
  ${Eigen_INCLUDE_DIRS}
)


qt5_wrap_cpp(player_QTLib_hdr_moc ${player_QTLib_hdr})
qt5_wrap_ui (player_QTLib_ui_moc  ${player_QTLib_ui})
#qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)



###########
## Build ##
###########

add_definitions(-std=c++11)

#add_definitions(${Qt5Widgets_DEFINITIONS})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_library(player_QTLib SHARED
 ${player_QTLib_src}
 ${player_QTLib_hdr_moc}
 ${player_QTLib_ui_moc}
)

target_link_libraries (player_QTLib Qt5::Widgets Qt5::Gui Qt5::OpenGL)

add_executable(player ${player_QTLib_src} ${player_QTBin_src} ${player_QTLib_hdr} ${SHADER_RSC_ADDED} ${player_QTLib_ui_moc})

#add_dependencies(recoder ${PROJECT_NAME}_gencfg)
add_dependencies(player ${catkin_EXPORTED_TARGETS})
add_dependencies(player_QTLib ${catkin_EXPORTED_TARGETS})

target_link_libraries(player

  ${catkin_LIBRARIES}
  ${QT_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  player_QTLib
  GL
  ${BOOST_CUSTOM_LIBS}
  ${Eigen_LIBRARIES}
)



