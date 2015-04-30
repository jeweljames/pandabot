cmake_minimum_required(VERSION 2.8.3)
project(sociobots_driver)

find_package(catkin REQUIRED COMPONENTS geometry_msgs
                                        roscpp
                                        std_msgs
                                        nav_msgs
                                        tf

)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rospy roscpp tf angles
                  geometry_msgs sensor_msgs nav_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(single_minion src/single_minion_node.cpp src/miniQ.cpp src/CerealPort.cpp)
target_link_libraries(single_minion ${catkin_LIBRARIES})

add_executable(tf_updater src/tf_updater.cpp src/miniQ.cpp src/CerealPort.cpp)
target_link_libraries(tf_updater ${catkin_LIBRARIES})

add_executable(multi_minion src/multi_minion.cpp src/miniQ.cpp src/CerealPort.cpp)
target_link_libraries(multi_minion ${catkin_LIBRARIES})


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   scripts/single_minion_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)