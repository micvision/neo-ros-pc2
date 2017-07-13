cmake_minimum_required(VERSION 2.8.3)
project(neo_ros_pc2)

find_package(catkin REQUIRED COMPONENTS
        rosconsole
        roscpp
        sensor_msgs
        pcl_msgs
        dynamic_reconfigure
        )

generate_dynamic_reconfigure_options(cfg/filter.cfg)

find_package(PCL REQUIRED)
find_package(Neo REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

include_directories(
	include ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
        ${PCL_LIBRARY_DIRS}
)

catkin_package()

add_executable(neo_node_pc2 src/node.cpp src/line_segment.cpp src/line_extraction.cpp)

add_dependencies(neo_node_pc2 neo_ros_pc2_gencfg)
target_link_libraries(neo_node_pc2 ${catkin_LIBRARIES} ${LIBNEO_LIBRARY} ${PCL_LIBRARIES})

target_include_directories(neo_node_pc2 SYSTEM PRIVATE ${LIBNEO_INCLUDE_DIR})

install(TARGETS neo_node_pc2
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
