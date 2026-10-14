cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

project(lio LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_ENABLE_TESTING OFF CACHE BOOL "Disable catkin tests" FORCE)
# Keep Eigen's allocation ABI compatible with the prebuilt PCL libraries.
# -march=native enables AVX alignment on capable CPUs while Ubuntu's PCL is
# built for the generic x86-64 baseline, which makes cross-library point-cloud
# allocation/free unsafe.
set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH "/usr/lib/x86_64-linux-gnu")

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
# Ubuntu 22.04's PCLConfig can append a directory-scoped QHULL::QHULL target
# to this legacy variable.  The requested common/io/filters shared libraries
# already carry their own qhull linkage, so do not propagate that broken item.
list(REMOVE_ITEM PCL_LIBRARIES QHULL::QHULL)
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

option(LIO_ENABLE_LOOP_CLOSURE "Build the MID360 Scan Context / ICP / GTSAM backend" ON)
if(LIO_ENABLE_LOOP_CLOSURE)
    find_package(GTSAM REQUIRED)
    find_package(Threads REQUIRED)
    find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
    list(REMOVE_ITEM PCL_LIBRARIES QHULL::QHULL)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Standard ROS setup exports 1/2. Livox build.sh overwrites the inherited
# environment variable with ROS1/ROS2, so accept both spellings here and in
# package.xml to preserve colcon's conditional dependencies and build order.
if("$ENV{ROS_VERSION}" STREQUAL "2" OR "$ENV{ROS_VERSION}" STREQUAL "ROS2")
    set(LIO_ROS_VERSION 2)
elseif("$ENV{ROS_VERSION}" STREQUAL "1" OR "$ENV{ROS_VERSION}" STREQUAL "ROS1")
    set(LIO_ROS_VERSION 1)
else()
    find_package(catkin QUIET)
    if(catkin_FOUND)
        set(LIO_ROS_VERSION 1)
    else()
        message(FATAL_ERROR "Cannot determine ROS version. Source a ROS 1 or ROS 2 setup file first.")
    endif()
endif()
message(STATUS "Building lio for ROS ${LIO_ROS_VERSION}")

set(LIO_ROS2_MESSAGE_FILES
    "msg/WheelInfo.msg"
    "msg/ElevatorState.msg"
)

set(LIO_ROS_DEPENDENCIES
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    pcl_ros
    pcl_conversions
    tf2_ros
    std_srvs
    pluginlib
)

if(LIO_ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        rospy
        ${LIO_ROS_DEPENDENCIES}
        message_generation
        rviz
    )

    add_message_files(FILES
        CustomMsg.msg
        CustomPoint.msg
        wheel_info.msg
        ElevatorState.msg
    )
    generate_messages(DEPENDENCIES geometry_msgs std_msgs nav_msgs sensor_msgs)

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES lio_rviz_plugins
        DEPENDS EIGEN3 PCL
        CATKIN_DEPENDS
            roscpp roslib std_msgs nav_msgs geometry_msgs sensor_msgs
            pcl_ros pcl_conversions tf2_ros message_runtime rviz pluginlib
    )
else()
    find_package(ament_cmake REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(rosidl_default_runtime REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(pcl_ros REQUIRED)
    find_package(pcl_conversions REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(rviz_common REQUIRED)
    find_package(livox_ros_driver2 REQUIRED)

    # pcl_ros locates PCL again and repopulates this legacy variable.
    list(REMOVE_ITEM PCL_LIBRARIES QHULL::QHULL)

    rosidl_generate_interfaces(${PROJECT_NAME}
        ${LIO_ROS2_MESSAGE_FILES}
        DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
    )
    rosidl_get_typesupport_target(LIO_TYPESUPPORT_TARGET ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

set(LIO_COMMON_INCLUDE_DIRS
    include
    include/ikd_tree
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(LIO_NODE_SOURCES
    include/support/YamlReader.cpp
    src/main.cpp
    src/support/TopicReceive.cpp
    src/support/common_lib.cpp
    src/estimator/ESEKF.cpp
    include/ikd_tree/ikd_Tree.cpp
    src/node/publish.cpp
    src/node/callbacks.cpp
    src/node/map.cpp
    src/node/save.cpp
    src/node/LidarPipeline.cpp
    src/estimator/IMUProcess.cpp
    src/elevator/ElevatorProcess.cpp
    src/elevator/ElevatorSelfExit.cpp
    src/node/service.cpp
    src/node/loop.cpp
)

if(LIO_ENABLE_LOOP_CLOSURE)
    # Compile the copied implementation directly into this project's node.
    list(APPEND LIO_NODE_SOURCES src/loop/LoopClosure.cpp src/loop/Scancontext.cpp)
endif()

function(lio_configure_target target)
    target_include_directories(${target} PRIVATE ${LIO_COMMON_INCLUDE_DIRS})
    target_compile_definitions(${target} PRIVATE
        LIO_ROS_VERSION=${LIO_ROS_VERSION}
        PACKAGE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_link_libraries(${target}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        yaml-cpp
        ${OpenCV_LIBS}
        TBB::tbb
        OpenMP::OpenMP_CXX
    )

    if(LIO_ROS_VERSION EQUAL 1)
        target_include_directories(${target} PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(${target} ${catkin_LIBRARIES})
        add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    else()
        ament_target_dependencies(${target}
            rclcpp ament_index_cpp geometry_msgs nav_msgs sensor_msgs std_msgs
            tf2_ros std_srvs livox_ros_driver2
        )
        target_link_libraries(${target} "${LIO_TYPESUPPORT_TARGET}")
        add_dependencies(${target} ${PROJECT_NAME}__rosidl_generator_cpp)
    endif()
endfunction()

# Keep the installed executable name "lio", while avoiding a CMake target-name
# collision with rosidl_generate_interfaces(${PROJECT_NAME}) on ROS 2.
add_executable(lio_node_bin ${LIO_NODE_SOURCES})
set_target_properties(lio_node_bin PROPERTIES OUTPUT_NAME lio)
lio_configure_target(lio_node_bin)
target_compile_definitions(lio_node_bin PRIVATE LIO_ENABLE_LOOP_CLOSURE=$<BOOL:${LIO_ENABLE_LOOP_CLOSURE}>)
if(LIO_ENABLE_LOOP_CLOSURE)
    target_include_directories(lio_node_bin PRIVATE ${GTSAM_INCLUDE_DIR} ${GTSAM_INCLUDE_DIRS})
    target_link_libraries(lio_node_bin gtsam Threads::Threads)
endif()

add_executable(sim_node
    src/sim/run_sim_node.cpp
    src/sim/ElevatorTrajectory.hpp
    src/sim/ObjectRelativeTrajectory.hpp
    src/sim/ObjectWorldTrajectory.hpp
    src/sim/SimSensorPublisher.hpp
)
lio_configure_target(sim_node)

qt5_wrap_cpp(ELEVATOR_STATUS_PANEL_MOC include/rviz/ElevatorStatusPanel.h
    OPTIONS -DLIO_ROS_VERSION=${LIO_ROS_VERSION}
)
add_library(lio_rviz_plugins SHARED
    src/rviz/ElevatorStatusPanel.cpp
    ${ELEVATOR_STATUS_PANEL_MOC}
)
target_include_directories(lio_rviz_plugins PRIVATE ${LIO_COMMON_INCLUDE_DIRS})
target_compile_definitions(lio_rviz_plugins PRIVATE LIO_ROS_VERSION=${LIO_ROS_VERSION})
target_link_libraries(lio_rviz_plugins Qt5::Widgets)

if(LIO_ROS_VERSION EQUAL 1)
    target_include_directories(lio_rviz_plugins PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(lio_rviz_plugins ${catkin_LIBRARIES})
    add_dependencies(lio_rviz_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
    ament_target_dependencies(
        lio_rviz_plugins rclcpp rviz_common pluginlib std_msgs livox_ros_driver2
    )
    target_link_libraries(lio_rviz_plugins "${LIO_TYPESUPPORT_TARGET}")
    add_dependencies(lio_rviz_plugins ${PROJECT_NAME}__rosidl_generator_cpp)
    pluginlib_export_plugin_description_file(rviz_common plugin_description_ros2.xml)
endif()

if(LIO_ROS_VERSION EQUAL 1)
    install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(TARGETS lio_node_bin sim_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(TARGETS lio_rviz_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
    install(FILES src/sim/config.jsonc DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sim)
    install(DIRECTORY launch rviz yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    install(DIRECTORY include/ DESTINATION include)
    install(TARGETS lio_node_bin sim_node lio_rviz_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    install(DIRECTORY launch rviz yaml DESTINATION share/${PROJECT_NAME})
    install(FILES plugin_description_ros2.xml DESTINATION share/${PROJECT_NAME})
    install(FILES src/sim/config.jsonc DESTINATION share/${PROJECT_NAME}/sim)
    ament_export_include_directories(include)
    ament_export_dependencies(rosidl_default_runtime livox_ros_driver2)
    ament_package()
endif()
