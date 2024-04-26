
find_package(Eigen3 REQUIRED NO_MODULE)

if(COMPILE_ROS1_MODULE)
    find_package(catkin REQUIRED COMPONENTS roscpp)
    catkin_package(
      INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
      LIBRARIES cnr_param_utilities
      CATKIN_DEPENDS roscpp
    ) 
endif()

if(COMPILE_ROS2_MODULE)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rmw REQUIRED)
    find_package(rosidl_runtime_c REQUIRED)
    find_package(rcl_interfaces REQUIRED)
endif()

if(COMPILE_MAPPED_FILE_MODULE)
  find_package(yaml-cpp REQUIRED)
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options iostreams regex)

list(APPEND DEPENDENCIES_INCLUDE_DIRS ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
if(COMPILE_ROS1_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
elseif(COMPILE_ROS2_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS} ${rmw_INCLUDE_DIRS} ${rosidl_runtime_c_INCLUDE_DIRS} ${rcl_interfaces_INCLUDE_DIRS})
endif()

list(APPEND DEPENDENCIES_ROS_LIBRARIES "")

message(STATUS "COMPILE_MAPPED_FILE_MODULE: ${COMPILE_MAPPED_FILE_MODULE}, COMPILE_ROS1_MODULE: ${COMPILE_ROS1_MODULE}, COMPILE_ROS2_MODULE: ${COMPILE_ROS2_MODULE}")
if(COMPILE_ROS1_MODULE)
  list(APPEND DEPENDENCIES_ROS_LIBRARIES ${catkin_LIBRARIES})
endif()

if(COMPILE_ROS2_MODULE)
    list(APPEND DEPENDENCIES_ROS_LIBRARIES ${rclcpp_LIBRARIES})
endif()



if(ENABLE_TESTING)
  if(COMPILE_ROS2_MODULE)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest REQUIRED)
  endif()
endif()