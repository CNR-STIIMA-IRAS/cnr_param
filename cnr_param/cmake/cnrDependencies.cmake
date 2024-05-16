#find_package(Eigen3 REQUIRED Core Dense)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense  Geometry)

if(ROS1_MODULE)
  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package(
    INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES
    cnr_param_core
    CATKIN_DEPENDS
    roscpp)
endif()

if(ROS2_MODULE)
  find_package(rclcpp REQUIRED)
  find_package(rmw REQUIRED)
  find_package(rosidl_runtime_c REQUIRED)
  find_package(rcl_interfaces REQUIRED)
endif()

find_package(yaml-cpp REQUIRED)
if(yaml-cpp VERSION_LESS "0.8")
  set(YAML_CPP_HAS_NAMESPACE 0)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(yaml-cpp_pkg_config REQUIRED yaml-cpp IMPORTED_TARGET)
else()
  set(YAML_CPP_HAS_NAMESPACE 1)
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options iostreams regex)

list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

if(NOT BOOL:${YAML_CPP_HAS_NAMESPACE})
  list(APPEND DEPENDENCIES_INCLUDE_DIRS  ${yaml-cpp_INCLUDE_DIRS})
endif()

if(ROS1_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
elseif(ROS2_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS} ${rosidl_runtime_c_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS})
endif()

if(BOOL:${YAML_CPP_HAS_NAMESPACE})
  list(APPEND DEPENDENCIES_ROS_LIBRARIES yaml-cpp::yaml-cpp)
else()
  list(APPEND DEPENDENCIES_ROS_LIBRARIES yaml-cpp)
endif()

message(
  STATUS
  "MODULES: MAPPED_FILE_MODULE=${MAPPED_FILE_MODULE} | "
  "ROS1_MODULE=${ROS1_MODULE} | "
  "ROS2_MODULE=${ROS2_MODULE}"
)
if(ROS1_MODULE)
  list(APPEND DEPENDENCIES_ROS_LIBRARIES ${catkin_LIBRARIES})
endif()

if(ROS2_MODULE)
  list(APPEND DEPENDENCIES_ROS_LIBRARIES ${rclcpp_LIBRARIES})
endif()