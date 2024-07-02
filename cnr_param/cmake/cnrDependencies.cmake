# Eigen3
# ######################################################################################
find_package(Eigen3 REQUIRED COMPONENTS Core Dense Geometry)

# Yaml-cpp
# ####################################################################################
if(yaml-cpp VERSION_LESS "0.8")
  set(YAML_CPP_HAS_NAMESPACE 0)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(yaml-cpp_pkg_config REQUIRED yaml-cpp IMPORTED_TARGET)
else()
  set(YAML_CPP_HAS_NAMESPACE 1)
endif()

# Boost
# ########################################################################################
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options
                                       iostreams regex)

# ROS 1 DEPENDENCIES
# ###########################################################################
if(ROS1_MODULE)
  message(STATUS "Loading the catkin dependencies...")
  find_package(catkin REQUIRED COMPONENTS roscpp)
endif()

# ROS 2 DEPENDENCIES
# ###########################################################################
if(ROS2_MODULE)
  find_package(rclcpp REQUIRED)
  find_package(rmw REQUIRED)
  find_package(rosidl_runtime_c REQUIRED)
  find_package(rcl_interfaces REQUIRED)
endif()

# Catkin is a fake dependency. It is used to test the cmake configuration
# ######################
find_package(catkin QUIET)
if(${catkin_FOUND})
  set(PACKAGE_LIB_DESTINATION     "${CATKIN_GLOBAL_LIB_DESTINATION}")
  set(PACKAGE_BIN_DESTINATION     "${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME}")
  set(PACKAGE_INCLUDE_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
  set(CONFIG_INSTALL_DIR          "share/${PROJECT_NAME}/cmake_alternative")
else()
  set(PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
  set(PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
  set(PACKAGE_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
  set(CONFIG_INSTALL_DIR "share/${PROJECT_NAME}/cmake")
endif()

# cnr_yaml
# #####################################################################################
find_package(cnr_yaml QUIET)
if(NOT ${cnr_yaml_FOUND})
  if(${catkin_FOUND})
    message(STATUS "CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
    find_package(cnr_yaml REQUIRED PATHS ${CMAKE_INSTALL_PREFIX}/.. NO_DEFAULT_PATH)
  else()
    find_package(cnr_yaml REQUIRED PATHS ${CMAKE_INSTALL_PREFIX}/.. NO_DEFAULT_PATH)
  endif()
endif()

# Keys: DEPENDENCIES_INCLUDE_DIRS
# ##############################################################
# DEPENDENCIES_LIBRARIES
# ##############################################################
list(APPEND DEPENDENCIES_INCLUDE_DIRS "${Boost_INCLUDE_DIRS}"
     "${EIGEN3_INCLUDE_DIRS}")

if(NOT BOOL:${YAML_CPP_HAS_NAMESPACE})
  list(APPEND DEPENDENCIES_INCLUDE_DIRS ${yaml-cpp_INCLUDE_DIRS})
endif()

if(ROS1_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS "${catkin_INCLUDE_DIRS}")
elseif(ROS2_MODULE)
  list(APPEND DEPENDENCIES_INCLUDE_DIRS "${rclcpp_INCLUDE_DIRS}"
       "${rmw_INCLUDE_DIRS}" "${rosidl_runtime_c_INCLUDE_DIRS}"
       "${rcl_interfaces_INCLUDE_DIRS}")
endif()

if(BOOL:${YAML_CPP_HAS_NAMESPACE})
  list(APPEND DEPENDENCIES_LIBRARIES yaml-cpp::yaml-cpp)
else()
  list(APPEND DEPENDENCIES_LIBRARIES yaml-cpp)
endif()

list(
  APPEND
  DEPENDENCIES_LIBRARIES
  Boost::system
  Boost::filesystem
  Boost::program_options
  Boost::iostreams
  Boost::regex)

if(ROS1_MODULE)
  list(APPEND DEPENDENCIES_LIBRARIES "${catkin_LIBRARIES}")
endif()

if(ROS2_MODULE)
  list(APPEND DEPENDENCIES_LIBRARIES "${rclcpp_LIBRARIES}")
endif()
