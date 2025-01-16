find_package(GTest REQUIRED)
if(${GTest_FOUND})
  include(GoogleTest)
  enable_testing()
  include(CTest REQUIRED)
else()
  message(FATAL_ERROR "unable to add gtest: missing pacakge GTest")
endif()

if(COMPILE_ROS1_MODULE)
  find_package(rostest REQUIRED)
endif()

if(COMPILE_ROS2_MODULE)
  find_package(ament_cmake_gtest REQUIRED)
endif()
#
# cnr_unit_gtest
#
macro(cnr_unit_gtest trg src deps)
  
  list(APPEND TEST_DEPENDENCIES ${deps})
  
  if(COMPILE_ROS1_MODULE)
    catkin_add_gtest(
      "${trg}"
      "${src}"
    )
    if(TARGET ${trg})
      target_link_libraries(${trg} ${TEST_DEPENDENCIES})
    endif()
  else()
    add_executable(${trg} "${src}")
    gtest_discover_tests(${trg})
    list(APPEND TEST_DEPENDENCIES
          Threads::Threads
          GTest::Main
          Boost::program_options
          Boost::system
          Boost::filesystem
          Boost::iostreams
          Boost::regex
    )
    target_link_libraries(${trg} PUBLIC ${TEST_DEPENDENCIES})
  endif()

  if(TARGET ${trg})
    target_compile_definitions(${trg} PRIVATE TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  endif()
    
endmacro()
#
# end cnr_unit_gtest
#

#
# cnr_integration_gtest
#
macro(cnr_integration_gtest trg src deps)
  
  add_executable(${trg} "${src}")
  gtest_discover_tests(${trg})
  list(APPEND TEST_DEPENDENCIES
        Threads::Threads
        GTest::Main
        Boost::program_options
        Boost::system
        Boost::filesystem
        Boost::iostreams
        Boost::regex
  )
  if(TARGET ${trg})
    target_link_libraries(${trg} PUBLIC ${TEST_DEPENDENCIES})
    target_compile_definitions(${trg} PRIVATE TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/")
  endif()
    
endmacro()
#
# end cnr_unit_gtest
#


if(BUILD_UNIT_TESTS)
  # EXECUTABLE #################################################################
  if(COMPILE_ROS2_MODULE)
    cnr_unit_gtest(test_ros2_yaml_formatter ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_yaml_formatter.cpp cnr_param::cnr_param)
    list(APPEND TARGETS_LIST test_ros2_yaml_formatter)
  endif()

  # EXECUTABLE #################################################################
  cnr_unit_gtest(test_mapped_file ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mapped_file_module.cpp cnr_param::cnr_param)

  list(APPEND TARGETS_LIST test_mapped_file)
endif()


if(BUILD_INTEGRATION_TESTS)

  # EXECUTABLE #################################################################
  if(COMPILE_ROS1_MODULE)
    add_rostest_gtest(
      test_ros_module 
      test/launch/ros1_test.test
      test/test_ros_module.cpp
    )
    target_link_libraries(test_ros_module  cnr_param::cnr_param ${catkin_LIBRARIES})
    list(APPEND TARGETS_LIST test_ros_module)

    # message(STATUS "Compiling test_cnr_param_ros_and_mapped_file.cpp")
    # add_rostest_gtest(
    #    test_cnr_param_ros_and_mapped_file
    #    test/launch/cnr_param_test.test
    #    test/test_cnr_param.cpp
    # )
    # target_link_libraries(test_cnr_param_ros_and_mapped_file  cnr_param::cnr_param ${catkin_LIBRARIES})
    # target_compile_definitions(test_cnr_param_ros_and_mapped_file PRIVATE TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/")
    # list(APPEND TARGETS_LIST test_cnr_param_ros_and_mapped_file)
  endif()

  if(COMPILE_ROS2_MODULE)
    # EXECUTABLE #################################################################
    cnr_integration_gtest(test_ros2_module ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros2_module.cpp cnr_param::cnr_param)

    target_compile_definitions(test_ros2_module
                PRIVATE TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/")
    list(APPEND TARGETS_LIST test_ros2_module)
    # EXECUTABLE #################################################################

    cnr_integration_gtest(test_cnr_param ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cnr_param.cpp cnr_param::cnr_param)

    target_compile_definitions(test_cnr_param
                PRIVATE TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/")
  endif()
endif()
