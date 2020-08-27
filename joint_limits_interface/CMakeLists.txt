cmake_minimum_required(VERSION 3.5.0)
project(joint_limits_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)

###########
## Build ##
###########

# Specify header include paths


#############
## Testing ##
#############
if(0)
#if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(joint_limits_interface_test test/joint_limits_interface_test.cpp)
  target_link_libraries(joint_limits_interface_test ${LIBS})

  ament_add_gmock(joint_limits_urdf_test test/joint_limits_urdf_test.cpp)
  target_link_libraries(joint_limits_urdf_test ${LIBS})

if(0) #todo: figure out how this ports over to ament
  add_rostest_gtest(joint_limits_rosparam_test
    test/joint_limits_rosparam.test
    test/joint_limits_rosparam_test.cpp
  )
  target_link_libraries(joint_limits_rosparam_test ${LIBS})
endif()
  
endif()

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_include_directories(include)

ament_package()
