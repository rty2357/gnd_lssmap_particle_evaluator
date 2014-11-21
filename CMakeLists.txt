##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(gnd_lssmap_particle_evaluator)

##############################################################################
# Catkin
##############################################################################

find_package(catkin
  REQUIRED COMPONENTS 
  roscpp
  std_msgs
  sensor_msgs
  gnd_msgs
  gndlib
  gnd_rosutil
  gnd_particle_localizer
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gnd_lssmap_particle_evaluator
  CATKIN_DEPENDS
   roscpp
   std_msgs
   sensor_msgs
   gnd_msgs
   gndlib
   gnd_rosutil
   gnd_particle_localizer
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# Sources
##############################################################################

#add_library(gnd_lssmap_particle_evaluator src/lib/gnd_lssmap_particle_evaluator.cpp)
#target_link_libraries(gnd_lssmap_particle_evaluator ${catkin_LIBRARIES})

#install(TARGETS gnd_lssmap_particle_evaluator
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/gnd_lssmap_particle_evaluator/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

##############################################################################
# Example
##############################################################################

add_executable(gnd_lssmap_particle_evaluator src/gnd_lssmap_particle_evaluator.cpp)
target_link_libraries(gnd_lssmap_particle_evaluator ${catkin_LIBRARIES})
install(TARGETS gnd_lssmap_particle_evaluator 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

##############################################################################
# Test
##############################################################################

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest rosunit)
#  include_directories(${GTEST_INCLUDE_DIRS})
#  catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  endif()
#endif()
