cmake_minimum_required(VERSION 2.8.3)
project(rob_auto_cal)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  tf2 
  tf2_ros
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(ur_modern_driver REQUIRED)

#ncurses
find_package(PkgConfig REQUIRED)
pkg_check_modules ( ncurses++ REQUIRED ncurses++)

find_package(OpenCV 3 REQUIRED)


## Third party libraries includes



find_library(SDPA 
 NAMES libsdpa sdpa
 PATHS ${CMAKE_CURRENT_LIST_DIR}/sdpa/lib
)

find_library(MUMPS
 NAMES libdmumps_seq dmumps_seq
 PATHS ${CMAKE_CURRENT_LIST_DIR}/sdpa/share/
)


find_library(BLAS
 NAMES libblas3 blas
 PATHS ${CMAKE_CURRENT_LIST_DIR}/sdpa/share/sdpa/blas
)

find_library(PTHREAD
 NAMES libpthread pthread
 PATHS ${CMAKE_CURRENT_LIST_DIR}
)

find_library(LAPACK
 NAMES liblapack lapack
 PATHS ${CMAKE_CURRENT_LIST_DIR}
)




if(SDPA)
 message("SDPA found")
else(SDPA)
 message("SDPA not found")
endif(SDPA)

if(MUMPS)
 message(${MUMPS})
 message("MUMPS found")
else(MUMPS)
 message("MUMPS not found")
endif(MUMPS)

if(BLAS)
 message("BLAS found")
else(BLAS)
 message("BLAS not found")
endif(BLAS)


if(PTHREAD)
 message("PTHREAD found")
else(PTHREAD)
 message("PTHREAD not found")
endif(PTHREAD)


if(LAPACK)
 message("LAPACK found")
else(LAPACK)
 message("LAPACK not found")
endif(LAPACK)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  vectorVector3.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   reqChessTransform.srv
   reqTransCalc.srv
   reqSquareTrans.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trans_calc
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime 
  DEPENDS system_lib Eigen sdpa dmumps_seq lapack blas pthread
)

###########
## Build ##
###########

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include -L/usr/lib -lsdpa -ldmumps_seq -llapack -lblas -lpthread")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${rob_auto_cal_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(rob-auto-cal
#   src/${PROJECT_NAME}/rob-auto-cal.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rob-auto-cal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(rob-auto-cal_node src/rob-auto-cal_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rob-auto-cal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(rob-auto-cal_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rob-auto-cal rob-auto-cal_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rob-auto-cal.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_executable(chess_features src/nodes/chess_features.cpp)
target_link_libraries(chess_features ${catkin_LIBRARIES})
add_dependencies(chess_features rob_auto_cal_chess_features_cpp)
add_dependencies(chess_features ${catkin_EXPORTED_TARGETS})

add_executable(save_vects_node src/nodes/save_vects_node.cpp)
target_link_libraries(save_vects_node ${catkin_LIBRARIES})
add_dependencies(save_vects_node rob_auto_cal_vects_vects_node_cpp)
add_dependencies(save_vects_node ${catkin_EXPORTED_TARGETS})

add_executable(save_trans_node src/nodes/save_trans_node.cpp)
target_link_libraries(save_trans_node ${catkin_LIBRARIES})
add_dependencies(save_trans_node rob_auto_cal_save_trans_node_cpp)


add_executable(image_publisher src/nodes/image_publisher.cpp)
target_link_libraries(image_publisher ${catkin_LIBRARIES})
add_dependencies(image_publisher rob_auto_cal_save_data_node_cpp)

add_executable(save_joints_node src/nodes/save_joints_node.cpp)
target_link_libraries(save_joints_node ${catkin_LIBRARIES} ncurses)
add_dependencies(save_joints_node rob_auto_cal_save_joints_node_cpp)


add_executable(move_ur10 src/nodes/move_ur10.cpp)
target_link_libraries(move_ur10 ${catkin_LIBRARIES})
add_dependencies(move_ur10 rob_auto_cal_move_ur10_cpp)

add_executable(chessboard_move_ur10 src/nodes/chessboard_move_ur10.cpp)
target_link_libraries(chessboard_move_ur10 ${catkin_LIBRARIES})
add_dependencies(chessboard_move_ur10 rob_auto_cal_chessboard_move_ur10_cpp)


add_executable(GR2_control src/nodes/GR2_control.cpp)
target_link_libraries(GR2_control ${catkin_LIBRARIES} ncurses)
add_dependencies(GR2_control rob_auto_cal_GR2_control_cpp)


add_executable(auto_cal_sequence_ur10 src/nodes/auto_cal_sequence_ur10.cpp)
target_link_libraries(auto_cal_sequence_ur10 ${catkin_LIBRARIES} )
add_dependencies(auto_cal_sequence_ur10 rob_auto_cal_auto_cal_sequence_ur10_cpp)
add_dependencies(auto_cal_sequence_ur10 ${catkin_EXPORTED_TARGETS})


add_executable(chess_pickup_sequence src/nodes/chess_pickup_sequence.cpp)
target_link_libraries(chess_pickup_sequence ${catkin_LIBRARIES} )
add_dependencies(chess_pickup_sequence rob_auto_cal_chess_pickup_sequence_cpp)
add_dependencies(chess_pickup_sequence ${catkin_EXPORTED_TARGETS})

add_executable(trans_calc_test src/nodes/trans_calc_test.cpp)
target_link_libraries(trans_calc_test ${catkin_LIBRARIES} )
add_dependencies(trans_calc_test rob_auto_cal_trans_calc_test_cpp)
add_dependencies(trans_calc_test ${catkin_EXPORTED_TARGETS})


add_executable(trans_calc src/nodes/trans_calc/herwc_solver.cpp src/nodes/trans_calc/herwc_problem3.cpp)
target_link_libraries(trans_calc 
${catkin_LIBRARIES} 
${Eigen_LIBRARIES}
${SDPA}
${MUMPS}
${BLAS}
${PTHREAD}
${LAPACK})
add_dependencies(trans_calc rob_auto_cal_trans_calc_cpp)
add_dependencies(trans_calc ${catkin_EXPORTED_TARGETS})
#add_dependencies(trans_calc rob_auto_cal_generate_messages_cpp)



#add_dependencies(package_name ${catkin_EXPORTED_TARGETS}) is needed because 
#a custom ros message that is a part of its own ros package as it is used. This is not considered good
#pratice but this is the only custom message in this package and creating a new separate package just 
#for the message would further complicate the program for low reusability.