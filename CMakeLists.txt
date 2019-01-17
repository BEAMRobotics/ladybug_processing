cmake_minimum_required(VERSION 3.0)
project(ladybug_processing)

add_compile_options(-std=c++14)# -Wall -Werror)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
#set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O0")

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        sensor_msgs
        pcl_ros
        pcl_conversions
        ladybug_msgs
        tf
        tf2
        tf2_ros
        ladybug_camera_driver
        )

## Find system libraries
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "debug 1: cmake_module_path: ${CMAKE_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")
message(STATUS "debug 2: cmake_module_path: ${CMAKE_MODULE_PATH}")

#find_package(Ladybug QUIET)
#message(STATUS "Debug3----------- ${Ladybug_FOUND}")

#find_package(DC1394 REQUIRED)
#find_package(DC1394 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(DC1394_PKGCONF REQUIRED libdc1394-2)
find_path(DC1394_INCLUDE_DIR
        NAMES dc1394.h
        PATHS ${DC1394_INCLUDE_DIRS}
        )
find_library(DC1394_LIBRARY
        NAMES dc1394
        PATHS ${DC1394_LIBRARIES}
        )


catkin_package(
        INCLUDE_DIRS
        include
        #    ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
        ${PROJECT_NAME}_core
        CATKIN_DEPENDS
        pcl_ros
        roscpp
        sensor_msgs
        ladybug_msgs
        #  DEPENDS
        #    Boost
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        #${Ladybug_INCLUDE_DIRS}
        ${DC1394_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
        src/Colorizer.cpp
        )
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})

## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/${PROJECT_NAME}_node.cpp
        src/LadybugColorizerNode.cpp
        )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(${PROJECT_NAME}_core
#        ${PCL_LIBRARIES}
#        #${catkin_EXPORTED_TARGETS}
#        #${BOOST_LIBRARIES}
#        )
#
#add_dependencies(${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS}
#        )

#add_dependencies(${PROJECT_NAME}_core ladybug_msgs_generate_messages_cpp)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
        #${Ladybug_LIBRARIES}
        ${OpenCV_LIBS}
        ${DC1394_LIBRARIES}
        ${catkin_LIBRARIES}
        )

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_core
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
        DIRECTORY doc
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test
            test/test_ladybug_processing.cpp
            test/ColorizerTest.cpp)
endif ()

if (TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif ()
