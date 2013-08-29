cmake_minimum_required(VERSION 2.8.3)
project(kobuki_dock_drive)
find_package(catkin REQUIRED COMPONENTS ecl_threads ecl_geometry ecl_linear_algebra)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki_dock_drive
   CATKIN_DEPENDS ecl_threads ecl_geometry ecl_linear_algebra
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(kobuki_dock_drive src/dock_drive.cpp)

target_link_libraries(kobuki_dock_drive ${catkin_LIBRARIES})

install(TARGETS kobuki_dock_drive
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
