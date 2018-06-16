##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_dock_drive)

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake_ros REQUIRED)

find_package(ecl_build REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
find_package(ecl_threads REQUIRED)

##############################################################################
# Platform Configuration
##############################################################################

ament_package_xml()
ecl_enable_cxx14_compiler()

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Exports
##############################################################################

ament_export_interfaces(${PROJECT_NAME})
ament_export_dependencies(
    ecl_geometry
    ecl_linear_algebra
    ecl_threads
)
# TODO: deprecate this line post 'ardent' (been fixed upstream)
ament_export_libraries(${PROJECT_NAME})
ament_package()
