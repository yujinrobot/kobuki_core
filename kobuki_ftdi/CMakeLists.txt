##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(kobuki_ftdi)

##############################################################################
# Find Packages
##############################################################################

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_command_line REQUIRED)

# pkg-config packages
find_package(PkgConfig)
pkg_search_module(libusb REQUIRED libusb)
pkg_search_module(libftdi REQUIRED libftdi)

##############################################################################
# Platform Configuration
##############################################################################

ament_package_xml()
ecl_enable_cxx14_compiler()


##############################################################################
# Sources
##############################################################################

add_subdirectory(src)

##############################################################################
# Install data files
##############################################################################

install(DIRECTORY bluetooth
        DESTINATION share
)
install(DIRECTORY eeproms
        DESTINATION share
)
install(PROGRAMS scripts/turtlebot_config
                 scripts/create_udev_rules
        DESTINATION bin
)

install(FILES 57-kobuki.rules
        DESTINATION share
)

# This has issues....hydro doing th same thing will result in
# a file conflict. We could call it 57-groovy-kobuki.rules and
# 57-hydro-kobuki.rules. But that seems kind of lame and could
# break things if they are actually different (which should
# be called - udev will have no idea).

#install(FILES 57-kobuki.rules
#        DESTINATION /etc/udev/rules.d/
#)

##############################################################################
# Exports
##############################################################################

ament_package()
