cmake_minimum_required(VERSION 3.0.2)
project(kobuki_ftdi)
find_package(catkin REQUIRED COMPONENTS ecl_command_line)

# pkg-config packages
find_package(PkgConfig)
pkg_check_modules(libusb REQUIRED libusb)
pkg_check_modules(libftdi REQUIRED libftdi)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS ecl_command_line
   DEPENDS libusb libftdi
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY bluetooth
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY eeproms
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/turtlebot_config
                 scripts/create_udev_rules
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 57-kobuki.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# This has issues....hydro doing th same thing will result in 
# a file conflict. We could call it 57-groovy-kobuki.rules and
# 57-hydro-kobuki.rules. But that seems kind of lame and could
# break things if they are actually different (which should
# be called - udev will have no idea).

#install(FILES 57-kobuki.rules
#        DESTINATION /etc/udev/rules.d/
#)

