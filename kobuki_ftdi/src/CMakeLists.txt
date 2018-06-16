###############################################################################
# Macro
###############################################################################

# The arguments beyond 'name' (ARGN) are the system libraries to link
macro(kobuki_ftdi_add_utility name)
  set(target_name kobuki_ftdi_utility_${name})
  add_executable(${target_name} ${name}.cpp)
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME ${name})
  target_include_directories(
    ${target_name}
    PRIVATE
      $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )
  target_link_libraries(
    ${target_name}
    ecl_command_line::ecl_command_line
    ${ARGN}
  )
  install(
    TARGETS
      ${target_name}
    RUNTIME DESTINATION
      lib/${PROJECT_NAME}
  )
endmacro()

##############################################################################
# Binaries
##############################################################################

kobuki_ftdi_add_utility(ftdi_scan ftdi)
kobuki_ftdi_add_utility(ftdi_kobuki ftdi)
kobuki_ftdi_add_utility(ftdi_read_eeprom ftdi)
kobuki_ftdi_add_utility(ftdi_write_eeprom ftdi)
kobuki_ftdi_add_utility(find_devices usb)
kobuki_ftdi_add_utility(get_serial_number usb)
kobuki_ftdi_add_utility(flasher usb ftdi)
kobuki_ftdi_add_utility(reset_device usb)
kobuki_ftdi_add_utility(overwrite_serial_number usb ftdi)
kobuki_ftdi_add_utility(unflasher usb ftdi)

