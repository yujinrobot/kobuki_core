###############################################################################
# Macro
###############################################################################

macro(kobuki_driver_add_demo name)
  set(target_name kobuki_driver_demo_${name})
  add_executable(${target_name} ${name}.cpp)
  target_link_libraries(${target_name}
    ${PROJECT_NAME}
  )
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME demo_${name})
  install(
    TARGETS
      ${target_name}
    RUNTIME DESTINATION
      lib/${PROJECT_NAME}
  )
endmacro()

###############################################################################
# Tests
###############################################################################

kobuki_driver_add_demo(velocity_commands)
kobuki_driver_add_demo(initialisation)
kobuki_driver_add_demo(sigslots)
kobuki_driver_add_demo(simple_loop)
