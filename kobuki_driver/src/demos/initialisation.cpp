/**
 * @file /kobuki_driver/src/test/initialisation.cpp
 *
 * @brief Demo program for kobuki initialisation.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <kobuki_driver/kobuki.hpp>
#include <ecl/time.hpp>
#include <ecl/command_line.hpp>

class KobukiManager {
public:
  KobukiManager(const std::string & device) {
    kobuki::Parameters parameters;
    // change the default device port from /dev/kobuki to /dev/ttyUSB0
    parameters.device_port = device;
    // Other parameters are typically happy enough as defaults
    // namespaces all sigslot connection names under this value, only important if you want to
    parameters.sigslots_namespace = "/kobuki";
    // Most people will prefer to do their own velocity smoothing/acceleration limiting.
    // If you wish to utilise kobuki's minimal acceleration limiter, set to true
    parameters.enable_acceleration_limiter = false;
    // If your battery levels are showing significant variance from factory defaults, adjust thresholds.
    // This will affect the led on the front of the robot as well as when signals are emitted by the driver.
    parameters.battery_capacity = 16.5;
    parameters.battery_low = 14.0;
    parameters.battery_dangerous = 13.2;

    // initialise - it will throw an exception if parameter validation or initialisation fails.
    try {
      kobuki.init(parameters);
    } catch ( ecl::StandardException &e ) {
      std::cout << e.what();
    }
  }
private:
  kobuki::Kobuki kobuki;
};

int main(int argc, char** argv) {
  ecl::CmdLine cmd_line("initialisation demo", ' ', "0.2");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the kobuki", false, "/dev/kobuki", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  KobukiManager kobuki_manager(device_port.getValue());
  ecl::Sleep()(5);
  return 0;
}
