/**
 * @file /kobuki_driver/src/test/sigslots.cpp
 *
 * @brief Example/test program for kobuki sigslots.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>

#include <ecl/command_line.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <iostream>
#include <kobuki_driver/kobuki.hpp>

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager(const std::string & device) :
      slot_stream_data(&KobukiManager::processStreamData, *this) // establish the callback
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/mobile_base"; // configure the first part of the sigslot namespace
    parameters.device_port = device;
    // configure other parameters here
    kobuki.init(parameters);
    slot_stream_data.connect("/mobile_base/stream_data");
  }

  void spin() {
    ecl::Sleep sleep(1);
    while ( true ) {
      sleep();
    }
  }

  /*
   * Called whenever the kobuki receives a data packet. Up to you from here to process it.
   *
   * Note that special processing is done for the various events which discretely change
   * state (bumpers, cliffs etc) and updates for these are informed via the xxxEvent
   * signals provided by the kobuki driver.
   */
  void processStreamData() {
    kobuki::CoreSensors::Data data = kobuki.getCoreSensorData();
    std::cout << "Encoders [" <<  data.left_encoder << "," << data.right_encoder << "]" << std::endl;
  }

private:
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv) {
  ecl::CmdLine cmd_line("sigslots demo", ' ', "0.2");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the kobuki", false, "/dev/kobuki", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  KobukiManager kobuki_manager(device_port.getValue());
  kobuki_manager.spin();
  return 0;
}
