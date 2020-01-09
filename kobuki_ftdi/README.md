Kobuki ftdi
===========

### Documentation ###

* [Official Web Page](http://kobuki.yujinrobot.com) - home page, sales, specifications and hardware howto.
* [Protocol, Usage and Api Documentation](http://yujinrobot.github.com/kobuki/doxygen/index.html) - in doxygen.

### Important Utilities ###

* create_udev_rules - creates /dev/kobuki link 
* get_serial_number
* flasher

### Trouble Shooting ###

* Does kobuki appear as USB device?

> lsusb # See if there is "0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC"

> dmesg # See what happen when kobuki usb is plugged.

* No /dev/kobuki?

# Directly via sources
> sudo cp 57-kobuki.rules /etc/udev/rules.d
> sudo service udev reload
> sudo service udev restart
# Or using the ROS2 client tools
> ros2 run kobuki_ftdi create_udev_rules

* Does kobuki stream data?

> cat /dev/kobuki # check if any data stream happens

* Is the serial number correct?

> sudo <install_location>/lib/kobuki_ftdi/get_serial_number

Check if it is different from below

<pre>
Device #0
  Manufacturer : Yujin Robot
  Product      : iClebo Kobuki
  Serial Number: kobuki_A601D86G
</pre>

If it is different,

> sudo <install_location>/lib/kobuki_ftdi/flasher

Then check the serial again.
