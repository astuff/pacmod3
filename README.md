# PACMod (Platform Actuation and Control MODule) Vehicle Interface for Board Revision 3 #

[![CircleCI](https://circleci.com/gh/astuff/pacmod3/tree/master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod3/tree/master)

This ROS node is designed to allow the user to control a vehicle (see SUPPORTED VEHICLES below) with the PACMod drive-by-wire system, board revision 3. For more information about the topics, parameters, and details on the implementation, see [the ROS wiki](http://wiki.ros.org/pacmod3).

For access to the DBC file which defines the CAN interface for the PACMod, see the [pacmod_dbc](https://github.com/astuff/pacmod_dbc) repo.

| Supported Vehicles | ROS Version Available | PACMod Version | ROS Driver Branch |
| - | - | - | - |
| Polaris GEM Series (e2/e4/e6) MY 2016+ | ROS | PACMod2 | [PACMod2 Driver](https://github.com/astuff/pacmod/tree/release) |
| Polaris eLXD MY 2016+ | ROS | PACMod2 | [PACMod2 Driver](https://github.com/astuff/pacmod/tree/release) |
| International Prostar+ 122 | ROS | PACMod2 | [PACMod2 Driver](https://github.com/astuff/pacmod/tree/release) |
| Lexus RX-450h MY 2016+ | ROS and ROS2 | PACMod3 | [PACMod3 Driver](https://github.com/astuff/pacmod3) and [ROS2 Driver (DBC 3.4)](https://github.com/astuff/pacmod3/tree/dashing-devel) |
| Lexus RX-450h MY 2016+ V3| ROS | PACMod3 | [PACMod3 Driver with message migration](https://github.com/astuff/pacmod3/tree/maint/pacmod_msg_migration) |
| Kenworth T680 Semi 2017+ |ROS | PACMod3 | [PACMod3 Driver](https://github.com/astuff/pacmod3) |
| Freightliner Cascadia DD13 DayCab/Sleeper/Extended-Sleeper | ROS | PACMod3 | [PACMod3 Driver](https://github.com/astuff/pacmod3)|
| Tractor 2017+ | ROS | PACMod3 | [PACMod3 Driver (Hexagon Tractor)](https://github.com/astuff/pacmod3/tree/maint/hexagon_tractor) |
| Ford Ranger 2019+ | ROS | PACMod3 |  [PACMod3 Driver with message migration](https://github.com/astuff/pacmod3/tree/maint/pacmod_msg_migration) |
| Polaris Ranger X900 | ROS | PACMod3 |  [PACMod3 Driver with message migration](https://github.com/astuff/pacmod3/tree/maint/pacmod_msg_migration) |
| Toyota Minivan 2019+ | ROS | PACMod3 |  [PACMod3 Driver with message migration](https://github.com/astuff/pacmod3/tree/maint/pacmod_msg_migration) |
| VEHICLE_HCV | ROS | PACMod3 | [PACMod3 Driver with message migration](https://github.com/astuff/pacmod3/tree/maint/pacmod_msg_migration) |
| VEHICLE_FTT | ROS | PACMod3 | [PACMod3 Driver with message migration](https://github.com/astuff/pacmod3/tree/maint/pacmod_msg_migration) |
More coming soon...
