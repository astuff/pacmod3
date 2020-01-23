^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pacmod3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2020-01-23)
------------------
* Merge pull request `#59 <https://github.com/astuff/pacmod3/issues/59>`_ from astuff/maint/hybrid_ros1_ros2_msgs
  ROS1/ROS2 Hybrid Messages
* CI: Updating CI to build with hybrid messages.
* Merge pull request `#63 <https://github.com/astuff/pacmod3/issues/63>`_ from astuff/maint/add_engine_report
  Adding Engine Report
* Merge pull request `#64 <https://github.com/astuff/pacmod3/issues/64>`_ from astuff/maint/remove_unused_pid_rpts
  Removing unused PID reports.
* Adding Hazard Lights Cmd and Rpt Message (`#61 <https://github.com/astuff/pacmod3/issues/61>`_)
* Merge pull request `#62 <https://github.com/astuff/pacmod3/issues/62>`_ from astuff/maint/make_parse_const
  Maint/make parse const
* Merge branch 'master' into maint/make_parse_const
* Merge pull request `#60 <https://github.com/astuff/pacmod3/issues/60>`_ from astuff/maint/add_gearN_w_avail
  Add gear number and availability.
* Use master branch of astuff_sensor_msgs
* Adding gear number availability and number.
* Adding Gear number to Shift Aux Report.
* Make the input to all of the parse functions a const pointer since they shouldn't be modifying the raw data
* Merge pull request `#57 <https://github.com/astuff/pacmod3/issues/57>`_ from astuff/maint/cpp_11_updates
  Maint/cpp 11 updates
* Moving CAN_ID values to header file.
* Simplifying syntax in pacmod3_node.cpp.
* Simplifying the can_read function.
* Simplifying publishers.
* templatizing lookup_and_encode.
* Merge pull request `#56 <https://github.com/astuff/pacmod3/issues/56>`_ from astuff/fix/give_cmd_msgs_dlcs
  Adding DATA_LENGTH. Removing unused code.
* Merge pull request `#53 <https://github.com/astuff/pacmod3/issues/53>`_ from astuff/feature/freightliner_cascadia
  Initial Commit - Adding Freightliner Cascadia Trucks
* Reordering CAN messages, adding cruise control to freightliner.
* Merge branch 'master' into feature/freightliner_cascadia
* Adding freightliner specific messages - engine braking, marker lamp, sprayer.
* Making DATA_LENGTH constexpr.
* Adding DATA_LENGTH. Removing unused code.
  DATA_LENGTH is a property which defines the length of the data payload
  on a given CAN message. This is now used to initialize command message
  data storage.
* Merge pull request `#55 <https://github.com/astuff/pacmod3/issues/55>`_ from astuff/feature/door_system_remake
  Feature/door system remake
* Merge branch 'master' into feature/freightliner_cascadia
* Merge pull request `#54 <https://github.com/astuff/pacmod3/issues/54>`_ from astuff/feature/add_Jupiter_Spirit
  Feature/add jupiter spirit
* added vehicle Jupiter_Spirit
* Adding Freightliner Cascadia Trucks
* Merge pull request `#50 <https://github.com/astuff/pacmod3/issues/50>`_ from astuff/maint/dlc_agnosticism
  Making driver DLC-agnostic.
* Merge pull request `#48 <https://github.com/astuff/pacmod3/issues/48>`_ from astuff/maint/add_roslint
  Maint/add roslint
* Replaced all int64_t CAN IDs with uint32_ts.
* Shared Pointers everywhere!
* Fixing more C-style casts.
* Lint and copyright clean-up.
* Update src/pacmod3_ros_msg_handler.cpp
  Recommendation from @zoakesAStuff
  Co-Authored-By: JWhitleyAStuff <jwhitley@autonomoustuff.com>
* CI: Adding catkin_test_results.
* Added roslint. Implemented suggestions. Changed to raw pointers.
* Merge pull request `#47 <https://github.com/astuff/pacmod3/issues/47>`_ from astuff/maint/ci_remove_lunar
  CI: Removing Lunar build AND fixing tests.
* Merge pull request `#46 <https://github.com/astuff/pacmod3/issues/46>`_ from astuff/maint/can_api_changes
  Changed motor_rpt_2 scale values to match new dbc
* Removing DBC. CI: Removing DBC validation.
* Updating reference to DBC in README.
* Contributors: Daniel Stanek, Joshua Whitley, Sam Rustan, Sanaz Fattahalhosseini, Zach Oakes, Sneha Ganesh, Mike Lemm

1.2.1 (2018-12-07)
------------------
* Merge pull request `#43 <https://github.com/astuff/pacmod3/issues/43>`_ from astuff/maint/add_urls
* Adding URLs to package.xml and upadating README.
* Contributors: Daniel-Stanek, Joshua Whitley

1.2.0 (2018-11-19)
------------------
* Merge pull request `#41 <https://github.com/astuff/pacmod3/issues/41>`_ from astuff/fix/vector_comp_dbc_file
  Removing unnecessary line preventing Vector CANdb++ editor from opening
* Merge pull request `#39 <https://github.com/astuff/pacmod3/issues/39>`_ from astuff/feat/add_comp_rpt
  Feature add component report for each PACMod component.
* DBC: Fixing errors. CI: Adding DBC validation to workflow.
* DBC: Typo on two lines.
* Adding encoding for CLEAR_FAULTS flag.
* Adding parsing and publishing for COMPONENT_RPT.
* DBC: Add COMPONENT_RPT and CLEAR_FAULTS flag.
* Merge pull request `#38 <https://github.com/astuff/pacmod3/issues/38>`_ from astuff/feature/add_veh_6
  Adding VEHICLE_6.
* Contributors: Daniel-Stanek, Joshua Whitley, Mike Lemm, Nate Imig

1.1.1 (2018-08-30)
------------------
* Merge pull request `#34 <https://github.com/astuff/pacmod3/issues/34>`_ from astuff/maint/add_none_shift_cmd
* Removing unused COMMANDED_VALUE values on SHIFT_RPT.
* Adding NONE shift value.
* Merge pull request `#33 <https://github.com/astuff/pacmod3/issues/33>`_ from astuff/fix/percent-signs
* Removed erroneous percent sign from signal units
  Before: Percentages were being reported as a decimal value between 0 and 1
  with a percentage sign.  E.g. 0.5 would be reported, but the intent
  was to convey 50%.  i.e. the report or command would read 0.5%
  when it should've read 50% or just simply 0.5.
  After: Values will be reported without the '%' sign.  They report as decimal
  values.
* Forgot to bump DBC after last minor change.
* Merge pull request `#32 <https://github.com/astuff/pacmod3/issues/32>`_ from astuff/fix/steering-rpt-units
* Corrected Steering report units JIRA: LEXUS-131
  Before: steering_rpt reported commanded position in rad/s.
  After: steering_rpt reports commanded position in rad.
* Contributors: Daniel-Stanek, Joshua Whitley, Sam Rustan, Zach Oakes, driscoll85

1.1.0 (2018-08-15)
------------------
* A boost::shared_ptr error would occur on some vehicles on shutdown.
  Moving the allocation of all of the optional subscribers to after
  ros::init is called fixes this error and should not impact functionality
  or performance.
* Temporarily disabling Vehicle 5 steering wheel controls.
* First attempt at adding an All System Statuses topic.
* Changing CAN factors for Steer Aux rpt values.
* Removed incorrect conversion factor for as_tx/vehicle_speed
* Adding publishing for door, interior lights, rear lights, and occupancy rpt.
* Removing wipers from VEHICLE_5.
  This system will not be availble in the initial vehicle release.
* Changes for vehicle 4
* Adding unknown vehicle mfg and year to VIN rpt.
* Adds the ability to parse Dash Controls, CC Btns, and Media Btns.
* Add config_fault_active to global report.
  This indicates that a fault occurred while reading the configuration file.
* Creating framework for publishing of all current Aux msgs.
  Created parsing and publishing framework for Aux messages on the
  following systems: Accel, Brake, Headlights, Shift, Steer, Turn, and
  Wipers.
* Adding VehSpecificRpt1. Removing SteerRpt2 and SteerRpt3.
  VehicleControlsRpt was replaced with VehicleSpecificRpt1. SteerRpt2
  and SteerRpt3 were deemed unnecessary given the addition of SteerAuxRpt.
* Command values for SystemCmdBools were reversed.
  This commit fixes the reversal (commanded true now encodes a 1
  in the CAN message instead of a 0 and vice versa).
* Added report messages DetectedObjectRpt, VehicleControlsRpt, and VehicleDynamicsRpt
* This commit removes state mgmt and the global enable.
  If any system is disabled, it should either be due to a disable
  being sent from the user, an override, or a fault. This means that
  there is no need for us to maintain all systems' states in the driver.
  We can just continue to spam the most recent command and only modify
  it if we receive an override_active or fault_active flag on the global
  command. If either of those are true, we immediately disable all
  systems.
* CAN ID reorganization.
  After talking with the team, reorganizing the CAN IDs prior to
  the use of a PACMod3 in production made sense. This includes
  consideration for priority, grouping based on function, and leaving
  space for future additions.
* Adds clear_override flag to all PACMod 3 command messages.
  This requires approval of the maint/add_clear_override_flag branch
  on astuff_sensor_messages - hence the change to .travis.rosinstall.
  Will have to change this back to master once that branch is approved
  and this is merged into master here.
* Adding Aux rpts for brake/accel/shift/steer. Door/Occ/IntLights/ExtLights rpts.
  Adding framework for parsing Aux reports from brake, accel, shift, and steer systems.
  Adding parsing framework for DoorRpt, OccupancyRpt, InteriorLightsRpt,
  and ExteriorLightsRpt.
* Adding state_change_debounce_counts for each system.
  This will help to prevent quick enable/disable flashes
  on the PACMods and PACMinis by stopping listening to their reports
  for X number of loops after a state change (enable->disable/disable
  ->enable.
* Turn signal was defaulting to 0 (TURN_LEFT). Fixed.
* Fixing Horn cmd type.
* Only listen to system reported state if PACMod is disabling the system.
* Removing recent_state_change stuff. It isn't helping anyway.
* Only saving output value to command if disabled and no recent state change.
* Start debounce count with high number to avoid missing the first state change.
* Finished implementing state change debouncing for all systems.
* Implementing state change debouncing.
* Filling commands with no matching parser with 0s instead of 255s (much safter).
* Added proper class initialization.
* Fixing type difference in can_id value. Fixing bug in SystemRptBool parsing.
* Horn is Bool, not Int.
* Adding additional fault reporting to global rpt and system reports.
* Changing name of CruiseControlSystem to be more accurate (CruiseControlButtonsSystem).
* Adding support for additional vehicle systems.
* Fixing enable/disable problem.
* Setting command = output while disabled for each system.
* Adding clear_override flag.
* First commit with most things changed to pacmod3 (untested).
* Contributors: Daniel-Stanek, Joe Driscoll, Joe Kale, Josh Whitley, Joshua F WHitley, Joshua Whitley, Kyle Rector, Lucas Buckland, Nishanth Samala, Sam Rustan, Samuel Rustan, driscoll85
