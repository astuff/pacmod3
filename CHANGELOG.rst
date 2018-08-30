^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pacmod3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
