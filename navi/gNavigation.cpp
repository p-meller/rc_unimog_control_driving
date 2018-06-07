//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    projects/rc_unimog_control_group2/navi/gNavigation.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/gNavigation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//#include "projects/rc_unimog_control_group2/navi/mChooseScenario.h"
//#include "projects/rc_unimog_control_group2/navi/mDrive.h"
#include "projects/rc_unimog_control_group2/navi/mSpeedComputer.h"
#include "projects/rc_unimog_control_group2/navi/mOdometry.h"
#include "projects/rc_unimog_control_group2/navi/mComputeWheelDistance.h"

#include "projects/rc_unimog_control_group2/navi/mMap2D.h"
#include "projects/rc_unimog_control_group2/navi/mDrive.h"
#include "libraries/closed_loop_control/mPIDController.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace rc_unimog_control_group2
{
namespace navi
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gNavigation> cCREATE_ACTION_FOR_G_NAVIGATION("Navigation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gNavigation constructor
//----------------------------------------------------------------------
gNavigation::gNavigation(core::tFrameworkElement *parent, const std::string &name,
                         const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its sensor outputs and controller inputs)
{

	/*
	// create modules
   mChooseScenario* choose_scenario = new mChooseScenario(this, "ChooseScenario");
	mDrive* drive = new mDrive(this, "Drive");

	// connect modules

    si_list_detected_object.ConnectTo(choose_scenario->in_list_detected_objects);
	choose_scenario->out_dist_bridge.ConnectTo(drive->in_dist2_bridge);
	choose_scenario->out_dist_stop_sign.ConnectTo(drive->in_dist2_stopsign);
	choose_scenario->out_middle_mark_found.ConnectTo(drive->in_middle_mark_found);
	choose_scenario->out_side_mark_found_right.ConnectTo(drive->in_side_mark_right_found);
	choose_scenario->out_side_mark_found_left.ConnectTo(drive->in_side_mark_left_found);

	choose_scenario-> out_bridge_found.ConnectTo(drive->in_bridge_found);
	choose_scenario->out_stopsign_found.ConnectTo(drive->in_stopsign_found);
	choose_scenario->out_junction_found.ConnectTo(drive->in_junction_found);
	choose_scenario->out_no_mark_found.ConnectTo(drive->in_no_mark_found);
	*/


	auto map2D = new mMap2D(this);
	auto drive = new mDrive(this);
	auto pid_controller = new closed_loop_control::mPIDController<float,float>(this);

	map2D->middle_line_input.ConnectTo("/Main Thread/MainControl/ObjectDetection/DetectMiddleLine/Output/Lines Vector");
	map2D->outside_line_input.ConnectTo("/Main Thread/MainControl/ObjectDetection/DetectOutsideLine/Output/Lines Vector");
	drive->in.ConnectTo(this-pid_controller->co_control_value);
	pid_controller->si_measured_value.ConnectTo(map2D->distance);





	pid_controller->par_kp.Set(0.1);
	pid_controller->par_kI.Set(0.1);
	pid_controller->par_kD.Set(0.05);

	pid_controller->par_max_value.Set(22);
	pid_controller->par_min_value.Set(-22);
	pid_controller->ci_target_value.Set(0);
	pid_controller->par_cut_max_min.Set(true);

}

//----------------------------------------------------------------------
// gNavigation destructor
//----------------------------------------------------------------------
gNavigation::~gNavigation()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
