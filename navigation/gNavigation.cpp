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
/*!\file    projects/rc_unimog_control_group2/navigation/gNavigation.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-12
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navigation/gNavigation.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "projects/rc_unimog_control_group2/navigation/mMap2D.h"
#include "projects/rc_unimog_control_group2/navigation/mDrive.h"
#include "projects/rc_unimog_control_group2/navigation/mScenarioChooser.h"
#include "libraries/closed_loop_control/mPIDController.h"


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
namespace navigation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gNavigation> cCREATE_ACTION_FOR_G_NAVIGATION(
		"Navigation");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gNavigation constructor
//----------------------------------------------------------------------
gNavigation::gNavigation(core::tFrameworkElement *parent,
		const std::string &name, const std::string &structure_config_file) :
		tGroup(parent, name, structure_config_file, false) // change to 'true' to make group's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
	auto map2D = new mMap2D(this);
	auto drive = new mDrive(this);
	auto pid_controller = new closed_loop_control::mPIDController<float, float>(
			this, "PID_Controller");
	auto scenario_chooser = new mScenarioChooser(this);
	//auto pid_controller_velocity= new closed_loop_control::mPIDController<rrlib::si_units::tVelocity<>,rrlib::si_units::tVelocity<>>(this , "velocity pid");




	scenario_chooser->distance_to_left_lane.ConnectTo(map2D->distance_to_left_lane);
	scenario_chooser->distance_to_right_lane.ConnectTo(map2D->distance_to_right_lane);
	scenario_chooser->distance_error.ConnectTo(pid_controller->si_measured_value);
	scenario_chooser->velocity.ConnectTo("/Main Thread/RemoteInterface/Controller Input/Forward Velocity");
	//scenario_chooser->velocity.ConnectTo(pid_controller_velocity->si_measured_value);
	//pid_controller_velocity->si_measured_value.ConnectTo("/Main Thread/MainControl/Sensor Output/Wheel Velocity Rear Right");
	scenario_chooser->stop_detected.ConnectTo("/Main Thread/MainControl/ObjectDetection/StopSignDetection/Output/Stop Detected");
	map2D->middle_line_input.ConnectTo(
			"/Main Thread/MainControl/ObjectDetection/DetectMiddleLine/Output/Lines Vector");
	map2D->outside_line_input.ConnectTo(
			"/Main Thread/MainControl/ObjectDetection/DetectOutsideLine/Output/Lines Vector");
	drive->in.ConnectTo(pid_controller->co_control_value);
	//pid_controller->si_measured_value.ConnectTo(map2D->distance_to_right_lane);

	drive->curvature.ConnectTo("/Main Thread/MainControl/Controller Output/Curvature");
	//pid_controller_velocity->co_control_value.ConnectTo("/Main Thread/RemoteInterface/Controller Input/Forward Velocity");

	pid_controller->par_kp.Set(0.349);
	pid_controller->par_kI.Set(0.047);
	pid_controller->par_kD.Set(0.084);

	pid_controller->par_max_value.Set(22);
	pid_controller->par_min_value.Set(-22);
	pid_controller->ci_target_value.Set(0);
	pid_controller->par_cut_max_min.Set(true);

//	pid_controller_velocity->par_max_value.Set(rrlib::si_units::tVelocity<>(0.2));
//	pid_controller_velocity->par_min_value.Set(rrlib::si_units::tVelocity<>(-0.1));
//	pid_controller_velocity->par_cut_max_min.Set(true);

}

//----------------------------------------------------------------------
// gNavigation destructor
//----------------------------------------------------------------------
gNavigation::~gNavigation()
{
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
