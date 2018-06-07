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
/*!\file    projects/rc_unimog_control_group2/navi/mDrive.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/mDrive.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <math.h>
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>
#include <time.h>
#include <ctime>
#include <cstdlib>
//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc {
namespace rc_unimog_control_group2 {
namespace navi {

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<mDrive> cCREATE_ACTION_FOR_M_DRIVE(
		"Drive");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mDrive constructor
//----------------------------------------------------------------------
mDrive::mDrive(core::tFrameworkElement *parent, const std::string &name) :
		tSenseControlModule(parent, name, false),

		cal_curvature(0), cal_velocity(0), max_velocity(0.2), dist_stop_sign(0), dist_bridge(
				0), current_middle_mark_found(false), current_side_mark_right_found(
				false), current_side_mark_left_found(false), current_bridge_found(
				false), current_stopsign_found(false), current_junction_found(
				false), current_no_mark_found(false), stop_sign_has_stopped(
				false)

{
//	dist_threshold(10);
//	current_middle_mark_found = in_middle_mark_found.Get();
//	current_side_mark_right_found = in_side_mark_right_found.Get();
//	current_side_mark_left_found =	in_side_mark_left_found.Get();
//	current_bridge_found = in_bridge_found.Get();
//	current_stopsign_found = in_stopsign_found.Get();
//	current_junction_found = in_junction_found.Get();
//	current_no_mark_found =	in_no_mark_found.Get();
//	stop_sign_has_stopped = false;
}

//----------------------------------------------------------------------
// mDrive destructor
//----------------------------------------------------------------------
mDrive::~mDrive() {
}

//----------------------------------------------------------------------
// mDrive OnStaticParameterChange
//----------------------------------------------------------------------
void mDrive::OnStaticParameterChange() {

}

//----------------------------------------------------------------------
// mDrive OnParameterChange
//----------------------------------------------------------------------
void mDrive::OnParameterChange() {
	//If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
}

//----------------------------------------------------------------------
// mDrive Sense
//----------------------------------------------------------------------
void mDrive::Sense() {
	if (this->SensorInputChanged()) {
		in_middle_mark_found.Get(current_middle_mark_found);
		in_side_mark_right_found.Get(current_side_mark_right_found);
		in_side_mark_left_found.Get(current_side_mark_left_found);
		in_bridge_found.Get(current_bridge_found);
		in_stopsign_found.Get(current_stopsign_found);
		in_junction_found.Get(current_junction_found);
		in_no_mark_found.Get(current_no_mark_found);
		in_dist2_bridge.Get(dist_bridge); // import dist to bridge if brigde is detected
		in_dist2_stopsign.Get(dist_stop_sign); // import dits to stop sign if detected

	}

	if (this->in_stopsign_found.HasChanged()) {
		stop_sign_has_stopped = false;
	}

	// publish the computed angle to output port
	// here implement the command to follow the middle markings

	// Do something each cycle independent from changing ports.

	//this->so_signal_1.Publish(some meaningful value); can be used to publish data via your sensor output ports.
}

//----------------------------------------------------------------------
// mDrive Control
//----------------------------------------------------------------------

// for timing
/*void pause(int dur) {
	int temp = time(NULL) + dur;

	while (temp > time(NULL))
		;
}

*/
void mDrive::Control() {
	drive();

}

rrlib::si_units::tCurvature<double> mDrive::Calculate_curvature() {

	if (in_Angle2_Mark.Get().Value() > 0) {
		// for positive winkel
		// right curve with positive curvature

		if ((in_Angle2_Mark.Get().Value() > 10)
				&& (in_Angle2_Mark.Get().Value() <= 20)) {	// light steering
			cal_curvature = 6;// straight

		}

		if ((in_Angle2_Mark.Get().Value() > 20)
				&& (in_Angle2_Mark.Get().Value() <= 30)) {  // moderate steering

			cal_curvature = 12; //

		}
		if ((in_Angle2_Mark.Get().Value() > 30)
				&& (in_Angle2_Mark.Get().Value() <= 70)) { // full steering

			cal_curvature = 22;

		}

	}

	if (in_Angle2_Mark.Get().Value() < 0) {

		// for negative angles
		// left curve with negative curvature

		if ((in_Angle2_Mark.Get().Value() > -10)
				&& (in_Angle2_Mark.Get().Value() <= -20)) { // light steering

			cal_curvature = -5;

		}

		if ((in_Angle2_Mark.Get().Value() > -20)
				&& (in_Angle2_Mark.Get().Value() <= -30)) { // moderate steering

			cal_curvature = -10;

		}
		if ((in_Angle2_Mark.Get().Value() > -30)
				&& (in_Angle2_Mark.Get().Value() <= -70)) { // full steering

			cal_curvature = -21;

		}

	}

	return cal_curvature;
}

rrlib::si_units::tVelocity<double> mDrive::Calculate_velocity() {

	// light steering max velocity

	if ((double) cal_curvature > 0.0 && (double) cal_curvature <= 7.0) {

		cal_velocity = max_velocity;

	}
	// moderate steering max/2

	if ((double) cal_curvature > 8.0 && (double) cal_curvature <= 14.0) {

		cal_velocity = max_velocity / 2;

	}
	//full steering min velocity

	if ((double) cal_curvature > 15.0 && (double) cal_curvature <= 21.0) {

		cal_velocity = max_velocity * 0.7;

	}

	if (current_stopsign_found == true) {
		if ((dist_stop_sign.Value()) < (dist_threshold.Get().Value())
				&& !stop_sign_has_stopped) {
			out_velocity.Publish(0);
			//pause(2); // pause for seconds at the stop sign
			out_velocity.Publish(max_velocity);
			stop_sign_has_stopped = true;
		}

	}

	if (current_bridge_found == true) {
		if (dist_bridge.Value() < dist_threshold.Get().Value()) {
			out_velocity.Publish(max_velocity); // untill the edge

		}

	}
	return cal_velocity;
}

// think of adding function that calculates the angle to the middle marking
// depending on this angle u may turn left or stay whre you are and follow the line

void mDrive::drive() {
// control commands

	out_velocity.Publish(Calculate_velocity());

	out_steering.Publish(Calculate_curvature());

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
