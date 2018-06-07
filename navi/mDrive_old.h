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
/*!\file    projects/rc_unimog_control_group2/navi/mDrive.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 * \brief Contains mDrive
 *
 * \b mDrive
 *
 * This module receivexs a list of detected objects and computesangle to middle marking
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mDrive_h__
#define __projects__rc_unimog_control_group2__navi__mDrive_h__

#include "plugins/structure/tSenseControlModule.h"
#include "rrlib/math/tVector.h"
#include <math.h>

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/math/tAngle.h"
//#include "libraries/tDetectedObject.h"

//----------------------------------------------------------------------
// Internal includes with ""
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module receivexs a list of detected objects and computesangle to middle marking
 */
class mDrive: public structure::tSenseControlModule {

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:
// input ports

	tSensorInput<rrlib::math::tAngle<double>> in_Angle2_Mark;
	tSensorInput<rrlib::si_units::tLength<double>> in_dist2_bridge;
	tSensorInput<rrlib::si_units::tLength<double>> in_dist2_stopsign;
	tSensorInput<bool> in_middle_mark_found;
	tSensorInput<bool> in_side_mark_right_found;
	tSensorInput<bool> in_side_mark_left_found;
	tSensorInput<bool> in_bridge_found;
	tSensorInput<bool> in_stopsign_found;
	tSensorInput<bool> in_junction_found;
	tSensorInput<bool> in_no_mark_found;
	tControllerOutput<rrlib::si_units::tVelocity<double>> out_velocity;
	tControllerOutput<rrlib::si_units::tCurvature<double>> out_steering;
	tParameter<rrlib::si_units::tLength<double>> dist_threshold;
	//tParameter<bool> par_started;	// Parameter to start drive eight
	//tParameter<double> par_current_angle;// for testing, for switching direction
	//tParameter<rrlib::si_units::tVelocity<double>> par_velocity;
	//tParameter<rrlib::si_units::tCurvature<double>> par_curvature;// Radius of curve
	//tParameter<rrlib::si_units::tLength<double>> par_straight_length;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	mDrive(core::tFrameworkElement *parent, const std::string &name = "Drive");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of modules is declared protected to avoid accidental deletion. Deleting
	 * modules is already handled by the framework.
	 */
	~mDrive();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	// pose
	//rrlib::math::tAngle<double> current_in_Angle2Mark;

	//bool current_boo;
	rrlib::si_units::tCurvature<double> cal_curvature;
	rrlib::si_units::tVelocity<double> cal_velocity;
	rrlib::si_units::tVelocity<double> max_velocity;
	rrlib::si_units::tLength<double> dist_stop_sign;
	rrlib::si_units::tLength<double> dist_bridge;

	bool current_middle_mark_found;
	bool current_side_mark_right_found;
	bool current_side_mark_left_found;
	bool current_bridge_found;
	bool current_stopsign_found;
	bool current_junction_found;
	bool current_no_mark_found;

	bool stop_sign_has_stopped;	// temp variable if robot has stopped in front of stop sign

	//rrlib::localization::tPose3D<> start_pose;
	//rrlib::localization::tPose3D<> current_pose;
	//rrlib::localization::tPose3D<> ObjectDetectedPose;
	//rrlib::localization::tPose3D<> last_pose;

	//double current_angle;   // current turn angle in deg
	//double Angle2_Mark;

	//rrlib::si_units::tLength<double> driven_straight; //straight track which is already driven when robot drives an eight
	//double angle_to_drive;

	virtual void OnStaticParameterChange() override;

	virtual void OnParameterChange() override;

	virtual void Sense() override;

	virtual void Control() override;

	//double ComputeAngle2middleMarking();
	rrlib::si_units::tCurvature<double> Calculate_curvature();
	rrlib::si_units::tVelocity<double> Calculate_velocity();

	void drive();
	void pause(int dur);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif

/// testen vom git, mal sehen was passiert!!!!!!!!!!!!!!!!!!!!!!!!!!
