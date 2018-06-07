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
/*!\file    projects/rc_unimog_control_group2/object_detection/mBridgeDetection.h
 *
 * \author  Paul Dornhof
 *
 * \date    2018-02-07
 *
 * \brief Contains mBridgeDetection
 *
 * \b mBridgeDetection
 *
 * this module detects a bridge and sends its as a detected_object further
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__object_detection__mBridgeDetection_h__
#define __projects__rc_unimog_control_group2__object_detection__mBridgeDetection_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/distance_data/tDistanceData.h"
#include "projects/rc_unimog_control_group2/libraries/tDetectedObject.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/math/tAngle.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc {
namespace rc_unimog_control_group2 {
namespace object_detection {

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this module detects a bridge and sends its as a detected_object further
 */
class mBridgeDetection: public structure::tModule {

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<rrlib::distance_data::tDistanceData> in_point_cloud;
	tOutput<rrlib::distance_data::tDistanceData> output_point_cloud;

	tOutput<std::vector<libraries::tDetectedObject>> out_detected_bridge;

//	tParameter<rrlib::si_units::tLength<double>> par_min_height;// min height for detection
//	tParameter<rrlib::si_units::tLength<double>> par_max_height;// max height for detection

	tParameter<rrlib::si_units::tLength<double>> par_left_threshold;
	tParameter<rrlib::si_units::tLength<double>> par_right_threshold;

	tParameter<rrlib::si_units::tLength<double>> par_max_detection_distance;
	tParameter<rrlib::si_units::tLength<double>> par_mount_height;
	tParameter<rrlib::math::tAngleDeg> par_slope;
//	tParameter<rrlib::math::tAngleDeg> par_slope_deviation;

	tParameter<int> par_counter;

	tParameter<bool> par_debug;	// parameter, if enable, then output in terminal
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	mBridgeDetection(core::tFrameworkElement *parent, const std::string &name =
			"BridgeDetection");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of modules is declared protected to avoid accidental deletion. Deleting
	 * modules is already handled by the framework.
	 */
	~mBridgeDetection();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!
//	rrlib::si_units::tLength<double> min_height;
//	rrlib::si_units::tLength<double> max_height;

//	rrlib::si_units::tLength<double> left_threshold;
//	rrlib::si_units::tLength<double> right_threshold;
//
//	rrlib::si_units::tLength<double> max_distance_detection;
//	rrlib::si_units::tLength<double> mount_height;
//	rrlib::math::tAngleDeg slope;
//	rrlib::math::tAngleDeg slope_deviation;

//	int counter;
//
//	bool debug;

//  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

//	virtual void OnParameterChange() override; //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

	virtual void Update() override;

//  void calculate_slope(); // calculates slope between points
	void calculate_middle();	// calculate the middle of the bridge

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
