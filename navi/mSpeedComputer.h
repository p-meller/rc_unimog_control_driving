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
/*!\file    projects/rc_unimog_control_group2/navi/mSpeedComputer.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 * \brief Contains mSpeedComputer
 *
 * \b mSpeedComputer
 *
 * This module computes the velocity of the robot  wrt detected objects ie middle marking, bridge or junction
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mSpeedComputer_h__
#define __projects__rc_unimog_control_group2__navi__mSpeedComputer_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/math/tAngle.h"
//----------------------------------------------------------------------
// Internal includes with ""
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
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This module computes the velocity of the robot  wrt detected objects ie middle marking, bridge or junction
 */
class mSpeedComputer : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

    //input port
	//tInput<std::vector::vec3d<>> points_detected;
	//tInput<rrlib::math::tAngle<double>> Angle2Middle_Mark; // this port imports angle to middle line from Drive Module
	//tInput<rrlib::localization::tPose3D<>> in_position;
	//tInput<rrlib::si_units::tCurvature<>> in_curvature; // input port for the curvature
	tInput<rrlib::math::tAngle<double>> delta_Theta; // angle from middle point of the robot(camera) to the tangent line of the detected point
	tInput<rrlib::si_units::tLength<double>>delta_error; // difference between set point and the measured position(actual position)





		    // here you shud put a data port that loads the list of detected objects
   //tParameter<rrlib::localization::tPose3D<>> in_ObjectDetectedPose;
	//output ports
	// output ports
	 tOutput<rrlib::si_units::tVelocity<>>O_fwd_velocity;
	 //tOutput<rrlib::si_units::tCurvature<>>O_Curvature;






//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mSpeedComputer(core::tFrameworkElement *parent, const std::string &name = "SpeedComputer");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mSpeedComputer();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


  rrlib::si_units::tCurvature<> par_curvature;
  rrlib::math::tAngle<double>par_delta_Theta;
  rrlib::si_units::tLength<double>par_delta_error;
  double vmax;
  double K, ke, kth;
  double err;
  double err_sum, err_past,y, kp, Ta, kd;
  rrlib::si_units::tVelocity<double>adj_velocity;
  rrlib::si_units::tAngularVelocity<double>adj_angularVelocity;
  rrlib::si_units::tVelocity<double>desired_velocity;
  rrlib::si_units::tAngularVelocity<double>desired_angularVelocity;






  virtual void OnStaticParameterChange() override;
  virtual void OnParameterChange() override;

  virtual void Update() override;
  double PID_Controller(double err);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
