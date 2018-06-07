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
/*!\file    projects/rc_unimog_control_group2/navi/mComputeWheelDistance.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-25
 *
 * \brief Contains mComputeWheelDistance
 *
 * \b mComputeWheelDistance
 *
 * this module computes the wheel distance from Angular velocity
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mComputeWheelDistance_h__
#define __projects__rc_unimog_control_group2__navi__mComputeWheelDistance_h__

#include "plugins/structure/tModule.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/time/time.h"
#include "libraries/sikola/gGenericHardwareInterface.h"
#include "rrlib/time/time.h"


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include<tgmath.h>
#include <math.h>
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
 * this module computes the wheel distance from Angular velocity
 */
class mComputeWheelDistance : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:


	 // sensor Input
	 tParameter<rrlib::si_units::tLength<>>wheelDiameter;
	  tInput<rrlib::si_units::tAngularVelocity<>> in_Left_AngularVelocity;
	  tInput<rrlib::si_units::tAngularVelocity<>> in_Right_AngularVelocity;



	  	//sensor output
	    tOutput<rrlib::si_units::tLength<>> O_distance_left; // ibyo na calculatinze muri mComputeWheel.cpp will be connected here in the output ports
	    tOutput<rrlib::si_units::tLength<>> O_distance_right;
	    tOutput<rrlib::si_units::tVelocity<>>O_velocity_left;
	    tOutput<rrlib::si_units::tVelocity<>>O_velocity_right;



//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mComputeWheelDistance(core::tFrameworkElement *parent, const std::string &name = "ComputeWheelDistance");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */

  ~mComputeWheelDistance();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:







  rrlib::si_units::tAngularVelocity<>current_Right_AngularVelocity;
  rrlib::si_units::tAngularVelocity<>current_Left_AngularVelocity;
  rrlib::si_units::tLength<>distance_left;// distance covered by the left wheel
 rrlib::si_units::tLength<> wheelRadius;
  rrlib::si_units::tLength<>distance_right;// distance of right wheel
  rrlib::si_units::tLength<>distance_center;// distance of the robot
  rrlib::si_units::tLength<>distance_track;// distance between left and right wheel
  rrlib::si_units::tLength<>radius_center;// distance from the centre of the curve to the middle point of robot
  rrlib::si_units::tLength<>radius_leftWheel;// distance from center to the left wheel
  rrlib::si_units::tLength<>radius_rightWheel;// distance of the robot


  //std::vector<2,double> Buffer_x; 
  rrlib::math::tVector<2,double> Buffer_x;
  //std::vector<2,double> Buffer_y;

    //iteration variable
    int i;


    // Movement durations

    rrlib::time::tTimestamp last_time;
    rrlib::time::tTimestamp current_time;
    rrlib::time::tDuration delta_t;
    double delta_time;

    rrlib::si_units::tVelocity<double>current_Right_Velocity;
    rrlib::si_units::tVelocity<double>current_Left_Velocity;
    
   
















  //Here is the right place for your variables. Replace this line by your declarations!

  //virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
