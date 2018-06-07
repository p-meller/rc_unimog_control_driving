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
/*!\file    projects/rc_unimog_control_group2/navi/mSpeedComputer.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/mSpeedComputer.h"
#include "rrlib/math/tVector.h"
#include <math.h>
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
runtime_construction::tStandardCreateModuleAction<mSpeedComputer> cCREATE_ACTION_FOR_M_SPEEDCOMPUTER("SpeedComputer");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mSpeedComputer constructor
//----------------------------------------------------------------------
mSpeedComputer::mSpeedComputer(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), par_curvature() , vmax(0.5), K(1),ke(1),kth(0.6), err(0),err_sum(0),err_past(0),y(0),kp(),Ta(),kd(), adj_velocity(), adj_angularVelocity(),
  desired_velocity(),desired_angularVelocity()
{
	//current_Angle2Middle_Mark= this->Angle2Middle_Mark.Get();
	//par_curvature=this->in_curvature.Get();
	par_delta_Theta=this->delta_Theta.Get();
	par_delta_error=this->delta_error.Get();


	//ObjectDetectedPose=this->in_ObjectDetectedPose.Get();
}

//----------------------------------------------------------------------
// mSpeedComputer destructor
//----------------------------------------------------------------------
mSpeedComputer::~mSpeedComputer()
{}

//----------------------------------------------------------------------
// mSpeedComputer OnStaticParameterChange
//----------------------------------------------------------------------
void mSpeedComputer::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mSpeedComputer OnParameterChange
//----------------------------------------------------------------------
void mSpeedComputer::OnParameterChange()
{

}

//----------------------------------------------------------------------
// mSpeedComputer Update
//----------------------------------------------------------------------
void mSpeedComputer::Update()
{

// here velocity and the angular velocity are adjusted
adj_velocity= (double)vmax*exp(-(double)K/(double)par_curvature);
adj_angularVelocity= (double) adj_velocity*(double)par_curvature;

PID_Controller(par_delta_error.Value());

// desired velocity and desired angular velocity
desired_velocity=(double)adj_velocity*cos(par_delta_Theta.Value())+(double)ke*(double)par_delta_error;
desired_angularVelocity=(double)adj_angularVelocity +(double)adj_velocity*((double)kth*(double)par_delta_error+sin(par_delta_Theta.Value()));
// ke and kth are feedback gain and positive constants

this->O_fwd_velocity.Publish(desired_velocity);

// the curvature aint be calculated
// think of lookup table



// the the calculated  desired velocity and desired_angularVelocity are fed int a PID controller
/*The proportional control stabilizes the gain but
produces a steady state error. The integral control
reduces or eliminates the steady state error. The
derivative control reduces the rate of change of error.
For example, in steering control, if the reference or set
direction is 180 degrees and if the robot is moving off
from the set direction, say, to 182 degrees, then the
controller dynamically corrects the wheel speeds in such
a manner that the robot moves closer to 180 degrees.
*/



  
  
}

double mSpeedComputer::PID_Controller(double err){
	//static double ke, kth;
	static double err_sum, err_past,y, kp, Ta, kd, ki;

	err_sum += err;
	y=kp*err+ki*Ta*err_sum +kd*(err-err_past)/Ta;
	err_past=err;
	return y;

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
