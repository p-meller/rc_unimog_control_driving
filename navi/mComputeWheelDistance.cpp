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
/*!\file    projects/rc_unimog_control_group2/navi/mComputeWheelDistance.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-25
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/mComputeWheelDistance.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tTime.h"
#include "plugins/scheduling/tThreadContainerThread.h"
#include "rrlib/si_units/si_units.h"
#include "rrlib/time/time.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include <cassert>
#include <stdlib.h>
#include<tgmath.h>
#include <math.h>
//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::si_units;
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
runtime_construction::tStandardCreateModuleAction<mComputeWheelDistance> cCREATE_ACTION_FOR_M_COMPUTEWHEELDISTANCE("ComputeWheelDistance");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mComputeWheelDistance constructor
//----------------------------------------------------------------------
mComputeWheelDistance::mComputeWheelDistance(core::tFrameworkElement *parent,
                                             const std::string &name) :
    tModule(parent, name, false),


    // Note that the order in initialisation should match the order in deklaration lest u get errors
    wheelDiameter(0.078), distance_left(0), wheelRadius(0), distance_right(
                                                                0), distance_center(0), distance_track(0.175), radius_center(0), // distance from the centre of the curve to the middle point of robot
    radius_leftWheel(0), radius_rightWheel(0),
    // distance from center to the left wheel

    // Buffer_x(),
    //Buffer_y(7),
    i(0),

    delta_t(), delta_time(), current_Right_Velocity(0), current_Left_Velocity(
                                                            0)

{
    current_Right_AngularVelocity = this->in_Right_AngularVelocity.Get();
    current_Left_AngularVelocity = this->in_Left_AngularVelocity.Get();

    current_time = rrlib::time::Now();
    last_time = rrlib::time::Now();
}

//----------------------------------------------------------------------
// mComputeWheelDistance destructor
//----------------------------------------------------------------------
mComputeWheelDistance::~mComputeWheelDistance() {
}

//----------------------------------------------------------------------
// mComputeWheelDistance OnParameterChange
//----------------------------------------------------------------------
void mComputeWheelDistance::OnParameterChange() {
    wheelRadius = (wheelDiameter.Get()) / 2; // access the given wheeldiameter port
}

//----------------------------------------------------------------------
// mComputeWheelDistance Update
//----------------------------------------------------------------------
void mComputeWheelDistance::Update()

{

    current_time = rrlib::time::Now();

    delta_t = current_time - last_time;
    delta_time = (double) std::chrono::duration_cast < std::chrono::milliseconds
            > (delta_t).count() / 1000; // casting delta_t to double

    std::cout << "delta_time" << "\t" << delta_time << "\t" << "\t"
              << "delta_time" << delta_time << std::endl;
    //distance_left = (double )distance_left - Buffer_x[i];
    //distance_right = (double )distance_right -Buffer_y[i];
    wheelRadius = (wheelDiameter.Get()) / 2;// access the given wheeldiameter port

    current_Right_AngularVelocity = this->in_Right_AngularVelocity.Get();
    current_Left_AngularVelocity = this->in_Left_AngularVelocity.Get();

    //Buffer_x[0] = (double) current_Left_AngularVelocity.Value()
    //(//double) delta_time * (double) wheelRadius.Value();//Buffer_x[0]=(double)current_Left_AngularVelocity.Value()*(double)delta_t*((double) wheelDiameter.Get()/2); //Buffer_x[i]=(double)current_Left_AngularVelocity.Value()*(double)delta_t*(double)WheelRadius.Value(); // distance covered by left wheel
    distance_left = fabs((double) distance_left )+( (double) current_Left_AngularVelocity.Value()* (double) delta_time* (double) wheelRadius.Value()/24);// distance_left =(double)distance_left + Buffer_x[i];// add all distance covered by left wheel

    //Buffer_x[1] = (double) current_Right_AngularVelocity.Value()
    //	*(double) delta_time * (double) wheelRadius.Value();//Buffer_x[1]=(double)current_Right_AngularVelocity.Value()*(double)delta_t*((double) wheelDiameter.Get()/2); //Buffer_y[i]=(double)current_Right_AngularVelocity.Value()*(double)delta_t*(double)WheelRadius.Value(); // distance covered by left wheel
    distance_right = fabs((double )distance_right) + ((double) current_Right_AngularVelocity.Value()
            * (double) delta_time * (double) wheelRadius.Value())/24;	//distance_right = (double)distance_right+ Buffer_y[i];// add all distance covered by right wheel

    radius_center = (double) (distance_left
                              / (distance_right - distance_left))
            * (distance_track.Value() / 2);	// distance from middle point of the robot to the center of the curve

    radius_leftWheel = (double) radius_center
            - (double) (distance_track.Value() / 2);// distance from left wheel to the center
    radius_rightWheel = (double) radius_center
            + (double) (distance_track.Value() / 2);// distance from left wheel to the center

    last_time = current_time;	// time update

    //std::cout << "right" << "\t" << Buffer_x[0]<< "\t" << "\t" << "wheelRadius" << wheelRadius << std::endl;
    FINROC_LOG_PRINTF(USER, "current_Right_AngularVelocity: \t %f \n", current_Right_AngularVelocity.Value().Value()); //static_cast<double>(current_Right_AngularVelocity));
    FINROC_LOG_PRINTF(USER, "current_Left_AngularVelocity: \t %f", current_Left_AngularVelocity.Value().Value());

    std::cout << "distance_left" << "\t" << distance_left << "\t"
              << "\t" << "distance_right" << distance_right << std::endl;
    this->O_distance_left.Publish(distance_left, rrlib::time::Now()); // publish the current value
    this->O_distance_right.Publish(distance_right, rrlib::time::Now());

    current_Left_Velocity =
            (double) current_Left_AngularVelocity.Value()
            * (double) wheelRadius.Value(); // caculate the velocity of the left wheel frm angular velocity
    current_Right_Velocity =
            (double) current_Right_AngularVelocity.Value()
            * (double) wheelRadius.Value();

    this->O_velocity_left.Publish(current_Left_Velocity,current_time);
    this->O_velocity_right.Publish(current_Right_Velocity, last_time);


}

//this -> O_distance_left.Publish(distance_left,current_time); // publish the current value
//this -> O_distance_right.Publish(distance_right,last_time);

}

}
}

