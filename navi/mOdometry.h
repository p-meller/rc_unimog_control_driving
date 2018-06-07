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
/*!\file    projects/rc_unimog_control_group2/navi/mOdometry.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 * \brief Contains mOdometry
 *
 * \b mOdometry
 *
 * This module computes the pose of the robot
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mOdometry_h__
#define __projects__rc_unimog_control_group2__navi__mOdometry_h__

#include "plugins/structure/tModule.h"
#include "rrlib/localization/tPose.h"
#include "rrlib/si_units/si_units.h"

#include "rrlib/math/tAngle.h"
#include "rrlib/math/tVector.h"
#include <math.h>

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
using namespace rrlib::si_units;
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
 * This module computes the pose of the robot
 */
class mOdometry : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

                tInput<rrlib::si_units::tLength<>> dleft;
		tInput<rrlib::si_units::tLength<>> dright;
		tInput<rrlib::si_units::tVelocity<>> lVelocity;
		tInput<rrlib::si_units::tVelocity<>> rVelocity;
		tParameter<rrlib::si_units::tLength<>> WheelBase;

		tOutput<rrlib::localization::tPose3D<>> O_Pose;


//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mOdometry(core::tFrameworkElement *parent, const std::string &name = "Odometry");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mOdometry();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:



  rrlib::si_units::tLength<>distance_track;
       rrlib::si_units::tLength<>current_dleft;
       rrlib::si_units::tLength<>current_dright;
       rrlib::si_units::tLength<> differenceDistance;
       rrlib::si_units::tLength<>radius_center;
       rrlib::si_units::tLength<> radius_right;
       rrlib::si_units::tLength<> radius_left;
       rrlib::si_units::tLength<> delta_x,delta_y ;

//current_Dtrack,
       rrlib::localization::tPose3D<> current_o_pose;
       rrlib::math::tAngle<double> inner_Angle,outer_Angle,delta_outer_Angle;

        rrlib::math::tVector<3,float> Position_vector,Position_initial; //to store positions
        rrlib::math::tMat3x3f rotMat;
      double actual_x, actual_y;
       rrlib::si_units::tVelocity<double>current_lvelocity;
        rrlib::si_units::tVelocity<double>current_rvelocity;

        rrlib::time::tTimestamp last_time;
        rrlib::time::tTimestamp current_time;
         rrlib::time::tDuration delta_t;
         double delta_time;


virtual void Update() override;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
