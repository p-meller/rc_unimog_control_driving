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
/*!\file    projects/rc_unimog_control_group2/navi/mOdometry.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/mOdometry.h"
//#include "projects/rc_unimog_control_group2/navi/mComputeWheelDistance.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <math.h>
#include <cmath>
#include <iostream>
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
runtime_construction::tStandardCreateModuleAction<mOdometry> cCREATE_ACTION_FOR_M_ODOMETRY("Odometry");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mOdometry constructor
//----------------------------------------------------------------------
mOdometry::mOdometry(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false),
  WheelBase(0.045),
     distance_track(0.029),
     current_dleft(0),
     current_dright(0),
     differenceDistance(0),

     radius_center(0),

     radius_right(0),
     radius_left(0),

     delta_x(0),
     delta_y(0),

     current_o_pose(),
     inner_Angle(0),
     outer_Angle(0),
     delta_outer_Angle(0),

     Position_vector(0,0,0),
     rotMat(),
     actual_x(0),
     actual_y(0),
     current_lvelocity(0),
     current_rvelocity(0),
     last_time(),
     current_time(),
     delta_t(),
     delta_time()
   //Position_initial(0,0,0)
{

    current_dleft=this->dleft.Get(); // u give the caculated distance from computeDistance Module to current_dleft so u can use it to compute the positions.
      current_dright=this->dright.Get();
      current_time=rrlib::time::Now();
      last_time=rrlib::time::Now();
      current_lvelocity=this->lVelocity.Get();
      current_rvelocity=this->rVelocity.Get();
}

//----------------------------------------------------------------------
// mOdometry destructor
//----------------------------------------------------------------------
mOdometry::~mOdometry()
{}



//----------------------------------------------------------------------
// mOdometry Update
//----------------------------------------------------------------------
void mOdometry::Update()
{
    current_dleft=this->dleft.Get(); // u give the caculated distance from computeDistance Module to current_dleft so u can use it to compute the positions.
        current_dright=this->dright.Get();

        current_lvelocity=this->lVelocity.Get();
        current_rvelocity=this->rVelocity.Get();
        differenceDistance= current_dright.Value() - current_dleft.Value();

        inner_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)((double)differenceDistance.Value()/(double)WheelBase.Get());

        current_time=rrlib::time::Now();

        delta_t= current_time -last_time;
        delta_time=(double)std::chrono::duration_cast<std::chrono::milliseconds>(delta_t).count()/1000; // casting delta_t to double




        if((double)inner_Angle.Value()>0 || (double)differenceDistance.Value()<0){ //left curve

            radius_center=(double)current_dright.Value()/(double)inner_Angle.Value(); // distance from the center of both wheel to the center of the curve
            //Position_vector[0]=(double)Position_initial[0]-(double)radius_center*((double)(outer_Angle-M_PI/2).Cosine);
            //Position_vector[1]=(double)Position_initial[1]-(double)radius_center*((double)(outer_Angle-M_PI/2).Sine);

            //radius_right=((double)radius_center.Value()+(double)distance_track.Value()/2);// distance from the center of the right wheel to the center of the curve
            //radius_left=((double)radius_center.Value()-(double)(double)distance_track.Value()/2);// distance from the center of the  left wheel to the center of the curve

            delta_x=(double)radius_center*(double)inner_Angle.Sine();//delta_x=(double)radius_left*(double)inner_Angle.Sine();
            delta_y=(double)radius_center-(double)radius_center*(double)inner_Angle.Cosine();//delta_y=(double)radius_left*(double)inner_Angle.Cosine();
            delta_outer_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)((double)current_lvelocity.Value())/((double)WheelBase.Get()*(double)inner_Angle.Sine()*(double)delta_time);
            //delta_outer_Angle is change in heading or orientation
            outer_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)(atan2((double)delta_x.Value(),(double)delta_y.Value()));// steering Angle

            outer_Angle +=delta_outer_Angle ;//% (double)(2 * M_PI); // this heading or orientation angle
            actual_x +=((double)delta_x)*(double)outer_Angle.Cosine()-(double)delta_y*outer_Angle.Sine(); // actual coordinates of the robot
            actual_y+=(double)delta_x*(double)outer_Angle.Sine()+(double)delta_y*outer_Angle.Cosine();




        }
        else if ((double)inner_Angle.Value()<0.1 || (double)differenceDistance.Value()==0){ //straight motion




            delta_x= ((double)current_dleft.Value()+(double)current_dright)/2 ;
            delta_y=0;
            outer_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)0;




        }
        else {
            // right curve


            radius_center=(double)current_dleft.Value()/(double)inner_Angle.Value();
            //radius_right=(double)radius_center.Value()-(double)(double)distance_track.Value()/2;
            //radius_left=(double)radius_center.Value()+(double)(double)distance_track.Value()/2;

            delta_x=(double)radius_center*(double)inner_Angle.Sine();//delta_x=(double)radius_right*(double)inner_Angle.Sine();
            delta_y=(double)radius_center*(double)inner_Angle.Cosine()-(double)radius_center;//delta_y=(double)radius_right*(double)inner_Angle.Cosine();

            delta_outer_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)((double)current_lvelocity.Value())/((double)WheelBase.Get()*(double)inner_Angle.Sine()*(double)delta_time);
            // delta_outer_Angle is change in heading or orientation
           // outer_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)(atan2((double)delta_x.Value(),(double)delta_y.Value()));// steering Angle

            //outer_Angle += delta_outer_Angle; // this heading or orientation angle
            outer_Angle=(rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)(atan2((double)delta_x.Value(),(double)delta_y.Value()));
            outer_Angle += delta_outer_Angle; // this heading or orientation angle
            actual_x +=(double)delta_x*(double)outer_Angle.Cosine()-(double)delta_y*outer_Angle.Sine(); // actual coordinates of the robot
            actual_y+=(double)delta_x*(double)outer_Angle.Sine()+(double)delta_y*outer_Angle.Cosine();
            //  Position_vector.Set((double)actual_x,(double)actual_y,0);




        }


        rotMat=Rot3DZ((rrlib::math::tAngle<double, rrlib::math::angle::Radian, rrlib::math::angle::Signed>)current_o_pose.Yaw().Value());

        Position_vector.Set((double)actual_x,(double)actual_y,0);

        Position_vector=rotMat*Position_vector;

        current_o_pose.X()+=(double)Position_vector[0];
        current_o_pose.Y()+=(double)Position_vector[1];
        current_o_pose.Z()+=(double)Position_vector[2];

        current_o_pose.Yaw()= (double)delta_outer_Angle+((double)outer_Angle.Value()*(M_PI/180));
        current_o_pose.Roll()= 0;
        current_o_pose.Pitch()= 0;

        this->O_Pose.Publish(current_o_pose); // publishing to the output port O_pose current_o_pose





    }

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
