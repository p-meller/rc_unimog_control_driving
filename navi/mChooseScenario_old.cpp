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
/*!\file    projects/rc_unimog_control_group2/navi/mChooseScenario.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-30
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/mChooseScenario.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>
#include <math.h>

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
runtime_construction::tStandardCreateModuleAction<mChooseScenario> cCREATE_ACTION_FOR_M_CHOOSESCENARIO(
		"ChooseScenario");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mChooseScenario constructor
//----------------------------------------------------------------------
mChooseScenario::mChooseScenario(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false),
		middle_mark_found(false),
		side_mark_found_right(false),
		side_mark_found_left(false),
		bridge_found(false),
		stopsign_found(false),
		junction_found(false),
		i(0),
		no_mark_found(false),
		temp_list(),
		temp_list_current(),
		temp_width(),
		temp_length(),
		temp_mid(),
		temp_wid(),
		temp_leng(),
		angle2Mark(),
		dist_bridge(),
		dist_stop_sign()

{
//	temp_list = in_list_detected_objects.Get();
	RRLIB_LOG_PRINT(USER, "constructor");
}

//----------------------------------------------------------------------
// mChooseScenario destructor
//----------------------------------------------------------------------
mChooseScenario::~mChooseScenario() {
}

//----------------------------------------------------------------------
// mChooseScenario OnStaticParameterChange
//----------------------------------------------------------------------
void mChooseScenario::OnStaticParameterChange() {

}

//----------------------------------------------------------------------
// mChooseScenario OnParameterChange
//----------------------------------------------------------------------
void mChooseScenario::OnParameterChange() {

}

void mChooseScenario::CheckList() {
//	temp_list = in_list_detected_objects.Get();
	RRLIB_LOG_PRINT(USER, "temp_list");
}

//----------------------------------------------------------------------
// mChooseScenario Update
//----------------------------------------------------------------------
void mChooseScenario::Update() {
	//CheckList();
	std::cout << "Line 1" << std::endl;
	in_list_detected_objects.Get(temp_list); // this works same temp_list=in_list_detected_objects.Get()
//	RRLIB_LOG_PRINT(USER, "in_list_detected_objects-Get()");

  if (temp_list.empty()) {
	  RRLIB_LOG_PRINT(USER, "temp_list is empty");
  }
//	std::cout << << std::endl;
	for (unsigned int i=0; i<temp_list.size(); i++) {
		//temp_list.size()
		std::cout << "Line 3" << std::endl;
		temp_list_current[i] = temp_list[i];
		temp_mid = temp_list_current[i].position_middle;
		temp_wid = temp_list_current[i].width; // for the obtacle detection
		temp_leng = temp_list_current[i].length; // for the obtacle detection
		std::cout << "Line 3" << std::endl;
	}
	if (middle_mark_found==true) {
		// calculate angle to middlemark
		// publish it
		angle2Mark = (rrlib::math::tAngle<double, rrlib::math::angle::Radian,
				rrlib::math::angle::Signed>) atan(
				(double) temp_mid.Y() / (double) temp_mid.X());

		Angle2Middle_Mark.Publish(angle2Mark);
		out_middle_mark_found.Publish(middle_mark_found);
		out_side_mark_found_right.Publish(false);
		out_side_mark_found_left.Publish(false);
		out_bridge_found.Publish(false);
		out_stopsign_found.Publish(false);
		out_no_mark_found.Publish(false);
		std::cout << "Line 4" << std::endl;

	}
	if (side_mark_found_right==true) {
		// calculate angle to side_mark_found_right
		// publish it
		angle2Mark = (rrlib::math::tAngle<double, rrlib::math::angle::Radian,
				rrlib::math::angle::Signed>) atan(
				(double) temp_mid.Y() / (double) temp_mid.X());
		Angle2Middle_Mark.Publish(angle2Mark);
		out_side_mark_found_right.Publish(side_mark_found_right);
		out_middle_mark_found.Publish(false);

		out_side_mark_found_left.Publish(false);
		out_bridge_found.Publish(false);
		out_stopsign_found.Publish(false);
		out_no_mark_found.Publish(false);
		std::cout << "Line 5" << std::endl;
	}

	if (side_mark_found_left==true) {
		// calculate angle to side_mark_found_right
		// publish it
		angle2Mark = (rrlib::math::tAngle<double, rrlib::math::angle::Radian,
				rrlib::math::angle::Signed>) atan(
				(double) temp_mid.Y() / (double) temp_mid.X());
		Angle2Middle_Mark.Publish(angle2Mark);
		out_side_mark_found_left.Publish(side_mark_found_left);
		out_middle_mark_found.Publish(false);
		out_side_mark_found_right.Publish(false);

		out_bridge_found.Publish(false);
		out_stopsign_found.Publish(false);
		out_no_mark_found.Publish(false);
		std::cout << "Line 6" << std::endl;
	}

	if (side_mark_found_left==true && middle_mark_found==true) {
			// calculate angle to side_mark_found_right
			// publish it
		// middle mark has priority
			angle2Mark = (rrlib::math::tAngle<double, rrlib::math::angle::Radian,
					rrlib::math::angle::Signed>) atan(
					(double) temp_mid.Y() / (double) temp_mid.X());
			Angle2Middle_Mark.Publish(angle2Mark);
			out_side_mark_found_left.Publish(false);
			out_middle_mark_found.Publish(middle_mark_found);
			out_side_mark_found_right.Publish(false);

			out_bridge_found.Publish(false);
			out_stopsign_found.Publish(false);
			out_no_mark_found.Publish(false);
			std::cout << "Line 7" << std::endl;
		}
	if (side_mark_found_right==true && middle_mark_found==true) {
				// calculate angle to side_mark_found_right
				// publish it
		// middle mark has priority
				angle2Mark = (rrlib::math::tAngle<double, rrlib::math::angle::Radian,
						rrlib::math::angle::Signed>) atan(
						(double) temp_mid.Y() / (double) temp_mid.X());
				Angle2Middle_Mark.Publish(angle2Mark);
				out_side_mark_found_left.Publish(false);
				out_middle_mark_found.Publish(middle_mark_found);
				out_side_mark_found_right.Publish(false);

				out_bridge_found.Publish(false);
				out_stopsign_found.Publish(false);
				out_no_mark_found.Publish(false);
				std::cout << "Line 8" << std::endl;
			}


	if (bridge_found==true) {
		// calculate angle to brigde_found
		// publish it
		out_bridge_found.Publish(bridge_found);
		out_middle_mark_found.Publish(false);
		out_side_mark_found_right.Publish(false);
		out_side_mark_found_left.Publish(false);

		out_stopsign_found.Publish(false);
		out_no_mark_found.Publish(false);

		dist_bridge=sqrt((double)(temp_mid.X())*(double)(temp_mid.X()) +(double)(temp_mid.Y())*((double)temp_mid.Y()));

		out_dist_bridge.Publish(dist_bridge.Value());
		std::cout << "Line 9" << std::endl;



	}
	if (stopsign_found==true) {
		// calculate angle to brigde_found
		// publish it
		out_stopsign_found.Publish(stopsign_found);
		out_middle_mark_found.Publish(false);
		out_side_mark_found_right.Publish(false);
		out_side_mark_found_left.Publish(false);
		out_bridge_found.Publish(false);

		dist_stop_sign=sqrt((double)(temp_mid.X())*(double)(temp_mid.X()) +(double)(temp_mid.Y())*(double)(temp_mid.Y()));

	    out_dist_stop_sign.Publish(dist_stop_sign.Value());

		out_no_mark_found.Publish(false);
		std::cout << "Line 10" << std::endl;
	}

	if (no_mark_found==true) {
		// calculate angle to no mark found
		// publish it
		out_no_mark_found.Publish(no_mark_found);
		out_middle_mark_found.Publish(false);
		out_side_mark_found_right.Publish(false);
		out_side_mark_found_left.Publish(false);
		out_bridge_found.Publish(false);
		out_stopsign_found.Publish(false);
		std::cout << "Line 11" << std::endl;

	}
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
