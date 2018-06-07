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
/*!\file    projects/rc_unimog_control_group2/object_detection/mTransmitterDummy.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-02-01
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mTransmitterDummy.h"

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
namespace object_detection
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mTransmitterDummy> cCREATE_ACTION_FOR_M_TRANSMITTERDUMMY("TransmitterDummy");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mTransmitterDummy constructor
//----------------------------------------------------------------------
mTransmitterDummy::mTransmitterDummy(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
//  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
  	  par_x_position_left_line(110),
	  par_y_position_left_line(100),

	  par_x_position_middle_line(300),
	  par_y_position_middle_line(100),

	  par_x_position_right_line(450),
	  par_y_position_right_line(100),

	  par_x_position_stop_sign(),
	  par_y_position_stop_sign(),

	  par_debug(false),

	  list_fake_detected_objects(),

	  left_line(),
	  middle_line(),
	  right_line(),
	  stop_sign()//,

//	angle_null(0)
  {}

//----------------------------------------------------------------------
// mTransmitterDummy destructor
//----------------------------------------------------------------------
mTransmitterDummy::~mTransmitterDummy()
{}

//----------------------------------------------------------------------
// mTransmitterDummy OnStaticParameterChange
//----------------------------------------------------------------------
//void mTransmitterDummy::OnStaticParameterChange()
//{
//  if (this->static_parameter_1.HasChanged())
//  {
//    As this static parameter has changed, do something with its value!
//  }
//}

//----------------------------------------------------------------------
// mTransmitterDummy OnParameterChange
//----------------------------------------------------------------------
//void mTransmitterDummy::OnParameterChange()
//{
//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.

//}

//----------------------------------------------------------------------
// mTransmitterDummy Update
//----------------------------------------------------------------------
void mTransmitterDummy::Update()
{
	if (par_debug.Get()) {
		RRLIB_LOG_PRINT(USER, "TransmitterDummy - update\n");
	}
	// clear list
	list_fake_detected_objects.clear();
	if (par_debug.Get()) {
		RRLIB_LOG_PRINT(USER, "TransmitterDummy - clear_list\n");
	}

	// left line detected
	if(par_x_position_left_line.Get() != 0 || par_y_position_left_line.Get() !=0) {
		left_line.position_middle.SetPosition(par_x_position_left_line.Get(), par_y_position_left_line.Get(), 0);
		left_line.width = 0;
		left_line.length = 0;
		left_line.detected_object_type = left_line.possible_object_type.at(0); // left line = pos#0 in verctor
		list_fake_detected_objects.push_back(left_line);
		if (par_debug.Get()) {
			RRLIB_LOG_PRINTF(USER, "left_line added with pos_x=%lf and pos_y=%lf\n", par_x_position_left_line.Get(), par_y_position_left_line.Get());
		}
	}

	// middle line detected
	if(par_x_position_middle_line.Get() !=0 || par_y_position_middle_line.Get() != 0) {
		middle_line.position_middle.SetPosition(par_x_position_middle_line.Get(), par_y_position_middle_line.Get(), 0);
		middle_line.width = 0;
		middle_line.length = 0;
		middle_line.detected_object_type = middle_line.possible_object_type.at(1); // middle line = pos#1 in vector
		list_fake_detected_objects.push_back(middle_line);
		if (par_debug.Get()) {
			RRLIB_LOG_PRINTF(USER, "middle_line added with pos_x=%lf and pos_y=%lf\n", par_x_position_middle_line.Get(), par_y_position_middle_line.Get());
		}
	}

	// right line
	if(par_x_position_right_line.Get() !=0 || par_y_position_middle_line.Get() !=0) {
		right_line.position_middle.SetPosition(par_x_position_right_line.Get(), par_y_position_right_line.Get(), 0);
		right_line.width = 0;
		right_line.length = 0;
		right_line.detected_object_type = right_line.possible_object_type.at(2); // right mark = pos#2 in vector
		list_fake_detected_objects.push_back(right_line);
		if (par_debug.Get()) {
			RRLIB_LOG_PRINTF(USER, "right_line added with pos_x=%lf and pos_y=%lf\n", par_x_position_right_line.Get(), par_y_position_right_line.Get());
		}
	}

	// stop sign
	if(par_x_position_stop_sign.Get() !=0 || par_y_position_stop_sign.Get() !=0) {
		stop_sign.position_middle.SetPosition(par_x_position_stop_sign.Get(), par_y_position_stop_sign.Get(), 0);
		stop_sign.width = 0;
		stop_sign.length = 0;
		stop_sign.detected_object_type = stop_sign.possible_object_type.at(5); // stop_sign = pos#5 in vector
		list_fake_detected_objects.push_back(stop_sign);
		if (par_debug.Get()) {
			RRLIB_LOG_PRINTF(USER, "stop_sign added with pos_x=%lf and pos_y=%lf\n", par_x_position_stop_sign.Get(), par_y_position_stop_sign.Get());
		}
	}
	// send list
	if (list_fake_detected_objects.size()==0){
		RRLIB_LOG_PRINT(USER, "list is empty");
	}
	out_list_detected_objects.Publish(list_fake_detected_objects);

	if (par_debug.Get()) {
		RRLIB_LOG_PRINT(USER, "list is send");
	}
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
