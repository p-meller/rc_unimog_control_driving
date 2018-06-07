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
/*!\file    projects/rc_unimog_control_group2/object_detection/mTransmitter.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-02-01
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mTransmitter.h"

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
namespace finroc {
namespace rc_unimog_control_group2 {
namespace object_detection {

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mTransmitter> cCREATE_ACTION_FOR_M_TRANSMITTER(
		"Transmitter");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mTransmitter constructor
//----------------------------------------------------------------------
mTransmitter::mTransmitter(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
//  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
		list_detected_objects() {
}

//----------------------------------------------------------------------
// mTransmitter destructor
//----------------------------------------------------------------------
mTransmitter::~mTransmitter() {
}

//----------------------------------------------------------------------
// mTransmitter OnStaticParameterChange
//----------------------------------------------------------------------
//void mTransmitter::OnStaticParameterChange()
//{
//  if (this->static_parameter_1.HasChanged())
//  {
//    As this static parameter has changed, do something with its value!
//  }
//}

//----------------------------------------------------------------------
// mTransmitter OnParameterChange
//----------------------------------------------------------------------
//void mTransmitter::OnParameterChange()
//{
//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
//}

//----------------------------------------------------------------------
// mTransmitter Update
//----------------------------------------------------------------------
void mTransmitter::Update() {

	//    At least one of your input ports has changed. Do something useful with its data.
	//    However, using the .HasChanged() method on each port you can check in more detail.

	if (this->InputChanged()) {
		list_detected_objects.clear();
		if (!in_lanes.Get().empty()) {
			for (int i = 0; i < in_lanes.Get().size(); i++) {
				list_detected_objects.push_back(in_lanes.Get().at(i));
			}
		}
		if (!in_bridge.Get().empty()) {
			for (int i = 0; i < in_bridge.Get().size(); i++) {
				list_detected_objects.push_back(in_bridge.Get().at(i));
			}
		}
		if (!in_pylon.Get().empty()) {
			for (int i = 0; i < in_pylon.Get().size(); i++) {
				list_detected_objects.push_back(in_pylon.Get().at(i));
			}
		}
		if (!in_stop_sign.Get().empty()) {
			for (int i = 0; i < in_stop_sign.Get().size(); i++) {
				list_detected_objects.push_back(in_stop_sign.Get().at(i));
			}
		}
		if (!in_unimog.Get().empty()) {
			for (int i = 0; i < in_unimog.Get().size(); i++) {
				list_detected_objects.push_back(in_unimog.Get().at(i));
			}
		}

		// publish list with all detected objects
		out_list_detected_objects.Publish(list_detected_objects);
	}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
}
