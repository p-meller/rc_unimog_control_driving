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
/*!\file    projects/rc_unimog_control_group2/libraries/tStateManager.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-14
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tStateManager.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
namespace libraries
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

void tStateManager::ChangeState(States state)
{
	switch (state)
	{
	case States::doNothing:
		if (current_state != States::doNothing and current_state != States::stopForSomeTime)
		{
			state_ptr = std::move(
					std::unique_ptr < tStateAbstract > (new tDoNothing()));
			current_state = States::doNothing;
		}
		break;

	case States::stopForSomeTime:
		if (current_state != States::stopForSomeTime)
		{
			state_ptr = std::move(
					std::unique_ptr < tStateAbstract
							> (new tStopForSomeTime()));
			current_state = States::stopForSomeTime;
		}
		break;

	case States::driveOnLeftLane:

		if (current_state != States::driveOnLeftLane and current_state != States::stopForSomeTime)
		{
			state_ptr = std::move(
					std::unique_ptr < tStateAbstract
							> (new tDriveOnLeftLane()));
			current_state = States::driveOnLeftLane;
		}
		break;
	case States::driveOnRightLane:

		if (current_state != States::driveOnRightLane and current_state != States::stopForSomeTime)
		{
			state_ptr = std::move(
					std::unique_ptr < tStateAbstract
							> (new tDriveOnRightLane()));
			current_state = States::driveOnRightLane;
		}
		break;
	default:
		break;

	}
}

void tStateManager::update(float distance_to_left_lane,
		float distance_to_right_lane)
{
	this->distance_to_left_lane=distance_to_left_lane;
	this->distance_to_right_lane=distance_to_right_lane;
	state_ptr->update(this);
}

//----------------------------------------------------------------------
// tStateManager constructors
//----------------------------------------------------------------------
tStateManager::tStateManager() :
		current_state(States::doNothing)
{
	state_ptr = std::move(
			std::unique_ptr < tStateAbstract > (new tDoNothing()));
}

//----------------------------------------------------------------------
// tStateManager destructor
//----------------------------------------------------------------------
tStateManager::~tStateManager()
{
}

//----------------------------------------------------------------------
// tStateManager SomeExampleMethod
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
