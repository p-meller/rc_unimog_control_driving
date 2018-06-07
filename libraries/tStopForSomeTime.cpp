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
/*!\file    projects/rc_unimog_control_group2/libraries/tStopForSomeTime.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-14
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tStopForSomeTime.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "projects/rc_unimog_control_group2/libraries/tStateManager.h"

#include <thread>
#include <chrono>
#include <iostream>

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

void tStopForSomeTime::update(tStateManager* state)
{
	state->distance_error = 0;
	state->velocity = 0;

	if(wait())
	{
	state->state_ptr = std::move(
			std::unique_ptr < tStateAbstract > (new tDoNothing()));
	state->current_state = States::doNothing;
	}
}

bool tStopForSomeTime::wait()
{

	bool ok = false;

	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed_time = std::chrono::duration_cast < std::chrono::milliseconds
			> (now - current_time);
	current_time = now;
	time_count = time_count + elapsed_time;
	if (time_count > std::chrono::milliseconds(2000))
	{
		//std::cout << time_count.count() << std::endl;
		time_count = std::chrono::milliseconds(0);
		ok = true;
	}

	return ok;
}

//----------------------------------------------------------------------
// tStopForSomeTime constructors
//----------------------------------------------------------------------
tStopForSomeTime::tStopForSomeTime(): time_count(0)
{
	current_time = std::chrono::high_resolution_clock::now();
}

//----------------------------------------------------------------------
// tStopForSomeTime destructor
//----------------------------------------------------------------------
tStopForSomeTime::~tStopForSomeTime()
{
}

//----------------------------------------------------------------------
// tStopForSomeTime SomeExampleMethod
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
