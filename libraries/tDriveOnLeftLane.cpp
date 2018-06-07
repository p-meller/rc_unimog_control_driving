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
/*!\file    projects/rc_unimog_control_group2/libraries/tDriveOnLeftLane.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-14
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tDriveOnLeftLane.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "projects/rc_unimog_control_group2/libraries/tStateManager.h"

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

void tDriveOnLeftLane::update(tStateManager* state)
{
state->distance_error=state->distance_to_left_lane;
state->velocity=0.06;
}

//----------------------------------------------------------------------
// tDriveOnLeftLane constructors
//----------------------------------------------------------------------
tDriveOnLeftLane::tDriveOnLeftLane()
{}

//----------------------------------------------------------------------
// tDriveOnLeftLane destructor
//----------------------------------------------------------------------
tDriveOnLeftLane::~tDriveOnLeftLane()
{}

//----------------------------------------------------------------------
// tDriveOnLeftLane SomeExampleMethod
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
