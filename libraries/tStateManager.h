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
/*!\file    projects/rc_unimog_control_group2/libraries/tStateManager.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-14
 *
 * \brief   Contains tStateManager
 *
 * \b tStateManager
 *
 * class that can manage different states
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__libraries__tStateManager_h__
#define __projects__rc_unimog_control_group2__libraries__tStateManager_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "projects/rc_unimog_control_group2/libraries/tStateAbstract.h"
#include "projects/rc_unimog_control_group2/libraries/tDoNothing.h"
#include "projects/rc_unimog_control_group2/libraries/tStopForSomeTime.h"
#include "projects/rc_unimog_control_group2/libraries/tDriveOnRightLane.h"
#include "projects/rc_unimog_control_group2/libraries/tDriveOnLeftLane.h"

#include <memory>
#include <utility>

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
namespace libraries
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

enum class States
{
	doNothing, stopForSomeTime, driveOnRightLane, driveOnLeftLane
};

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * class that can manage different states
 */
class tStateManager
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	tStateManager();

	~tStateManager();

	void ChangeState(States state);

	void update(float distance_to_left_lane, float distance_to_right_lane);

	float distance_to_left_lane, distance_to_right_lane, velocity,distance_error;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

	std::unique_ptr<tStateAbstract> state_ptr;

	States current_state;


	friend tStopForSomeTime;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
