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
/*!\file    projects/rc_unimog_control_group2/object_detection/mDrive.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-07
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navigation/mDrive.h"

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
namespace navigation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mDrive> cCREATE_ACTION_FOR_M_DRIVE(
		"Drive");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mDrive constructor
//----------------------------------------------------------------------
mDrive::mDrive(core::tFrameworkElement *parent, const std::string &name) :
		tModule(parent, name, false), reverse_steering(false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)

{
}

//----------------------------------------------------------------------
// mDrive destructor
//----------------------------------------------------------------------
mDrive::~mDrive()
{
}

//----------------------------------------------------------------------
// mDrive OnStaticParameterChange
//----------------------------------------------------------------------
void mDrive::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mDrive OnParameterChange
//----------------------------------------------------------------------
void mDrive::OnParameterChange()
{
	if (reverse_steering.Get() == true)
		reverse = -1;
	else
		reverse = 1;
}

//----------------------------------------------------------------------
// mDrive Update
//----------------------------------------------------------------------
void mDrive::Update()
{
	if (this->InputChanged())
	{

		this->curvature.Publish(reverse * (this->in.Get() + 6));
	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
