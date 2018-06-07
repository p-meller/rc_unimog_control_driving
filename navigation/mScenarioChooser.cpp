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
/*!\file    projects/rc_unimog_control_group2/navigation/mScenarioChooser.cpp
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-12
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navigation/mScenarioChooser.h"

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
runtime_construction::tStandardCreateModuleAction<mScenarioChooser> cCREATE_ACTION_FOR_M_SCENARIOCHOOSER(
		"ScenarioChooser");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mScenarioChooser constructor
//----------------------------------------------------------------------
mScenarioChooser::mScenarioChooser(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false),stop_sign_detected(false) ,vel(0) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
{
}

//----------------------------------------------------------------------
// mScenarioChooser destructor
//----------------------------------------------------------------------
mScenarioChooser::~mScenarioChooser()
{
}

//----------------------------------------------------------------------
// mScenarioChooser OnStaticParameterChange
//----------------------------------------------------------------------
void mScenarioChooser::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mScenarioChooser OnParameterChange
//----------------------------------------------------------------------
void mScenarioChooser::OnParameterChange()
{
	vel = velocity_par.Get();
}

//----------------------------------------------------------------------
// mScenarioChooser Update
//----------------------------------------------------------------------
void mScenarioChooser::Update()
{
	if (this->InputChanged())
	{
		bool d = drive.Get();

		if (d == false)
		{
			manager.ChangeState(libraries::States::doNothing);
		}
		else
		{
			bool det = obstacle_detected.Get();
			if (det == true)
			{
				manager.ChangeState(libraries::States::driveOnLeftLane);
			}
			else
			{
				manager.ChangeState(libraries::States::driveOnRightLane);
			}

			bool st = stop_detected.Get();
			if (stop_sign_detected == true)
			{
				if (st == false)
				{
					this->manager.ChangeState(
							libraries::States::stopForSomeTime);
					stop_sign_detected = false;
				}
			}
			else
			{
				if (st == true)
				{
					stop_sign_detected = true;
				}
			}

		}

		manager.update(this->distance_to_left_lane.Get(),
				this->distance_to_right_lane.Get());

		this->distance_error.Publish(this->manager.distance_error);
		if (this->manager.velocity == 0)
		{
			this->velocity.Publish(rrlib::si_units::tVelocity<>(0));
		}
		else
		{
			this->velocity.Publish(rrlib::si_units::tVelocity<>(vel));
		}

	}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
