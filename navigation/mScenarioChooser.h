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
/*!\file    projects/rc_unimog_control_group2/navigation/mScenarioChooser.h
 *
 * \author  Piotr Meller
 *
 * \date    2018-02-12
 *
 * \brief Contains mScenarioChooser
 *
 * \b mScenarioChooser
 *
 * this module can change robot behavior
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navigation__mScenarioChooser_h__
#define __projects__rc_unimog_control_group2__navigation__mScenarioChooser_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "projects/rc_unimog_control_group2/libraries/tStateManager.h"
#include "rrlib/si_units/si_units.h"

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
namespace navigation
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this module can change robot behavior
 */
class mScenarioChooser : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<float> distance_to_left_lane,distance_to_right_lane;

	tOutput<float> distance_error;
	tOutput<rrlib::si_units::tVelocity<>> velocity;

	tInput<bool> stop_detected;
	tInput<bool> obstacle_detected;
	tInput<bool> drive;

	tParameter<float> velocity_par;



//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mScenarioChooser(core::tFrameworkElement *parent, const std::string &name = "ScenarioChooser");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mScenarioChooser();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  libraries::tStateManager manager;

  bool stop_sign_detected;

  float vel;

  virtual void OnStaticParameterChange() override;

  virtual void OnParameterChange() override;

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
