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
/*!\file    projects/rc_unimog_control_group2/navi/gNavigation.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-23
 *
 * \brief Contains gNavigation
 *
 * \b gNavigation
 *
 * This group module puts to togather all modules for navigation
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__gNavigation_h__
#define __projects__rc_unimog_control_group2__navi__gNavigation_h__

#include "plugins/structure/tSenseControlGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include "rrlib/si_units/si_units.h"
#include "projects/rc_unimog_control_group2/libraries/tDetectedObject.h"
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
namespace navi
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * This group module puts to togather all modules for navigation
 */
class gNavigation : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

//  tControllerInput<double> ci_signal_1;
	//tControllerOutput<rrlib::si_units::tVelocity<double>> co_velocity;
	//tControllerOutput<rrlib::si_units::tCurvature<double>> co_curvature;

//	tControllerInput<rrlib::si_units::tVelocity<double>> ci_velocity;
//		tControllerInput<rrlib::si_units::tCurvature<double>> ci_curvature;
//
  ///tSensorInput<std::vector<libraries::tDetectedObject>> si_list_detected_object;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  gNavigation(core::tFrameworkElement *parent, const std::string &name = "Navigation",
              const std::string &structure_config_file = __FILE__".xml");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of groups is declared protected to avoid accidental deletion. Deleting
   * groups is already handled by the framework.
   */
  ~gNavigation();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
