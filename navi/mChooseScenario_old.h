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
/*!\file    projects/rc_unimog_control_group2/navi/mChooseScenario.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-01-30
 *
 * \brief Contains mChooseScenario
 *
 * \b mChooseScenario
 *
 * this module evaluates scenarios to be considered
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mChooseScenario_h__
#define __projects__rc_unimog_control_group2__navi__mChooseScenario_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tDetectedObject.h"
#include "rrlib/localization/tPose.h"
#include "rrlib/math/tAngle.h"
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
 * this module evaluates scenarios to be considered
 */
class mChooseScenario : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<std::vector<libraries::tDetectedObject>> in_list_detected_objects;

	tOutput<rrlib::math::tAngle<double>> Angle2Middle_Mark;// in radians
	tOutput<rrlib::si_units::tLength<double>>out_dist_bridge;
	tOutput<rrlib::si_units::tLength<double>>out_dist_stop_sign;
	tOutput<bool> out_middle_mark_found;
	// direction to be figured out

	tOutput<bool> out_side_mark_found_right;
	tOutput<bool> out_side_mark_found_left;
	tOutput<bool> out_bridge_found;
	tOutput<bool> out_stopsign_found;
	tOutput<bool> out_junction_found;
	tOutput<bool> out_no_mark_found;



//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mChooseScenario(core::tFrameworkElement *parent, const std::string &name = "ChooseScenario");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mChooseScenario();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


  virtual void OnStaticParameterChange() override;
  virtual void OnParameterChange() override;
  virtual void Update() override;
  void CheckList();

	bool middle_mark_found;
	bool side_mark_found_right;
	bool side_mark_found_left;
	bool bridge_found;
	bool stopsign_found;
	bool junction_found;
	int i;
bool no_mark_found;
std::vector<libraries::tDetectedObject> temp_list;
std::vector<libraries::tDetectedObject> temp_list_current;
std::vector<libraries::tDetectedObject> temp_width;
std::vector<libraries::tDetectedObject> temp_length;
rrlib::localization::tPose3D<double> temp_mid;
rrlib::si_units::tLength<double>temp_wid;
rrlib::si_units::tLength<double>temp_leng;

rrlib::math::tAngle<double>angle2Mark;
rrlib::si_units::tLength<double>dist_bridge;
rrlib::si_units::tLength<double>dist_stop_sign;



};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
