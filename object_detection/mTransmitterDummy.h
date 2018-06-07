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
/*!\file    projects/rc_unimog_control_group2/object_detection/mTransmitterDummy.h
 *
 * \author  Paul Dornhof
 *
 * \date    2018-02-01
 *
 * \brief Contains mTransmitterDummy
 *
 * \b mTransmitterDummy
 *
 * this module is a dummy which sends fake list with detected object, need for tests
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__object_detection__mTransmitterDummy_h__
#define __projects__rc_unimog_control_group2__object_detection__mTransmitterDummy_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/libraries/tDetectedObject.h"
#include "rrlib/math/tAngle.h"

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
namespace object_detection
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! SHORT_DESCRIPTION
/*!
 * this module is a dummy which sends fake list with detected object, need for tests
 */
class mTransmitterDummy : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:
	tOutput<std::vector<libraries::tDetectedObject>> out_list_detected_objects;

	tParameter<double> par_x_position_left_line;
	tParameter<double> par_y_position_left_line;

	tParameter<double> par_x_position_middle_line;
	tParameter<double> par_y_position_middle_line;

	tParameter<double> par_x_position_right_line;
	tParameter<double> par_y_position_right_line;

	tParameter<double> par_x_position_stop_sign;
	tParameter<double> par_y_position_stop_sign;

	tParameter<bool> par_debug;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mTransmitterDummy(core::tFrameworkElement *parent, const std::string &name = "TransmitterDummy");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mTransmitterDummy();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

//  Here is the right place for your variables. Replace this line by your declarations!

  std::vector<libraries::tDetectedObject> list_fake_detected_objects;

  libraries::tDetectedObject left_line;
  libraries::tDetectedObject middle_line;
  libraries::tDetectedObject right_line;
  libraries::tDetectedObject stop_sign;

//  rrlib::math::tAngle angle_null;
//  virtual void OnStaticParameterChange() override;   //Might be needed to process static parameters. Delete otherwise!

//  virtual void OnParameterChange() override;   //Might be needed to react to changes in parameters independent from Update() calls. Delete otherwise!

  virtual void Update() override;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}



#endif
