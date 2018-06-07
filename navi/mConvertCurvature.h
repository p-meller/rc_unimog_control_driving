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
/*!\file    projects/rc_unimog_control_group2/navi/mConvertCurvature.h
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-02-02
 *
 * \brief Contains mConvertCurvature
 *
 * \b mConvertCurvature
 *
 * this module converts curveturethis  module converts curvature because carvature zero doesnt give staright motion
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__navi__mConvertCurvature_h__
#define __projects__rc_unimog_control_group2__navi__mConvertCurvature_h__

#include "plugins/structure/tModule.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
 * this module converts curveturethis  module converts curvature because carvature zero doesnt give staright motion
 */
class mConvertCurvature : public structure::tModule
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:
//tInput<rrlib::si_units::tCurvature<double> in_curvature;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  mConvertCurvature(core::tFrameworkElement *parent, const std::string &name = "ConvertCurvature");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  /*! Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  ~mConvertCurvature();

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


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
