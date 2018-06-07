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
/*!\file    projects/rc_unimog_control_group2/navi/mConvertCurvature.cpp
 *
 * \author  Harold Moses Mutabazi
 *
 * \date    2018-02-02
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/navi/mConvertCurvature.h"

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
namespace navi
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mConvertCurvature> cCREATE_ACTION_FOR_M_CONVERTCURVATURE("ConvertCurvature");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mConvertCurvature constructor
//----------------------------------------------------------------------
mConvertCurvature::mConvertCurvature(core::tFrameworkElement *parent, const std::string &name) :
  tModule(parent, name, false) // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)

{}

//----------------------------------------------------------------------
// mConvertCurvature destructor
//----------------------------------------------------------------------
mConvertCurvature::~mConvertCurvature()
{}

//----------------------------------------------------------------------
// mConvertCurvature OnStaticParameterChange
//----------------------------------------------------------------------
void mConvertCurvature::OnStaticParameterChange()
{

}

//----------------------------------------------------------------------
// mConvertCurvature OnParameterChange
//----------------------------------------------------------------------
void mConvertCurvature::OnParameterChange()
{

}

//----------------------------------------------------------------------
// mConvertCurvature Update
//----------------------------------------------------------------------
void mConvertCurvature::Update()
{

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
