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
/*!\file    projects/rc_unimog_control_group2/pUnimogControl.cpp
 *
 * \author  Tobias Groll
 *
 * \date    2017-03-31
 *
 * \brief Contains mUnimogControl
 *
 * \b pUnimogControl
 *
 * controller program for the rc unimog
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <chrono>
#include <cassert>


//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/rc_unimog/gRemoteInterface.h"
#include "projects/rc_unimog_control_group2/gMainControl.h"
#include "projects/rc_unimog_control_group2/object_detection/gObjectDetection.h"
#include "projects/rc_unimog_control_group2/navi/gNavigation.h"

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the UnimogControl module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{}

//----------------------------------------------------------------------
// CreateMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
  auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);


  main_thread->SetCycleTime(std::chrono::milliseconds(20));

  auto interface = new finroc::rc_unimog::gRemoteInterface(main_thread);
  auto control = new finroc::rc_unimog_control_group2::gMainControl(main_thread);
  interface->GetSensorOutputs().ConnectByName(control->GetSensorInputs(),true);
  interface->GetControllerInputs().ConnectByName(control->GetControllerOutputs(),true);

  // connect interface to main_control_group
  interface->GetSensorOutputs().ConnectByName(control->GetSensorInputs(), true);
  control->GetSensorInputs().ConnectByName(control->GetSensorOutputs(), true);
}
