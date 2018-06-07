//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    gMainControl.cpp
 *
 * \author  Unknown
 *
 * \date    2011-11-07
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/gMainControl.h"

#include "projects/rc_unimog_control_group2/object_detection/gObjectDetection.h"
#include "projects/rc_unimog_control_group2/navigation/gNavigation.h"
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
namespace finroc
{
namespace rc_unimog_control_group2
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<gMainControl> cCREATE_ACTION_FOR_G_MAIN_CONTROL("MainControl");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gMainControl constructors
//----------------------------------------------------------------------
gMainControl::gMainControl(finroc::core::tFrameworkElement *parent, const std::string &name,
                           const std::string &structure_config_file)
  : tSenseControlGroup(parent, name, structure_config_file)
{
	// create groups
	auto object_detection = new finroc::rc_unimog_control_group2::object_detection::gObjectDetection(this, "ObjectDetection");
	auto navigation = new finroc::rc_unimog_control_group2::navigation::gNavigation(this, "Navigation");

	navigation->GetName();

	// connect ports
	// object detection to mainConrol
	object_detection->in_images_pi.ConnectTo("/Main Thread/MainControl/Sensor Input/Camera 0");
	object_detection->in_images_xtion.ConnectTo("/Main Thread/MainControl/Sensor Input/Stereo Camera 0");
	object_detection->in_point_cloud.ConnectTo("/Main Thread/MainControl/Sensor Input/Stereo Point Cloud 0");
	// object_detection to navi
//	object_detection->out_list_detected_objects.ConnectTo(navi->si_list_detected_object);

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
