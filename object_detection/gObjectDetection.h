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
/*!\file    projects/rc_unimog_control_group2/object_detection/gObjectDetection.h
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-23
 *
 * \brief Contains gObjectDetection
 *
 * \b gObjectDetection
 *
 * this group contains modules which tasks are to detect and recognize objects
 *
 */
//----------------------------------------------------------------------
#ifndef __projects__rc_unimog_control_group2__object_detection__gObjectDetection_h__
#define __projects__rc_unimog_control_group2__object_detection__gObjectDetection_h__

#include "plugins/structure/tGroup.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/coviroa/tImage.h"
#include "projects/rc_unimog_control_group2/libraries/tDetectedObject.h"
#include "rrlib/distance_data/tDistanceData.h"
//#include "projects/camera_control/mStopSignDetection.h"
#include "projects/rc_unimog_control_group2/object_detection/mBridgeDetection.h"
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
 * this group contains modules which tasks are to detect and recognize objects
 */
class gObjectDetection: public structure::tGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:

	tInput<std::vector<rrlib::coviroa::tImage>> in_images_xtion;	// Input xtion rgb camera
	tInput<std::vector<rrlib::coviroa::tImage>> in_images_pi;	// input pi camera
	tInput<rrlib::distance_data::tDistanceData> in_point_cloud;	// input point cloud

	tOutput<rrlib::coviroa::tImage> out_image;
	tOutput<std::vector<libraries::tDetectedObject>> out_list_detected_objects;

	//tParameter<float> par_min_distance_distance_data;
	//tParameter<float> par_max_distance_distance_data;
//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

	gObjectDetection(core::tFrameworkElement *parent, const std::string &name =
			"ObjectDetection", const std::string &structure_config_file =
			__FILE__".xml");

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

	/*! Destructor
	 *
	 * The destructor of groups is declared protected to avoid accidental deletion. Deleting
	 * groups is already handled by the framework.
	 */
	~gObjectDetection();

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
