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
/*!\file    projects/rc_unimog_control_group2/object_detection/gObjectDetection.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-01-23
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/gObjectDetection.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
//#include "projects/rc_unimog_control_group2/object_detection/mTransmitter.h"
//#include "projects/rc_unimog_control_group2/object_detection/mLaneDetector.h"
//#include "projects/rc_unimog_control_group2/object_detection/mPylonDetector.h"
#include "projects/rc_unimog_control_group2/object_detection/mBridgeDetection.h"

#include "projects/rc_unimog_control_group2/object_detection/mTransmitterDummy.h"
#include "projects/rc_unimog_control_group2/utilities/camera_utilities/mMirrorImage.h"
#include "projects/rc_unimog_control_group2/utilities/camera_utilities/mToBirdEyeView.h"
#include "projects/rc_unimog_control_group2/utilities/camera_utilities/mFromVectorToImage.h"
//#include "projects/rc_unimog_control_group2/object_detection/mLineDetection.h"
#include "projects/rc_unimog_control_group2/object_detection/mDetectOutsideLine.h"
#include "projects/rc_unimog_control_group2/object_detection/mDetectMiddleLine.h"
#include "projects/rc_unimog_control_group2/object_detection/mStopSignDetection.h"
#include "projects/rc_unimog_control_group2/object_detection/mCollisionDetection.h"


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
namespace object_detection
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
static runtime_construction::tStandardCreateModuleAction<gObjectDetection> cCREATE_ACTION_FOR_G_OBJECTDETECTION("ObjectDetection");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gObjectDetection constructor
//----------------------------------------------------------------------
gObjectDetection::gObjectDetection(core::tFrameworkElement *parent, const std::string &name,
                                   const std::string &structure_config_file) :
  tGroup(parent, name, structure_config_file, false)
// change to 'true' to make group's ports shared (so that ports in other
// processes can connect to its output and/or input ports)
{
	// create modules
	//mTransmitterDummy* transmitter_dummy = new mTransmitterDummy(this, "TransmitterDummy");

	auto mirror_image = new utilities::camera_utilities::mMirrorImage(this);
	auto to_bird_eye_view = new utilities::camera_utilities::mToBirdEyeView(this);
	//auto object_detection = new mLineDetection(this);group2
	//auto pid_controller =  new closed_loop_control::mPIDController<float,float>(this);
	//auto drive = new mDrive(this);
	auto from_vector_to_image = new utilities::camera_utilities::mFromVectorToImage(this ,"xtion camera");
	//auto from_vector_to_image2 = new utilities::camera_utilities::mFromVectorToImage(this ,"pi camera");
	auto bridge_detection = new mBridgeDetection(this, "BridgeDetection");
	auto detect_outside_line = new mDetectOutsideLine(this);
	auto detect_middle_line = new mDetectMiddleLine(this);
	auto stop_sign_detection = new mStopSignDetection(this);
	auto collision_detection = new mCollisionDetection(this);


	//from_vector_to_image->par_number_of_outputs.Set(1);
	// connect ports
	// transmitter_dummy
	//transmitter_dummy->out_list_detected_objects.ConnectTo(out_list_detected_objects);

	from_vector_to_image->in_images.ConnectTo("/Main Thread/MainControl/Sensor Input/Stereo Camera 0");
	mirror_image->in_image.ConnectTo(from_vector_to_image->out_image);
	mirror_image->out_image.ConnectTo(to_bird_eye_view->in_image);

	//to_bird_eye_view->out_image.ConnectTo(object_detection->in_image);
	//object_detection->distance_to_center.ConnectTo(pid_controller->si_measured_value);
	//drive->in.ConnectTo(pid_controller->co_control_value);
	// object_detection to bridge_detection
	this->in_point_cloud.ConnectTo(bridge_detection->in_point_cloud);
	to_bird_eye_view->out_image.ConnectTo(detect_outside_line->in_image);
	to_bird_eye_view->out_image.ConnectTo(detect_middle_line->in_image);
	stop_sign_detection->in_image.ConnectTo(mirror_image->out_image);

	//collision_detection->input_rgb_image.ConnectTo(from_vector_to_image2->out_image);

	collision_detection->input_rgb_image.ConnectTo(mirror_image->out_image);
	collision_detection->stop_center.ConnectTo(stop_sign_detection->stop_center);
	collision_detection->input_us_front.ConnectTo("/Main Thread/MainControl/Sensor Input/Us Front");




	//from_vector_to_image->in_images.ConnectTo("/Main Thread/MainControl/Sensor Input/Stereo Camera 0");
	//from_vector_to_image2->in_images.ConnectTo("/Main Thread/MainControl/Sensor Input/Camera 0");
	//drive->curvature.ConnectTo("/Main Thread/MainControl/Controller Output/Curvature");


	//pid_controller->par_kp.Set(0.1);
	//pid_controller->par_kI.Set(0.1);
//	pid_controller->par_kD.Set(0.05);
//
//	pid_controller->par_max_value.Set(22);
//	pid_controller->par_min_value.Set(-22);
//	pid_controller->ci_target_value.Set(0);
//	pid_controller->par_cut_max_min.Set(true);

	//this->in_images_xtion.ConnectTo("/Main Thread/MainControl/Sensor Input/Stereo Camera 0");
}

//----------------------------------------------------------------------
// gObjectDetection destructor
//----------------------------------------------------------------------
gObjectDetection::~gObjectDetection()
{}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
