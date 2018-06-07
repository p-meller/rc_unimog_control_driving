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
/*!\file    projects/rc_unimog_control_group2/object_detection/mBridgeDetection.cpp
 *
 * \author  Paul Dornhof
 *
 * \date    2018-02-07
 *
 */
//----------------------------------------------------------------------
#include "projects/rc_unimog_control_group2/object_detection/mBridgeDetection.h"

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
namespace finroc {
namespace rc_unimog_control_group2 {
namespace object_detection {

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<mBridgeDetection> cCREATE_ACTION_FOR_M_BRIDGEDETECTION(
		"BridgeDetection");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBridgeDetection constructor
//----------------------------------------------------------------------
mBridgeDetection::mBridgeDetection(core::tFrameworkElement *parent,
		const std::string &name) :
		tModule(parent, name, false), // change to 'true' to make module's ports shared (so that ports in other processes can connect to its output and/or input ports)
//  If you have some member variables, please initialize them here. Especially built-in types (like pointers!). Delete this line otherwise!
//		par_min_height(10),
//		par_max_height(30),
		par_left_threshold(-0.5),
		par_right_threshold(0.5),
		par_max_detection_distance(2),
		par_mount_height(0.3),
//		par_slope(20.0),
//		par_slope_deviation(5),
		par_counter(4000),
		par_debug(false)
{
par_slope.Set(rrlib::math::tAngleDeg(20.0));

}

//----------------------------------------------------------------------
// mBridgeDetection destructor
//----------------------------------------------------------------------
mBridgeDetection::~mBridgeDetection() {
}

//----------------------------------------------------------------------
// mBridgeDetection OnStaticParameterChange
//----------------------------------------------------------------------
//void mBridgeDetection::OnStaticParameterChange()
//{
//  if (this->static_parameter_1.HasChanged())
//  {
//    As this static parameter has changed, do something with its value!
//  }
//}

//----------------------------------------------------------------------
// mBridgeDetection OnParameterChange
//----------------------------------------------------------------------
//void mBridgeDetection::OnParameterChange() {
//  If this method is called, at least on of your parameters has changed. However, each can be checked using its .HasChanged() method.
//	if (par_min_height.HasChanged()) {
//		min_height = par_min_height.Get();
//	}
//	if (par_max_height.HasChanged()) {
//		max_height = par_max_height.Get();
//	}
//	if (par_left_threshold.HasChanged()) {
//		left_threshold = par_left_threshold.Get();
//	}
//	if (par_right_threshold.HasChanged()) {
//		right_threshold = par_right_threshold.Get();
//	}
//	if (par_max_detection_distance.HasChanged()) {
//		max_distance_detection = par_max_detection_distance.Get();
//	}
//	if (par_mount_height.HasChanged()) {
//		mount_height = par_mount_height.Get();
//	}
//	if (par_slope.HasChanged()) {
//		slope = par_slope.Get();
//	}
////	if (par_slope_deviation.HasChanged()) {
////		slope_deviation = par_slope_deviation.Get();
////	}
//	if (par_counter.HasChanged()) {
//		counter = par_counter.Get();
//	}
//	if (par_debug.HasChanged()) {
//		debug = par_debug.Get();
//	}

//}

//----------------------------------------------------------------------
// mBridgeDetection Update
//----------------------------------------------------------------------
void mBridgeDetection::Update() {
	if (this->in_point_cloud.HasChanged()) {
		data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> cloud =
				in_point_cloud.GetPointer();
		const rrlib::math::tVec3f* data =
				reinterpret_cast<const rrlib::math::tVec3f*>(cloud->DataPtr());

		//################# import start, imported from mSimpleGroundPlaneTransformAndFilter

		data_ports::tPortDataPointer<rrlib::distance_data::tDistanceData> output =
				output_point_cloud.GetUnusedBuffer();
		output->Resize(rrlib::distance_data::eDF_CARTESIAN_3D_FLOAT,
				cloud->Capacity(), false, cloud->ExtraDataSize());

		// ground plane is function mx + b;
		float b = static_cast<float>(par_mount_height.Get().Value() * 1000);
		double alpha = rrlib::math::tAngleRad(par_slope.Get()).Value();
		//float m = static_cast<float>(-tan(alpha));
		float threshold = 0.0;//static_cast<float>(ground_plane_threshold.Get().Value());

		// rotation matrix
		float r11 = static_cast<float>(cos(alpha));
		float r12 = static_cast<float>(-sin(alpha));
		float r21 = static_cast<float>(sin(alpha));
		float r22 = static_cast<float>(cos(alpha));

//		// no filter ground
//		if (!filter_ground_plane.Get()) {
//			output->SetDimension(cloud->Dimension());
//			rrlib::math::tVec3f* ovec =
//					reinterpret_cast<rrlib::math::tVec3f*>(output->DataPtr());
//
//			for (size_t i = 0; i < cloud->Dimension(); i++) {
//				if (data->Z() == 0.0f) {
//					ovec->Set(0, 0, 0);
//				} else {
//					ovec->Set(r11 * data->Z() + r12 * data->Y(), data->X(),
//							r21 * data->Z() + r22 * data->Y() + b);
//				}
//				data++;
//				ovec++;
//			}
//			memcpy(output->ExtraDataPtr(), cloud->ExtraDataPtr(),
//					cloud->ExtraDataSize()); // copy any RGB information
//		} else
		// copy colors from input to output
		if (cloud->ExtraDataSize() >= cloud->Dimension() * 3) {
			// TODO optimize
			int point_count = 0;
			rrlib::math::tVec3f* ovec =
					reinterpret_cast<rrlib::math::tVec3f*>(output->DataPtr());
			const uint8_t* input_colors =
					static_cast<const uint8_t*>(cloud->ExtraDataPtr());
			uint8_t* output_colors =
					static_cast<uint8_t*>(output->ExtraDataPtr());
			for (size_t i = 0; i < cloud->Dimension(); i++) {
				// filter point cloud
				if (data->Z() != 0.0f && data->X() <= par_max_detection_distance.Get().Value()*1000 && data->Y() >= par_left_threshold.Get().Value()*1000 && data->Y() <= par_right_threshold.Get().Value()*1000) {
					ovec->Set(r11 * data->Z() + r12 * data->Y(), data->X(),
							r21 * data->Z() + r22 * data->Y() + b);
					if (ovec->Z() > threshold) {
						point_count++;
						ovec++;
						output_colors[0] = input_colors[0];
						output_colors[1] = input_colors[1];
						output_colors[2] = input_colors[2];
						output_colors += 3;
					}
				}
				input_colors += 3;
				data++;
			}
			output->SetDimension(point_count);
		} else {
			// TODO optimize
			int point_count = 0;
			rrlib::math::tVec3f* ovec =
					reinterpret_cast<rrlib::math::tVec3f*>(output->DataPtr());
			for (size_t i = 0; i < cloud->Dimension(); i++) {
				if (data->Z() != 0.0f && data->X() <= par_max_detection_distance.Get().Value()*1000 && data->Y() >= par_left_threshold.Get().Value()*1000 && data->Y() <= par_right_threshold.Get().Value()*1000) {
					ovec->Set(r11 * data->Z() + r12 * data->Y(), data->X(),
							r21 * data->Z() + r22 * data->Y() + b);
					if (ovec->Z() > threshold) {
						point_count++;
						ovec++;
					}
				}
				data++;
			}
			output->SetDimension(point_count);
		}

		output.SetTimestamp(cloud.GetTimestamp());
		output_point_cloud.Publish(output);

		// ######	filter output
		// filtered point cloud data
//		/*const*/ std::vector<rrlib::math::tVec3f> filtered_data;
//		// reset filtered_data;
//		filtered_data.clear();
//
//		for (unsigned int i = 0; i < cloud->Dimension(); i++) {
//			rrlib::math::tVec3f point = data[i];
//			double x_value = point.X(); //.Value();
//			double y_value = point.Y(); //.Value();
//			double z_value = point.Z(); //.Value();
//			if (debug) {
//				RRLIB_LOG_PRINTF(USER,
//						"data position=%i\t x_value=%lf\t y_value=%lf\t z_value=%lf\n ",
//						i, x_value, y_value, z_value);
//			}
//
//			if (data[i].X() <= max_distance_detection.Value()*1000 && data[i].Y() >= left_threshold.Value()*1000 && data[i].Y() <= right_threshold.Value()*1000) {
//				filtered_data.push_back(data[i]);
//
//			}
//
			// check if enough points for bridge
//			bool bridge_found;
//			if (output->Dimension() >= static_cast<uint>(counter) && output->Dimension() != 0) {
//				bridge_found = true;
//			} else {
//				bridge_found = false;
//			}
//			if (bridge_found) {
//				RRLIB_LOG_PRINT(USER, "bridge_found");
//			}

//		}


	}
//}

// ###### import end

//    At least one of your input ports has changed. Do something useful with its data.
//    However, using the .HasChanged() method on each port you can check in more detail.
//  }
//
//  Do something each cycle independent from changing ports.
//
//  this->out_signal_1.Publish(some meaningful value); can be used to publish data via your output ports.
}

//----------------------------------------------------------------------
// mBridgeDetection calculate_slope
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// mBridgeDetection calculate middle
//----------------------------------------------------------------------
void mBridgeDetection::calculate_middle() {

}
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}
